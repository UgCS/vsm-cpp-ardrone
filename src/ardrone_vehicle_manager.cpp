// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <ardrone_vehicle_manager.h>
#include <ardrone_vehicle.h>
#include <ugcs/vsm/transport_detector.h>
#include <ugcs/vsm/param_setter.h>

using namespace ugcs::vsm;

constexpr std::chrono::seconds Ardrone_vehicle_manager::MAVLINK_TIMEOUT;

constexpr std::chrono::seconds Ardrone_vehicle_manager::CONFIG_RETRY_TIMEOUT;

constexpr std::chrono::seconds Ardrone_vehicle_manager::TCP_TIMEOUT;

Ardrone_vehicle_manager::Ardrone_vehicle_manager() :
Mavlink_vehicle_manager(
        "Ardrone",
        "vehicle.ardrone",
        mavlink::Extension::Get())
{

}

void
Ardrone_vehicle_manager::Register_detectors()
{
    /* Bind a detector to udp port specified in config. */
    Transport_detector::Get_instance()->Add_detector(
            config_prefix + ".detector",
            ugcs::vsm::Transport_detector::Make_connect_handler(
                    &Ardrone_vehicle_manager::On_connection_detected,
                    Shared_from_this()),
                    Shared_from_this());
}

void
Ardrone_vehicle_manager::Detection_ended(Detection_ctx::Ptr ctx)
{
	detectors.erase(ctx);
}

void
Ardrone_vehicle_manager::On_manager_disable()
{
	detectors.clear();
}

Mavlink_vehicle::Ptr
Ardrone_vehicle_manager::Create_mavlink_vehicle(
        Mavlink_demuxer::System_id system_id,
        Mavlink_demuxer::Component_id component_id,
        mavlink::MAV_TYPE type,
        Io_stream::Ref stream,
        ugcs::vsm::Socket_address::Ptr peer_addr,
        ugcs::vsm::Optional<std::string> mission_dump_path,
        std::string serial_number,
        std::string model_name,
        bool)
{
    return Ardrone_vehicle::Create(
            system_id,
            component_id,
            type,
            Vehicle::Capabilities(
            Vehicle::Capability::LAND_AVAILABLE,
            Vehicle::Capability::AUTO_MODE_AVAILABLE,
            Vehicle::Capability::PAUSE_MISSION_AVAILABLE,
            Vehicle::Capability::RESUME_MISSION_AVAILABLE,
            Vehicle::Capability::EMERGENCY_LAND_AVAILABLE),
            stream,
            peer_addr,
            mission_dump_path,
            serial_number,
            model_name);
}

void
Ardrone_vehicle_manager::On_connection_detected(
        std::string name,
        int ,
        ugcs::vsm::Socket_address::Ptr peer,
        ugcs::vsm::Io_stream::Ref stream)
{
    /** Ardrone specific tweak.
     * Sometimes your dhcp address can change but ArDrone keeps sending telemetry
     * to previous address. This can happen when more than one client connects
     * to ardrone wifi access-point.
     * To fix this we send zero-length UDP packet to ardrone which makes Ardrone
     * to start sending telemetry to our new address.
     */
    Io_result client_result;
    stream->Write(Io_buffer::Create(""), Make_setter(client_result));

    Detection_ctx::Ptr ctx = Detection_ctx::Create(stream, name, peer);
    detectors.insert(ctx);
    Start_detection(ctx);
}

Ardrone_vehicle_manager::Detection_ctx::Detection_ctx(
		ugcs::vsm::Io_stream::Ref mav_stream,
		const std::string& port_name,
		ugcs::vsm::Socket_address::Ptr peer) :
				mav_stream(mav_stream),
				drone_addr(peer),
				port_name(port_name)
{
	drone_udp_addr = Socket_address::Create(
			peer->Get_name_as_c_str(),
			std::to_string(AT_PORT));

	drone_tcp_addr = Socket_address::Create(
			peer->Get_name_as_c_str(),
			std::to_string(CONFIG_PORT));
}

void
Ardrone_vehicle_manager::Start_detection(Detection_ctx::Ptr ctx)
{
	Io_result result;
	Socket_processor::Get_instance()->Bind_udp(
			Socket_address::Create("0.0.0.0", std::to_string(AT_PORT)),
			Make_setter(ctx->at_stream, result));
	if (result != Io_result::OK) {
		LOG_WARNING("Could not bind to UDP 0.0.0.0:%d", AT_PORT);
		Detection_ended(ctx);
	} else {
		/* Wait for some data from Mavlink stream to make sure something is there. */
	    /* max_read must be the largest possible mavlink packet here
	     * otherwise the read fails on windows.*/
	    ctx->mav_waiter = ctx->mav_stream->Read(
	            ugcs::vsm::MIN_UDP_PAYLOAD_SIZE_TO_READ,
	            16,
				Make_read_callback(
						&Ardrone_vehicle_manager::On_mavlink_read,
						this,
						Detection_ctx::Weak_ptr(ctx)),
				Get_worker());
		ctx->mav_waiter.Timeout(MAVLINK_TIMEOUT);
	}
}

Ardrone_vehicle_manager::Detection_ctx::~Detection_ctx()
{
	connect_waiter.Abort();
	mav_waiter.Abort();
	config_stream = nullptr;
	at_stream = nullptr;
	if (mav_stream) {
		Transport_detector::Get_instance()->Protocol_not_detected(mav_stream);
		mav_stream = nullptr;
	}
	if (filter) {
		filter->Disable();
		filter = nullptr;
	}
	if (config_timer) {
		config_timer->Cancel();
		config_timer = nullptr;
	}
}

void
Ardrone_vehicle_manager::On_tcp_connected(
    			ugcs::vsm::Socket_processor::Stream::Ref stream,
    			ugcs::vsm::Io_result result,
    			Detection_ctx::Weak_ptr ctx_weak)
{
	auto ctx = ctx_weak.lock();
	if (result != Io_result::OK) {
		Reconnect_tcp(ctx);
	} else {
		ctx->config_stream = stream;
		if (ctx->filter) {
			ctx->filter->Disable();
		}
		ctx->filter = Text_stream_filter::Create(ctx->config_stream, Get_worker());
		ctx->filter->Add_entry(
				regex::regex("network:ssid_single_player *= *(.*)"),
				ugcs::vsm::Text_stream_filter::Make_match_handler(
						&Ardrone_vehicle_manager::String_param_read_cb,
						this,
						ctx_weak,
						&ctx->ssid));
		ctx->filter->Add_entry(
				regex::regex("general:ardrone_name *= *(.*)"),
				ugcs::vsm::Text_stream_filter::Make_match_handler(
						&Ardrone_vehicle_manager::String_param_read_cb,
						this,
						ctx_weak,
						&ctx->name));
		ctx->filter->Enable();
		Request_config(ctx);
	}
}

bool
Ardrone_vehicle_manager::Request_config(Detection_ctx::Weak_ptr ctx_weak)
{
	auto ctx = ctx_weak.lock();
	if(!ctx->read_attempts_left--) {
		LOG_DEBUG("Could not read drone configuration.");
		Detection_ended(ctx);
		return false;
	}
	if (ctx->config_timer) {
		ctx->config_timer->Cancel();
	}
	ctx->config_timer = Timer_processor::Get_instance()->Create_timer(
			CONFIG_RETRY_TIMEOUT,
			Make_callback(
					&Ardrone_vehicle_manager::Request_config,
					this,
					Detection_ctx::Weak_ptr(ctx)),
			Get_worker());
	Send_at(ctx, "AT*CTRL", "4,0");
	return false;
}

void
Ardrone_vehicle_manager::On_mavlink_read(
		ugcs::vsm::Io_buffer::Ptr,
		ugcs::vsm::Io_result result,
		Detection_ctx::Weak_ptr ctx_weak)
{
	auto ctx = ctx_weak.lock();
	if (result != Io_result::OK) {
		/* Seems, ArDrone is not present. */
		Detection_ended(ctx);
	} else {
		/* Always acknowledge any pending configuration receiving state. */
		Send_at(ctx, "AT*CTRL", "5,0");
		Reconnect_tcp(ctx);
	}
}

void
Ardrone_vehicle_manager::Reconnect_tcp(Detection_ctx::Ptr ctx)
{
	if (ctx->config_stream) {
		ctx->config_stream->Close();
	}
	if (ctx->config_timer) {
		ctx->config_timer->Cancel();
	}
	ctx->connect_waiter.Abort();

	if (!ctx->connect_attempts_left--) {
		LOG_DEBUG("Could not establish TCP connection to %s.",
				ctx->drone_tcp_addr->Get_as_string().c_str());
		Detection_ended(ctx);
		return;
	}

	ctx->connect_waiter = Socket_processor::Get_instance()->Connect(
			ctx->drone_tcp_addr,
			Make_socket_connect_callback(
					&Ardrone_vehicle_manager::On_tcp_connected,
					this,
					Detection_ctx::Weak_ptr(ctx)),
			Get_worker());
	ctx->connect_waiter.Timeout(TCP_TIMEOUT);
}

void
Ardrone_vehicle_manager::Try_finish_detecton(Detection_ctx::Ptr ctx)
{
	if (ctx->ssid && ctx->name) {
		/* We don't want it do be closed anymore. */
		auto mav_stream = std::move(ctx->mav_stream);
		Detection_ended(ctx);
		Handle_new_connection(
				ctx->port_name,
				0,
				ctx->drone_addr,
				mav_stream,
				ugcs::vsm::mavlink::MAV_AUTOPILOT_GENERIC,
				false,
				ctx->name,
				ctx->ssid);
	}
}

void
Ardrone_vehicle_manager::Send_at(
		Detection_ctx::Ptr ctx,
		const std::string& prefix,
		const std::string& args)
{
	Io_result result;
	ctx->at_stream->Write_to(
			Io_buffer::Create(prefix + "=" + std::to_string(ctx->at_seq++) + "," + args + "\r"),
			ctx->drone_udp_addr,
			Make_setter(result));
}

bool
Ardrone_vehicle_manager::String_param_read_cb(
		regex::smatch* match,
		ugcs::vsm::Text_stream_filter::Lines_list*,
		ugcs::vsm::Io_result result,
		Detection_ctx::Weak_ptr ctx_weak,
		ugcs::vsm::Optional<std::string>* param)
{
	auto ctx = ctx_weak.lock();
	Send_at(ctx, "AT*CTRL", "5,0");
	if (result == Io_result::OK) {
		*param = (*match)[1].str();
		Try_finish_detecton(ctx);
	} else {
		Reconnect_tcp(ctx);
	}
	return false;
}
