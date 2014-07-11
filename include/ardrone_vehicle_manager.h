// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file ardrone_vehicle_manager.h
 */
#ifndef _ARDRONE_VEHICLE_MANAGER_H_
#define _ARDRONE_VEHICLE_MANAGER_H_

#include <mavlink_vehicle_manager.h>
#include <ugcs/vsm/text_stream_filter.h>

class Ardrone_vehicle_manager: public Mavlink_vehicle_manager {
    DEFINE_COMMON_CLASS(Ardrone_vehicle_manager, Mavlink_vehicle_manager)

public:

    /** Constructor. */
    Ardrone_vehicle_manager();

    /** Handler for a new transport connection. */
    void
    On_connection_detected(
    		std::string,
    		int,
    		ugcs::vsm::Socket_address::Ptr,
    		ugcs::vsm::Io_stream::Ref);

private:

    /** ArDrone specific detection context used for reading network SSID and
     * drone name from TCP connection. */
    class Detection_ctx: public std::enable_shared_from_this<Detection_ctx> {
    	DEFINE_COMMON_CLASS(Detection_ctx, Detection_ctx)
    public:
		/** Construct using Mavlink stream, port name and drone address. */
    	Detection_ctx(
    			ugcs::vsm::Io_stream::Ref,
    			const std::string&,
    			ugcs::vsm::Socket_address::Ptr);

    	/** Close all. */
    	~Detection_ctx();

    	/** TCP stream for receiving configuration data. */
    	ugcs::vsm::Io_stream::Ref config_stream;

    	/** UDP steam for sending AT commands. */
    	ugcs::vsm::Socket_processor::Stream::Ref at_stream;

    	/** Mavlink stream from the detector. */
    	ugcs::vsm::Io_stream::Ref mav_stream;

    	/** Connection operation waiter. */
    	ugcs::vsm::Operation_waiter connect_waiter;

    	/** Mavlink data waiter. */
    	ugcs::vsm::Operation_waiter mav_waiter;

    	/** Drone UDP address from transport detector. */
    	ugcs::vsm::Socket_address::Ptr drone_addr;

    	/** Drone UDP address. */
    	ugcs::vsm::Socket_address::Ptr drone_udp_addr;

    	/** Drone TCP address. */
    	ugcs::vsm::Socket_address::Ptr drone_tcp_addr;

    	/** Configuration TCP stream filter. */
    	ugcs::vsm::Text_stream_filter::Ptr filter;

    	/** Configuration read retry timer. */
    	ugcs::vsm::Timer_processor::Timer::Ptr config_timer;

    	/** Port name from the detector. */
    	std::string port_name;

    	/** Attempts for TCP connection establishments. */
    	size_t connect_attempts_left = 3;

    	/** Attempts for configuration reading. */
    	size_t read_attempts_left = 3;

    	/** AT command sequence number to use next. */
    	uint32_t at_seq = 1;

    	/** Drone SSID, when received. */
    	ugcs::vsm::Optional<std::string> ssid;

    	/** Drone name, when received. */
    	ugcs::vsm::Optional<std::string> name;
    };

    /** UDP port for AT commands. */
    constexpr static unsigned AT_PORT = 5556;

    /** TCP port for configuration reading. */
    constexpr static unsigned CONFIG_PORT = 5559;

    /** Timeout for waiting for any data from Mavlink stream. Configuration
     * reading is started only after some data is read from the Mavlink
     * stream. This is to avoid TCP SYN flooding when drone is absent.
     */
    constexpr static std::chrono::seconds MAVLINK_TIMEOUT = std::chrono::seconds(2);

    /** Configuration reading retry timeout. */
    constexpr static std::chrono::seconds CONFIG_RETRY_TIMEOUT = std::chrono::seconds(3);

    /** TCP connection timeout. */
    constexpr static std::chrono::seconds TCP_TIMEOUT = std::chrono::seconds(5);

    /** Start detection by waiting some data from Mavlink stream first. */
    void
    Start_detection(Detection_ctx::Ptr);

    /** TCP connection handler. */
    void
    On_tcp_connected(
    		ugcs::vsm::Socket_processor::Stream::Ref,
    		ugcs::vsm::Io_result,
    		Detection_ctx::Weak_ptr);

    /** Send AT command to request full drone config from TCP stream and
     * schedule retry timer. */
    bool
    Request_config(Detection_ctx::Weak_ptr);

    /** Reading from Mavlink stream. */
    void
    On_mavlink_read(
    		ugcs::vsm::Io_buffer::Ptr,
    		ugcs::vsm::Io_result,
    		Detection_ctx::Weak_ptr);

    /** Reconnect TCP stream for configuration reading. */
    void
    Reconnect_tcp(Detection_ctx::Ptr);

    /** Finishes detection when both SSID and drone name are recieved. */
    void
    Try_finish_detecton(Detection_ctx::Ptr);

    /** Send AT command. */
    void
    Send_at(
    		Detection_ctx::Ptr,
    		const std::string& prefix,
    		const std::string& args);

    /** Handler for reading string parameter from TCP. */
    bool
    String_param_read_cb(
    		regex::smatch* match,
    		ugcs::vsm::Text_stream_filter::Lines_list*,
    		ugcs::vsm::Io_result,
    		Detection_ctx::Weak_ptr,
    		ugcs::vsm::Optional<std::string>*);

    virtual Mavlink_vehicle::Ptr
    Create_mavlink_vehicle(
            ugcs::vsm::Mavlink_demuxer::System_id system_id,
            ugcs::vsm::Mavlink_demuxer::Component_id component_id,
            ugcs::vsm::mavlink::MAV_TYPE type,
            ugcs::vsm::Io_stream::Ref stream,
            ugcs::vsm::Socket_address::Ptr peer_addr,
            ugcs::vsm::Optional<std::string> mission_dump_path,
            std::string serial_number,
            std::string model_name,
            bool id_overridden) override;

    virtual void
    Register_detectors() override;

    /** Detection ended (successfully or not), do cleanup. */
    void
    Detection_ended(Detection_ctx::Ptr);

    /** Disable handler. */
    virtual void
    On_manager_disable() override;

    /** Currently ongoing detections. */
    std::unordered_set<Detection_ctx::Ptr> detectors;

};

#endif /* _ARDRONE_VEHICLE_MANAGER_H_ */
