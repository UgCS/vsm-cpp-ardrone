// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <ardrone_vehicle_manager.h>
#include <ardrone_vehicle.h>
#include <vsm/transport_detector.h>

using namespace vsm;

Ardrone_vehicle_manager::Ardrone_vehicle_manager() :
Mavlink_vehicle_manager(
        "Ardrone",
        "vehicle.ardrone",
        mavlink::Extension::Get(),
        vsm::mavlink::MAX_MAVLINK_PACKET_SIZE)
{

}

void
Ardrone_vehicle_manager::Register_detectors()
{
    /* Bind a detector to udp port specified in config. */
    Transport_detector::Get_instance()->Add_detector(
            config_prefix + ".detector",
            vsm::Transport_detector::Make_connect_handler(
                    &Ardrone_vehicle_manager::On_new_connection,
                    Shared_from_this()),
                    Shared_from_this());
}

Mavlink_vehicle::Ptr
Ardrone_vehicle_manager::Create_mavlink_vehicle(
        Mavlink_demuxer::System_id system_id,
        Mavlink_demuxer::Component_id component_id,
        mavlink::MAV_TYPE type,
        Io_stream::Ref stream,
        std::string serial_number,
        std::string model_name)
{
    return Ardrone_vehicle::Create(
            system_id,
            component_id,
            type,
            stream,
            serial_number,
            model_name);
}
