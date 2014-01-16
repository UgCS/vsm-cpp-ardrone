// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file ardrone_vehicle_manager.h
 */
#ifndef _ARDRONE_VEHICLE_MANAGER_H_
#define _ARDRONE_VEHICLE_MANAGER_H_

#include <mavlink_vehicle_manager.h>

class Ardrone_vehicle_manager: public Mavlink_vehicle_manager {
    DEFINE_COMMON_CLASS(Ardrone_vehicle_manager, Mavlink_vehicle_manager)

public:

    /** Constructor. */
    Ardrone_vehicle_manager();

private:

    virtual Mavlink_vehicle::Ptr
    Create_mavlink_vehicle(
            vsm::Mavlink_demuxer::System_id system_id,
            vsm::Mavlink_demuxer::Component_id component_id,
            vsm::mavlink::MAV_TYPE type,
            vsm::Io_stream::Ref stream,
            std::string serial_number,
            std::string model_name) override;

    virtual void
    Register_detectors() override;

};

#endif /* _ARDRONE_VEHICLE_MANAGER_H_ */
