// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file ardrone_vehicle.h
 */
#ifndef _ARDRONE_VEHICLE_H_
#define _ARDRONE_VEHICLE_H_

#include <mavlink_vehicle.h>

/** Vehicle supporting Ardrone specific flavor of Mavlink. */
class Ardrone_vehicle: public Mavlink_vehicle {
    DEFINE_COMMON_CLASS(Ardrone_vehicle, Mavlink_vehicle)

public:
    template<typename... Args>
    Ardrone_vehicle(
            vsm::Mavlink_demuxer::System_id system_id,
            vsm::Mavlink_demuxer::Component_id component_id,
            vsm::mavlink::MAV_TYPE type,
            vsm::Io_stream::Ref stream,
            Args &&... args) :
            Mavlink_vehicle(
                    system_id, component_id, type,
                    vsm::mavlink::MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY,
                    stream, vsm::mavlink::MAX_MAVLINK_PACKET_SIZE,
                    std::forward<Args>(args)...),
            vehicle_command(*this),
            task_upload(*this)
    {

    }

    /** UCS has sent a task for a vehicle. */
    virtual void
    Handle_vehicle_request(vsm::Vehicle_task_request::Handle request) override;

    /**
     * UCS requesting to clear up all missions on a vehicle.
     */
    virtual void
    Handle_vehicle_request(vsm::Vehicle_clear_all_missions_request::Handle request) override;

    /**
     * UCS requesting command execution on a vehicle.
     */
    virtual void
    Handle_vehicle_request(vsm::Vehicle_command_request::Handle request) override;

    /** Data related to vehicle command processing. */
    class Vehicle_command_act : public Activity {
    public:

        using Activity::Activity;

        /** Related constants. */
        enum {
            ATTEMPTS = 3,
            /** In seconds. */
            RETRY_TIMEOUT = 1,
        };

        /** Try execute command a vehicle. */
        bool
        Try();

        /** Command ack received. */
        void
        On_command_ack(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::COMMAND_ACK>::Ptr);

        /** Enable class and start command execution. */
        void
        Enable(vsm::Vehicle_command_request::Handle vehicle_command_request);

        /** Disable this class and cancel any existing request. */
        virtual void
        On_disable() override;

        /** Schedule timer for retry operation. */
        void
        Schedule_timer();

        /** Current command request. */
        vsm::Vehicle_command_request::Handle vehicle_command_request;

        /** Remaining attempts towards vehicle. */
        size_t remaining_attempts = 0;

        /** Retry timer. */
        vsm::Timer_processor::Timer::Ptr timer;
    } vehicle_command;

    /** Data related to task upload processing. */
    class Task_upload: public Activity {
    public:

        using Activity::Activity;

        /** Calls appropriate prepare action based on type. */
        void
        Prepare_action(vsm::Action::Ptr);

        /** Add mission item to prepared actions. Common mission item
         * initialization are made, like sequence number generation.
         */
        void
        Add_mission_item(vsm::mavlink::Pld_mission_item::Ptr);

        //@{
        /** Prepare methods for different types of actions. These methods
         * create an item in the prepared actions list.
         * @return Created mission item. */

        void
        Prepare_move(vsm::Action::Ptr&);

        void
        Prepare_wait(vsm::Action::Ptr&);

        void
        Prepare_payload_steering(vsm::Action::Ptr&);

        void
        Prepare_takeoff(vsm::Action::Ptr&);

        void
        Prepare_landing(vsm::Action::Ptr&);

        void
        Prepare_change_speed(vsm::Action::Ptr&);

        void
        Prepare_set_home(vsm::Action::Ptr&);

        void
        Prepare_POI(vsm::Action::Ptr&);

        void
        Prepare_heading(vsm::Action::Ptr&);

        void
        Prepare_panorama(vsm::Action::Ptr&);
        //@}

        /** Build waypoint mission item based on move action. */
        vsm::mavlink::Pld_mission_item::Ptr
        Build_wp_mission_item(vsm::Action::Ptr&);

        /** Previous activity is completed, enable class and start task upload. */
        void
        Enable(bool success, vsm::Vehicle_task_request::Handle);

        /** Disable this class and cancel any existing request. */
        virtual void
        On_disable() override;

        /** Calculate launch elevation which is assumed to be the first
         * waypoint.
         * @return true on success, false if no sufficient data found in
         * the mission.
         */
        bool
        Calculate_launch_elevation();

        /** Prepare the task for uploading to the vehicle. */
        void
        Prepare_task();

        /** Mission upload handler. */
        void
        Mission_uploaded(bool success);

        /**
         * Fill coordinates into Mavlink message based on vsm::Geodetic_tuple and
         * some other common mission item data structures.
         * @param msg Mavlink message.
         * @param tuple Geodetic tuple.
         * @param heading Vehicle heading.
         */
        void
        Fill_mavlink_mission_item_coords(vsm::mavlink::Pld_mission_item& msg,
                const vsm::Geodetic_tuple& tuple, double heading);

        /**
         * Fill Mavlink mission item common parameters.
         * @param msg Mavlink message.
         */
        void
        Fill_mavlink_mission_item_common(vsm::mavlink::Pld_mission_item& msg);


        /** Current task for uploading, if any. */
        vsm::Vehicle_task_request::Handle request;

        /** Prepared Mavlink actions to be uploaded to the vehicle and built based
         * on the actions from the original request. Original actions could be
         * extended/removed/updated to meet the Mavlink mission protocol
         * requirements. Example is adding of magical "dummy waypoints" and
         * special processing of waypoint zero.
         */
        std::vector<vsm::mavlink::Payload_base::Ptr> prepared_actions;

        /** Task attributes to be written to the vehicle. */
        Write_parameters::List task_attributes;

        /** Previous move action, if any. */
        vsm::Action::Ptr last_move_action;

        /** Elevation (ground level) of the vehicle launch position which is
         * assumed to be first waypoint. Used to compensate absolute altitude
         * sent from UCS. It is assumed that vehicle is started 'close enough'
         * to the first waypoint.
         */
        double launch_elevation = 0;
    } task_upload;
};

#endif /* _ARDRONE_VEHICLE_H_ */
