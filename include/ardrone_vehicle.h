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
            ugcs::vsm::Mavlink_demuxer::System_id system_id,
            ugcs::vsm::Mavlink_demuxer::Component_id component_id,
            ugcs::vsm::mavlink::MAV_TYPE type,
            const Vehicle::Capabilities& capabilities,
            ugcs::vsm::Io_stream::Ref stream,
            ugcs::vsm::Socket_address::Ptr drone_addr,
            ugcs::vsm::Optional<std::string> mission_dump_path,
            Args &&... args) :
            Mavlink_vehicle(
                    system_id, component_id, type,
                    ugcs::vsm::mavlink::MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY,
                    capabilities,
                    stream, mission_dump_path, ugcs::vsm::mavlink::MAX_MAVLINK_PACKET_SIZE,
                    std::forward<Args>(args)...),
            vehicle_command(*this),
            task_upload(*this),
            mode_switch(*this),
            emergency_land(*this),
            drone_addr(drone_addr)
    {}

    /** UCS has sent a task for a vehicle. */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_task_request::Handle request) override;

    /**
     * UCS requesting to clear up all missions on a vehicle.
     */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_clear_all_missions_request::Handle request) override;

    /**
     * UCS requesting command execution on a vehicle.
     */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_command_request::Handle request) override;

private:
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
        On_command_ack(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::COMMAND_ACK>::Ptr);

        /** Enable class and start command execution. */
        void
        Enable(ugcs::vsm::Vehicle_command_request::Handle vehicle_command_request);

        /** Disable this class and cancel any existing request. */
        virtual void
        On_disable() override;

        /** Schedule timer for retry operation. */
        void
        Schedule_timer();

        /** Current command request. */
        ugcs::vsm::Vehicle_command_request::Handle vehicle_command_request;

        /** Remaining attempts towards vehicle. */
        size_t remaining_attempts = 0;

        /** Retry timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;
    } vehicle_command;

    /** Data related to task upload processing. */
    class Task_upload: public Activity {
    public:

        using Activity::Activity;

        /** Calls appropriate prepare action based on type. */
        void
        Prepare_action(ugcs::vsm::Action::Ptr);

        /** Add mission item to prepared actions. Common mission item
         * initialization are made, like sequence number generation.
         */
        void
        Add_mission_item(ugcs::vsm::mavlink::Pld_mission_item::Ptr);

        //@{
        /** Prepare methods for different types of actions. These methods
         * create an item in the prepared actions list.
         * @return Created mission item. */

        void
        Prepare_move(ugcs::vsm::Action::Ptr&);

        void
        Prepare_wait(ugcs::vsm::Action::Ptr&);

        void
        Prepare_payload_steering(ugcs::vsm::Action::Ptr&);

        void
        Prepare_takeoff(ugcs::vsm::Action::Ptr&);

        void
        Prepare_landing(ugcs::vsm::Action::Ptr&);

        void
        Prepare_change_speed(ugcs::vsm::Action::Ptr&);

        void
        Prepare_set_home(ugcs::vsm::Action::Ptr&);

        void
        Prepare_POI(ugcs::vsm::Action::Ptr&);

        void
        Prepare_heading(ugcs::vsm::Action::Ptr&);

        void
        Prepare_panorama(ugcs::vsm::Action::Ptr&);
        //@}

        /** Build waypoint mission item based on move action. */
        ugcs::vsm::mavlink::Pld_mission_item::Ptr
        Build_wp_mission_item(ugcs::vsm::Action::Ptr&);

        /** Previous activity is completed, enable class and start task upload. */
        void
        Enable(bool success, ugcs::vsm::Vehicle_task_request::Handle);

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
         * Fill coordinates into Mavlink message based on ugcs::vsm::Geodetic_tuple and
         * some other common mission item data structures.
         * @param msg Mavlink message.
         * @param tuple Geodetic tuple.
         * @param heading Vehicle heading.
         */
        void
        Fill_mavlink_mission_item_coords(ugcs::vsm::mavlink::Pld_mission_item& msg,
                const ugcs::vsm::Geodetic_tuple& tuple, double heading);

        /**
         * Fill Mavlink mission item common parameters.
         * @param msg Mavlink message.
         */
        void
        Fill_mavlink_mission_item_common(ugcs::vsm::mavlink::Pld_mission_item& msg);


        /** Current task for uploading, if any. */
        ugcs::vsm::Vehicle_task_request::Handle request;

        /** Prepared Mavlink actions to be uploaded to the vehicle and built based
         * on the actions from the original request. Original actions could be
         * extended/removed/updated to meet the Mavlink mission protocol
         * requirements. Example is adding of magical "dummy waypoints" and
         * special processing of waypoint zero.
         */
        std::vector<ugcs::vsm::mavlink::Payload_base::Ptr> prepared_actions;

        /** Task attributes to be written to the vehicle. */
        Write_parameters::List task_attributes;

        /** Previous move action, if any. */
        ugcs::vsm::Action::Ptr last_move_action;

        /** Elevation (ground level) of the vehicle launch position which is
         * assumed to be first waypoint. Used to compensate absolute altitude
         * sent from UCS. It is assumed that vehicle is started 'close enough'
         * to the first waypoint.
         */
        double launch_elevation = 0;
    } task_upload;

    /** Control mode switching requests. */
    class Mode_switch: public Activity {
    public:
        using Activity::Activity;
        static constexpr int MAX_ATTEMPTS = 3;

        /** Current mode updated by heartbeat. */
        Sys_status::Control_mode current_mode;

        /** Enable class and start command execution. */
        void
        Enable(ugcs::vsm::Vehicle_command_request::Handle request);

        /** Disable this class and cancel any existing request. */
        virtual void
        On_disable() override;

        /** Send mode toggling message if necessary. */
        void
        Toggle_mode();

    private:
        /** Current command request. */
        ugcs::vsm::Vehicle_command_request::Handle request;
        /** Number of attempts in scope of command execution. */
        int num_attempts;
        /** Target control mode. */
        Sys_status::Control_mode target_mode;
    } mode_switch;

    /** Initiate emergency landing. */
    class Emergeny_land: public Activity {
    public:
    	using Activity::Activity;

    	/** Delay for sending emergency land command. */
    	static constexpr std::chrono::milliseconds RETRY_DELAY =
    			std::chrono::milliseconds(250);

    	/** How many times to send emergency land command. */
    	static constexpr size_t RETRIES = 12;

    	/** Port for AT commands. */
    	static constexpr size_t AT_PORT = 5556;

    	/** Enable class and start command execution. */
    	void
    	Enable(ugcs::vsm::Vehicle_command_request::Handle request,
    			ugcs::vsm::Socket_address::Ptr);

    	/** Disable this class and cancel any existing request. */
    	virtual void
    	On_disable() override;

    	bool
    	Send_command();

    	/** Current command request. */
    	ugcs::vsm::Vehicle_command_request::Handle request;

    	/** Retry timer. */
    	ugcs::vsm::Timer_processor::Timer::Ptr timer;

    	/** UDP stream for sending AT commands. */
    	ugcs::vsm::Socket_processor::Stream::Ref udp_stream;

    	/** How many more times to send the command. */
    	size_t attempts_left = RETRIES;

    	/** Current AT sequence. */
    	size_t current_seq;

    	/** Address for sending AT commands. */
    	ugcs::vsm::Socket_address::Ptr at_addr;

    } emergency_land;

    /**
     * Minimal waypoint acceptance radius to use.
     */
    constexpr static double ACCEPTANCE_RADIUS_MIN = 1;

    /** Process heartbeat message by setting system status according to it. */
    virtual void
    Process_heartbeat(
        ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::HEARTBEAT>::Ptr) override;

    /** Address of the drone received from transport detector. */
    ugcs::vsm::Socket_address::Ptr drone_addr;
};

#endif /* _ARDRONE_VEHICLE_H_ */
