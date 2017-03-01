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
                    static_cast<ugcs::vsm::mavlink::MAV_AUTOPILOT>(ugcs::vsm::mavlink::ugcs::MAV_AUTOPILOT_AR_DRONE),
                    capabilities,
                    stream, mission_dump_path,
                    std::forward<Args>(args)...),
            vehicle_command(*this),
            task_upload(*this),
            mode_switch(*this),
            emergency_land(*this, 11, 250),
            camera_trigger(*this),
            set_max_altitude(*this, 2, 250),
            drone_addr(drone_addr)
    {
        autopilot_type = "ardrone";
        frame_type = "ardrone";
    }

    /** UCS has sent a task for a vehicle. */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_task_request::Handle request) override;

    /**
     * UCS requesting command execution on a vehicle.
     */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_command_request::Handle request) override;

    size_t
    Get_next_at_seq();

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

        Task_upload(Ardrone_vehicle& v);

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

        /* Calculate maximum elevation which is set in vehicle along with
         * mission upload.
         */
        void
        Calculate_max_altitude();

        /** Prepare the task for uploading to the vehicle. */
        void
        Prepare_task();

        /** Mission upload handler. */
        void
        Mission_uploaded(bool success, std::string error_msg);

        /** Mission upload complete handler. */
        void
        Max_altitude_set(bool success, std::string error_msg);

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

        static constexpr float INVALID_ELEVATION = -13000.0;

        /** Elevation (ground level) of the vehicle launch position which is
         * assumed to be first waypoint. Used to compensate absolute altitude
         * sent from UCS. It is assumed that vehicle is started 'close enough'
         * to the first waypoint.
         */
        double launch_elevation = INVALID_ELEVATION;

        /** Max elevation of the mission. ArDrone will not fly above this
         * in any circumstances.
         */
        double max_altitude = 0;

        Ardrone_vehicle& ardrone_vehicle;

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
    class At_command: public Activity {
    public:
        At_command(Ardrone_vehicle& v, size_t retries = 0, size_t delay_ms = 0);

        /** Port for AT commands. */
        static constexpr size_t AT_PORT = 5556;

        /** Enable class and start command execution. */
        void
        Enable( ugcs::vsm::Vehicle_command_request::Handle request,
                ugcs::vsm::Socket_address::Ptr);

        /** Disable this class and cancel any existing request. */
        virtual void
        On_disable() override;

        bool
        Send_command();

        virtual ugcs::vsm::Io_buffer::Ptr
        Prepare_command();

        /** Current command request. */
        ugcs::vsm::Vehicle_command_request::Handle request;

        /** Retry timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;

        /** UDP stream for sending AT commands. */
        ugcs::vsm::Socket_processor::Stream::Ref udp_stream;

        /** How many more times to send the command. */
        size_t retries;

        /** How many times left*/
        size_t attempts_left;

        std::chrono::milliseconds delay;

        /** Address for sending AT commands. */
        ugcs::vsm::Socket_address::Ptr at_addr;

        Ardrone_vehicle& ardrone_vehicle;
    };

    /** Initiate emergency landing. */
    class Emergeny_land: public At_command {
    public:
        using At_command::At_command;
        virtual ugcs::vsm::Io_buffer::Ptr
        Prepare_command();
    } emergency_land;

    /** Take picture. */
    class Camera_trigger: public At_command {
    public:
        using At_command::At_command;
        virtual ugcs::vsm::Io_buffer::Ptr
        Prepare_command();
    } camera_trigger;

    /** Take picture. */
    class Set_max_altitude: public At_command {
    public:
        using At_command::At_command;
        void Set_altitude(double);
        virtual ugcs::vsm::Io_buffer::Ptr
        Prepare_command();
        int altitude = 0;
    } set_max_altitude;

    /** Current AT sequence.
     * Initially set to something bigger because vehicle detection involves
     * sending AT commands. */
    std::atomic_size_t current_at_seq = { 100 };

    /**
     * Minimal waypoint acceptance radius to use.
     */
    constexpr static double ACCEPTANCE_RADIUS_MIN = 1;

    /** current vehicle mode in MAV_MODE_FLAG syntax. */
    int current_vehicle_mode = 0;

    /** current vehicle mode in MAV_MODE_FLAG syntax. */
    int current_vehicle_custom_mode = 0;

    /** Process heartbeat message by setting system status according to it. */
    virtual void
    Process_heartbeat(
        ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::HEARTBEAT>::Ptr) override;

    /** Address of the drone received from transport detector. */
    ugcs::vsm::Socket_address::Ptr drone_addr;
};

#endif /* _ARDRONE_VEHICLE_H_ */
