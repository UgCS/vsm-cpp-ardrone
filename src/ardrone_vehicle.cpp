// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <ardrone_vehicle.h>

/* Ardrone uses only this value in ACK messages. */
const vsm::Mavlink_demuxer::System_id Mavlink_vehicle::VSM_SYSTEM_ID = 255;

/* Ardrone uses only this value in ACK messages. */
const vsm::Mavlink_demuxer::Component_id Mavlink_vehicle::VSM_COMPONENT_ID = 0;

using namespace vsm;

void
Ardrone_vehicle::Handle_vehicle_request(Vehicle_task_request::Handle request)
{
    LOG_INFO("Starting to handle %ld tasks...", request->actions.size());
    ASSERT(!task_upload.request);
    task_upload.Disable();
    clear_all_missions.Disable();
    /* Mission upload will be done after all existing missions are cleared. */
    clear_all_missions.Set_next_action(
            Activity::Make_next_action(
                    &Task_upload::Enable,
                    &task_upload,
                    request));
    clear_all_missions.Enable(vsm::Clear_all_missions());
}

void
Ardrone_vehicle::Handle_vehicle_request(Vehicle_clear_all_missions_request::Handle request)
{
    ASSERT(!task_upload.request);
    clear_all_missions.Disable();
    /* Only clear. */
    clear_all_missions.Enable(*request);
}

void
Ardrone_vehicle::Handle_vehicle_request(Vehicle_command_request::Handle request)
{
    ASSERT(!vehicle_command.vehicle_command_request);
    vehicle_command.Disable();
    vehicle_command.Enable(request);
}

bool
Ardrone_vehicle::Vehicle_command_act::Try()
{
    if (!remaining_attempts--) {
        LOG_WARN("Vehicle_command all attempts failed.");
        Disable();
        return false;
    }

    mavlink::Pld_command_long cmd;
    Fill_target_ids(cmd);
    cmd->param1 = 0;
    cmd->param2 = 0;
    cmd->param3 = 0;
    cmd->param4 = 0;
    cmd->param5 = 0;
    cmd->param6 = 0;
    cmd->param7 = 0;

    switch (vehicle_command_request->Get_type()) {
    case Vehicle_command::Type::HOLD:
        cmd->command = mavlink::MAV_CMD::MAV_CMD_NAV_LAND;
        break;
    case Vehicle_command::Type::GO:
        cmd->command = mavlink::MAV_CMD::MAV_CMD_NAV_TAKEOFF;
        break;
    }


    Send_message(cmd);
    Schedule_timer();
    return false;
}

void
Ardrone_vehicle::Vehicle_command_act::On_command_ack(
        mavlink::Message<mavlink::MESSAGE_ID::COMMAND_ACK>::Ptr message)
{
    LOG_INFO("Vehicle command executed, result %d", message->payload->result.Get());

    if (message->payload->result == mavlink::MAV_RESULT::MAV_RESULT_ACCEPTED) {
        vehicle_command_request = Vehicle_request::Result::OK;
    } else {
        vehicle_command_request = Vehicle_request::Result::NOK;
    }
    Disable();
}

void
Ardrone_vehicle::Vehicle_command_act::Enable(
        Vehicle_command_request::Handle vehicle_command_request)
{
    this->vehicle_command_request = vehicle_command_request;
    remaining_attempts = ATTEMPTS;

    Register_mavlink_handler<mavlink::MESSAGE_ID::COMMAND_ACK>(
            &Vehicle_command_act::On_command_ack,
            this,
            Mavlink_demuxer::COMPONENT_ID_ANY);

    Try();
}

void
Ardrone_vehicle::Vehicle_command_act::On_disable()
{
    Unregister_mavlink_handlers();

    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
    if (vehicle_command_request) {
        vehicle_command_request = Vehicle_request::Result::NOK;
    }
}

void
Ardrone_vehicle::Vehicle_command_act::Schedule_timer()
{
    if (timer) {
        timer->Cancel();
    }
    timer = Timer_processor::Get_instance()->Create_timer(
                std::chrono::seconds(RETRY_TIMEOUT),
                Make_callback(&Vehicle_command_act::Try, this),
                vehicle.Get_completion_ctx());
}


void
Ardrone_vehicle::Task_upload::Enable(
        bool success,
        Vehicle_task_request::Handle request)
{
    if (!success) {
        LOG_INFO("Previous activity failed, failing also task upload.");
        request = Vehicle_request::Result::NOK;
        Disable();
        return;
    }
    /* Discard unsupported actions. */
    for (auto iter = request->actions.begin(); iter != request->actions.end();) {
        switch ((*iter)->Get_type()) {
        case Action::Type::MOVE:
        case Action::Type::LANDING:
            iter++;
            continue;
        default:
            LOG_INFO("%s action unsupported, ignored.", (*iter)->Get_name().c_str());
            break;
        }
        iter = request->actions.erase(iter);
    }
    this->request = request;

    if (!Calculate_launch_elevation()) {
        LOG_ERROR("Unable to calculate launch elevation.");
        Disable();
    } else {
        Prepare_task();
        vehicle.mission_upload.Disable();
        vehicle.mission_upload.mission_items = std::move(prepared_actions);
        vehicle.mission_upload.Set_next_action(
                Activity::Make_next_action(
                        &Task_upload::Mission_uploaded,
                        this));
        vehicle.mission_upload.Enable();
    }
}

void
Ardrone_vehicle::Task_upload::Mission_uploaded(bool success)
{
    if (!success) {
        LOG_INFO("Mission upload to vehicle failed, failing vehicle request.");
        request = Vehicle_request::Result::NOK;
        Disable();
        return;
    }
    /* Everything is OK. */
    request = Vehicle_request::Result::OK;
    Disable();
}

void
Ardrone_vehicle::Task_upload::Fill_mavlink_mission_item_coords(
        mavlink::Pld_mission_item& msg,
        const Geodetic_tuple& tuple, double heading)
{
    msg->x = (tuple.latitude * 180.0) / M_PI;
    msg->y = (tuple.longitude * 180.0) / M_PI;
    /* APM treats all altitudes relative to home position, which
     * is presumably the point of arming (launch), so compensate
     * absolute altitude sent from UCS by launch point elevation.
     */
    msg->z = tuple.altitude - launch_elevation;
    msg->param4 = (heading * 180.0) / M_PI;
}

void
Ardrone_vehicle::Task_upload::Fill_mavlink_mission_item_common(
        mavlink::Pld_mission_item& msg)
{
    ASSERT(vehicle.real_system_id != Mavlink_demuxer::SYSTEM_ID_ANY);

    Fill_target_ids(msg);
    msg->seq = prepared_actions.size();
    /* APM firmware treats all altitudes as relative. Always. */
    msg->frame = mavlink::MAV_FRAME::MAV_FRAME_GLOBAL_RELATIVE_ALT;
    msg->current = 0;
    msg->autocontinue = 1;
}

void
Ardrone_vehicle::Task_upload::On_disable()
{
    if (request) {
        request = Vehicle_request::Result::NOK;
    }
    vehicle.write_parameters.Disable();
    vehicle.mission_upload.Disable();
    prepared_actions.clear();
    task_attributes.clear();
}

bool
Ardrone_vehicle::Task_upload::Calculate_launch_elevation()
{
    for (auto& action: request->actions) {
        /* The first move or takeoff action. */
        if (action->Get_type() == Action::Type::MOVE) {
            Move_action::Ptr move = action->Get_action<Action::Type::MOVE>();
            launch_elevation = move->elevation;
            return true;
        }
        /* Or first takeoff. */
        if (action->Get_type() == Action::Type::TAKEOFF) {
            Takeoff_action::Ptr takeoff = action->Get_action<Action::Type::TAKEOFF>();
            launch_elevation = takeoff->elevation;
            return true;
        }
    }
    return false;
}

void
Ardrone_vehicle::Task_upload::Prepare_task()
{
    prepared_actions.clear();
    last_move_action = nullptr;
    for (auto& iter:request->actions) {
        Prepare_action(iter);
    }
}

void
Ardrone_vehicle::Task_upload::Prepare_action(Action::Ptr action)
{
    switch (action->Get_type()) {
    case Action::Type::MOVE:
        Prepare_move(action);
        return;
    case Action::Type::WAIT:
        Prepare_wait(action);
        return;
    case Action::Type::PAYLOAD_STEERING:
        Prepare_payload_steering(action);
        return;
    case Action::Type::TAKEOFF:
        Prepare_takeoff(action);
        return;
    case Action::Type::LANDING:
        Prepare_landing(action);
        return;
    case Action::Type::CHANGE_SPEED:
        Prepare_change_speed(action);
        return;
    case Action::Type::SET_HOME:
        Prepare_set_home(action);
        return;
    case Action::Type::POI:
        Prepare_POI(action);
        return;
    case Action::Type::HEADING:
        Prepare_heading(action);
        return;
    case Action::Type::PANORAMA:
        Prepare_panorama(action);
        return;
    case Action::Type::CAMERA_CONTROL:
        VSM_EXCEPTION(Internal_error_exception, "CAMERA_CONTROL action not supported.");
    case Action::Type::CAMERA_TRIGGER:
        VSM_EXCEPTION(Internal_error_exception, "CAMERA_TRIGGER action not supported.");
    case Action::Type::TASK_ATTRIBUTES:
        VSM_EXCEPTION(Internal_error_exception, "TASK_ATTRIBUTES action not supported.");
    }
    VSM_EXCEPTION(Internal_error_exception, "Unsupported action [%s]",
            action->Get_name().c_str());
}

void
Ardrone_vehicle::Task_upload::Add_mission_item(mavlink::Pld_mission_item::Ptr mi)
{
    Fill_mavlink_mission_item_common(*mi);
    prepared_actions.push_back(mi);
}

void
Ardrone_vehicle::Task_upload::Prepare_move(Action::Ptr& action)
{
    Add_mission_item(Build_wp_mission_item(action));
    last_move_action = action;
}

void
Ardrone_vehicle::Task_upload::Prepare_wait(Action::Ptr& action)
{
    /* Create additional waypoint on the current position to wait. */
    if (last_move_action) {
        auto wp = Build_wp_mission_item(last_move_action);
        Wait_action::Ptr wa = action->Get_action<Action::Type::WAIT>();
        (*wp)->param1 = wa->wait_time;
        Add_mission_item(wp);
    } else {
        LOG_WARN("No move action before wait action, ignored.");
    }
}

void
Ardrone_vehicle::Task_upload::Prepare_payload_steering(Action::Ptr&)
{
    ASSERT(false); /* Not implemented yet */
}

void
Ardrone_vehicle::Task_upload::Prepare_takeoff(Action::Ptr& action)
{
    /* Ardupilot does not fly to the takeoff position after takeoff
     * is done. Add explicit waypoint after the takeoff command with
     * target coordinates.
     */
    auto takeoff = action->Get_action<Action::Type::TAKEOFF>();
    auto explicit_wp = Move_action::Create(
            takeoff->position,
            0,
            takeoff->acceptance_radius,
            0,
            takeoff->heading,
            takeoff->elevation
    );

    mavlink::Pld_mission_item::Ptr mi = mavlink::Pld_mission_item::Create();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_NAV_TAKEOFF;
    (*mi)->param1 = 0; /* No data for pitch. */
    Fill_mavlink_mission_item_coords(*mi, takeoff->position.Get_geodetic(), takeoff->heading);
    Add_mission_item(mi);
    Prepare_action(explicit_wp);
}

void
Ardrone_vehicle::Task_upload::Prepare_landing(Action::Ptr& action)
{
    /* Ardupilot does not take the altitude of the landing, so
     * add explicit waypoint guiding vehicle to the landing start
     * position.
     */
    auto land = action->Get_action<Action::Type::LANDING>();
    auto explicit_wp = Move_action::Create(
            land->position,
            0,
            land->acceptance_radius,
            0,
            land->heading,
            land->elevation
    );
    Prepare_action(explicit_wp);

    mavlink::Pld_mission_item::Ptr mi = mavlink::Pld_mission_item::Create();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_NAV_LAND;
    Fill_mavlink_mission_item_coords(*mi, land->position.Get_geodetic(), land->heading);
    Add_mission_item(mi);

    /* Don't duplicate waypoint if last action is land. */
    last_move_action = nullptr;
}

void
Ardrone_vehicle::Task_upload::Prepare_change_speed(Action::Ptr& action)
{
    Change_speed_action::Ptr la = action->Get_action<Action::Type::CHANGE_SPEED>();
    mavlink::Pld_mission_item::Ptr mi = mavlink::Pld_mission_item::Create();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_CHANGE_SPEED;
    switch (vehicle.type) {
    case mavlink::MAV_TYPE::MAV_TYPE_QUADROTOR:
    case mavlink::MAV_TYPE::MAV_TYPE_HEXAROTOR:
    case mavlink::MAV_TYPE::MAV_TYPE_OCTOROTOR:
    case mavlink::MAV_TYPE::MAV_TYPE_TRICOPTER:
        /* Copters use p1 as navigation speed always. */
        (*mi)->param1 = la->speed;
        (*mi)->param2 = 0; /* unused */
        break;
    case mavlink::MAV_TYPE::MAV_TYPE_GROUND_ROVER:
    default:
        /* Ground rover takes only airspeed into account, others seems to
         * take both, but we have only airspeed from UCS, so use only air. */
        (*mi)->param1 = 0; /* Airspeed. */
        (*mi)->param2 = la->speed;
        break;
    }

    (*mi)->param3 = -1; /* Throttle no change. */
    Add_mission_item(mi);
}

void
Ardrone_vehicle::Task_upload::Prepare_set_home(Action::Ptr& action)
{
    Set_home_action::Ptr sa = action->Get_action<Action::Type::SET_HOME>();
    mavlink::Pld_mission_item::Ptr mi = mavlink::Pld_mission_item::Create();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_SET_HOME;
    (*mi)->param1 = sa->use_current_position ? 1 : 0;
    Fill_mavlink_mission_item_coords(*mi, sa->home_position.Get_geodetic(), 0);
    Add_mission_item(mi);
}

void
Ardrone_vehicle::Task_upload::Prepare_POI(Action::Ptr& action)
{
    /* Apply only active POI. */
    if (action->Get_action<Action::Type::POI>()->active) {
        Poi_action::Ptr pa = action->Get_action<Action::Type::POI>();
        mavlink::Pld_mission_item::Ptr mi = mavlink::Pld_mission_item::Create();
        (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_SET_ROI;
        (*mi)->param1 = mavlink::MAV_ROI::MAV_ROI_LOCATION;
        Fill_mavlink_mission_item_coords(*mi, pa->position.Get_geodetic(), 0);
        Add_mission_item(mi);
    }
}

void
Ardrone_vehicle::Task_upload::Prepare_heading(Action::Ptr& action)
{
    Heading_action::Ptr ha = action->Get_action<Action::Type::HEADING>();
    mavlink::Pld_mission_item::Ptr mi = mavlink::Pld_mission_item::Create();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_CONDITION_YAW;
    (*mi)->param1 = (ha->heading * 180.0) / M_PI;
    (*mi)->param2 = 0; /* Default auto speed. */
    (*mi)->param3 = 1; /* clockwise. */
    (*mi)->param4 = 0; /* absolute angle. */
    Add_mission_item(mi);
}

void
Ardrone_vehicle::Task_upload::Prepare_panorama(Action::Ptr& action)
{
    Panorama_action::Ptr panorama = action->Get_action<Action::Type::PANORAMA>();

    if (!last_move_action) {
        LOG_WARNING("No previous move action found to generate panorama action, ignored.");
        return;
    }
    /* Create additional waypoint at the current position with wait to
     * stabilize before doing panorama. */
    auto panorama_pre_wait = Build_wp_mission_item(last_move_action);
    (*panorama_pre_wait)->param1 = 3; /* seconds. */
    Add_mission_item(panorama_pre_wait);

    /* Panorama is always done in steps less then 180 degree to make sure that
     * turns over 180 degrees are supported.
     */
    double min_step = 170;
    double full_angle = (std::abs(panorama->angle) * 180.0) / M_PI;
    double speed = (std::abs(panorama->speed) * 180.0) / M_PI;
    double step = 0;
    int delay = 0;

    if (!speed || speed > 60) {
        speed = 60; /* Degrees/second assumed max speed. */
    }

    double panorama_duration = full_angle / speed;

    switch (panorama->trigger_state) {
    case Panorama_action::Trigger_state::ON:
        step = min_step;
        delay = 0;
        break;
    case Panorama_action::Trigger_state::SERIAL:
        step = (std::abs(panorama->step) * 180.0) / M_PI;
        /* Per-sector delay. */
        delay = std::chrono::duration_cast<std::chrono::seconds>(panorama->delay).count();
        break;
    }

    if (!step) {
        LOG_WARN("Zero step angle, ignoring panorama.");
        return;
    }

    double completed_angle = 0;
    while (completed_angle + 0.1 < full_angle) {
        double split_angle = completed_angle;
        /* Add step desired by user. */
        completed_angle += step;
        /* Don't exceed the maximum angle, even if it does not evenly divides
         * to desired step intervals.
         */
        if (completed_angle > full_angle) {
            completed_angle = full_angle;
        }
        /* If desired step is more that 180 degrees, it should be split in
         * min_step intervals, otherwise Ardupilot makes the shorter turn.
         */
        while (split_angle + 0.1 < completed_angle) {
            double diff = split_angle;
            split_angle += min_step;
            if (split_angle > completed_angle) {
                split_angle = completed_angle;
            }
            diff = split_angle - diff;
            /* Turn is relative to current position. */
            mavlink::Pld_mission_item::Ptr mi = mavlink::Pld_mission_item::Create();
            (*mi)->command = mavlink::MAV_CMD::MAV_CMD_CONDITION_YAW;
            (*mi)->param1 = diff;
            double val = diff / speed;
            if (val < 1) {
                val = 1; /* Fastest possible speed. */
            }
            (*mi)->param2 = val;
            (*mi)->param3 = panorama->angle > 0 ? 1 /* clockwise. */ : 0 /* ccw */;
            (*mi)->param4 = 1; /* relative angle. */
            Add_mission_item(mi);
        }
        if (delay) {
            /* Add delay after each segment. */
            auto wait_segment = mavlink::Pld_mission_item::Create();
            (*wait_segment)->command = mavlink::MAV_CMD::MAV_CMD_CONDITION_DELAY;
            (*wait_segment)->param1 = delay;
            Add_mission_item(wait_segment);
        }
    }

/* Needed only if jump is used. */
#if 0
    /* Delay for 3 seconds to stabilize after panorama is done. */
    auto wait_after = mavlink::Pld_mission_item::Create();
    (*wait_after)->command = mavlink::MAV_CMD::MAV_CMD_CONDITION_DELAY;
    (*wait_after)->param1 = 3;
    Add_mission_item(wait_after);
#endif

/* Ardupilot jump command is broken (bug). It works only once, so don't use it. */
#if 0
    /* Jump to next waypoint and interrupt "pseudo indefinite wait" of
     * 255 seconds.
     */
    mavlink::Pld_mission_item::Ptr jump = mavlink::Pld_mission_item::Create();
    (*jump)->command = mavlink::MAV_CMD::MAV_CMD_DO_JUMP;
    (*jump)->param2 = 1; /* Repeat once. */
    /* Skip next long wait waypoint and jump to 'jump_to'. */
    (*jump)->param1 = prepared_actions.size() + 2;
    Add_mission_item(jump);
#endif

    /* Create a waypoint with hold time slightly more than estimated
     * panorama duration.
     */
    auto long_wait = Build_wp_mission_item(last_move_action);
    if (panorama_duration + 5 > 255) {
        LOG_WARNING("Estimated panorama duration is truncated to 255 seconds.");
        (*long_wait)->param1 = 255; /* Max possible wait for Ardupilot. */
    } else {
        (*long_wait)->param1 = panorama_duration + 5;
    }
    Add_mission_item(long_wait);

/* Not needed without jump. */
#if 0
    auto jump_to = Build_wp_mission_item(last_move_action);
    /* Don't wait. Ardupilot does not support (most probably bug) jumping
     * to waypoints with non-zero wait. */
    (*jump_to)->param1 = 0;
    Add_mission_item(jump_to);
#endif
}

mavlink::Pld_mission_item::Ptr
Ardrone_vehicle::Task_upload::Build_wp_mission_item(Action::Ptr& action)
{
    auto Normalize = [](double value, double min, double max, double zero_val = 0)
    {
        if (value == 0 && zero_val != 0) {
            return zero_val;
        } else {
            return std::max(min, std::min(max, value));
        }
    };

    Move_action::Ptr ma = action->Get_action<Action::Type::MOVE>();
    mavlink::Pld_mission_item::Ptr mi = mavlink::Pld_mission_item::Create();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_NAV_WAYPOINT;
    (*mi)->param1 = ma->wait_time * 10;
    // Set acceptance radius to something reasonable
    (*mi)->param2 = Normalize(
            ma->acceptance_radius,
            ACCEPTANCE_RADIUS_MIN,
            ACCEPTANCE_RADIUS_MAX,
            ACCEPTANCE_RADIUS_DEFAULT);
    if ((*mi)->param2 != ma->acceptance_radius) {
        LOG("Acceptance radius normalized from %f to %f", ma->acceptance_radius, (*mi)->param2.Get());
    }
    (*mi)->param3 = ma->loiter_orbit;
    Fill_mavlink_mission_item_coords(*mi, ma->position.Get_geodetic(), ma->heading);

    return mi;
}

