/**
 * Module: my_mission
 * Description: Trigger mission when Pilot switches to MISSION mode.
 * Workflow:
 * 1. Upload dummy mission to allow switching to Mission Mode.
 * 2. Arm vehicle.
 * 3. Switch to MISSION mode -> Code takes over (Offboard).
 * 4. Switch to HOLD to Abort.
 */

#include "MyMission.hpp"
#include "MissionPlanner.hpp"

// uORB Topics
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;
using my_mission::MAX_WAYPOINTS;
using my_mission::MissionConfig;
using my_mission::Waypoint;

void MyMission::run() {
    // Subscriptions
    int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    int status_sub    = orb_subscribe(ORB_ID(vehicle_status));

    // Publications
    uORB::Publication<offboard_control_mode_s> _offboard_mode_pub{ORB_ID(offboard_control_mode)};
    uORB::Publication<trajectory_setpoint_s>   _traj_sp_pub{ORB_ID(trajectory_setpoint)};
    uORB::Publication<vehicle_command_s>       _cmd_pub{ORB_ID(vehicle_command)};

    vehicle_local_position_s pos{};
    vehicle_status_s status{};
    
    px4_pollfd_struct_t fds[] = { { .fd = local_pos_sub, .events = POLLIN } };

    uint64_t start_time = 0;
    uint64_t state_time = 0;
    uint64_t last_position_valid_time = 0;
    float home_x = 0.0f, home_y = 0.0f, home_z = 0.0f;
    bool home_set = false;

    // Mission params are sampled once at trigger time and expanded into a fixed
    // waypoint list. This makes the active mission deterministic until landing.
    MissionConfig config{};
    Waypoint waypoints[MAX_WAYPOINTS]{};
    int waypoint_count = 0;
    int current_waypoint = 0;

    PX4_INFO("Mission Ready. Upload a Dummy Mission, Arm, then switch to MISSION mode to Start.");

    while (!should_exit()) {
        px4_poll(fds, 1, 100);

        if (fds[0].revents & POLLIN) {
            orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &pos);
            orb_copy(ORB_ID(vehicle_status), status_sub, &status);
        }

        // --- TRIGGER LOGIC ---

        // 1. Safety Abort: If user switches to HOLD (Auto Loiter) -> Reset
        if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) {
            if (_state != IDLE && _state != MONITOR_LANDING) {
                PX4_WARN("HOLD Switch Detected! Mission Aborted.");
                _state = IDLE;
                home_set = false;
            }
        }

        // 2. Start Condition: User switched to MISSION mode (Auto Mission)
        // We only trigger if we are currently IDLE
	if (_state == IDLE) {
            if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
                bool is_position_valid = pos.xy_valid && pos.z_valid;
                if (is_position_valid) {
                    PX4_INFO("GPS OK. Hijacking control to Offboard...");
                    updateParams();
                    start_time = hrt_absolute_time();
                    _state = WARMUP;
                } else {
                    // Print error only once per second to avoid console flooding
                    static uint64_t last_print = 0;
                    if (hrt_absolute_time() - last_print > 1_s) {
                        PX4_ERR("REJECTED: GPS/Position NOT valid yet! Waiting for Lock...");
                        last_print = hrt_absolute_time();
                    }
                }
            }
        }

        // Used by the Offboard mission as an internal watchdog. The commander
        // still owns the real failsafe policy; this only stops us sending stale
        // position setpoints if local position drops out.
        if (pos.xy_valid && pos.z_valid) {
            last_position_valid_time = hrt_absolute_time();
        }

        // --- Offboard Heartbeat ---
        if (_state != IDLE && _state != MONITOR_LANDING && _state != EXIT) {
            offboard_control_mode_s ocm{};
            ocm.position = true;
            ocm.timestamp = hrt_absolute_time();
            _offboard_mode_pub.publish(ocm);
        }

        // --- State Machine ---
        trajectory_setpoint_s sp{};
        sp.timestamp = hrt_absolute_time();
        sp.yaw = 0.0f;

        switch (_state) {
        case IDLE:
            break;

        case WARMUP:
            // Lock the local-frame home once. All generated waypoints are
            // relative to this point, not to later vehicle drift.
            if (!home_set && pos.xy_valid && pos.z_valid) {
                home_x = pos.x; home_y = pos.y; home_z = pos.z; home_set = true;
                config = mission_config();
                waypoint_count = my_mission::build_waypoints(waypoints, MAX_WAYPOINTS, home_x, home_y, home_z, config);
                current_waypoint = 0;

                PX4_INFO("Mission pattern %d, %d waypoints: alt %.1fm dist %.1fm",
                         config.pattern, waypoint_count, (double)config.altitude_m, (double)config.distance_m);
            }
            sp.position[0] = home_x; sp.position[1] = home_y; sp.position[2] = pos.z;

            // PX4 requires a stream of Offboard setpoints before accepting the
            // mode switch, so WARMUP publishes hold-position setpoints first.
            if (home_set && waypoint_count > 0 && hrt_absolute_time() - start_time > 1_s) {
                PX4_INFO("Switching to Offboard...");
                send_vehicle_command(_cmd_pub, vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, 6); // 6 = Offboard
                
                // If not armed, try to arm (Safety: usually pilot arms first)
                if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
                    send_vehicle_command(_cmd_pub, vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
                }
                
                _state = GOTO_WAYPOINT;
                state_time = hrt_absolute_time();
            }
            break;

        case GOTO_WAYPOINT: {
            const Waypoint &target = waypoints[current_waypoint];
            sp.position[0] = target.x; sp.position[1] = target.y; sp.position[2] = target.z; sp.yaw = target.yaw;

            // If position becomes invalid after Offboard takeover, hand back to
            // PX4 landing instead of continuing to publish old setpoints.
            if (hrt_absolute_time() - last_position_valid_time > 1_s) {
                PX4_ERR("Position invalid during mission. Landing.");
                _state = TRIGGER_LAND;
                break;
            }

            if (my_mission::reached_waypoint(pos, target, config.acceptance_radius_m)) {
                PX4_INFO("Waypoint %d/%d reached. Holding.", current_waypoint + 1, waypoint_count);
                _state = HOLD_WAYPOINT;
                state_time = hrt_absolute_time();
                break;
            }

            if (hrt_absolute_time() - state_time > config.waypoint_timeout_us) {
                PX4_ERR("Waypoint %d timeout. Landing.", current_waypoint + 1);
                _state = TRIGGER_LAND;
            }
            break;
        }

        case HOLD_WAYPOINT: {
            const Waypoint &target = waypoints[current_waypoint];
            sp.position[0] = target.x; sp.position[1] = target.y; sp.position[2] = target.z; sp.yaw = target.yaw;

            if (hrt_absolute_time() - state_time > config.hold_time_us) {
                current_waypoint++;

                if (current_waypoint >= waypoint_count) {
                    _state = TRIGGER_LAND;
                } else {
                    PX4_INFO("Moving to waypoint %d/%d.", current_waypoint + 1, waypoint_count);
                    _state = GOTO_WAYPOINT;
                    state_time = hrt_absolute_time();
                }
            }
            break;
        }

        case TRIGGER_LAND:
            PX4_INFO("Mission Done. Auto Landing...");
            send_vehicle_command(_cmd_pub, vehicle_command_s::VEHICLE_CMD_NAV_LAND);
            _state = MONITOR_LANDING;
            break;

        case MONITOR_LANDING:
            if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
                PX4_INFO("Disarmed. Resetting to IDLE.");
                _state = IDLE;
                home_set = false;
            }
            break;

        case EXIT:
            request_stop();
            break;
        }

        if (_state != IDLE && _state != MONITOR_LANDING && _state != EXIT) {
            _traj_sp_pub.publish(sp);
        }
    }

    orb_unsubscribe(local_pos_sub);
    orb_unsubscribe(status_sub);
}

MissionConfig MyMission::mission_config() const
{
    MissionConfig config{};
    config.altitude_m = my_mission::constrain_float(_param_mymis_alt.get(), 0.5f, 20.0f);
    config.distance_m = my_mission::constrain_float(_param_mymis_dist.get(), 0.5f, 50.0f);
    config.hold_time_us = my_mission::seconds_to_us(my_mission::constrain_float(_param_mymis_hold_t.get(), 0.0f, 30.0f));
    config.acceptance_radius_m = my_mission::constrain_float(_param_mymis_acc_rad.get(), 0.1f, 2.0f);
    config.waypoint_timeout_us = my_mission::seconds_to_us(my_mission::constrain_float(_param_mymis_timeout.get(), 2.0f, 120.0f));
    config.pattern = my_mission::normalized_pattern(_param_mymis_pattern.get());

    return config;
}

void MyMission::send_vehicle_command(uORB::Publication<vehicle_command_s>& pub, uint16_t command, float p1, float p2) {
    vehicle_command_s cmd{};
    cmd.command = command;
    cmd.param1 = p1;
    cmd.param2 = p2;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = false;
    cmd.timestamp = hrt_absolute_time();
    pub.publish(cmd);
}

MyMission *MyMission::instantiate(int argc, char *argv[]) {
    return new MyMission(0, false);
}

int MyMission::task_spawn(int argc, char *argv[]) {
    _task_id = px4_task_spawn_cmd("my_mission", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2048, (px4_main_t)&run_trampoline, (char *const *)argv);
    if (_task_id < 0) { _task_id = -1; return -errno; }
    return 0;
}

int MyMission::print_usage(const char *reason) {
    if (reason) PX4_WARN("%s\n", reason);
    PRINT_MODULE_DESCRIPTION("Custom Mission (Trigger: Mission Mode)");
    PRINT_MODULE_USAGE_NAME("my_mission", "command");
    PRINT_MODULE_USAGE_COMMAND("start");
    return 0;
}

extern "C" __EXPORT int my_mission_main(int argc, char *argv[]) {
    return MyMission::main(argc, argv);
}
