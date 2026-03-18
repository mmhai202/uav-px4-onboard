/**
 * Module: my_mission
 * Description: Trigger mission when Pilot switches to MISSION mode.
 * Workflow:
 * 1. Upload dummy mission to allow switching to Mission Mode.
 * 2. Arm vehicle.
 * 3. Switch to MISSION mode -> Code takes over (Offboard).
 * 4. Switch to HOLD to Abort.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>

// uORB Topics
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class MyMission : public ModuleBase<MyMission>, public ModuleParams {
public:
    MyMission(int example_param, bool example_flag) : ModuleParams(nullptr) {}
    virtual ~MyMission() = default;

    static int task_spawn(int argc, char *argv[]);
    static MyMission *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
    static int print_usage(const char *reason = nullptr);

    void run() override;

private:
    // --- Configuration ---
    static constexpr float TARGET_ALT   = -2.0f; // 2m Altitude
    static constexpr float MOVE_DIST    =  2.0f; // 2m Distance
    static constexpr time_t WAIT_TIME   =  2_s;  // Wait time at waypoints

    // --- State Machine ---
    enum State {
        IDLE,            // Waiting for MISSION mode
        WARMUP,          // Switching to Offboard
        TAKEOFF,
        HOLD_TOP,
        MOVE_RIGHT,
        HOLD_SIDE,
        TRIGGER_LAND,
        MONITOR_LANDING,
        EXIT
    } _state = IDLE;

    void send_vehicle_command(uORB::Publication<vehicle_command_s>& pub, uint16_t command, float p1 = 0.0f, float p2 = 0.0f);
};

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
    float home_x = 0.0f, home_y = 0.0f;
    bool home_set = false;

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
            // Lock Home
            if (!home_set && pos.z_valid) {
                home_x = pos.x; home_y = pos.y; home_set = true;
            }
            sp.position[0] = home_x; sp.position[1] = home_y; sp.position[2] = pos.z;

            // Wait 1s to ensure Offboard signal is stable, then Switch Mode
            if (hrt_absolute_time() - start_time > 1_s) {
                PX4_INFO("Switching to Offboard...");
                send_vehicle_command(_cmd_pub, vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, 6); // 6 = Offboard
                
                // If not armed, try to arm (Safety: usually pilot arms first)
                if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
                    send_vehicle_command(_cmd_pub, vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
                }
                
                _state = TAKEOFF;
                state_time = hrt_absolute_time();
            }
            break;

        case TAKEOFF:
            sp.position[0] = home_x; sp.position[1] = home_y; sp.position[2] = TARGET_ALT;
            if (pos.z < (TARGET_ALT + 0.2f)) {
                PX4_INFO("Altitude Reached. Hovering...");
                _state = HOLD_TOP;
                state_time = hrt_absolute_time();
            }
            break;

        case HOLD_TOP:
            sp.position[0] = home_x; sp.position[1] = home_y; sp.position[2] = TARGET_ALT;
            if (hrt_absolute_time() - state_time > WAIT_TIME) {
                PX4_INFO("Moving Right...");
                _state = MOVE_RIGHT;
            }
            break;

        case MOVE_RIGHT:
            sp.position[0] = home_x; sp.position[1] = home_y + MOVE_DIST; sp.position[2] = TARGET_ALT;
            if (pos.y > (home_y + MOVE_DIST - 0.2f)) {
                PX4_INFO("Target Reached. Hovering...");
                _state = HOLD_SIDE;
                state_time = hrt_absolute_time();
            }
            break;

        case HOLD_SIDE:
            sp.position[0] = home_x; sp.position[1] = home_y + MOVE_DIST; sp.position[2] = TARGET_ALT;
            if (hrt_absolute_time() - state_time > WAIT_TIME) {
                _state = TRIGGER_LAND;
            }
            break;

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
