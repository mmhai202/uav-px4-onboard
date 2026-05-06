#pragma once

#include "MissionTypes.hpp"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_command.h>

// PX4 module shell and task wrapper. Mission geometry lives in MissionPlanner.hpp
// so this class can stay focused on vehicle mode handling and uORB IO.
class MyMission : public ModuleBase<MyMission>, public ModuleParams {
public:
	MyMission(int, bool) : ModuleParams(nullptr) {}
	virtual ~MyMission() = default;

	static int task_spawn(int argc, char *argv[]);
	static MyMission *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	void run() override;

private:
	// Offboard is only active between WARMUP and landing handoff. HOLD from the
	// pilot resets this state machine and lets PX4 hold/loiter normally.
	enum State {
		IDLE,            // Waiting for MISSION mode
		WARMUP,          // Switching to Offboard
		GOTO_WAYPOINT,
		HOLD_WAYPOINT,
		TRIGGER_LAND,
		MONITOR_LANDING,
		EXIT
	} _state = IDLE;

	void send_vehicle_command(uORB::Publication<vehicle_command_s> &pub, uint16_t command, float p1 = 0.0f,
				  float p2 = 0.0f);
	my_mission::MissionConfig mission_config() const;

	// Params are defined in my_mission_params.c and generated into
	// px4_parameters.hpp by the PX4 build system.
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MYMIS_ALT>) _param_mymis_alt,
		(ParamFloat<px4::params::MYMIS_DIST>) _param_mymis_dist,
		(ParamFloat<px4::params::MYMIS_HOLD_T>) _param_mymis_hold_t,
		(ParamFloat<px4::params::MYMIS_ACC_RAD>) _param_mymis_acc_rad,
		(ParamFloat<px4::params::MYMIS_TIMEOUT>) _param_mymis_timeout,
		(ParamInt<px4::params::MYMIS_PATTERN>) _param_mymis_pattern
	)
};
