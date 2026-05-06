#pragma once

#include <drivers/drv_hrt.h>

namespace my_mission
{

// Keep mission sizing fixed and stack-friendly: PX4 modules should avoid heap
// allocation in the flight loop unless there is a clear need.
static constexpr hrt_abstime USEC_PER_SEC = 1000000ULL;
static constexpr int MAX_WAYPOINTS = 6;

// Pattern IDs are PX4 param values. Keep these stable so saved airframe params
// do not change meaning after firmware updates.
enum Pattern {
	PATTERN_LINE = 0,
	PATTERN_SQUARE = 1,
	PATTERN_OUT_AND_BACK = 2
};

// Runtime config copied from PX4 params when a mission starts. The active
// mission should not change mid-flight if someone edits params from the shell.
struct MissionConfig {
	float altitude_m;
	float distance_m;
	hrt_abstime hold_time_us;
	float acceptance_radius_m;
	hrt_abstime waypoint_timeout_us;
	int pattern;
};

// Local NED waypoint. In PX4 local position, x/y are horizontal meters and z is
// positive down, so flying above home means target_z < home_z.
struct Waypoint {
	float x;
	float y;
	float z;
	float yaw;
};

} // namespace my_mission
