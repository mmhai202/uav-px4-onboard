#pragma once

#include "MissionTypes.hpp"

#include <math.h>

#include <uORB/topics/vehicle_local_position.h>

namespace my_mission
{

// Small header-only planner helpers keep the module at one .cpp while still
// separating mission geometry from PX4/uORB control flow.
inline float constrain_float(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}

	if (value > max_value) {
		return max_value;
	}

	return value;
}

inline hrt_abstime seconds_to_us(float seconds)
{
	return static_cast<hrt_abstime>(seconds * static_cast<float>(USEC_PER_SEC));
}

// Reject unknown param values by falling back to the smallest, safest pattern.
inline int normalized_pattern(int pattern)
{
	if (pattern < PATTERN_LINE || pattern > PATTERN_OUT_AND_BACK) {
		return PATTERN_LINE;
	}

	return pattern;
}

inline int build_waypoints(Waypoint *waypoints, int max_waypoints, float home_x, float home_y, float home_z,
			   const MissionConfig &config)
{
	// PX4 local frame is NED: decreasing z climbs above the home altitude.
	const float z = home_z - config.altitude_m;
	const float d = config.distance_m;
	int count = 0;

	auto add_waypoint = [&](float x, float y) {
		if (count >= max_waypoints) {
			return;
		}

		Waypoint &wp = waypoints[count];
		wp.x = x;
		wp.y = y;
		wp.z = z;

		// Point the nose toward the next leg. The first waypoint is vertical
		// takeoff, so yaw is kept neutral until horizontal motion starts.
		if (count == 0) {
			wp.yaw = 0.0f;

		} else {
			const Waypoint &prev = waypoints[count - 1];
			wp.yaw = atan2f(wp.y - prev.y, wp.x - prev.x);
		}

		count++;
	};

	add_waypoint(home_x, home_y);

	switch (config.pattern) {
	case PATTERN_SQUARE:
		add_waypoint(home_x, home_y + d);
		add_waypoint(home_x + d, home_y + d);
		add_waypoint(home_x + d, home_y);
		add_waypoint(home_x, home_y);
		break;

	case PATTERN_OUT_AND_BACK:
		add_waypoint(home_x, home_y + d);
		add_waypoint(home_x, home_y);
		break;

	case PATTERN_LINE:
	default:
		add_waypoint(home_x, home_y + d);
		break;
	}

	return count;
}

// Use 3D distance so takeoff altitude and horizontal legs share one acceptance
// rule. This keeps state transitions independent of the selected pattern.
inline bool reached_waypoint(const vehicle_local_position_s &pos, const Waypoint &waypoint, float acceptance_radius_m)
{
	const float dx = pos.x - waypoint.x;
	const float dy = pos.y - waypoint.y;
	const float dz = pos.z - waypoint.z;
	const float distance_sq = dx * dx + dy * dy + dz * dz;
	return distance_sq <= acceptance_radius_m * acceptance_radius_m;
}

} // namespace my_mission
