/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * My Mission altitude above home
 *
 * Target mission altitude relative to the local home position.
 *
 * @group My Mission
 * @unit m
 * @decimal 1
 * @min 0.5
 * @max 20.0
 */
PARAM_DEFINE_FLOAT(MYMIS_ALT, 2.0f);

/**
 * My Mission horizontal distance
 *
 * Horizontal leg length used by the selected mission pattern.
 *
 * @group My Mission
 * @unit m
 * @decimal 1
 * @min 0.5
 * @max 50.0
 */
PARAM_DEFINE_FLOAT(MYMIS_DIST, 2.0f);

/**
 * My Mission waypoint hold time
 *
 * Time to hold after reaching each waypoint.
 *
 * @group My Mission
 * @unit s
 * @decimal 1
 * @min 0.0
 * @max 30.0
 */
PARAM_DEFINE_FLOAT(MYMIS_HOLD_T, 2.0f);

/**
 * My Mission acceptance radius
 *
 * Position error radius used to decide that a waypoint has been reached.
 *
 * @group My Mission
 * @unit m
 * @decimal 2
 * @min 0.1
 * @max 2.0
 */
PARAM_DEFINE_FLOAT(MYMIS_ACC_RAD, 0.25f);

/**
 * My Mission waypoint timeout
 *
 * Maximum time allowed to reach one waypoint before landing.
 *
 * @group My Mission
 * @unit s
 * @decimal 1
 * @min 2.0
 * @max 120.0
 */
PARAM_DEFINE_FLOAT(MYMIS_TIMEOUT, 15.0f);

/**
 * My Mission pattern
 *
 * Selects the waypoint pattern.
 *
 * @value 0 Line
 * @value 1 Square
 * @value 2 Out and back
 * @group My Mission
 * @min 0
 * @max 2
 */
PARAM_DEFINE_INT32(MYMIS_PATTERN, 0);
