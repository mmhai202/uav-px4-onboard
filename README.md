# UAV PX4 Onboard Mission

This repository provides a custom PX4 onboard module (`my_mission`) as a clean starting point for developing and testing autonomous flight algorithms directly on Pixhawk.
- Build a shared repo where everyone can develop onboard UAV algorithms.
- Run mission logic inside PX4 firmware (no external offboard control loop required).
- Keep deployment simple for both SITL and real hardware.

## Behavior
When vehicle enters **Mission mode**, `my_mission` runs automatically.
If switched to **HOLD**, the mission is aborted and the vehicle hovers.

## Setup
### SITL
1. Add `my_mission` to `PX4-Autopilot/src/modules/`.
2. Add to `PX4-Autopilot/boards/px4/sitl/default.px4board`:

```txt
CONFIG_MODULES_MY_MISSION=y
```

### Real flight (FMU-v6x)
1. Add `my_mission` to `PX4-Autopilot/src/modules/`.
2. Add to `PX4-Autopilot/boards/px4/fmu-v6x/default.px4board`:

```txt
CONFIG_MODULES_MY_MISSION=y
```

3. Add to `PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.mc_apps`:

```txt
my_mission start
```

## Recommended params (real flight)
- `SENS_TFMINI_CFG = TELEM 3`
- `EKF2_HGT_REF = 2`
- `EKF2_RNG_CTRL = 2`
- `EKF2_RNG_A_HMAX = 10.000`
- `EKF2_RNG_A_VMAX = 2.0`

## Build and flash (Pixhawk)
```bash
make -j px4_fmu-v6x_default
make px4_fmu-v6x_default upload
```
