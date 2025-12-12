# Final Deployment Checklist

## System Status
- **Wall Following V2**: Fully Implemented (8 Stages).
  - Logic: RT(Start) -> Top(Arc) -> LT(Corner) -> Long(Blind) -> LB -> Bottom(Arc) -> RB -> Long -> Finish.
  - Verification: Syntax fixed. Alignment logic added.

- **Navigation System**: Implemented `vive_navigation.cpp`.
  - Localization: EKF (Encoders + Vive).
  - Config: Calibration points set. Map Goals set.
  - Motion: Pure Pursuit + A* Planning.

- **Chassis Control**: V2 PID Tuned.
  - PID: Right Motor adjusted for hardware issue (Kd increased, Ki decreased).
  - Kinematics: Differential Drive model verified.

## Pre-Deployment Steps (Tomorrow)
1. **Calibration**: Measure the 4 calibration points (plus center) and update `g_nav_calib_points` in `nav_config.cpp`.
2. **Vive Check**: Ensure Vive sensors are visible and providing valid (X, Y) < 8192.
3. **Motor Test**: Verify PID response. If Right motor oscillates, reduce `PID_RIGHT_KP` or increase `PID_RIGHT_KD`.
4. **Wall Distances**: Wall following targets are set to **50mm** (general) and **70mm** (Long Edge Left). Adjust if robot drifts too close/far.

## Logic Flow Verification
- **Start**: `process_convex_corner_rt` aligns distance (8-12cm) before the second 90-degree turn. This ensures the robot enters the loop squarely.
- **Loop**: Counter-clockwise traversal (Left Turns) with Right Wall Following. Matches standard competition setups.
- **Finish**: Hard stop at end of `process_long_edge_right`.

**Ready for deployment.**
