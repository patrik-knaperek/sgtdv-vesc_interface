# **Racecar VESC interface**

___

© **SGT Driverless**

**Authors:** Matej Dudák, Patrik Knaperek

**Objective:** Interface for motor control and odometry with VESC (MIT) driver.
___

### Related packages

* `path_tracking` (SGT-DV)
* `vesc` (RC car)

### Topic conversions
* `/pathtracking_commands [sgtdv_msgs::Control]` → `vesc/commands/motor/speed [std_msgs::Float64]`, `vesc/commands/servo/position [std_msgs::Float64]`
* `/odom [nav_msgs::Odometry]` → `pose_estimate [sgtdv_msgs::CarPose]`, `velocity_estimate [sgtdv_msgs::CarVel]` (optional)

### Configuration

- sgt command parameters: [`sgt.yaml`](./src/racecar/config/sgt.yaml)
- vesc commands parameters: [`vesc.yaml`](./src/racecar/config/vesc.yaml)
- publish odometry: macro `#define VESC_ODOMETRY` in `RaceCar.h`

## Build

* `./build.bash`

## Launch
* `./launch.bash`

### Issues
* [SGT - RC car - Manuals, error solutions](https://docs.google.com/document/d/1M7zWvItjHyNsSe2zlgr-lzvzVdgYPm58dYw4mHXGxWc/edit?usp=sharing)