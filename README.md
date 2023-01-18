# **Racecar VESC interface**

___

&copy; **SGT Driverless**

**Authors:** Matej Dudák

**Objective:** Interface for motor control with VESC driver.
___

### Used packages

* [vesc](https://github.com/mit-racecar/vesc)

### Supported message and data

- The node uses [Control.msg](src/racecar/msg/Control.msg).
- `speed` value range is 0-100
- `steeringAngle` value range is +-20 degrees

## Launch

``./start.bash``
