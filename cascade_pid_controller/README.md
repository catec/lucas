# Cascade PID Controller

## Overview

This repository contains a PID controller. The position controller is based on a cascade architecture. The input can be a pose, a velocity or a trajectory (position, velocity, acceleration) reference, while the output will be the angle commanded to the drone.

## 1. Dependencies


* APT Dependencies
```
sudo apt install ros-${ROS_DISTRO}-control-toolbox ros-${ROS_DISTRO}-tf-conversions
```

* [cascade_pid_controller_msgs](https://github.com/catec/lucas/tree/main/cascade_pid_controller_msgs)

* [catec_control_manager_msgs](https://github.com/catec/lucas/tree/main/catec_control_manager_msgs)

## 2. Control modes

Position control: The velocity loop reference is given by the output of position PID.

Velocity control: The velocity loop reference is given by the subscribed topic.

Trajectory control: It behaves like position control but using the derivatives of position (velocity and acceleration) to execute a feedforward control.

## 3. Usage

### Subscribed topics

- odometry_topic ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))
> Odometry estimation from drone.

- pose_reference_topic ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
> Use this topic to activate position controller. 

- twist_reference_topic ([geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))
> Use this topic to activate velocity controller. 

- trajectory_reference_topic ([cascade_pid_controller_msgs/TrajCommand](https://github.com/catec/lucas/blob/main/cascade_pid_controller_msgs/msg/TrajCommand.msg))
> Use this topic to activate trajectory controller (feedforward). 

- state_topic ([catec_control_manager_msgs/State](https://github.com/catec/lucas/blob/main/catec_control_manager_msgs/msg/State.msg))
> Receives state information from CATEC Control Manager node by this topic.

### Published topics

> Publish output angles.

### Parameters

- loop_freq (double)
> Main controller loop frequency (hertz).

- max_vel_xy (double)
> Saturation of horizontal velocity reference (m/s).

- max_vel_z (double)
> Saturation of vertical velocity reference (m/s).

- WPNAV_SPEED_DN (double)
> Output saturation of negative vertical velocity (m/s).

- WPNAV_SPEED_UP (double)
> Output saturation of positive vertical velocity (m/s).

##### Position PID (xy | z), velocity PID (x | y | z):

- p (double)
> Proportional term.

- i (double)
> Integral term.

- d (double)
> Derivative term.

- i clamp min and max (double)
> Saturation of acumulated error.

- antiwindup (bool)
> Activate antiwindup (clamping).

- publish_state (bool)
> Publish PIDs state for debug purposes.


### Contact

- Raúl Zahínos [rzahinos@catec.aero]
- José I. Murillo [jimurillo@catec.aero]