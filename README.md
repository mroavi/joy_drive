# joy_drive

A minimal ROS 2 package for joystick-based robot teleoperation using custom logic.

This node listens to `/joy` messages (from a `joy_node`) and publishes velocity commands  
(`geometry_msgs/Twist`) to `/diff_cont/cmd_vel_unstamped` based on button presses and  
joystick input.

## Features

- Use X (`button 0`) to move forward, O (`button 1`) to move backward  
- Use right joystick (axis index configurable) to turn  
- Configurable parameters via YAML  

## Usage

Launch with a compatible `joy_node`, for example:

```bash
ros2 launch my_bot joy_launch.py
```

Make sure your YAML file includes parameters for both `joy_node` and `joy_drive`.

## Parameters

Example YAML:

```bash
joy_drive:
  ros__parameters:
    forward_button: 0
    backward_button: 1
    angular_axis: 0
    speed: 0.5
    turn_speed: 0.5
```
