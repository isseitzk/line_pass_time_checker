# time_checker

## Purpose

This package measures the time elapsed for a vehicle to travel from a designated start line to a finish line.

- The vehicle's footprint is approximated as a rectangular polygon.
- The timer starts when any vertex of the rectangle crosses the start line.
- The timer stops after all vertices of the rectangle have passed the finish line, at which point it outputs the total time taken.

## How to Use

If you want to launch time_checker node:

```sh
ros2 launch time_checker time_checker.launch.py
```

If you want to set the start/finish line by ROS2 service call: 

```sh
ros2 service call /reset_timer time_checker_msgs/srv/ResetTimer "{location: '[location where you set start/goal line]'}"
```

For example: 
```sh
ros2 service call /reset_timer time_checker_msgs/srv/ResetTimer "{location: 'shiojiri.reference_intersection'}"
```

You can set start/goal line when launching:
```sh
ros2 launch time_checker time_checker.launch.py location:='shiojiri.reference_intersection'
```

The start/goal line can be designed from config/turn_spot.yaml.
