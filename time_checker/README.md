# time_checker

## Purpose

This package measures the time elapsed for a vehicle to travel from a designated start line to a finish line.

- The vehicle's footprint is approximated as a rectangular polygon.
- The timer starts when any vertex of the rectangle crosses the start line.
- The timer stops after all vertices of the rectangle have passed the finish line, at which point it outputs the total time taken.

## How to Use

If you want to set the start/finish line from config file:

```sh
ros2 launch time_checker time_checker.launch.py location:='shiojiri.reference_intersection'
```

If you want to set the start/finish line directly: 

```sh
ros2 run time_checker polygon_pass_opposite_line_time_measurer \
--ros-args \
-p vehicle_length:=7.24 \
-p vehicle_width:=2.30 \
-p start_line_p1_x:=65779.1159 \
-p start_line_p1_y:=723.4907 \
-p start_line_p2_x:=65779.6364 \
-p start_line_p2_y:=746.2223 \
-p finish_line_p1_x:=65783.3713 \
-p finish_line_p1_y:=746.1353 \
-p finish_line_p2_x:=65782.851 \
-p finish_line_p2_y:=723.4006
```