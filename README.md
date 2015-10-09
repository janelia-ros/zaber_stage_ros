zaber_stage_ros
===============

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD

##Running

```shell
roslaunch zaber_stage zaber_stage.launch x_serial_number:=123 x_alias:=10 y_serial_number:=123 y_alias:=11
```

```shell
rosrun zaber_stage home.py
rosservice call /zaber_stage_node/get_pose
rosservice call /zaber_stage_node/moving
```

```shell
rosrun zaber_stage key_teleop.py
```

###Use units of mm with T-LSR450B actuators and Travel of 450mm

```shell
roslaunch zaber_stage zaber_stage.launch x_serial_number:=123 x_alias:=10 x_microstep_size:=0.000496 x_travel:=450 y_serial_number:=123 y_alias:=11 y_microstep_size:=0.000496 y_travel:=450
```

```shell
rosrun zaber_stage key_teleop.py _up_y_rate:=5 _down_y_rate:=-5 _left_x_rate:=-5 _right_x_rate:=5
```

```shell
rosrun zaber_stage move_absolute.py _x:=100 _y:=200
```

```shell
rosrun zaber_stage move_relative.py _x:=-75 _y:=-100
```

```shell
rosrun zaber_stage move_absolute_percent.py _x:=50 _y:=25
```

```shell
rosrun zaber_stage move_relative_percent.py _x:=-25 _y:=25
```

##Zaber Device Python Module

For more information, see [zaber_device_python](https://github.com/janelia-pypi/zaber_device_python)
