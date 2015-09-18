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

