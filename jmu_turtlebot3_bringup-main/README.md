# jmu_turtlebot3_bringup
Package that addresses some rough edges with using TB3 with Dashing

Usage:

Start the turtelbot as usual, then use this package to bring up rviz2
visualizations:

Basic visualization:

```
ros2 launch jmu_turtlebot3_bringup rviz2.launch.py
```

Nav2 visualization:

```
ros2 launch jmu_turtlebot3_bringup nav2_rviz2.launch.py
```


These will launch the `tb_fixer` node along with rviz2 with an
appropriate configuration file. The `tb_fixer` node fixes the
following issues:

- Scan messages don't show up in rviz because they run ahead of tf.
    - This node delays and republishes `/scan` messages on `/scan_viz`.
- Rviz expects camera info messages on a different topic (when using Gazebo), and the frame_id is incorrect.
    - Republish `camera/image_raw` to `camera/image_raw_viz`
    - Republish `camera/camera_info` to `camera/image_raw_viz/camera_info`
    - `frame_id` is corrected for both
- Turtlebot continues to obey last `cmd_vel` message forever, which can
   be awkward if a control node crashes or exits while the robot is moving.
     - Periodically check to see if no other nodes are publishing to `cmd_vel`
       if not, publish a stop command.
