# HexapodElevationMapping

Elevation mapping hardware implementation with **velodyne VLP16 Lidar**, **D435 RGBD camera** and **T265 motion tracking camera**.

## Dependencies

Apt install

```bash
sudo apt install ros-$ROS_DISTRO-realsense2-camera ros-noetic-realsense2-description
```

Git submodules

- [grid_map](https://github.com/ANYbotics/grid_map)
- [elevation_mapping](https://github.com/ANYbotics/elevation_mapping)
- [kindr](https://github.com/ANYbotics/kindr)
- [kindr_ros](https://github.com/ANYbotics/kindr_ros)
- [message_logger](https://github.com/ANYbotics/message_logger)
- [velodyne](https://github.com/ros-drivers/velodyne)
- [realsense_ros_gazebo](https://github.com/nilseuropa/realsense_ros_gazebo)
- [velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator.git)

## Acknowledgement

Many thanks to [Tipriest](https://github.com/Tipriest) for him providing the original solution.

Origin repository:

- [Tipriest/ElevationMap](https://github.com/Tipriest/ElevationMap)
- [Tipriest/velodyne](https://github.com/Tipriest/velodyne)
- [Tipriest/kindr_ros](https://github.com/Tipriest/kindr_ros)
- [Tipriest/realsense_ros_gazebo](https://github.com/Tipriest/realsense_ros_gazebo)
- [Tipriest/realsense-ros](https://github.com/Tipriest/realsense-ros)
