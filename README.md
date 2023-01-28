# PlantMap3D-ROS

## Creating a ROS workspace

1. Clone the repository
2. Build the project and all dependencies
```
catkin_make
```

## To run the camera driver

Start a ros master in a new terminal window
```
roscore
```

In a new terminal

```
rosrun oakd_camera_driver oakd_camera_wrapper.py
```
