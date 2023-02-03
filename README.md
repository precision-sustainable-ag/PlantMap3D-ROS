# PlantMap3D-ROS

## Creating a ROS workspace

1. Clone the repository
2. Build the project and all dependencies
```
cd PlantMap3D-ROS
catkin_make
```
3. Source the ROS workspace
```
source PlantMap3D-ROS/devel/setup.bash
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
