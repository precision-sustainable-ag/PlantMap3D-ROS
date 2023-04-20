## README for the drivers

### Start ROS environment

```
roscore
```
### Start the system 

```
roslaunch pm3d_system_config system_start.launch
```

### Cleanly shutdown all nodes

```
roslaunch pm3d_system_config system_shutdown.launch
```

### GPS Module 

The GPS module uses the ```nmea_navsat_driver``` package to read GPSRMC messages from ublox gps sensor. The sensor is connected through serial-UART on the Jetson AGX orin.

Reference :
1. More info on the driver : http://wiki.ros.org/nmea_navsat_driver
2. Some commonly found issues : https://github.com/ros-drivers/nmea_navsat_driver

### Camera location module

This module translates GPS coordinates based on the camera offset given in 'configs/config/cam_location.yaml' file.

This module is a ROS service with the PM3DCameraLocation.srv as the service message structure. 

> PM3DCameraLocation.srv 
```
float32[] gpscoords #lat and lon as list 
float32 gpsheading 
int32 cameraid
- - - 
float32[] newgpscoords 
```
The camera location service is defined as 

```
from camera_location_module.srv import PM3DCameraLocation, PM3DCameraLocationResponse
cam_location_service = rospy.Service('camera_location_service',PM3DCameraLocation,self.camera_location_callback)
```

Example : calling the service 

```
from camera_location_module.srv import PM3DCameraLocation, PM3DCameraLocationRequest
cam_location = rospy.ServiceProxy("camera_location_service",PM3DCameraLocation)
        req = PM3DCameraLocationRequest()
        req.gpscoords = [1.0,50.0]
        req.gpsheading = 90.0
        req.cameraid = 4
        
        resp = cam_location(req)
        
        rospy.loginfo(f"Received Camera 2 Location : {resp.newgpscoords}")
```

### Camera Trigger Driver

The ```camera_trigger_node.cpp```file executes to read GPSRMC data published by the GPS sensor.

> The 'camera_trigger' node is subscribed to ``` /fix ``` topic from the 'nmea_serial_driver'
> The 'camera_trigger' node publishes PM3DGPSData at ``` /camera_trigger ``` topic 

There is a set distance variable ``` float totalDistance ``` which calculates the total distance travelled between the two data collection points. This variable resets after each iteration.

The camera trigger node publishes PM3DGPSData msg type.

> PM3DGPSData.msg
```
float32 latitude
float32 longitude
float32 gps_heading
float32 distance 
bool camera_trigger
```
Based on the previous GPS readings, the heading is computed in angles. A ```PM3DGPSHeading.srv``` service is called to get the GPS heading. 

> PM3DGPSHeading.srv
```
float32 northing
float32 lnorthing
float32 easting
float32 leasting
---
float32 gps_heading
```
### OakD camera driver

#### 1. Working with the OakD cameras

The OakD S2 PoE cameras are used, more info on cameras can be read here : 

The camera_{1,2,,,8} nodes run parallely all the specified cameras and publish to a single topic called ```camera_data```. 
>> Topic ```camera_data``` is subscribed by node ```height_map``

The ```camera_data``` topic uses a custom message called PM3DCameraData.

> PM3DCameraData.msg
```
Header header # this field contains time stamp 
int64[] rgb_data
int64[] depth_map
int64[] segmentation_labels
int64[] rgb_dims
int64[] depth_map_dims
int64[] segmentation_label_dims
int64[] height_map
int64[] height_map_dims
float32[] biomass_estimate
int32 camera_id
camera_trigger_driver/PM3DGPSData gps_data
```

Setting up the camera's :


- Setting up the config files :

Navigate to 'configs/config/image_config.yaml' file , add or remove cameras.

> Example :
```
camera_1:
  node_name: "camera_1"
  camera_id: 1
  ip: "169.254.54.200"
```
- Test camera stream :

Run the camera wrapper :

```
rosrun oakd_camera_driver oakd_camera_wrapper.py
```

### BioMassEstimator module
ROS wrapper will collect the output of the segmentation module for a segmentation array and a height above ground array from a height estimation module

Inputs:
- a segmentation numpy 2D array
- a height above ground 2D array (not used in version 1)

Process:
1. count pixels for each species from segmentation label input
2. apply a linear y = mx+b correlation between species pixel count and biomass (by species if necessary)

Outputs:
- a dictionary of Biomass estimates for each possible species (floats)

Roadmap:
1. Version 1 release date: 2/10/2023
2. Version 2: change biomass estimation to include height information to calculate volume per species. Then the biomass estimate would be a correlation between species volume and biomass. No release date yet.
3. Version 3: change biomass correlation method to be either more statistically sophisticated, or introduce a neural network for biomass estimation.


### Biomass map module

"""
    Enter Biomass map module description
"""

