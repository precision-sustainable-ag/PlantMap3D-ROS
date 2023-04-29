# PlantMap3D-ROS


"""
Add project introduction

"""

## This branch acts as a beta development branch, create a new branch for feature development and create a pull request.

## Setting up the repository

Create a workspace

```
mkdir ROS_dev_ws
cd ROS_dev_ws
```

Clone the repository

```
git clone https://github.com/precision-sustainable-ag/PlantMap3D-ROS.git
```

Build the project

```
cd PlantMap3D-ROS
catkin_make
```

Source the repository

```
source devel/setup.bash
```

Add the setup.bash file to .bashrc file

```
echo "source ~/ROS_dev_ws/PlantMap3D-ROS/devel/setup.bash" >> ~/.bashrc
```

Check if all requirements are installed

```
pip3 install -r requirements.txt
```

## Current development branch : ```setup/ros_drivers```
### Initial source code PR link : https://github.com/precision-sustainable-ag/PlantMap3D-ROS/pull/38

## Creating a ROS workspace


## Intended System Flow-diagram : 

[Note : This diagram is expected to be updated with design upgrades]
<img src="https://user-images.githubusercontent.com/71589098/223761652-da859b3b-0a5c-4f2f-87a6-755c1b02e5df.png" width=85% height=75% />

## Sample interpolation results :

<img src="https://user-images.githubusercontent.com/71589098/234415197-434c50d7-e88f-4c58-88ef-951392a8840e.png" width=45% height=45% /> <img src="https://user-images.githubusercontent.com/71589098/234415530-8fb0a08b-708e-4fe0-bc6f-1058f79256b7.png" width=48% height=45% />



## Some common erros

1. To run camera trigger node

```rosrun camera_trigger_driver camera_trigger_driver```

2.

<img src="https://user-images.githubusercontent.com/71589098/218086068-499a5122-87de-429d-a2ca-c7f20ace9aa5.png" width=80% height=85% />

Answer : Try the following command line :

```
sudo apt-get install ros-noetic-catkin-virtualenv
```
2. 
>> python module pynmea not found

```
pip install pynmea2
```

