# PlantMap3D-ROS

## Current development branch : ```setup/ros_drivers```
### Initial source code PR link : https://github.com/precision-sustainable-ag/PlantMap3D-ROS/pull/38

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

## Current development branch : ```debug/segmentation_fps``
### Initial source code PR link : https://github.com/precision-sustainable-ag/PlantMap3D-ROS/pull/38

## Creating a ROS workspace


## Intended System Flow-diagram : 

[Note : This diagram is expected to be updated with design upgrades]
<img src="https://user-images.githubusercontent.com/71589098/223761652-da859b3b-0a5c-4f2f-87a6-755c1b02e5df.png" width=85% height=75% />

## Sample interpolation results :

<img src="https://user-images.githubusercontent.com/71589098/234415197-434c50d7-e88f-4c58-88ef-951392a8840e.png" width=45% height=45% /> <img src="https://user-images.githubusercontent.com/71589098/234415530-8fb0a08b-708e-4fe0-bc6f-1058f79256b7.png" width=48% height=45% />

```
pip install pynmea2
```

