# PlantMap3D-ROS



## This branch acts as a beta development branch, create a new branch for feature development and create a pull request.


## Intended System Flow-diagram : 

[Note : This diagram is expected to be updated with design upgrades]
<img src="https://user-images.githubusercontent.com/71589098/223761652-da859b3b-0a5c-4f2f-87a6-755c1b02e5df.png" width=95%% height=75% />

## Current working pipeline : 


<img src="https://user-images.githubusercontent.com/71589098/229500208-06b18871-3812-44b6-901f-64430809140a.png" width=95%% height=95% />

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

## Some common erros

1. To run camera trigger node

```rosrun camera_trigger_driver camera_trigger_driver```

2.

<img src="https://user-images.githubusercontent.com/71589098/218086068-499a5122-87de-429d-a2ca-c7f20ace9aa5.png" width=50%% height=50% />

Answer : Try the following command line :

```
sudo apt-get install ros-noetic-catkin-virtualenv
```
2. 
>> python module pynmea not found

```
pip install pynmea2
```

