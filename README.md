# PlantMap3D-ROS

Robot Operating System (ROS):

System Requirements : 

```
lsb_release - a
```
>> Description: Ubuntu 20.04.2 LTS

2. ROS version

```
rosversion -d
```
>> noetic

Installing ROS :

Add ROS noetic to your Ubuntu source file list.

```
echo "deb http://packages.ros.org/ros/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros-focal.list
```
Add official ROS keyring.

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
or

```
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
```

Updating the ROS package index

```
sudo apt update
```

Install ROS noetic (desktop full version recommended)

```
sudo apt install ros-noetic-desktop-full
```

Setting up ROS : 

Source the setup.bash file

```
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

Verify the changes by running the following command

```
tail ~/.basbhrc
```

Update changes

```
source ~/.bashrc
```

Finally, verify the installation 

```
roscore
```

Setting up the repository

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


## Some common erros

1. 
![image](https://user-images.githubusercontent.com/71589098/218086068-499a5122-87de-429d-a2ca-c7f20ace9aa5.png)

Answer : Try the following command line :

```
sudo apt-get install ros-noetic-catkin-virtualenv
```

# BioMassEstimator module
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
