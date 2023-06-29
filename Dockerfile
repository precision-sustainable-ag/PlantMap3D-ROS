FROM nvidia/cudagl:11.1.1-base-ubuntu20.04

FROM ros:noetic

RUN apt-get update
 
# Install ROS Noetic
RUN sh -c echo "deb http://packages.ros.org/ros/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros-focal.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full  
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN apt-get update && apt-get install -y \
    python3-pip  
RUN apt-get update && apt-get install -y ros-noetic-catkin-virtualenv


RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

COPY . /ROS_ws
WORKDIR /ROS_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /ROS_ws ; catkin_make'

RUN /bin/bash -c "source devel/setup.bash" 
RUN echo "source ~/ROS_ws/devel/setup.bash" >> ~/.bashrc

# RUN pip install -r requirements.txt




