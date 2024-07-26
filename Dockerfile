# Use the ROS Iron Perception base image
FROM ros:iron-perception

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

ENV USER=root

# Setlocale
RUN apt update && apt install locales -y
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt install software-properties-common -y
RUN add-apt-repository universe

RUN apt update && apt install curl -y

# Fetch and add ROS keyring file
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg

# Update the sources.list to point to the ROS Iron repositories
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt upgrade -y

# Install ROS Iron desktop and python3-argcomplete
RUN apt install ros-iron-desktop python3-argcomplete -y

# Source ROS setup.bash in bashrc
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
RUN source /opt/ros/iron/setup.bash
RUN source ~/.bashrc

# Install git and wget
RUN apt-get install git wget -y

# Clone the repository and install dependencies
WORKDIR /home/Stage
RUN git clone https://github.com/nameguin/UMI_RTX_Demos.git
WORKDIR /home/Stage/UMI_RTX_Demos
RUN apt update

# Pinocchio for the inverse kinematics
RUN apt install ros-iron-pinocchio -y
RUN apt install ros-iron-xacro -y

# Install python3-colcon-common-extensions
RUN apt install python3-colcon-common-extensions -y
RUN apt-get update \
 && apt-get install -y \
    ros-iron-librealsense2* \
    ros-iron-realsense2-* \
 && rm -rf /var/lib/apt/lists/*

# Update LD_LIBRARY_PATH, PATH, PYTHONPATH
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/iron/lib:/opt/ros/iron/opt/rviz_ogre_vendor:/opt/ros/iron/opt/aml_cpp_vendor' >> ~/.bashrc
RUN echo 'export PATH=$PATH:/opt/ros/iron/bin' >> ~/.bashrc
RUN echo 'export PYTHONPATH=$PYTHONPATH:/opt/ros/iron/lib/python3.10/site-packages' >> ~/.bashrc

# Install nano
#RUN apt install nano -y

