# Use the ROS Iron Perception base image
FROM ros:iron-perception

# Set the working directory to /ros2_ws
ENV WS_DIR="/ros2_ws"
WORKDIR ${WS_DIR}

# Set the default shell to /bin/bash
SHELL ["/bin/bash", "-c"]

# Install locales
RUN apt update && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Set ARG to handle frontend during installation
ARG DEBIAN_FRONTEND=noninteractive

# Update and install necessary tools and libraries
RUN apt-get update \
 && apt-get install -y \
    build-essential \
    cmake \
    git-all \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

# Add universe repository
RUN add-apt-repository universe

# Install librealsense2 and realsense2 ROS packages
RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
 && rm -rf /var/lib/apt/lists/*

# Install rviz2 ROS package
RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
 && rm -rf /var/lib/apt/lists/*

# Update and upgrade the system
RUN apt update && apt upgrade -y

# Install ROS Iron desktop packages and other dependencies
RUN apt install -y \
    ros-iron-desktop \
    python3-argcomplete \
    ros-iron-pinocchio \
    ros-iron-xacro \
    python3-colcon-common-extensions

# Source ROS Iron setup.bash in .bashrc
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
RUN source /opt/ros/iron/setup.bash
RUN source ~/.bashrc

# Set environment variable USER to root
ENV USER=root

# Set working directory to /home/ros_user and clone UMI_RTX_Demos repository
WORKDIR /home/ros_user
RUN git clone --depth=1 https://github.com/nameguin/UMI_RTX_Demos

# Set working directory to /home/ros_user/UMI_RTX_Demos and create logs directory
WORKDIR /home/ros_user/UMI_rtx_Demos
RUN mkdir logs

# Append LD_LIBRARY_PATH, PATH, and PYTHONPATH to .bashrc
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/iron/lib:/opt/ros/iron/opt/rviz_ogre_vendor:/opt/ros/iron/opt/aml_cpp_vendor' >> ~/.bashrc
RUN echo 'export PATH=$PATH:/opt/ros/iron/bin' >> ~/.bashrc
RUN echo 'export PYTHONPATH=$PYTHONPATH:/opt/ros/iron/lib/python3.10/site-packages' >> ~/.bashrc

# Set working directory to /home/ros_user/UMI_RTX_Demos and install nano
WORKDIR /home/ros_user/UMI_RTX_Demos
RUN apt install nano -y
