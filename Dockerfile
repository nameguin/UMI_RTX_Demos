FROM ros:iron-perception

ENV WS_DIR="/ros2_ws"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]

# Install locales
RUN apt update && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
    build-essential \
    cmake \
    git-all \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe

RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
 && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
RUN source /opt/ros/iron/setup.bash
RUN source ~/.bashrc

# Update and install ROS
RUN apt update && apt upgrade -y
RUN apt install -y \
    ros-iron-desktop \
    python3-argcomplete \
    ros-iron-pinocchio \
    ros-iron-xacro \
    python3-colcon-common-extensions

# Configure ROS environment
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
RUN source /opt/ros/iron/setup.bash
RUN source ~/.bashrc

# Create non-root user
#RUN groupadd -r rosuser && useradd -r -g rosuser rosuser
ENV USER=root

# Download UMI RTX project
WORKDIR /Project/umi_rtx
RUN git clone https://github.com/nameguin/UMI_RTX_Demos
WORKDIR /Project/umi_rtx/UMI_RTX_Demos
RUN mkdir logs

# Configure environment variables
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/iron/lib:/opt/ros/iron/opt/rviz_ogre_vendor:/opt/ros/iron/opt/aml_cpp_vendor' >> ~/.bashrc
RUN echo 'export PATH=$PATH:/opt/ros/iron/bin' >> ~/.bashrc
RUN echo 'export PYTHONPATH=$PYTHONPATH:/opt/ros/iron/lib/python3.10/site-packages' >> ~/.bashrc

# Set working directory
WORKDIR /Project/umi_rtx/UMI_RTX_Demos
RUN apt install nano -y
