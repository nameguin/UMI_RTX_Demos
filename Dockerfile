FROM stereolabs/zed:4.0-devel-cuda11.8-ubuntu22.04

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

ENV USER=root
# Setlocale
RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt install software-properties-common -y
RUN add-apt-repository universe


RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


RUN apt update
RUN apt upgrade -y


RUN apt install ros-iron-desktop python3-argcomplete -y


RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
RUN source /opt/ros/iron/setup.bash
RUN source ~/.bashrc

RUN apt-get install git wget -y

WORKDIR /Project/umi_rtx
RUN git clone https://github.com/nameguin/UMI_RTX_Demos
WORKDIR /Project/umi_rtx/UMI_RTX_Demos

RUN apt update && \
    apt install ros-iron-pinocchio ros-iron-xacro -y && \
    cd /opt/ros/iron/include/rviz_common && \
    if ! [ -e ./tool_manager.hpp ]; then \
        wget "https://raw.githubusercontent.com/ros2/rviz/foxy/rviz_common/src/rviz_common/tool_manager.hpp"; \
    fi
RUN mkdir logs

RUN apt install python3-colcon-common-extensions -y
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/iron/lib:/opt/ros/iron/opt/rviz_ogre_vendor:/opt/ros/iron/opt/aml_cpp_vendor' >> ~/.bashrc
RUN echo 'export PATH=$PATH:/opt/ros/iron/bin' >> ~/.bashrc
RUN echo 'export PYTHONPATH=$PYTHONPATH:/opt/ros/iron/lib/python3.10/site-packages' >> ~/.bashrc

WORKDIR /Project/umi_rtx/UMI_RTX_Demos
RUN apt install nano -y
