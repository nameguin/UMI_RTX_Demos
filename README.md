# UMI RTX Robotic Arm Demos

## Installation

This project is built and tested with **Ubuntu 22.04** and **ROS2 Iron**.

1. **ROS2 Installation**

    First, ensure you have ROS2 Iron installed. If not, follow the instructions provided [here](https://docs.ros.org/en/iron/Installation.html) for installation. 
    After installation, set up your environment by sourcing ROS2. If you're not using bash, adjust accordingly by replacing ".bash" with your shell type in the following command:
    
    ```bash
    source /opt/ros/iron/setup.bash
    ```
    To avoid sourcing it every time, consider adding this line to the end of your `~/.bashrc` file.

2. **Install Dependencies**

    Start by updating your package lists:
    ```bash
    sudo apt update
    ```
   Then, install the necessary dependencies including Pinocchio for inverse kinematics or xacro:
    ```bash
    sudo apt install ros-iron-pinocchio -y
    sudo apt install ros-iron-xacro -y
    ```

3. **Colcon installation**

    Install Colcon, the build tool required for this project:
    ```bash
    sudo apt udpate
    sudo apt install python3-colcon-common-extensions
   ```
4. **Autocompletion**

    Enable autocompletion for Colcon:
    ```bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    ```

   To avoid sourcing it every time, add this line to the end of your `~/.bashrc` file.

### Build the Package

Navigate to your ROS workspace directory:

```bash
cd ROS_ws
```

Build the package using Colcon:

```bash
colcon build
```

Once built, source it to set up your environment:
```bash
source install/setup.bash
```

### Run simulation

To run the simulation, execute the following command:
```bash
ros2 launch umi_rtx_controller simu.launch.py
```

## Docker

### 1. Docker installation

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-docker2 nvidia-container-toolkit
sudo systemctl daemon-reload
sudo systemctl restart docker
```

### 2. Build the docker image

```bash
# Place yourself in umi_rtx_demos
docker build -t "name" .
```

Be careful to replace "name" with the name you want, and everything is ready !

### 3. Run the image into a container

```bash
# Give the permission to use the screen
xhost +

# Launch the container (replace "name" with the name you chose)
docker run --gpus all -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --rm "name":latest
```

### 4. Run the program

Start by running these commands:

```bash
cd ROS_ws/
colcon build
source install/setup.bash
cd ..
```

Then, if you want to launch only the simulation, run this:

```bash
ros2 launch umi_rtx_controller simu.launch.py
```

If you want to use the arm, run this:

```bash
./start_arm.sh
```


