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


