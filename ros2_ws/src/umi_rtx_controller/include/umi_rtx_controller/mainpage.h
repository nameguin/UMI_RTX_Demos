/**
 * @mainpage UMI-RTX Robotic Arm for Tic-Tac-Toe
 *
 * @section authors_sec Authors
 * - MEGUIN Nathan \<nathan.meguin@etu.uca.fr\> (ISIMA - Software Engineering)
 *
 * @section description_sec Description
 * This repository is designed to set up a ROS2 interface to make the UMI-RTX Arm play a Tic-Tac-Toe game against a human.
 *
 * @section config_sec Manual Installation
 * This project is built and tested with **Ubuntu 22.04** and **ROS2 Iron**.
 *
 * @subsection ros_install_subsec ROS Installation
 * Ensure you have ROS2 Iron installed. If not, follow the instructions provided [here](https://docs.ros.org/en/iron/Installation.html).
 *
 * Set up your environment by sourcing ROS2. If you're not using bash, replace ".bash" with your shell type:
 * 
 * \code{.sh}
 *   source /opt/ros/iron/setup.bash
 * \endcode
 * 
 * To avoid sourcing it every time, consider adding this line to the end of your `~/.bashrc` file.
 *
 * @subsection dependencies_subsec Install Dependencies
 * Update your package lists:
 * 
 * \code{.sh}
 *   sudo apt update
 * \endcode
 * 
 * Install necessary dependencies including Pinocchio for inverse kinematics:
 * 
 * \code{.sh}
 *   sudo apt install ros-iron-pinocchio -y
 *   sudo apt install ros-iron-xacro -y
 * \endcode
 *
 * @subsection colcon_subsec Colcon Installation
 * Install Colcon, the build tool required for this project:
 * 
 * \code{.sh}
 *   sudo apt update
 *   sudo apt install python3-colcon-common-extensions
 * \endcode
 *
 * @subsection auto_subsec Autocompletion
 * Enable autocompletion for Colcon:
 * 
 * \code{.sh}
 *   source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
 * \endcode
 * 
 * To avoid sourcing it every time, add this line to the end of your `~/.bashrc` file.
 *
 * @subsection build_subsec Build the Package
 * Navigate to your ROS workspace directory:
 * 
 * \code{.sh}
 *   cd ROS_ws
 * \endcode
 * 
 * Build the package using Colcon:
 * 
 * \code{.sh}
 *   colcon build
 * \endcode
 * 
 * Once built, source it to set up your environment:
 * 
 * \code{.sh}
 *   source install/setup.bash
 * \endcode
 *
 * @subsection run_subsec Run the Package
 * To run the simulation, execute:
 * 
 * \code{.sh}
 *   ros2 launch umi_rtx_controller simu.launch.py
 * \endcode
 * 
 * To use the arm, execute:
 * 
 * \code{.sh}
 *   ./start_arm.sh
 * \endcode
 *
 * @section docker_sec Docker
 *
 * @subsection install_subsec Docker Installation
 * Install Docker and the NVIDIA Docker toolkit:
 * 
 * \code{.sh}
 *   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
 *   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
 *   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
 * 
 *   sudo apt-get update && sudo apt-get install -y nvidia-docker2 nvidia-container-toolkit
 *   sudo systemctl daemon-reload
 *   sudo systemctl restart docker
 * \endcode
 *
 * @subsection build_docker_subsec Build the Docker Image
 * 
 * \code{.sh}
 *   # Navigate to umi_rtx_demos
 *   docker build -t "name" .
 * \endcode
 * 
 * Replace "name" with your chosen name for the Docker image.
 *
 * @subsection run_image_docker_subsec Run the Docker Container
 * 
 * \code{.sh}
 *   # Grant permission to use the screen
 *   xhost +
 * 
 *   # Launch the container (replace "name" with the name you chose)
 *   docker run --gpus all -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --rm "name":latest
 * \endcode
 *
 * @subsection run_docker_subsec Run the Program
 * Start by running:
 * 
 * \code{.sh}
 *   cd ROS_ws/
 *   colcon build
 *   source install/setup.bash
 *   cd ..
 * \endcode
 * 
 * Then, to launch the simulation:
 * 
 * \code{.sh}
 *   ros2 launch umi_rtx_controller simu.launch.py
 * \endcode
 * 
 * And to use the arm:
 * 
 * \code{.sh}
 *   ./start_arm.sh
 * \endcode
 */
