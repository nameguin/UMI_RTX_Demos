cmake_minimum_required(VERSION 3.5)
project(umi_rtx_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
 set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Requis pour utiliser Qt5 avec CMake
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(umi_rtx_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(realsense2 REQUIRED)
#find_package(ZED 3 REQUIRED)
#find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)


include_directories(
        include
        ${rclcpp_INCLUDE_DIRS}
        ${rviz_common_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
        ${realsense2_INCLUDE_DIR}
        #${CUDA_INCLUDE_DIRS}
        #${ZED_INCLUDE_DIRS}
)

add_library(armcmds src/drivers/armlib/armcmds.cpp)
target_include_directories(armcmds PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_library(armraw src/drivers/armlib/armraw.cpp)
target_include_directories(armraw PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_library(armerror src/drivers/armlib/armerror.cpp)
target_include_directories(armerror PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

##################################### Node camera
add_executable(nodeCamera
        src/node_camera.cpp
)

ament_target_dependencies(nodeCamera
        rclcpp
        sensor_msgs
        OpenCV
        cv_bridge
        geometry_msgs
        ament_index_cpp
        std_msgs
        umi_rtx_interfaces
)

target_link_libraries(nodeCamera "${cpp_typesupport_target}" ${OpenCV_LIBS}  ${realsense2_LIBRARY})
target_include_directories(nodeCamera PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

##################################### Node camera API
#[===[
add_executable(nodeCameraAPI
        src/node_camera_API.cpp
)

ament_target_dependencies(nodeCameraAPI
        rclcpp
        sensor_msgs
        OpenCV
        cv_bridge
        geometry_msgs
        ament_index_cpp
        std_msgs
        ZED
        CUDA
)

target_link_libraries(nodeCameraAPI "${cpp_typesupport_target}" ${OpenCV_LIBS} ${ZED_LIBRARY_DIR} ${CUDA_LIBRARY_DIRS})
target_include_directories(nodeCameraAPI PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
#]===]

##################################### Node Arm
add_executable(nodeArm
        src/node_arm.cpp

        src/drivers/armlib/armcmds.cpp
        src/drivers/armlib/armraw.cpp
        src/drivers/armlib/armerror.cpp
        src/drivers/umi/umiinit.cpp
        src/drivers/umi/umicmds.cpp

        src/arm_parts/arm.cpp
        src/arm_parts/forearm.cpp
        src/arm_parts/joint.cpp
        src/arm_parts/wrist.cpp
)

ament_target_dependencies(nodeArm
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        umi_rtx_interfaces

)

target_link_libraries(nodeArm armcmds armraw armerror)
target_link_libraries(nodeArm "${cpp_typesupport_target}" )
target_include_directories(nodeArm PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

##################################### Node Inverse Kinematics
add_executable(nodeInverseKinematics
        src/node_invkin.cpp
)

ament_target_dependencies(nodeInverseKinematics
        rclcpp
        geometry_msgs
        std_msgs
        sensor_msgs
        pinocchio
        ament_index_cpp
        umi_rtx_interfaces
)

target_link_libraries(nodeInverseKinematics "${cpp_typesupport_target}" )
target_include_directories(nodeInverseKinematics PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

##################################### Node Simulation
add_executable(nodeSimu
        src/node_simu.cpp
)

ament_target_dependencies(nodeSimu
        rclcpp
        std_msgs
        sensor_msgs
        ament_index_cpp
        umi_rtx_interfaces
)

target_link_libraries(nodeSimu "${cpp_typesupport_target}" )
target_include_directories(nodeSimu PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


##################################### GUI
add_executable(GUI
        src/main_gui.cpp
        src/node_commands.cpp
)

ament_target_dependencies(GUI
        rclcpp
        geometry_msgs
        std_msgs
        sensor_msgs
        ament_index_cpp
        rviz_common
        rviz_default_plugins
        rviz_rendering
        rviz_ogre_vendor
        OpenCV
        cv_bridge
        umi_rtx_interfaces
)

target_link_libraries(GUI "${cpp_typesupport_target}" Qt5::Core
        Qt5::Widgets
        rviz_common::rviz_common
        rviz_rendering::rviz_rendering
        rviz_ogre_vendor::OgreMain
        rviz_ogre_vendor::OgreOverlay)
target_include_directories(GUI PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(nodeGame
        src/node_game.cpp
)

ament_target_dependencies(nodeGame
        rclcpp
        std_msgs
        ament_index_cpp
        umi_rtx_interfaces
)
target_link_libraries(nodeGame "${cpp_typesupport_target}")
target_include_directories(nodeGame PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


if(BUILD_TESTING)
 find_package(ament_lint_auto REQUIRED)
 # the following line skips the linter which checks for copyrights
 # uncomment the line when a copyright and license is not present in all source files
 #set(ament_cmake_copyright_FOUND TRUE)
 # the following line skips cpplint (only works in a git repo)
 # uncomment the line when this package is not in a git repo
 #set(ament_cmake_cpplint_FOUND TRUE)
 ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS nodeCamera nodeArm nodeInverseKinematics nodeSimu GUI nodeGame
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include
        DESTINATION include)

install(DIRECTORY launch rviz urdf images
        DESTINATION share/${PROJECT_NAME}
)

ament_package()