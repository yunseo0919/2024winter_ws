# 2024winter_ws
## Imitation learning simulation environment setup
![Screenshot from 2024-02-26 14-41-51](https://github.com/yunseo0919/2024winter_ws/assets/161008012/77f31cd6-d6d1-4df2-8cda-71aaa69505f0)

## Edited from packages below

    https://github.com/saga0619/dyros_tocabi_v2
    https://github.com/saga0619/mujoco_ros_sim
    https://github.com/saga0619/tocabi_gui
    https://github.com/saga0619/tocabi_cc
    https://github.com/minong22/haptic_driver
    https://github.com/Hokyun-Lee/2024winter_ws

## Installation
### Haptic device
Use omega.6 ros driver

You can get the pose of the haptic device.

    git clone https://github.com/minong22/haptic_driver.git
    cd catkin_ws/src/sdk-3.14.0/bin
    sudo ./HapticInit

reference: (https://github.com/minong22/haptic_driver)

### Dyros_tocabi_v2
1. Clone repository
   
```cpp
cd catkin_ws/src
git clone --recurse-submodules https://github.com/saga0619/dyros_tocabi_v2
```

2. Install CustomController, GUI, Simulator and All-in-One Requirements installation

    reference: (https://github.com/saga0619/dyros_tocabi_v2)

3. Change
```cpp
mujoco_ros_sim/mujoco_ros/CMakeLists.txt (Edit)
mujoco_ros_sim/mujoco_ros/package.xml (Edit)
mujoco_ros_sim/mujoco_ros/src/main.cpp (Edit)
mujoco_ros_sim/mujoco_ros/src/mjros.cpp (Edit)
mujoco_ros_sim/mujoco_ros/include/mjros.h (Edit)
tocabi_cc/CMakeLists.txt (Edit)
tocabi_cc/package.xml (Edit)
tocabi_cc/src/cc.cpp (Edit)
tocabi_cc/include/cc.h (Edit)

mujoco_ros_sim/mujoco_ros/src/mujoco_rgbd_camera.cpp (New)
mujoco_ros_sim/mujoco_ros/include/mujoco_rgbd_camera.hpp (New)
dyros_tocabi_v2/tocabi_description/mujoco_model/dyros_tocabi_with_object_2024winter.xml (New)
dyros_tocabi_v2/tocabi_controller/launch/simulation_with_camera.launch (New)
```

## How to Use
1. dependency(OpenCV)
```cpp
sudo apt update
sudo apt install libopencv-dev
```
2. Build & Simulation Start
```cpp
cd catkin_ws/
catkin_make
rosrun haptic_ros_driver haptic_ros_driver
```

   Ctrl+Alt+T (New Terminal)
```cpp
roslaunch tocabi_controller simulation.launch hand:=false
```
