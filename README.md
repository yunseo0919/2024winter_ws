# 2024winter_ws
## Imitation learning simulation environment setup
![Screenshot from 2024-02-26 14-41-51](https://github.com/yunseo0919/2024winter_ws/assets/161008012/77f31cd6-d6d1-4df2-8cda-71aaa69505f0)

### Edited from packages below

    https://github.com/saga0619/dyros_tocabi_v2
    https://github.com/saga0619/mujoco_ros_sim
    https://github.com/saga0619/tocabi_gui
    https://github.com/saga0619/tocabi_cc
    https://github.com/minong22/haptic_driver

## Installation
### 1. Haptic device
Use omega.6 ros driver

You can get the pose of the haptic device.

    git clone https://github.com/minong22/haptic_driver.git
    cd catkin_ws/src/sdk-3.14.0/bin
    sudo ./HapticInit
   
    //cd catkin_ws/
    //catkin_make
    //rosrun haptic_ros_driver haptic_ros_driver

reference: (https://github.com/minong22/haptic_driver)

### Dyros_tocabi_v2
1. Clone repository
'''
    cd catkin_ws/src
    git clone --recurse-submodules 
'''
2. Install CustomController, GUI, Simulator and All-in-One Requirements installation

reference: (https://github.com/saga0619/dyros_tocabi_v2)

3. Change

    


roslaunch tocabi_controller simulation.launch hand:=false

