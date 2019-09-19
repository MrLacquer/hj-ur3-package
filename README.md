# hj-ur3-package
for backup repository


### update
- 2019.05.07 첫번째 백업

### update 
- 2019.05.17 두번째 백업
    - inculde the [ur_moder_driver](https://github.com/ros-industrial/ur_modern_driver) for ROS Kinetic:
        - editing $ ur_modern_driver\src\ur_hardware_interface.cpp
    - include the version realsense d435 moveit package
    

### update 
- 2019.06.05 세번째 백업
    - inculde the ur_moveit rosnode packages.
    - inculde the stookroom gazebo world [programming robots with ROS](https://github.com/Jpub/ROS)
    - inculde the ar_track_alvar pacakge.
    - inculde the ur3_bringup.launch in the "my_ur3_d435_control.launch"

### update 
- 2019.06.20 네번째 백업
    - update the new ur3_hj3_moveit package
        - this package's urdf include the hj hand.
        - editing the urdf which is shoulder pan joint limit pi/2 → pi
    - update the ur_moveit rosnode.
        - the node latest version is 6. 
        - test code(node) name is ur_marker_test.py
    - update the stookroom gazebo world. 
    
### update 
- 2019.07.15 다섯번째 백업
    - update the hj_hand communication python code
    ```
    pc +++ serial2usb module +++ bluetooth(M)
        <----->
            bluetooth(S) +++ stm32f303
    ```
    

### How to start?
```
Before simulation setting:
registration the "GAZEBO_MODEL_PATH env." [ref. link](https://github.com/Jpub/ROS/tree/master/stockroom_bot)

$ export GAZEBO_MODEL_PATH=${HOME}/catkin_ws/src/stockroom_bot/models

or

$ export GAZEBO_MODEL_PATH=${HOME}/catkin_ws/src/hj-ur3-package/ur3_my_ros/ur3_hj3_moveit_config/hj3_gazebo_urhand/models

Simulation:
$ roslaunch ur3_hj3_moveit_config ur3_stockroom.launch

with hammer simulation
$ roslaunch ur3_hj3_moveit_config ur3_hammer_stockroom.launch

$ roslaunch ur3_hj3_moveit_config ur3_hj3_moveit_planning_execution.launch sim:=true
$ roslaunch ur3_hj3_moveit_config moveit_rviz_sim.launch

$ rosrun my_pcl_tutorial example input:=/d435/depth/points
$ rosrun ur3_moveit marker_move_demo.py
```

```
Real robot
$ roslaunch ur3_hj3_moveit_config my_ur3_d435_control.launch
$ roslaunch ur3_hj3_moveit_config ur3_hj3_moveit_planning_execution.launch
$ roslaunch ur3_hj3_moveit_config moveit_rviz.launch

$ rosrun my_pcl_tutorial example input:=/camera/depth/color/points
$ rosrun ur3_moveit marker_move_demo_6.py
```

    