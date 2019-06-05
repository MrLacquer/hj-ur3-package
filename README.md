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
    - inculde the stookroom gazebo world[programming robots with ROS](https://github.com/Jpub/ROS)
    - inculde the ar_track_alvar pacakge.
    - inculde the ur3_bringup.launch in the "my_ur3_d435_control.launch"

    '''
    Simulation:
    $ roslaunch ur3_hj2_moveit_config ur3_stockroom.launch
    $ roslaunch ur3_hj2_moveit_config ur3_hj2_moveit_planning_execution.launch sim:=true
    $ roslaunch ur3_hj2_moveit_config moveit_rviz_sim.launch
    
    $ rosrun my_pcl_tutorial example input:=/d435/depth/points
    $ rosrun ur3_moveit marker_move_demo.py
    '''

    '''
    Real robot
    $ roslaunch ur3_hj2_moveit_config my_ur3_d435_control.launch
    $ roslaunch ur3_hj2_moveit_config ur3_hj2_moveit_planning_execution.launch
    $ roslaunch ur3_hj2_moveit_config moveit_rviz.launch
    '''

    