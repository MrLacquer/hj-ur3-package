#!/usr/bin/env python

import sys
import rospy
import copy, math
from math import pi

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

GROUP_NAME_ARM = "manipulator"
FIXED_FRAME = 'world'
#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"

class TestMove():
    def __init__(self):
        roscpp_initialize(sys.argv)        
        rospy.init_node('ur3_move',anonymous=True)

        scene = PlanningSceneInterface()
        robot = RobotCommander()

        robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
        #robot_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        robot_arm.set_goal_orientation_tolerance(0.005)
        robot_arm.set_planning_time(5)
        robot_arm.set_num_planning_attempts(5)
        ###eef = robot.get_end_effector_link()  #-----error 01

        rospy.sleep(2)
        # Allow replanning to increase the odds of a solution
        robot_arm.allow_replanning(True)
        #scene.remove_attached_object(GRIPPER_FRAME, "part")

   
        # clean the scene
        #scene.remove_world_object("table")
        #scene.remove_world_object("part")
   
        robot_arm.set_named_target("home")  #go to goal state.
        robot_arm.go(wait=True)
        '''
        right_gripper.set_named_target("open")
        right_gripper.go(wait=True)
        '''
        rospy.sleep(1)
    
        print("====== move plan go to home 1 ======")
        
        # publish a demo scene
        p = PoseStamped()
        p.header.frame_id = robot.get_planning_frame()       
   

        # add an object to be grasped
        p.pose.position.x = 0.170
        p.pose.position.y = 0.04
        p.pose.position.z = 0.3
        scene.add_box("part", p, (0.07, 0.01, 0.2))
      
        rospy.sleep(1)
             
        start_pose = PoseStamped()
        start_pose.header.frame_id = FIXED_FRAME
   
        # start the gripper in a neutral pose part way to the target
        start_pose.pose.position.x = -0.00756585784256
        start_pose.pose.position.y = -0.225419849157
        start_pose.pose.position.z = 0.117192693055
        start_pose.pose.orientation.x = 0.95493721962
        start_pose.pose.orientation.y = -0.0160209629685
        start_pose.pose.orientation.z = -0.00497157918289
        start_pose.pose.orientation.w = 0.296333402395
        print("going to pick up pose")

        robot_arm.set_pose_target(start_pose)
	    #right_gripper.set_named_target("close")
        robot_arm.go(wait=True)
        #right_gripper.go(wait=True)
       
        rospy.sleep(1)
        print("====== move plan go to up ======")

        ''' test code 04/20'''
        pose_zero = PoseStamped()
        pose_zero.header.frame_id = FIXED_FRAME
        pose_zero.pose.position.x = 0.0
        pose_zero.pose.position.y = 0.194525
        pose_zero.pose.position.z = 0.694149
        pose_zero.pose.orientation.x = 0.0
        pose_zero.pose.orientation.y = 0.0
        pose_zero.pose.orientation.x = 0.707062
        pose_zero.pose.orientation.x = 0.707062

        robot_arm.set_pose_target(pose_zero)
        robot_arm.go(wait=True)

        rospy.sleep(3)
        print("===== move plan test code =====")
        '''                '''
        
        #robot_arm.set_named_target("up")
        #robot_arm.go(wait=True)
        
        
        next_pose = PoseStamped()
        next_pose.header.frame_id = FIXED_FRAME
        next_pose.pose.position.x = -0.100732862949
        next_pose.pose.position.y = -0.210876911879
        next_pose.pose.position.z = 0.244678631425
        next_pose.pose.orientation.x = 0.784905433655
        next_pose.pose.orientation.y = -0.177844554186
        next_pose.pose.orientation.z = -0.131161093712
        next_pose.pose.orientation.w = 0.578870952129
        
        robot_arm.set_pose_target(next_pose)
        #right_gripper.set_named_target("open")
        robot_arm.go(wait=True)
        #right_gripper.go(wait=True)
        print("going to pick up pose 2")

        rospy.sleep(3)

        print("====== move plan go to home 3 ======")          
        robot_arm.set_named_target("home")
        robot_arm.go(wait=True)        
        
        rospy.spin()
        roscpp_shutdown()

        
if __name__=='__main__':
    TestMove()