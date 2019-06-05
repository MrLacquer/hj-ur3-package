#!/usr/bin/env python

import sys
import rospy
import copy, math
from math import pi

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, CollisionObject
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

from ar_pose.msg import ARMarker
from std_srvs.srv import Empty

GROUP_NAME_ARM = "manipulator"
FIXED_FRAME = 'world'
#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"

class TestMove():

    def __init__(self):
        roscpp_initialize(sys.argv)        
        rospy.init_node('ur3_move',anonymous=True)

        self.clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)

        self.scene = PlanningSceneInterface()
        self.robot_cmd = RobotCommander()

        self.robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
        #robot_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        self.robot_arm.set_goal_orientation_tolerance(0.005)
        self.robot_arm.set_planning_time(5)
        self.robot_arm.set_num_planning_attempts(5)

        rospy.sleep(2)
        # Allow replanning to increase the odds of a solution
        self.robot_arm.allow_replanning(True)      
        
        init_table_goal = self.robot_arm.get_current_joint_values()
        init_table_goal[0] = 0.2
        init_table_goal[1] = -1.983025375996725
        init_table_goal[2] = -2.4233086744891565
        init_table_goal[3] = 0.9490636587142944
        init_table_goal[4] = 1.4068996906280518
        init_table_goal[5] = -3.060608450566427
        self.robot_arm.go(init_table_goal, wait=True)
        rospy.sleep(0.5)
        
        self.clear_octomap()
        print("====== move plan go to init table goal ======")   

    def move_code(self):
        
        self.robot_arm.set_named_target("home")  #go to goal state.
        self.robot_arm.go(wait=True)
        print("====== move plan go to home 1 ======")        
        rospy.sleep(0.5)        
        self.clear_octomap()
        print("===== octomap clear =====")
#        print("====== move plan go to up ======")

        ''' test code 04/20'''
        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = 0.2
        joint_goal[1] = -pi/2
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = 0      
        
        self.robot_arm.go(joint_goal, wait=True)

        rospy.sleep(0.5)
        print("===== move plan test code =====")
        ''' ======================================= '''        
        self.clear_octomap()
        print("===== octomap clear =====")

        robot_state = self.robot_arm.get_current_pose();
        robot_angle = self.robot_arm.get_current_joint_values();

        last_goal = self.robot_arm.get_current_joint_values()
        last_goal = [0.2, -1.98, -2.4233, 0.9490, 1.4068, -3.0606]
        self.robot_arm.go(last_goal, wait=True)
        rospy.sleep(0.5)
        
        print(robot_state)
        print(robot_angle)

        self.clear_octomap()
        print("===== octomap clear =====")


#        robot_arm.set_named_target("up")
#        robot_arm.go(wait=True)

class Marker_detect():
    def marker_pose(self, data):
        self.posx = data.pose.position.x
        self.posy = data.pose.position.y
        self.posz = data.pose.position.z
            
          
        
if __name__=='__main__':
    tm = TestMove()
    tm.__init__()
    tm.move_code()


    rospy.spin()
    roscpp_shutdown()