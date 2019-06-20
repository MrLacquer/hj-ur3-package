#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import sys, time
import rospy
import copy, math
import tf
from math import pi

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import *
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, CollisionObject
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

from ar_track_alvar_msgs.msg import AlvarMarkers
from std_srvs.srv import Empty

GROUP_NAME_ARM = "manipulator"
FIXED_FRAME = 'world'
#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"

if __name__ == '__main__':
      roscpp_initialize(sys.argv)        
      rospy.init_node('ur3_wrist3_tester',anonymous=True)

      scene = PlanningSceneInterface()
      robot_cmd = RobotCommander()

      robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
      #robot_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
      robot_arm.set_goal_orientation_tolerance(0.005)
      robot_arm.set_planning_time(5)
      robot_arm.set_num_planning_attempts(5)

      rospy.sleep(2)
      # Allow replanning to increase the odds of a solution
      robot_arm.allow_replanning(True)              

      robot_joints = robot_arm.get_current_joint_values()

      robot_joints[5] = math.radians(90)
      print "robot_joints wrist3: deg, rad", math.degrees(robot_joints[5]), robot_joints[5]
      robot_arm.go(robot_joints, wait=True)

      while not rospy.is_shutdown(): 
            robot_joints = robot_arm.get_current_joint_values()
            if robot_joints[5] > abs(math.radians(170)):
                  if robot_joints[5] < 0:
                        robot_joints[5] = robot_joints[5] + math.radians(90) # wrist3
                        print "---- robot_joints wrist3: deg, rad", math.degrees(robot_joints[5]), robot_joints[5]
                  elif robot_joints[5] > 0:
                        robot_joints[5] = robot_joints[5] - math.radians(90)
                        print "++++ robot_joints wrist3: deg, rad", math.degrees(robot_joints[5]), robot_joints[5]
            else:
                  robot_joints[5] = robot_joints[5] - math.radians(90) # wrist3
                  print "<<<< robot_joints wrist3: deg, rad", math.degrees(robot_joints[5]), robot_joints[5]

            #robot_joints[5] = math.radians(90)
            #print "robot_joints wrist3: deg, rad", math.degrees(robot_joints[5]), robot_joints[5]
            robot_arm.go(robot_joints, wait=True)


            #time.sleep(0.05)
      