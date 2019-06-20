#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import sys
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

class TestMove():

      def __init__(self):
            roscpp_initialize(sys.argv)        
            rospy.init_node('ur3_move',anonymous=True)
            
            self.listener = tf.TransformListener()
            self.goal_x = 0
            self.goal_y = 0
            self.goal_z = 0
            self.goal_roll = 0
            self.goal_pitch = 0
            self.goal_yaw = 0

            self.goal_ori_x = 0
            self.goal_ori_y = 0
            self.goal_ori_z = 0
            self.goal_ori_w = 0

            self.wpose = []
            self.marker = []
            self.tf_list = []

            self.m_idd = []
            self.m_pose_x = []
            self.m_pose_y = []
            self.m_pose_z = []
            self.m_ori_w = []
            self.m_ori_x = []
            self.m_ori_y = []
            self.m_ori_z = []

            self.trans = []
            self.rot = []

            self.pose_goal = Pose()
            #rospy.loginfo("Waiting for ar_pose_marker topic...")
            #rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

            #rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback)
            #rospy.loginfo("Maker messages detected. Starting followers...")


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
            
            self.clear_octomap()
            

      def ar_callback(self, msg):
            #marker = msg.markers[1]
            self.marker = msg.markers
            ml = len(self.marker)
            m_id = self.marker[0].id
            #print "marker length: ", ml, ml-1
            #print "marker id: ", m_id, self.marker[1].id
            #self.m_idd = []
            #self.m_pose_x = []
            #self.m_pose_y = []
            #self.m_pose_z = []
            #self.m_ori_w = []
            #self.m_ori_x = []
            #self.m_ori_y = []
            #self.m_ori_z = []
            
            for ii in range(0, ml):
                  self.m_idd.append(self.marker[ii].id)
                  self.m_pose_x.append(self.marker[ii].pose.pose.position.x)
                  self.m_pose_y.append(self.marker[ii].pose.pose.position.y)
                  self.m_pose_z.append(self.marker[ii].pose.pose.position.z)

                  self.m_ori_w.append(self.marker[ii].pose.pose.orientation.w)
                  self.m_ori_x.append(self.marker[ii].pose.pose.orientation.x)
                  self.m_ori_y.append(self.marker[ii].pose.pose.orientation.y)
                  self.m_ori_z.append(self.marker[ii].pose.pose.orientation.z)
                  
            '''
            print "id: ", self.m_idd
            print "pose_x: ", self.m_pose_x
            print "pose_y: ", self.m_pose_y
            print "pose_z: ", self.m_pose_z
            print "ori_w: ", self.m_ori_w                        
            print "ori_x: ", self.m_ori_x
            print "ori_y: ", self.m_ori_y
            print "ori_z: ", self.m_ori_z
            '''

            m_pose_goal = Pose()
            m_pose_goal.orientation = Quaternion(*quaternion_from_euler(0, -1.5, 0))
            m_pose_goal.position.x = self.m_pose_x[0] + 0.029# red line      0.2   0.2
            m_pose_goal.position.y = self.m_pose_y[0] - 0.7 # green line  0.15   0.15
            m_pose_goal.position.z = self.m_pose_z[0] + 0.3 # blue line   # 0.35   0.6
            '''
            print "goal_pose: ", m_pose_goal
            self.robot_arm.set_pose_target(m_pose_goal)
            self.robot_arm.go(True)

            print "==== check check"

            marker_joint_goal = [0.07100913858593216, -1.8767615298285376, 2.0393206555899503, -1.8313959190971882, -0.6278395875738125, 1.6918219826764682]
            self.robot_arm.go(marker_joint_goal, wait=True)
            print "====== robot joint value: \n"
            print marker_joint_goal

            #self.robot_arm.go(marker_joint_goal, wait=True)                        
            
            pos_x = self.marker[0].pose.pose.position.x
            pos_y = self.marker[0].pose.pose.position.y
            pos_z = self.marker[0].pose.pose.position.z

            dist = math.sqrt(
                  (pos_x*pos_x) + (pos_y * pos_y)
                  )
'''
            #ori_x = marker.pose.pose.orientation.x
            #ori_y = marker.pose.pose.orientation.y
            #ori_z = marker.pose.pose.orientation.z
            #ori_w = marker.pose.pose.orientation.w
            #print(m_id)
            #print "==========="
            #print 'id: ', m_id
            #print 'pos: ', pos_x, pos_y, pos_z
            
            #print('ori: ', ori_x, ori_y, ori_z, ori_w)


      def move_code(self):
            planning_frame = self.robot_arm.get_planning_frame()
            print "========== plannig frame: ", planning_frame

            self.wpose = self.robot_arm.get_current_pose()
            print"====== current pose : ", self.wpose

            #self.pose_goal = self.robot_arm.get_current_pose()
            self.pose_goal.position.x = 0.062
            self.pose_goal.position.y = 0.194
            self.pose_goal.position.z = 0.878
            self.pose_goal.orientation.x = 0.673
            self.pose_goal.orientation.y = 0.673
            self.pose_goal.orientation.z = -0.217
            self.pose_goal.orientation.w = 0.217
            
            #print "========== goal frame: ", self.pose_goal
            self.robot_arm.set_pose_target(self.pose_goal)
            self.robot_arm.go(True)           

      def look_up_down(self):
            self.clear_octomap()
            print "======== clear_octomap"
            look_up_down = self.robot_arm.get_current_joint_values()
            #print "before: ", look_up_down
                        
            look_up_down[4] = look_up_down[4] + (math.radians(30)) # wrist2
            self.robot_arm.go(look_up_down, wait=True)

            look_up_down[4] = look_up_down[4] - (math.radians(60)) # wrist2
            self.robot_arm.go(look_up_down, wait=True)

            look_up_down[4] = look_up_down[4] + (math.radians(30)) # wrist2
            self.robot_arm.go(look_up_down, wait=True)

                       

      def plan_cartesian_path(self, scale = 1.0):
            
            waypoints = []
            ii = 1

            self.wpose = self.robot_arm.get_current_pose().pose
            #self.wpose.position.z -= scale * (0.695)  # First move up (z)
            #self.wpose.position.y += scale * (0.199)  # and sideways (y)
            #waypoints.append(copy.deepcopy(self.wpose))
            print "===== robot arm pose: ", self.wpose
            ''' # 19.06.11 10:45 분 코딩
            print self.wpose.position.x," + (",scale," * ",self.goal_x,")"
            print self.wpose.position.y," + (",scale," * ",self.goal_y,")"

            self.wpose.position.x = self.wpose.position.x - (scale * self.goal_x)
            self.wpose.position.y = self.wpose.position.y + (scale * self.goal_y)
            
            print "self.wpose ", ii, ": [",self.wpose.position.x, self.wpose.position.y, self.wpose.position.z,"]"
            waypoints.append(copy.deepcopy(self.wpose))
            ii += 1

            print self.wpose.position.z," + (",scale," * ",self.goal_z,")"
            self.wpose.position.z += (scale * self.goal_z)
            print "self.wpose ", ii, ": [",self.wpose.position.x, self.wpose.position.y, self.wpose.position.z,"]"
            waypoints.append(copy.deepcopy(self.wpose))
            '''
            self.wpose.position.x = (scale * self.wpose.position.x) - 0.10
            #self.wpose.position.y = (scale * self.goal_y) - 0.1
            
            #print "self.wpose ", ii, ": [",self.wpose.position.x, self.wpose.position.y, self.wpose.position.z,"]"
            waypoints.append(copy.deepcopy(self.wpose))
            ii += 1

            #print self.wpose.position.z," + (",scale," * ",self.goal_z,")"
            #self.wpose.position.z = (scale * self.goal_z) + 0.1

            self.wpose.position.y = (scale * self.goal_y) + 0.05
            waypoints.append(copy.deepcopy(self.wpose))
            ii += 1

            
            (plan, fraction) = self.robot_arm.compute_cartesian_path(waypoints, 0.01, 0.0)

            return plan, fraction

      def execute_plan(self, plan):
            group = self.robot_arm
            group.execute(plan, wait=True)

      def print_ar_pose(self):
            rospy.loginfo("Waiting for ar_pose_marker topic...")
            rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback)
            rospy.loginfo("Maker messages detected. Starting followers...")

      def go_to_move(self, scale = 1.0):
            self.wpose = self.robot_arm.get_current_pose()
            planning_frame = self.robot_arm.get_planning_frame()
            print "========== plannig frame: ", planning_frame

            self.pose_goal.position.x = (scale * self.goal_x) + 0.10 # base_link to wrist2 x-offset
            self.pose_goal.position.y = (scale * self.goal_y) - 0.25
            self.pose_goal.position.z = (scale * self.goal_z) + 0.72 - 0.115 # world to base_link z-offset and coke can offset
            self.pose_goal.orientation = Quaternion(*quaternion_from_euler(0, 0, 1.54))
            ''' 19.06.1 15:58 코딩
            self.pose_goal.orientation.x = self.rot[0]
            self.pose_goal.orientation.y = self.rot[1]
            self.pose_goal.orientation.z = self.rot[2]
            self.pose_goal.orientation.w = self.rot[3]
            '''
            #self.pose_goal.orientation.x = self.goal_roll + 1.57
            #self.pose_goal.orientation.y = self.goal_pitch + 1.57
            #self.pose_goal.orientation.z = self.goal_yaw 
            
            print "========== goal frame: ", self.pose_goal
            self.robot_arm.set_pose_target(self.pose_goal)
            #self.robot_arm.go(True)

      def tf_listener(self):  # 수정사안: ar_pose id의 가져와서 위치 값을 읽어와야함.
            try:
                  child_tf = '/ar_marker_9'
                  parent_tf = '/base_link'                 
                  
                  (self.trans,self.rot) = self.listener.lookupTransform(parent_tf, child_tf, rospy.Time(0)) 
                  # rosrun tf tf_echo /base_link /ar_marker_9 랑 같은 역할
                  # transformation: linear transformation, rotation: quaternion
                  print "======= trans[x, y, z]: ", self.trans
                  print "======= rotat[x, y, z, w]: ", self.rot
                  
                  (self.goal_roll, self.goal_pitch, self.goal_yaw) = euler_from_quaternion(self.rot)
                  print "======= rot(rad): ", self.goal_roll, self.goal_pitch, self.goal_yaw
                  
                  #convert 확인용
                  #quat = quaternion_from_euler(goal_roll, goal_pitch, goal_yaw)
                  #print quat
                                          
                  self.goal_x = self.trans[0]
                  self.goal_y = self.trans[1]
                  self.goal_z = self.trans[2]                  
                  
                  
            except Exception as ex:
                  print "======== tf_listener Exception!!"
                  rospy.loginfo(ex)

      def look_object(self):
            try:
                  
                  look_object = self.robot_arm.get_current_joint_values()
                  #look_object[4] = look_object[4] + (math.radians(90)) # wrist2
                  look_object[5] = look_object[5] + (3.1) # wrist3
                  #look_object[3] = look_object[3] - (math.radians(00)) # wrist1
                  self.robot_arm.go(look_object, wait=True)                 
                  #look_object[5] = look_object[5] + (math.radians(90)) # wrist3
                  #self.robot_arm.go(look_object, wait=True)                 

                 
            except Exception as ex:
                  print "======== look_object Exception!!"
                  rospy.loginfo(ex)

        
if __name__=='__main__':
      tm = TestMove()
      #tm.__init__()
      #tm.move_code()
      #tm.look_up_down()

      #tm.print_ar_pose()
      
      tm.tf_listener()

      print "============ Press `Enter` to plan and go to bin ..."
      raw_input()
      tm.go_to_move()

      #print "============ Press `Enter` to execute a saved path ..."
      #raw_input()
      

      #print "============ Look object ..."
      #tm.look_object()

      br = tf.TransformBroadcaster()
      rate = rospy.Rate(10.0)

      ## cartesian path 확인용      
      #m_wpose_pose = [tm.wpose.position.x, tm.wpose.position.y, tm.wpose.position.z]
      #m_wpose_ori = [tm.wpose.orientation.x, tm.wpose.orientation.y, tm.wpose.orientation.z, tm.wpose.orientation.w]
      ##      
      orientation_list = quaternion_from_euler(tm.pose_goal.orientation.x, tm.pose_goal.orientation.y, tm.pose_goal.orientation.z)      
      #print "======= quaternion from euler: ", orientation_list

      m_wpose_pose = [tm.pose_goal.position.x, tm.pose_goal.position.y, tm.pose_goal.position.z]
      #m_wpose_ori = orientation_list
      m_wpose_ori = [tm.pose_goal.orientation.x, tm.pose_goal.orientation.y, tm.pose_goal.orientation.z, tm.pose_goal.orientation.w]
      
      print "====== main pose: ",m_wpose_pose, m_wpose_ori


      ii = 0
      while ii < 5:
            ii += 1
            br.sendTransform(
                  m_wpose_pose,
                  m_wpose_ori,
                  rospy.Time.now(),
                  "goal_wpose",
                  "world")
            rate.sleep()

      tm.robot_arm.go(True)

      #print "============ Look object ..."
      tm.look_object()

      cartesian_plan, fraction = tm.plan_cartesian_path()
      tm.execute_plan(cartesian_plan)


      print "======= Press the Enter"
      raw_input()

      tm.move_code()
    
      #rospy.spin()
      #roscpp_shutdown()
