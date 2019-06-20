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

class TestMove():

      def __init__(self):
            roscpp_initialize(sys.argv)        
            rospy.init_node('ur3_move',anonymous=True)

            rospy.loginfo("Waiting for ar_pose_marker topic...")
            rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_tf_listener)
            rospy.loginfo("Maker messages detected. Starting followers...")

            #self.listener = tf.TransformListener()
            self.goal_x = 0
            self.goal_y = 0
            self.goal_z = 0

            self.goal_ori_x = 0
            self.goal_ori_y = 0
            self.goal_ori_z = 0
            self.goal_ori_w = 0

            
            self.marker = []
            self.position_list = []
            self.orientation_list = []

            self.m_idd = 0
            self.m_pose_x = []
            self.m_pose_y = []
            self.m_pose_z = []
            self.m_ori_w = []
            self.m_ori_x = []
            self.m_ori_y = []
            self.m_ori_z = []

            self.ar_pose = Pose()
            self.goalPoseFromAR = Pose()
            self.br = tf.TransformBroadcaster()
            #self.goalPose_from_arPose = Pose()

            self.trans = []
            self.rot = []

            self.calculed_coke_pose = Pose()
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
            

      def move_code(self):
            planning_frame = self.robot_arm.get_planning_frame()
            print "========== plannig frame: ", planning_frame

            self.wpose = self.robot_arm.get_current_pose()
            print"====== current pose : ", self.wpose                        

            marker_joint_goal = [0.07100913858593216, -1.8767615298285376, 2.0393206555899503, -1.8313959190971882, -0.6278395875738125, 1.6918219826764682]
            self.robot_arm.go(marker_joint_goal, wait=True)

      def look_object(self):
            
            look_object = self.robot_arm.get_current_joint_values()
            
            ## wrist3 현재 상태가 0도 인지, 360도인지에 따라 90도의 움직임을 -로 할지, + 할지 고려함
            print "wrist3 joint value(deg, rad): ", math.degrees(look_object[5]), look_object[5]
            look_object[5] = math.radians(90)
            '''
            if look_object[5] > abs(math.radians(180)):
                  if look_object[5] < 0:
                        look_object[5] = look_object[5] + math.radians(90) # wrist3
                  elif look_object[5] > 0:
                        look_object[5] = look_object[5] - math.radians(90)
            else:
                  look_object[5] = look_object[5] - math.radians(90) # wrist3
            '''
            print "wrist3 joint value(deg, rad): ", math.degrees(look_object[5]), look_object[5]
            #look_object[3] = look_object[3] - (math.radians(00)) # wrist1
            self.robot_arm.go(look_object, wait=True)                 

            #look_object[5] = look_object[5] + (math.radians(90)) # wrist3
            #self.robot_arm.go(look_object, wait=True)                 


      def look_up_down(self):
            self.clear_octomap()
            print "======== clear_octomap... Please wait...."
            look_up_down = self.robot_arm.get_current_joint_values()
            #print "before: ", look_up_down
                        
            look_up_down[4] = look_up_down[4] + (math.radians(30)) # wrist2
            self.robot_arm.go(look_up_down, wait=True)

            look_up_down[4] = look_up_down[4] - (math.radians(60)) # wrist2
            self.robot_arm.go(look_up_down, wait=True)

            look_up_down[4] = look_up_down[4] + (math.radians(30)) # wrist2
            self.robot_arm.go(look_up_down, wait=True)
                       
      def plan_cartesian_path(self, x_offset, y_offset, scale = 1.0):
            waypoints = []
            ii = 1

            self.wpose = self.robot_arm.get_current_pose().pose
            print "===== robot arm pose: ", self.wpose            
            self.wpose.position.x = (scale * self.wpose.position.x) + x_offset    #-0.10
            
            #print "self.wpose ", ii, ": [",self.wpose.position.x, self.wpose.position.y, self.wpose.position.z,"]"
            waypoints.append(copy.deepcopy(self.wpose))
            ii += 1

            self.wpose.position.y = (scale * self.goal_y) + y_offset  # + 0.05
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

            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_tf_listener)
            rospy.loginfo("Maker messages detected. Starting followers...")

            print "======= pos(meter): ", self.position_list
            print "======= orientation: ", self.orientation_list  

      def go_to_move(self, scale = 1.0):        # 로봇 팔: 마커 밑에 있는 물체 쪽으로 움직이기
            #self.calculed_coke_pose = self.robot_arm.get_current_pose()
            planning_frame = self.robot_arm.get_planning_frame()
            print "========== robot arm plannig frame: \n", planning_frame
            
            self.calculed_coke_pose.position.x = (scale * self.goal_x) + 0.10 # base_link to wrist2 x-offset
            self.calculed_coke_pose.position.y = (scale * self.goal_y) - 0.25
            #self.calculed_coke_pose.position.z = (scale * self.goal_z) + 0.72 - 0.115 # world to base_link z-offset and coke can offset
            self.calculed_coke_pose.position.z = (scale * self.goal_z) + 0.72 + 0.2# world to base_link z-offset real version offset
            self.calculed_coke_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 1.54))

                                   
            print "========== coke_pose goal frame: ", self.calculed_coke_pose
            self.robot_arm.set_pose_target(self.calculed_coke_pose)

            tf_display_position = [self.calculed_coke_pose.position.x, self.calculed_coke_pose.position.y, self.calculed_coke_pose.position.z]      
            tf_display_orientation = [self.calculed_coke_pose.orientation.x, self.calculed_coke_pose.orientation.y, self.calculed_coke_pose.orientation.z, self.calculed_coke_pose.orientation.w]      

            ii = 0
            while ii < 5:
                  ii += 1
                  self.br.sendTransform(
                        tf_display_position,
                        tf_display_orientation,
                        rospy.Time.now(),
                        "goal_wpose",
                        "world")
                  rate.sleep()
            print "============ Press `Enter` to if plan is correct!! ..."
            raw_input()
            self.robot_arm.go(True)

      def ar_tf_listener(self, msg):
            try:
                  self.marker = msg.markers
                  ml = len(self.marker)
                  target_id = 9
                  #self.m_idd = self.marker[0].id  # 임시용

                  for ii in range(0, ml): # 0 <= ii < ml
                        self.m_idd = self.marker[ii].id
                        #print "checked all id: ", self.m_idd
                        if self.m_idd != target_id:
                              pass
                              #target_id_flage = False
                        elif self.m_idd == target_id:
                              target_id_flage = True
                              target_id = self.m_idd
                              target_id_index = ii

                  #print "target id: ", target_id_index, target_id, target_id_flage
                  
                  if target_id_flage == True:
                        self.ar_pose.position.x = self.marker[target_id_index].pose.pose.position.x
                        self.ar_pose.position.y = self.marker[target_id_index].pose.pose.position.y
                        self.ar_pose.position.z = self.marker[target_id_index].pose.pose.position.z
                        self.ar_pose.orientation.x = self.marker[target_id_index].pose.pose.orientation.x
                        self.ar_pose.orientation.y = self.marker[target_id_index].pose.pose.orientation.y
                        self.ar_pose.orientation.z = self.marker[target_id_index].pose.pose.orientation.z
                        self.ar_pose.orientation.w = self.marker[target_id_index].pose.pose.orientation.w
                  
                  self.goal_x = self.ar_pose.position.x
                  self.goal_y = self.ar_pose.position.y
                  self.goal_z = self.ar_pose.position.z

                  self.position_list = [self.goal_x, self.goal_y, self.goal_z]
                  self.orientation_list = [self.ar_pose.orientation.x, self.ar_pose.orientation.y, self.ar_pose.orientation.z, self.ar_pose.orientation.w]
                  (self.goal_roll, self.goal_pitch, self.goal_yaw) = euler_from_quaternion(self.orientation_list) #list form으로 넘겨주어야 함
                  #print "======= pos(meter): ", self.goal_x, self.goal_y, self.goal_z
                  #print "======= rot(rad): ", self.goal_roll, self.goal_pitch, self.goal_yaw                 
                  #print "ar_pos(meter): \n", self.position_list
                  #print "ar_orientation: \n", self.orientation_list    
                  
            except:
                  return


        
if __name__=='__main__':
      tm = TestMove()
      rate = rospy.Rate(10.0)

      tm.move_code()    # go to initial pose
      time.sleep(1)
      ####tm.look_up_down()
      
      #print "============ Press `Enter` to if plan is correct!! ..."
      #raw_input()
      tm.go_to_move() 

      # (down arrow)외않됨? 19.06.12, 17:23 ;; ??? 17:25 ar print 부분 주식 시키니까 됨 ?? 19:44 try exception 없애니까 됨
      tm.look_object() 
        
      
      x_offset = -0.1
      y_offset = 0.05

      cartesian_plan, fraction = tm.plan_cartesian_path(x_offset, y_offset)
      tm.execute_plan(cartesian_plan)
      


      print "======= Press the Enter"
      raw_input()

      tm.move_code()
      
      #rospy.spin()
      #roscpp_shutdown()
