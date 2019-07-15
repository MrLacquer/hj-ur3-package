#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import sys, time
import rospy
import copy, math
import tf
import moveit_msgs.msg
from math import pi

import serial

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import *
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, CollisionObject
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

from ar_track_alvar_msgs.msg import AlvarMarkers
from std_srvs.srv import Empty

# joint name, rostopic ehco /joint_states
# look_object = self.robot_arm.get_current_joint_values()
#      look_object[0] = shoulder_pan_joint
#                 [1] = shoulder_lift_joint
#                 [2] = elbow_joint
#                 [3] = wrist_1_joint
#                 [4] = wrist_2_joint
#                 [5] = wrist_3_joint            

GROUP_NAME_ARM = "manipulator"
FIXED_FRAME = 'world'
#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"

class STM_serial():
      def __init__(self):
            self.rx_data = 0
            self.tx_data = 0
            self.PORT = '/dev/ttyUSB0'
            self.BaudRate = 115200
        
            #print 'serial', serial.__version__
            self.ARD= serial.Serial(self.PORT, self.BaudRate)

            self.ii = 0 

        

      def send_signal(self, hj_order):
            #A = [0x01, 0x02, 0x03, 0x04, 0x05]
            start = '2'
            stop = '3'

            if hj_order == 1:                
                  print "start list: ", start
                  self.ARD.write(start)    # list만 가능
            elif hj_order == 0:
                  print "stop list: ", stop
                  self.ARD.write(stop)    # list만 가능


class TestMove():

      def __init__(self):
            roscpp_initialize(sys.argv)        
            rospy.init_node('ur3_move',anonymous=True)

            rospy.loginfo("Waiting for ar_pose_marker topic...")
            rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_tf_listener)
            rospy.loginfo("Maker messages detected. Starting followers...")

            display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

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

            self.target_ar_id = 9

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

            self.display_trajectory_publisher = display_trajectory_publisher

            rospy.sleep(2)
            # Allow replanning to increase the odds of a solution
            self.robot_arm.allow_replanning(True)                 
            
            self.clear_octomap()
            

      def move_code(self):
            #self.clear_octomap()
            planning_frame = self.robot_arm.get_planning_frame()
            print "========== plannig frame: ", planning_frame

            self.wpose = self.robot_arm.get_current_pose()
            print"====== current pose : ", self.wpose                        

            marker_joint_goal = [-0.535054565144069, -2.009213503260451, 1.8350906250920112, -0.7794355413099039, -0.7980899690645948, 0.7782740454087982]
            print "INIT POSE: ", self.robot_arm.get_current_pose().pose.position
            self.robot_arm.go(marker_joint_goal, wait=True)

      def move_moveit_setting_pose(self, pose_name):
            if pose_name == "home":
                  self.robot_arm.set_named_target("home")
            elif pose_name == "zeros":
                  self.robot_arm.set_named_target("zeros")
            elif pose_name == "table":
                  self.robot_arm.set_named_target("table")
                  
            #print "Press the Enter"
            #raw_input()
            self.robot_arm.go(wait=True)

      def look_object_one_joint(self, joint_num, rad):
            
            look_object = self.robot_arm.get_current_joint_values()
            
            ## wrist3 현재 상태가 0도 인지, 360도인지에 따라 90도의 움직임을 -로 할지, + 할지 고려함
            print "wrist3 joint value(deg, rad): ", math.degrees(look_object[joint_num]), look_object[joint_num]
            #look_object[5] = math.radians(90)
            look_object[joint_num] = rad
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
      
      def display_trajectory(self, plan):
            display_trajectory_publisher = self.display_trajectory_publisher

            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot_cmd.get_current_state()
            display_trajectory.trajectory.append(plan)
            
            display_trajectory_publisher.publish(display_trajectory);
                       
      def plan_cartesian_path(self, x_offset, y_offset, z_offset, scale = 1.0):
            waypoints = []
            ii = 1

            self.wpose = self.robot_arm.get_current_pose().pose
            #print "===== robot arm pose: ", self.wpose            
            self.wpose.position.x = (scale * self.wpose.position.x) + x_offset    #-0.10
            
            #print "self.wpose ", ii, ": [",self.wpose.position.x, self.wpose.position.y, self.wpose.position.z,"]"
            waypoints.append(copy.deepcopy(self.wpose))
            ii += 1

            self.wpose.position.y = (scale * self.wpose.position.y) + y_offset  # + 0.05
            waypoints.append(copy.deepcopy(self.wpose))
            ii += 1

            self.wpose.position.z = (scale * self.wpose.position.z) + z_offset  # 
            waypoints.append(copy.deepcopy(self.wpose))
            ii += 1

            #print "waypoints:", waypoints
            (plan, fraction) = self.robot_arm.compute_cartesian_path(waypoints, 0.01, 0.0)

            return plan, fraction

      def plan_cartesian_x(self, x_offset, scale = 1.0):
            waypoints = []

            self.wpose = self.robot_arm.get_current_pose().pose
            #print "===== robot arm pose: ", self.wpose            
            self.wpose.position.x = (scale * self.wpose.position.x) + x_offset    #-0.10            
            waypoints.append(copy.deepcopy(self.wpose))            
            
            (plan, fraction) = self.robot_arm.compute_cartesian_path(waypoints, 0.01, 0.0)

            return plan, fraction

      def plan_cartesian_y(self, y_offset, scale = 1.0):
            waypoints = []

            self.wpose = self.robot_arm.get_current_pose().pose
            #print "===== robot arm pose: ", self.wpose            
            self.wpose.position.y = (scale * self.wpose.position.y) + y_offset    #-0.10            
            waypoints.append(copy.deepcopy(self.wpose))            
            
            (plan, fraction) = self.robot_arm.compute_cartesian_path(waypoints, 0.01, 0.0)

            return plan, fraction
      
      def plan_cartesian_z(self, z_offset, scale = 1.0):
            waypoints = []           

            self.wpose = self.robot_arm.get_current_pose().pose
            #print "===== robot arm pose: ", self.wpose            
            self.wpose.position.z = (scale * self.wpose.position.z) + z_offset    #-0.10            
            waypoints.append(copy.deepcopy(self.wpose))            
            
            (plan, fraction) = self.robot_arm.compute_cartesian_path(waypoints, 0.01, 0.0)

            return plan, fraction

      def execute_plan(self, plan):
            group = self.robot_arm
            group.execute(plan, wait=True)

      def ar_pose_subscriber(self):
            rospy.loginfo("Waiting for ar_pose_marker topic...")
            rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_tf_listener)
            rospy.loginfo("Maker messages detected. Starting followers...")

            #print "======= pos(meter): ", self.position_list
            #print "======= orientation: ", self.orientation_list  

      def go_to_move(self, scale = 1.0):        # 로봇 팔: 마커 밑에 있는 물체 쪽으로 움직이기
            #self.calculed_coke_pose = self.robot_arm.get_current_pose()
            planning_frame = self.robot_arm.get_planning_frame()
            coke_offset = [0, -0.35, -0.1] #x y z
            # gazebo_coke_offset = [0, -0.2875, -0.23] gazebo 에서의 마커와 코크 캔의 offset, 바로 명령하면 해를 못 품.
            # linear offset = abs([0, 0.0625, 0.13])
            robot_base_offset = 0.873
            base_wrist2_offset = 0.1      #for avoiding link contact error
            
            if self.target_ar_id == 9:
                  print ">> robot arm plannig frame: \n", planning_frame
                  print ">> move mode id: ", self.target_ar_id
                  
                  self.calculed_coke_pose.position.x = (scale * self.goal_x) # base_link to wrist2 x-offset
                  self.calculed_coke_pose.position.y = (scale * self.goal_y) + coke_offset[1]
                  #self.calculed_coke_pose.position.z = (scale * self.goal_z) + 0.72 + coke_offset# world to base_link z-offset
                  self.calculed_coke_pose.position.z = (scale * self.goal_z) + robot_base_offset # world to base_link z-offset and coke can offset
                  self.calculed_coke_pose.orientation = Quaternion(*quaternion_from_euler(3.14, 0, 1.57))

                  print "========== coke_pose goal frame: ", self.calculed_coke_pose
                  self.robot_arm.set_pose_target(self.calculed_coke_pose)


            elif self.target_ar_id == 10:
                  print ">> robot arm plannig frame: \n", planning_frame
                  print ">> move mode id: ", self.target_ar_id
                  
                  self.calculed_coke_pose.position.x = (scale * self.goal_x) + coke_offset[1]
                  self.calculed_coke_pose.position.y = (scale * self.goal_y) + 0
                  self.calculed_coke_pose.position.z = (scale * self.goal_z) + robot_base_offset # world to base_link z-offset and coke can offset
                  self.calculed_coke_pose.orientation = Quaternion(*quaternion_from_euler(3.14, 0, 0))
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

            ## ## ## show how to move on the Rviz
            coke_waypoints = []
            coke_waypoints.append(copy.deepcopy(self.calculed_coke_pose))
            (coke_plan, coke_fraction) = self.robot_arm.compute_cartesian_path(coke_waypoints, 0.01, 0.0)
            self.display_trajectory(coke_plan)
            ## ## ##

            print "============ Press `Enter` to if plan is correct!! ..."
            raw_input()
            self.robot_arm.go(True)
      
      def go_to_desired_coordinate(self, pose_x, pose_y, pose_z, roll, pitch, yaw):
            calculed_ar_id_10 = Pose()
            #desired_goal_pose = [0.171, -0.113, 1.039]
            #desired_goal_euler = [3.14, 0.17, 0]
            desired_goal_pose = [pose_x, pose_y, pose_z]
            desired_goal_euler = [roll, pitch, yaw]

            Cplanning_frame = self.robot_arm.get_planning_frame()
            print ">> current planning frame: \n", Cplanning_frame
            
            calculed_ar_id_10.position.x = desired_goal_pose[0] + 0.1
            calculed_ar_id_10.position.y = desired_goal_pose[1]
            calculed_ar_id_10.position.z = desired_goal_pose[2]
            calculed_ar_id_10.orientation = Quaternion(*quaternion_from_euler(desired_goal_euler[0], desired_goal_euler[1], desired_goal_euler[2]))

            print ">>> ar id 10 goal frame: ", desired_goal_pose
            self.robot_arm.set_pose_target(calculed_ar_id_10)

            tf_display_position = [calculed_ar_id_10.position.x, calculed_ar_id_10.position.y, calculed_ar_id_10.position.z]
            tf_display_orientation = [calculed_ar_id_10.orientation.x, calculed_ar_id_10.orientation.y, calculed_ar_id_10.orientation.z, calculed_ar_id_10.orientation.w]

            jj = 0
            while jj < 5:
                  jj += 1
                  self.br.sendTransform(
                        tf_display_position,
                        tf_display_orientation,
                        rospy.Time.now(),
                        "goal_wpose",
                        "world")
                  rate.sleep()

            ## ## ## show how to move on the Rviz
            ar_id_10_waypoints = []
            ar_id_10_waypoints.append(copy.deepcopy(calculed_ar_id_10))
            (ar_id_10_plan, ar_id_10_fraction) = self.robot_arm.compute_cartesian_path(ar_id_10_waypoints, 0.01, 0.0)
            self.display_trajectory(ar_id_10_plan)
            ## ## ##

            print "============ Press `Enter` to if plan is correct!! ..."
            raw_input()
            self.robot_arm.go(True)


      def ar_tf_listener(self, msg):
            try:
                  self.marker = msg.markers
                  
                  ml = len(self.marker)
                  target_start_point_id = self.target_ar_id
                  #target_id = target_ar_id
                  #self.m_idd = self.marker[0].id  # 임시용

                  for ii in range(0, ml): # 0 <= ii < ml
                        self.m_idd = self.marker[ii].id
                        #print "checked all id: ", self.m_idd
                        if self.m_idd != target_start_point_id:
                              pass
                              #target_id_flage = False
                        elif self.m_idd == target_start_point_id:
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
      shoulder_pan_joint = 0
      shoulder_lift_joint = 1
      elbow_joint = 2
      wrist_1_joint = 3
      wrist_2_joint = 4
      wrist_3_joint = 5
      
      tm = TestMove()
      stm_ser = STM_serial()

      rate = rospy.Rate(10.0)
      tm.move_code()    # go to initial pose
      time.sleep(1)
      #tm.look_up_down()
      
      #print "============ Press `Enter` to if plan is correct!! ..."
      #raw_input()
      tm.go_to_move() 
      
      #tm.look_object_one_joint(wrist_3_joint, 0) 
      #x_offset = -0.1      y_offset = -0.0625      z_offset = -0.08
      linear_path = [-0.0001, -0.0625, -0.08]
      print ">> Linear path planning 01 - pick the object"
      cartesian_plan, fraction = tm.plan_cartesian_path(linear_path[0], linear_path[1], linear_path[2]) # 모든 매개변수에 값을 줘야 함
      tm.display_trajectory(cartesian_plan)
      time.sleep(0.5)      
      #print "Press the Enter"
      #raw_input()
      tm.execute_plan(cartesian_plan)


      ###### serial test ######
      stm_ser.send_signal(1)
      time.sleep(5)
      #########################

      #x_offset = -0.1      y_offset = -0.0625      z_offset = -0.08
      linear_path = [+0.2, -0.2, +0.2]
      print ">> Linear path planning 02-z"
      cartesian_plan, fraction = tm.plan_cartesian_z(linear_path[2]) # 모든 매개변수에 값을 줘야 함
      tm.display_trajectory(cartesian_plan)
      time.sleep(2)      
      #print "Press the Enter"
      #raw_input()
      tm.execute_plan(cartesian_plan)


      ''' 안쓸 꺼 같음 
      print ">> Linear path planning 02-y"
      cartesian_plan, fraction = tm.plan_cartesian_y(linear_path[1]) # 모든 매개변수에 값을 줘야 함
      tm.display_trajectory(cartesian_plan)
      time.sleep(0.5)      
      #print "Press the Enter"
      #raw_input()
      #tm.execute_plan(cartesian_plan) 
      '''

      print ">> go to setting pose: home"      
      setting_pose = "home"
      tm.move_moveit_setting_pose(setting_pose)

      ### test code. for go to ar id 10
      ar_id10_goal_pose = [0.171, -0.113, 1.039]
      ar_id10_goal_euler = [3.14, 0.17, 0]
      
      tm.go_to_desired_coordinate(ar_id10_goal_pose[0], ar_id10_goal_pose[1], ar_id10_goal_pose[2],
                                    ar_id10_goal_euler[0], ar_id10_goal_euler[1], ar_id10_goal_euler[2])
      ####

      tm.target_ar_id = 10
      tm.ar_pose_subscriber()
      time.sleep(0.5)

      tm.go_to_move()

      #x_offset = -0.1      y_offset = -0.0625      z_offset = -0.08
      linear_path = [-0.0625, 0.0001, -0.08]
      print ">> Linear path planning 03-drop the object"
      cartesian_plan, fraction = tm.plan_cartesian_path(linear_path[0], linear_path[1], linear_path[2]) # 모든 매개변수에 값을 줘야 함
      tm.display_trajectory(cartesian_plan)
      time.sleep(0.5)      
      tm.execute_plan(cartesian_plan)
      ### ### ###
      
      ###### serial test ######
      stm_ser.send_signal(0)
      time.sleep(5)
      #########################



      print "Go to Init pose, Press the Enter"
      raw_input()
      tm.move_code()    # go to initial pose
      
      #rospy.spin()
      #roscpp_shutdown()
