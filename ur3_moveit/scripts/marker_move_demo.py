#!/usr/bin/env python

import sys
import rospy
import copy, math
from math import pi

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose
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
            self.goal_x = 0
            self.goal_y = 0
            self.goal_z = 0

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
            
            #self.clear_octomap()
            

      def ar_callback(self, msg):
            #marker = msg.markers[1]
            self.marker = msg.markers
            ml = len(self.marker)
            m_id = self.marker[0].id
            print "marker length: ", len(self.marker)
            print "marker id: ", m_id
            m_idd = []
            '''
            for ii in range(0, 2):
                  print(ii)
                  m_idd[ii] = marker[ii].id
                  #m_id[ii] = marker[ii].id
                  print(m_idd[ii])
            '''
            
            pos_x = self.marker[0].pose.pose.position.x
            pos_y = self.marker[0].pose.pose.position.y
            pos_z = self.marker[0].pose.pose.position.z

            dist = math.sqrt(
                  (pos_x*pos_x) + (pos_y * pos_y)
                  )

            #ori_x = marker.pose.pose.orientation.x
            #ori_y = marker.pose.pose.orientation.y
            #ori_z = marker.pose.pose.orientation.z
            #ori_w = marker.pose.pose.orientation.w
            #print(m_id)
            print "==========="
            print 'id: ', m_id
            print 'pos: ', pos_x, pos_y, pos_z
            
            #print('ori: ', ori_x, ori_y, ori_z, ori_w)

            self.goal_x = pos_x - 1.0
            self.goal_y = pos_y - pos_y
            self.goal_z = pos_z - pos_z

      def move_code(self):
            robot_state = self.robot_arm.get_current_pose()

            print "====== robot pose: \n", 
            print robot_state.pose.position
            
            
            #marker_joint_goal = self.robot_arm.get_current_joint_values()
            marker_joint_goal = [0.07100913858593216, -1.8767615298285376, 2.0393206555899503, -1.8313959190971882, -0.6278395875738125, 1.6918219826764682]
            self.robot_arm.go(marker_joint_goal, wait=True)
            print "====== robot joint value: \n"
            print marker_joint_goal

            self.robot_arm.go(marker_joint_goal, wait=True)            
            self.robot_arm.go(True)
            print "look at the markers"            

            pose_goal = Pose()
            pose_goal.orientation.w = 0.0
            pose_goal.position.x = 0.4 
            pose_goal.position.y = -0.4 
            pose_goal.position.z = 0.7

            planning_frame = self.robot_arm.get_planning_frame()
            print "========== plannig frame: ", planning_frame
            #self.robot_arm.set_pose_target(pose_goal)
            #7self.robot_arm.go(True)

      def plan_cartesian_path(self, scale=1):
            
            waypoints = []

            wpose = self.robot_arm.get_current_pose().pose
            wpose.position.z -= scale * 0.2  
            wpose.position.x -= scale * 0.2  
            

            waypoints.append(copy.deepcopy(wpose))

            (plan, fraction) = self.robot_arm.compute_cartesian_path(
                                                waypoints,   # waypoints to follow
                                                0.01,        # eef_step
                                                0.0)         # jump_threshold

            return plan, fraction

      def execute_plan(self, plan):
            group = self.robot_arm
            group.execute(plan, wait=True)

      def print_ar_pose(self):
            rospy.loginfo("Waiting for ar_pose_marker topic...")
            rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback)
            rospy.loginfo("Maker messages detected. Starting followers...")


        
if __name__=='__main__':
      tm = TestMove()
      tm.__init__()
      tm.move_code()

      tm.print_ar_pose()
    


    #print "============ Press `Enter` to plan and display a Cartesian path ..."
    #raw_input()
      cartesian_plan, fraction = tm.plan_cartesian_path()

    #print "============ Press `Enter` to execute a saved path ..."
    #raw_input()
      tm.execute_plan(cartesian_plan)

      rospy.spin()
      roscpp_shutdown()