#!/usr/bin/env python

import os, sys, rospy, tf, actionlib
import copy, math
from math import pi
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from control_msgs.msg import PointHeadAction, PointHeadGoal

from ar_track_alvar_msgs.msg import AlvarMarkers



class My_Test():
      def __init__(self):
            self.goal_x = 0
            self.goal_y = 0
            self.goal_z = 0
            

            rospy.loginfo("Waiting for ar_pose_marker topic...")
            rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback)
            rospy.loginfo("Maker messages detected. Starting followers...")


      def ar_callback(self, msg):
            #marker = msg.markers[1]
            self.marker = msg.markers
            ml = len(self.marker)
            m_id = self.marker[1].id
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
            
            pos_x = self.marker[1].pose.pose.position.x
            pos_y = self.marker[1].pose.pose.position.y
            pos_z = self.marker[1].pose.pose.position.z

            dist = math.sqrt(
                  (pos_x*pos_x) + (pos_y * pos_y)
                  )

            #ori_x = marker.pose.pose.orientation.x
            #ori_y = marker.pose.pose.orientation.y
            #ori_z = marker.pose.pose.orientation.z
            #ori_w = marker.pose.pose.orientation.w
            #print(m_id)
            print 'id: ', m_id
            print 'pos: ', pos_x, pos_y, pos_z
            print 'dis: ', dist
            #print('ori: ', ori_x, ori_y, ori_z, ori_w)

            self.goal_x = pos_x - 1.0
            self.goal_y = pos_y - pos_y
            self.goal_z = pos_z - pos_z
            '''
            move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            move_base.wait_for_server()
            goal = MoveBaseGoal()

            goal.target_pose.header.frame_id = "base_link"
            goal.target_pose.pose.position.x = self.goal_x
            goal.target_pose.pose.position.y = self.goal_y
            goal.target_pose.pose.position.z = self.goal_z
            orient = Quaternion(*quaternion_from_euler(0,0,0))

            goal.target_pose.pose.orientation = orient
            move_base.send_goal(goal)
            move_base.wait_for_result()
            '''
            

      def goto_my(self):
            print 'goal_x: ', self.goal_x

            move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            move_base.wait_for_server()
            goal = MoveBaseGoal()

            goal.target_pose.header.frame_id = "base_link"
            goal.target_pose.pose.position.x = self.goal_x
            goal.target_pose.pose.position.y = self.goal_y
            goal.target_pose.pose.position.z = self.goal_z
            orient = Quaternion(*quaternion_from_euler(0,0,0))

            goal.target_pose.pose.orientation = orient
            move_base.send_goal(goal)
            move_base.wait_for_result()

            print "finish!"


if __name__=='__main__':

      rospy.init_node('ar_Sub')

      mt = My_Test()
      '''
      rospy.loginfo("Waiting for ar_pose_marker topic...")
      rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

      rospy.Subscriber('ar_pose_marker', AlvarMarkers, mt.ar_callback)
      rospy.loginfo("Maker messages detected. Starting followers...")
      '''
      mt.goto_my()
      #ar_pose_sub = rospy.Subscriber('ar_pose_marker', ARMarker, ar_callback)
      rospy.spin()
