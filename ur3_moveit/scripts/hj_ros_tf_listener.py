#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import sys
import rospy
import copy, math
import tf
from math import pi

from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def tf_listener(parent_tf, child_tf):
      try:  
            listener = tf.TransformListener()      
            (trans, rot) = listener.lookupTransform(parent_tf, child_tf, rospy.Time(0)) 
            # rosrun tf tf_echo /base_link /ar_marker_9 랑 같은 역할
            # transformation: linear transformation, rotation: quaternion
            print "======= trans[x, y, z]: ", trans
            print "======= rotat[x, y, z, w]: ", rot
            
            
            (goal_roll, goal_pitch, goal_yaw) = euler_from_quaternion(rot)
            print "======= rot(rad): ", goal_roll, goal_pitch, goal_yaw
            
            #convert 확인용
            #quat = quaternion_from_euler(goal_roll, goal_pitch, goal_yaw)
            #print quat
                                    
            goal_x = trans[0]
            goal_y = trans[1]
            goal_z = trans[2]                  
      except Exception as ex:            
                  print "======== tf_listener Exception!!"
                  rospy.loginfo(ex)

      return [goal_x, goal_y, goal_z, goal_roll, goal_pitch, goal_yaw]
      
            
      