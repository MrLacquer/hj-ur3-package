#!/usr/bin/env python  
import roslib
#roslib.load_manifest('tf_adding')

import rospy
import tf

if __name__ == '__main__':
      rospy.init_node('tf_adding')
      br = tf.TransformBroadcaster()
      rate = rospy.Rate(10.0)
      while not rospy.is_shutdown():
            br.sendTransform(
                  (0.0, 2.0, 0.0),
                  (0.0, 0.0, 0.0, 1.0),
                  rospy.Time.now(),
                  "goal_wpose",
                  "tool0")            
            rate.sleep()