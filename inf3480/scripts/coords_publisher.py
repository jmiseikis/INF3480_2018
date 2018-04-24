#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

def talker():
    #pub = rospy.Publisher('/coords', Point, queue_size=10)
    pub = rospy.Publisher('coords', Point, queue_size=10) # <- What would be the difference when using launch file?
    rospy.init_node('coords_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    position = Point()
    position.x = 10
    position.y = 5
    position.z = 1

    while not rospy.is_shutdown():
        rospy.loginfo(position) # This is only for debugging
        
        pub.publish(position)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass