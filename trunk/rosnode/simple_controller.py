#!/usr/bin/env python
import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from geometry_msgs.msg import Point

# Add this node to ROS begineer tutorials
def talker():
    pub = rospy.Publisher('servocommand', Point)
    rospy.init_node('OctoroachCommand')
    while not rospy.is_shutdown():
        commandL = input('Enter a Left speed between 0.0 - 255.0:')
        commandR = input('Enter a Right speed between 0.0 - 255.0:')
    	pub.publish(Point(commandL,commandR,0.0))
       # rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
