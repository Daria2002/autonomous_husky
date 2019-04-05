#!/usr/bin/env python

import rospy;
from geometry_msgs.msg import Twist

def commander(l_x, l_y, l_z, a_x, a_y, a_z):
    while not rospy.is_shutdown():
        vel = Twist();
        vel.linear.x = l_x;
        vel.linear.y = l_y;
        vel.linear.z = l_z;
        
        vel.angular.x = a_x;
        vel.angular.y = a_y;
        vel.angular.z = a_z;
        pub.publish(vel);
        rospy.sleep(1.0);

if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    rospy.init_node('publisher')

    l_x = 2    
    l_y = 0
    l_z = 0
    
    a_x = 0
    a_y = 0
    a_z = 1

    try:
        while not rospy.is_shutdown():
            commander(l_x, l_y, l_z, a_x, a_y, a_z)
    except rospy.ROSInterruptException:
        pass
