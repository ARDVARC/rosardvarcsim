#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import Twist

if len(sys.argv) != 2:
    print("Specify name of RGV model as command line argument")
    exit(-1)

rgv_name = sys.argv[1]
print(f"rgv_controller_{rgv_name}")
rospy.init_node(f"rgv_controller_{rgv_name}")
    
twist_pub = rospy.Publisher("gazebo/set_model_state", Twist, queue_size=1)
rate = rospy.Rate(1)

msg = Twist()
msg.linear.x = 1
msg.angular.z = 1

while not rospy.is_shutdown():
    twist_pub.publish(msg)
    msg.angular.z = -msg.angular.z
    rate.sleep()