#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from rosardvarc.msg import EstimatedRgvState
from geometry_msgs.msg import Pose, Point, PoseStamped
from typing import Tuple
import numpy as np
from scipy.spatial.transform import Rotation


FAKE_MAVROS_RATE = 30 # Hz

rospy.init_node("fake_mavros")
rospy.wait_for_service("gazebo/get_model_state")
model_service = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
uas_model_request = GetModelStateRequest()
uas_model_request.model_name = "ardvarc_drone"
estimate_pub = rospy.Publisher("fake_mavros/local_position/pose", PoseStamped)
rate = rospy.Rate(FAKE_MAVROS_RATE)

msg = PoseStamped()

while not rospy.is_shutdown():
    rate.sleep()
    
    # Query model poses
    uas_response: GetModelStateResponse = model_service(uas_model_request)
    if not uas_response.success:
        rospy.logwarn(uas_response.status_message)
        continue
    
    # Create the message
    now = rospy.Time.now()
    msg.header.stamp = now
    msg.pose = uas_response.pose
    
    # Publish the message
    estimate_pub.publish(msg)