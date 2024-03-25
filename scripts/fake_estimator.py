#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from rosardvarc.msg import EstimatedRgvState
from geometry_msgs.msg import Pose, Point
from typing import Tuple
import numpy as np
from scipy.spatial.transform import Rotation


ESTIMATION_RATE = 5 # Hz
ESTIMATED_POSITION_STD = 0.01 # [m]
DISTANCE_ERROR_MULTIPLIER = 0.01 # [1/m]
MOVING_DISTANCE_THRESHOLD = 0.5 / ESTIMATION_RATE # [m]


def point_to_vector(point: Point) -> np.ndarray:
    return np.array([point.x, point.y, point.z])


rospy.init_node("fake_estimator")
rospy.wait_for_service("gazebo/get_model_state")
model_service = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
rgv1_model_request = GetModelStateRequest()
rgv1_model_request.model_name = "rgv1"
rgv2_model_request = GetModelStateRequest()
rgv2_model_request.model_name = "rgv2"
uas_model_request = GetModelStateRequest()
uas_model_request.model_name = "ardvarc_drone"
estimate_pub = rospy.Publisher("estimation/estimated_rgv_states", EstimatedRgvState)
rgv_estimate_msg = EstimatedRgvState()
rgv_estimate_msg.rgv1_confidence = 1
rgv_estimate_msg.rgv2_confidence = 1
rate = rospy.Rate(ESTIMATION_RATE)
prev_rgv1_position_estimate = np.array([0,0,0])
prev_rgv2_position_estimate = np.array([0,0,0])

while not rospy.is_shutdown():
    rate.sleep()
    
    # Query model poses
    rgv1_response: GetModelStateResponse = model_service(rgv1_model_request)
    if not rgv1_response.success:
        rospy.logwarn(rgv1_response.status_message)
        continue
    rgv2_response: GetModelStateResponse = model_service(rgv2_model_request)
    if not rgv2_response.success:
        rospy.logwarn(rgv2_response.status_message)
        continue
    uas_response: GetModelStateResponse = model_service(uas_model_request)
    if not uas_response.success:
        rospy.logwarn(uas_response.status_message)
        continue
    
    # Do math
    rgv1_position = point_to_vector(rgv1_response.pose.position)
    rgv2_position = point_to_vector(rgv2_response.pose.position)
    uas_position = point_to_vector(uas_response.pose.position)
    distance_to_rgv1 = np.linalg.norm(uas_position-rgv1_position)
    distance_to_rgv2 = np.linalg.norm(uas_position-rgv2_position)
    rgv1_position_estimate = rgv1_position + np.random.normal(0, ESTIMATED_POSITION_STD, 3) * distance_to_rgv1 * DISTANCE_ERROR_MULTIPLIER
    rgv2_position_estimate = rgv2_position + np.random.normal(0, ESTIMATED_POSITION_STD, 3) * distance_to_rgv2 * DISTANCE_ERROR_MULTIPLIER
    rgv1_moving = float(np.linalg.norm(rgv1_position_estimate-prev_rgv1_position_estimate)) > MOVING_DISTANCE_THRESHOLD
    rgv2_moving = float(np.linalg.norm(rgv2_position_estimate-prev_rgv2_position_estimate)) > MOVING_DISTANCE_THRESHOLD
    prev_rgv1_position_estimate = rgv1_position_estimate
    prev_rgv2_position_estimate = rgv2_position_estimate
    
    # Create the message
    now = rospy.Time.now()
    rgv_estimate_msg.timestamp = now
    rgv_estimate_msg.rgv1_moving = rgv1_moving
    rgv_estimate_msg.rgv1_position_local = rgv1_position_estimate
    rgv_estimate_msg.rgv2_moving = rgv2_moving
    rgv_estimate_msg.rgv2_position_local = rgv2_position_estimate
    
    # Publish the message
    estimate_pub.publish(rgv_estimate_msg)