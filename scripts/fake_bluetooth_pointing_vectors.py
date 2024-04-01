#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from rosardvarc.msg import UasToRgvDirectionVectorUasFrame
from geometry_msgs.msg import Pose
from typing import Tuple
import numpy as np
from scipy.spatial.transform import Rotation


BT_RATE = 40 # Hz, total (not per rgv)
POINTING_STD = 0.1 # Not really in any units, but bigger means more noise


def calculate_vector(uas_pose: Pose, rgv_pose: Pose) -> np.ndarray:
    # Calculate inertial pointing vector
    vec_uas2rgv_local = np.array([
        uas_pose.position.x - rgv_pose.position.x,
        uas_pose.position.y - rgv_pose.position.y,
        uas_pose.position.z - rgv_pose.position.z,
    ])
    vec_uas2rgv_local /= np.linalg.norm(vec_uas2rgv_local)
    
    # Add some noise to the pointing vector
    vec_noise_local = np.random.normal(0, POINTING_STD, 3)
    vec_uas2rgv_local += vec_noise_local
    vec_uas2rgv_local /= np.linalg.norm(vec_uas2rgv_local)
    
    # Convert it into the bluetooth sensor frame
    uas2local = Rotation.from_quat((uas_pose.orientation.x, uas_pose.orientation.y, uas_pose.orientation.z, uas_pose.orientation.w))
    vec_uas2rgv_bt = uas2local.apply(vec_uas2rgv_local, inverse=True)
    
    return vec_uas2rgv_bt


rospy.init_node("fake_bluetooth_pointing_vectors")
rospy.wait_for_service("gazebo/get_model_state")
model_service = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
rgv1_model_request = GetModelStateRequest()
rgv1_model_request.model_name = "rgv1"
rgv2_model_request = GetModelStateRequest()
rgv2_model_request.model_name = "rgv2"
uas_model_request = GetModelStateRequest()
uas_model_request.model_name = "ardvarc_drone"
vector_pub = rospy.Publisher("estimation/direction_vectors_uas", UasToRgvDirectionVectorUasFrame, queue_size=1)
rgv1_bluetooth_msg = UasToRgvDirectionVectorUasFrame()
rgv1_bluetooth_msg.rgv_id = 1
rgv2_bluetooth_msg = UasToRgvDirectionVectorUasFrame()
rgv2_bluetooth_msg.rgv_id = 2
next_rgv = 1
rate = rospy.Rate(BT_RATE)

while not rospy.is_shutdown():
    rate.sleep()
    
    # Query model poses
    if next_rgv == 1:
        rgv_response: GetModelStateResponse = model_service(rgv1_model_request)
        msg = rgv1_bluetooth_msg
        next_rgv = 2
    else:
        rgv_response: GetModelStateResponse = model_service(rgv2_model_request)
        msg = rgv2_bluetooth_msg
        next_rgv = 1
    if not rgv_response.success:
        rospy.logwarn(rgv_response.status_message)
        continue
    
    uas_response: GetModelStateResponse = model_service(uas_model_request)
    if not uas_response.success:
        rospy.logwarn(uas_response.status_message)
        continue
    
    # Do math to get the vector
    vec = calculate_vector(uas_response.pose, rgv_response.pose)
    
    # Create the message
    now = rospy.Time.now()
    msg.timestamp = now
    msg.direction = vec
    msg.measurement_source = 11
    
    # Publish the message
    vector_pub.publish(msg)