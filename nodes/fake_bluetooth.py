#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from rosardvarc.msg import BluetoothAzimuthElevation
from geometry_msgs.msg import Pose
from typing import Tuple
import numpy as np
from scipy.spatial.transform import Rotation


BT_RATE = 30 # Hz, per rgv (so twice this many will be published)
POINTING_STD = 0.1 # Not really in any units, but bigger means more noise
DCM_UAS2BT = Rotation.from_euler("xyz", (0, 0, 0)).as_matrix()


def calculate_angles(uas_pose: Pose, rgv_pose: Pose) -> Tuple[int, int]:
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
    dcm_local2uas = Rotation.from_quat((uas_pose.orientation.w, uas_pose.orientation.x, uas_pose.orientation.y, uas_pose.orientation.z)).inv().as_matrix()
    dcm_local2bt = DCM_UAS2BT @ dcm_local2uas
    vec_uas2rgv_bt = dcm_local2bt @ vec_uas2rgv_local
    
    # Calculate and return the two angles
    az = int(np.round(np.rad2deg(np.arctan2(vec_uas2rgv_bt[1],vec_uas2rgv_bt[0]))))
    el = int(np.round(np.rad2deg(np.arcsin(vec_uas2rgv_bt[2]))))
    return (az, el)


rospy.init_node("fake_bluetooth")
rospy.wait_for_service("gazebo/get_model_state")
model_service = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
rgv1_model_request = GetModelStateRequest()
rgv1_model_request.model_name = "rgv1"
rgv2_model_request = GetModelStateRequest()
rgv2_model_request.model_name = "rgv2"
uas_model_request = GetModelStateRequest()
uas_model_request.model_name = "ardvarc_drone"
bluetooth_pub = rospy.Publisher("bluetooth/az_els", BluetoothAzimuthElevation)
rgv1_bluetooth_msg = BluetoothAzimuthElevation()
rgv1_bluetooth_msg.rgv_id = 1
rgv2_bluetooth_msg = BluetoothAzimuthElevation()
rgv2_bluetooth_msg.rgv_id = 2
rate = rospy.Rate(BT_RATE)

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
    
    # Do math to get the angles
    rgv1_az, rgv1_el = calculate_angles(uas_response.pose, rgv1_response.pose)
    rgv2_az, rgv2_el = calculate_angles(uas_response.pose, rgv2_response.pose)
    
    # Create the messages with the angles
    now = rospy.Time.now()
    rgv1_bluetooth_msg.timestamp = now
    rgv1_bluetooth_msg.azimuth = rgv1_az
    rgv1_bluetooth_msg.elevation = rgv1_el
    rgv2_bluetooth_msg.timestamp = now
    rgv2_bluetooth_msg.azimuth = rgv2_az
    rgv2_bluetooth_msg.elevation = rgv2_el
    
    # Publish the messages
    bluetooth_pub.publish(rgv1_bluetooth_msg)
    bluetooth_pub.publish(rgv2_bluetooth_msg)