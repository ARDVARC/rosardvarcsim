#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from rosardvarc.msg import EstimatedRgvState
from geometry_msgs.msg import Pose, Point
from typing import Tuple, Optional
import numpy as np


ESTIMATION_RATE = 5 # Hz
ESTIMATED_POSITION_STD = 0.01 # [m]
DISTANCE_ERROR_MULTIPLIER = 0.01 # [1/m]
MOVING_DISTANCE_THRESHOLD = 0.5 / ESTIMATION_RATE # [m]

uas_pose: Optional[Pose] = None
rgv1_pose: Optional[Pose] = None
rgv2_pose: Optional[Pose] = None


def pose_to_position_vector(pose: Optional[Pose]) -> Tuple[bool, np.ndarray]:
    if pose is None:
        return (False, np.zeros(3))
    return (True, np.array([pose.position.x, pose.position.y, pose.position.z]))


def model_sub_callback(msg: ModelStates):
    global uas_pose, rgv1_pose, rgv2_pose
    try:
        rgv1_pose = msg.pose[msg.name.index("rgv1")] # type: ignore
        rgv2_pose = msg.pose[msg.name.index("rgv2")] # type: ignore
        uas_pose = msg.pose[msg.name.index("ardvarc_drone")] # type: ignore
    except:
        pass


rospy.init_node("fake_estimator")
model_sub = rospy.Subscriber("gazebo/model_states", ModelStates, model_sub_callback)
estimate_pub = rospy.Publisher("estimation/estimated_rgv_states", EstimatedRgvState)
rgv_estimate_msg = EstimatedRgvState()
rgv_estimate_msg.rgv1_confidence = 1
rgv_estimate_msg.rgv2_confidence = 1
rate = rospy.Rate(ESTIMATION_RATE)
prev_rgv1_position_estimate = np.array([0,0,0])
prev_rgv2_position_estimate = np.array([0,0,0])

while not rospy.is_shutdown():
    rate.sleep()
    
    # Try to convert to position vectors
    success1, rgv1_position = pose_to_position_vector(rgv1_pose)
    success2, rgv2_position = pose_to_position_vector(rgv2_pose)
    success3, uas_position = pose_to_position_vector(uas_pose)
    
    # Retry if they failed
    if not success1 or not success2 or not success3:
        rospy.loginfo("Some model is not yet loaded, retrying...")
        continue
    
    # Do math
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