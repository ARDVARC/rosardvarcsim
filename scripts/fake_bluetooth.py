#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from rosardvarc.msg import BluetoothAzimuthElevation
from geometry_msgs.msg import Pose
from typing import Tuple, Optional
import numpy as np
from scipy.spatial.transform import Rotation


BT_RATE = 30 # Hz, per rgv (so twice this many will be published)
POINTING_STD = 0 # Not really in any units, but bigger means more noise
ROT_UAS2BT = Rotation.from_euler("Y", 90, degrees=True)

uas_pose: Optional[Pose] = None
rgv1_pose: Optional[Pose] = None
rgv2_pose: Optional[Pose] = None


def calculate_angles(uas_pose: Optional[Pose], rgv_pose: Optional[Pose]) -> Tuple[bool, int, int]:
    # If any of the inputs are None, give up
    if uas_pose is None or rgv_pose is None:
        return (False, 0, 0)
    
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
    rot_uas2local = Rotation.from_quat((uas_pose.orientation.x, uas_pose.orientation.y, uas_pose.orientation.z, uas_pose.orientation.w))
    vec_uas2rgv_bt = ROT_UAS2BT.apply(rot_uas2local.apply(vec_uas2rgv_local, inverse=True))
    
    # Calculate and return the two angles
    angle1 = int(np.round(np.rad2deg(np.arcsin(-vec_uas2rgv_bt[1]))))
    angle2 = int(np.round(np.rad2deg(np.arcsin(vec_uas2rgv_bt[2]))))
    return (True, angle1, angle2)


def model_sub_callback(msg: ModelStates):
    global uas_pose, rgv1_pose, rgv2_pose
    try:
        rgv1_pose = msg.pose[msg.name.index("rgv1")] # type: ignore
        rgv2_pose = msg.pose[msg.name.index("rgv2")] # type: ignore
        uas_pose = msg.pose[msg.name.index("ardvarc_drone")] # type: ignore
    except:
        pass
        

rospy.init_node("fake_bluetooth")
model_sub = rospy.Subscriber("gazebo/model_states", ModelStates, model_sub_callback)
bluetooth_pub = rospy.Publisher("bluetooth/az_els", BluetoothAzimuthElevation)
rgv1_bluetooth_msg = BluetoothAzimuthElevation()
rgv1_bluetooth_msg.rgv_id = 1
rgv2_bluetooth_msg = BluetoothAzimuthElevation()
rgv2_bluetooth_msg.rgv_id = 2
rate = rospy.Rate(BT_RATE)

while not rospy.is_shutdown():
    rate.sleep()
    
    # Do math to get the angles
    success1, rgv1_angle1, rgv1_angle2 = calculate_angles(uas_pose, rgv1_pose)
    success2, rgv2_angle1, rgv2_angle2 = calculate_angles(uas_pose, rgv2_pose)
    
    # Retry if they failed
    if not success1 or not success2:
        rospy.loginfo("Some model is not yet loaded, retrying...")
        continue
    
    # Create the messages with the angles
    now = rospy.Time.now()
    rgv1_bluetooth_msg.timestamp = now
    rgv1_bluetooth_msg.azimuth = rgv1_angle1
    rgv1_bluetooth_msg.elevation = rgv1_angle2
    rgv2_bluetooth_msg.timestamp = now
    rgv2_bluetooth_msg.azimuth = rgv2_angle1
    rgv2_bluetooth_msg.elevation = rgv2_angle2
    
    # Publish the messages
    bluetooth_pub.publish(rgv1_bluetooth_msg)
    bluetooth_pub.publish(rgv2_bluetooth_msg)