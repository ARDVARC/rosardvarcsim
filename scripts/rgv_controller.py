#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Quaternion
import numpy as nd


UPDATE_RATE = 30
SPEED = 0.5

DX = SPEED/UPDATE_RATE

def fillQuaternionMessage(msg: Quaternion, values: nd.ndarray):
    msg.x = values[0]
    msg.y = values[1]
    msg.z = values[2]
    msg.w = values[3]


if len(sys.argv) < 5:
    print("Usage: rgv_controller.py <RGV_NAME> <X> <Y> <YAW>")
    exit(-1)

rgv_name = sys.argv[1]
rospy.init_node(f"rgv_controller_{rgv_name}")
rgv_x0 = float(sys.argv[2])
rgv_y0 = float(sys.argv[3])
rgv_Y0 = float(sys.argv[4])
    
rospy.wait_for_service("gazebo/set_model_state")
mover_service_proxy = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
rate = rospy.Rate(UPDATE_RATE)

rqst = SetModelStateRequest()
rqst.model_state.model_name = rgv_name
rqst.model_state.pose.position.x = rgv_x0
rqst.model_state.pose.position.y = rgv_y0
quat = Rotation.from_euler("xyz", [0, 0, rgv_Y0]).as_quat()
fillQuaternionMessage(rqst.model_state.pose.orientation, quat)

while not rospy.is_shutdown():
    resp: SetModelStateResponse = mover_service_proxy(rqst)
    if not resp.success:
        print(resp.status_message)
    rqst.model_state.pose.position.x += DX
    rate.sleep()