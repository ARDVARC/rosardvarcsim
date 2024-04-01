#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from rosardvarc.msg import EstimatedRgvState, RgvLocalProjection
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
import rospy
from typing import Tuple, Deque
from collections import deque
import numpy as np


fig, ax = plt.subplots()
ax.set_xlim(-22.86, 22.86)
ax.set_ylim(-22.86, 22.86)
ax.set_aspect('equal')
sc_rgv1_bt_proj = ax.scatter([], [], color='red', marker='+')
sc_rgv2_bt_proj = ax.scatter([], [], color='blue', marker='+')
sc_rgv1_cam_proj = ax.scatter([], [], color='red', marker='X')
sc_rgv2_cam_proj = ax.scatter([], [], color='blue', marker='X')
sc_rgv1_est = ax.scatter([], [], color='red', marker='o')
sc_rgv2_est = ax.scatter([], [], color='blue', marker='o')
sc_rgv1_true = ax.scatter([], [], color='red', marker='.')
sc_rgv2_true = ax.scatter([], [], color='blue', marker='.')
sc_uas_true = ax.scatter([], [], color='green', marker='.')
most_recent_rgv1_estimate: Tuple[float,float,float] = (0,0,0)
most_recent_rgv2_estimate: Tuple[float,float,float] = (0,0,0)
recent_rgv1_bt_projections: Deque[Tuple[float,float]] = deque(maxlen=30)
recent_rgv2_bt_projections: Deque[Tuple[float,float]] = deque(maxlen=30)
recent_rgv1_cam_projections: Deque[Tuple[float,float]] = deque(maxlen=30)
recent_rgv2_cam_projections: Deque[Tuple[float,float]] = deque(maxlen=30)


def est_callback(msg: EstimatedRgvState):
    global most_recent_rgv1_estimate, most_recent_rgv2_estimate
    most_recent_rgv1_estimate = msg.rgv1_position_local # type: ignore
    most_recent_rgv2_estimate = msg.rgv2_position_local # type: ignore

def proj_callback(msg: RgvLocalProjection):
    global recent_rgv1_bt_projections, recent_rgv2_bt_projections, recent_rgv1_cam_projections, recent_rgv2_cam_projections
    if msg.rgv_id == 1:
        if msg.measurement_source == 10:
            recent_rgv1_cam_projections.appendleft((-msg.rgv_position_local[1], msg.rgv_position_local[0]))
        else:
            recent_rgv1_bt_projections.appendleft((-msg.rgv_position_local[1], msg.rgv_position_local[0]))
    else:
        if msg.measurement_source == 10:
            recent_rgv2_cam_projections.appendleft((-msg.rgv_position_local[1], msg.rgv_position_local[0]))
        else:
            recent_rgv2_bt_projections.appendleft((-msg.rgv_position_local[1], msg.rgv_position_local[0]))


rospy.init_node("animator")
est_sub = rospy.Subscriber("estimation/estimated_rgv_states", EstimatedRgvState, est_callback)
proj_sub = rospy.Subscriber("estimation/rgv_local_projections", RgvLocalProjection, proj_callback)
rospy.wait_for_service("gazebo/get_model_state")
model_service = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
rgv1_model_request = GetModelStateRequest()
rgv1_model_request.model_name = "rgv1"
rgv2_model_request = GetModelStateRequest()
rgv2_model_request.model_name = "rgv2"
uas_model_request = GetModelStateRequest()
uas_model_request.model_name = "ardvarc_drone"

def animate(i):
    sc_rgv1_est.set_offsets((-most_recent_rgv1_estimate[1],most_recent_rgv1_estimate[0]))
    sc_rgv2_est.set_offsets((-most_recent_rgv2_estimate[1],most_recent_rgv2_estimate[0]))
    
    rgv1_response: GetModelStateResponse = model_service(rgv1_model_request)
    if not rgv1_response.success:
        rospy.logwarn(rgv1_response.status_message)
    rgv2_response: GetModelStateResponse = model_service(rgv2_model_request)
    if not rgv2_response.success:
        rospy.logwarn(rgv2_response.status_message)
    uas_response: GetModelStateResponse = model_service(uas_model_request)
    if not uas_response.success:
        rospy.logwarn(uas_response.status_message)
    
    sc_rgv1_true.set_offsets((-rgv1_response.pose.position.y,rgv1_response.pose.position.x))
    sc_rgv2_true.set_offsets((-rgv2_response.pose.position.y,rgv2_response.pose.position.x))
    sc_uas_true.set_offsets((-uas_response.pose.position.y,uas_response.pose.position.x))
    if len(recent_rgv1_bt_projections) > 0:
        sc_rgv1_bt_proj.set_offsets(list(recent_rgv1_bt_projections))
    if len(recent_rgv2_bt_projections) > 0:
        sc_rgv2_bt_proj.set_offsets(list(recent_rgv2_bt_projections))
    if len(recent_rgv1_cam_projections) > 0:
        sc_rgv1_cam_proj.set_offsets(list(recent_rgv1_cam_projections))
    if len(recent_rgv2_cam_projections) > 0:
        sc_rgv2_cam_proj.set_offsets(list(recent_rgv2_cam_projections))

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()