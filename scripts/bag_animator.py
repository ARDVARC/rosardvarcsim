#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from rosardvarc.msg import EstimatedRgvState, RgvLocalProjection
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
import rospy
from typing import Tuple, Deque
from collections import deque
import numpy as np
from geometry_msgs.msg import PoseStamped
from FSW.config.constants import MISSION_AREA_HALF_WIDTH


fig, ax = plt.subplots()
ax.set_xlim(-MISSION_AREA_HALF_WIDTH, MISSION_AREA_HALF_WIDTH)
ax.set_ylim(-MISSION_AREA_HALF_WIDTH, MISSION_AREA_HALF_WIDTH)
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
ax.grid(which='both')
most_recent_rgv1_estimate: Tuple[float,float,float] = (0,0,0)
most_recent_rgv2_estimate: Tuple[float,float,float] = (0,0,0)
most_recent_uas_estimate: Tuple[float,float,float] = (0,0,0)
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

def uas_callback(msg: PoseStamped):
    global most_recent_uas_estimate
    most_recent_uas_estimate = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) # type: ignore

rospy.init_node("animator")
est_sub = rospy.Subscriber("estimation/estimated_rgv_states", EstimatedRgvState, est_callback)
proj_sub = rospy.Subscriber("estimation/rgv_local_projections", RgvLocalProjection, proj_callback)
uas_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, uas_callback)

def animate(i):
    sc_rgv1_est.set_offsets((-most_recent_rgv1_estimate[1],most_recent_rgv1_estimate[0]))
    sc_rgv2_est.set_offsets((-most_recent_rgv2_estimate[1],most_recent_rgv2_estimate[0]))
    
    sc_uas_true.set_offsets((-most_recent_uas_estimate[1],most_recent_uas_estimate[0]))
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