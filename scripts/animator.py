import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
from rosardvarc.msg import EstimatedRgvState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
import rospy
from typing import Tuple

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
most_recent_rgv1_estimate: Tuple[float,float,float] = (0,0,0)
most_recent_rgv2_estimate: Tuple[float,float,float] = (0,0,0)


def callback(msg: EstimatedRgvState):
    global most_recent_rgv1_estimate, most_recent_rgv2_estimate
    most_recent_rgv1_estimate = msg.rgv1_position_local # type: ignore
    most_recent_rgv2_estimate = msg.rgv2_position_local # type: ignore

rospy.init_node("animator")
est_sub = rospy.Subscriber("estimation/estimated_rgv_states", EstimatedRgvState, callback)
rospy.wait_for_service("gazebo/get_model_state")
model_service = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
rgv1_model_request = GetModelStateRequest()
rgv1_model_request.model_name = "rgv1"
rgv2_model_request = GetModelStateRequest()
rgv2_model_request.model_name = "rgv2"
uas_model_request = GetModelStateRequest()
uas_model_request.model_name = "ardvarc_drone"
estimate_pub = rospy.Publisher("estimation/estimated_rgv_states", EstimatedRgvState)

def animate(i):
    ax1.clear()
    ax1.grid(visible=True, which="minor")
    ax1.set_xlim(-12.86, 12.86)
    ax1.set_ylim(-12.86, 12.86)
    ax1.axis(option="equal")
    ax1.plot(most_recent_rgv1_estimate[0],most_recent_rgv1_estimate[1], 'ro')
    ax1.plot(most_recent_rgv2_estimate[0],most_recent_rgv2_estimate[1], 'bo')
    
    rgv1_response: GetModelStateResponse = model_service(rgv1_model_request)
    if not rgv1_response.success:
        rospy.logwarn(rgv1_response.status_message)
    rgv2_response: GetModelStateResponse = model_service(rgv2_model_request)
    if not rgv2_response.success:
        rospy.logwarn(rgv2_response.status_message)
    uas_response: GetModelStateResponse = model_service(uas_model_request)
    if not uas_response.success:
        rospy.logwarn(uas_response.status_message)
    
    ax1.plot(rgv1_response.pose.position.x,rgv1_response.pose.position.y, 'r.')
    ax1.plot(rgv2_response.pose.position.x,rgv2_response.pose.position.y, 'b.')
    ax1.plot(uas_response.pose.position.x,uas_response.pose.position.y, 'g.')
    

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()
rospy.spin()