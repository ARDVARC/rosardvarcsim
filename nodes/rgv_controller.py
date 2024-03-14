#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Quaternion
import numpy as np
import csv
from rosardvarcsim.rgv_path_generator import RGV, getRgvStateAtTime
import argparse
from dataclasses import dataclass
import random
import rospkg
import os
from typing import Optional


@dataclass
class Path:
    rate: float
    xs: np.ndarray
    ys: np.ndarray
    yaws: np.ndarray

def fillQuaternionMessage(msg: Quaternion, values: np.ndarray):
    msg.x = values[0]
    msg.y = values[1]
    msg.z = values[2]
    msg.w = values[3]

def load_file_func(args: argparse.Namespace) -> Path:
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('rosardvarcsim')
    if "." in args.filename:
        file_name = args.filename
    else:
        file_name = f"{args.filename}.csv"

    file_path = os.path.join(package_path, "rgv_paths", file_name)
    
    with open(file_path) as csvfile:
        reader = csv.reader(csvfile)
        rate = float(next(reader)[0])
        xs = []
        ys = []
        yaws = []
        for line in reader:
            xs.append(float(line[0]))
            ys.append(float(line[1]))
            yaws.append(float(line[2]))
        return Path(rate, np.array(xs), np.array(ys), np.array(yaws))

def generate_func(args: argparse.Namespace) -> Path:
    if args.seed == None:
        seed = int(random.random()*(2**16))
    else:
        seed = int(args.seed)
    rgv = RGV(seed, np.array([args.x0, args.y0]), args.yaw0, args.duration)
    xs = []
    ys = []
    yaws = []
    for t in np.arange(0, args.duration, 1.0/args.sample_rate):
        yaw, pos = getRgvStateAtTime(rgv, t)
        xs.append(pos[0])
        ys.append(pos[1])
        yaws.append(yaw)
    return Path(args.sample_rate, np.array(xs), np.array(ys), np.array(yaws))
    

parser = argparse.ArgumentParser(prog="rgv_controller.py",
                                 description="Generates RGV paths and exports them as CSV files",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("node_name", help="The name of the ROS node.")
parser.add_argument("rgv_name", help="The name of the RGV model in the Gazebo simulation.")
subparsers = parser.add_subparsers(help='sub-command help')
load_file_parser = subparsers.add_parser("load_file", help="load_file help")
load_file_parser.add_argument("filename", type=str, help="The name of the input .csv file.")
load_file_parser.set_defaults(func=load_file_func)
generate_parser = subparsers.add_parser("generate", help="generate help")
generate_parser.add_argument("-s", "--seed", default=None, type=int, help="An integer used to set the random number generator for the path generation. If not specified, a random seed is used")
generate_parser.add_argument("-r", "--sample_rate", default=120, type=int, help="How often the RGV path should be sampled in the output CSV, in Hz.")
generate_parser.add_argument("-d", "--duration", default=3600, type=int, help="How long the RGV path should be generated for, in seconds.")
generate_parser.add_argument("--x0", default=0, type=float, help="The initial X position of the RGV in the local frame, in meters.")
generate_parser.add_argument("--y0", default=0, type=float, help="The initial Y position of the RGV in the local frame, in meters.")
generate_parser.add_argument("--yaw0", default=0, type=float, help="The initial yaw angle of the RGV, in degrees clockwise from north.")
generate_parser.set_defaults(func=generate_func)

# roslaunch adds weird stuff to the end, ignore it
if sys.argv[-1].startswith("__log:="):
    args = parser.parse_args(sys.argv[1:-2])
else:
    args = parser.parse_args()
path: Path = args.func(args)

rospy.init_node(args.node_name)

rospy.wait_for_service("gazebo/set_model_state")
mover_service_proxy = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
rate = rospy.Rate(path.rate)

rqst = SetModelStateRequest()
rqst.model_state.model_name = args.rgv_name
rqst.model_state.pose.position.x = path.xs[0]
rqst.model_state.pose.position.y = path.ys[0]
quat = Rotation.from_euler("xyz", [0, 0, path.yaws[0]]).as_quat()
fillQuaternionMessage(rqst.model_state.pose.orientation, quat)
i = 0

while not rospy.is_shutdown():
    rate.sleep()
    if i < len(path.xs) - 1:
        i += 1
        
    resp: SetModelStateResponse = mover_service_proxy(rqst)
    if not resp.success:
        rospy.logwarn(resp.status_message)
        continue
    
    rqst.model_state.pose.position.x = path.xs[i]
    rqst.model_state.pose.position.y = path.ys[i]
    quat = Rotation.from_euler("xyz", [0, 0, path.yaws[i]]).as_quat()
    fillQuaternionMessage(rqst.model_state.pose.orientation, quat)