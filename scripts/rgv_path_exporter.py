#!/usr/bin/env python3
from rosardvarcsim.rgv_path_generator import RGV, getRgvStateAtTime
import argparse
import numpy as np
import csv
import rospkg
import os


parser = argparse.ArgumentParser(prog="rosrun rosardvarcsim rgv_path_exporter.py",
                                 description="Generates RGV paths and exports them as CSV files",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("-s", "--seed", type=int, required=True, help="An integer used to set the random number generator for the path generation.")
parser.add_argument("-r", "--sample_rate", type=int, default=120, help="How often the RGV path should be sampled in the output CSV, in Hz.")
parser.add_argument("-d", "--duration", type=int, default=3600, help="How long the RGV path should be generated for, in seconds.")
parser.add_argument("--x0", default=0, type=float, help="The initial X position of the RGV in the local frame, in meters.")
parser.add_argument("--y0", default=0, type=float, help="The initial Y position of the RGV in the local frame, in meters.")
parser.add_argument("--yaw0", default=0, type=float, help="The initial yaw angle of the RGV, in radians CCW from east.")
parser.add_argument("-o", "--output", default="", type=str, help="The name of the output file. If not specified, name will be 'rgv_path_SEED.csv'.")

args = parser.parse_args()

rgv = RGV(seed=args.seed, vec_startPos_local_ne=np.array([args.x0, args.y0]), startYawAngle=args.yaw0, duration=args.duration)

rospack = rospkg.RosPack()
package_path = rospack.get_path('rosardvarcsim')
if args.output == "":
    file_name = f"rgv_path_{args.seed}.csv"
elif args.output.endswith(".csv"):
    file_name = args.output
else:
    file_name = f"{args.output}.csv"

file_path = os.path.join(package_path, "rgv_paths", file_name)

with open(file_path, 'w') as csvfile:
    writer = csv.writer(csvfile)
    for t in np.arange(0.0, args.duration, 1/args.sample_rate):
        rgv_state = getRgvStateAtTime(rgv=rgv, t=t)
        writer.writerow([1/args.sample_rate, rgv_state[1][0], rgv_state[1][1], rgv_state[0]])