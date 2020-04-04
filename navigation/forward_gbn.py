import sys,os
import argparse

sys.path.append(os.getenv("MARRTINO_APPS_HOME")+"/program")

import robot_cmd_ros
from robot_cmd_ros import *

robot_cmd_ros.use_audio = False
robot_cmd_ros.tv_good = 0.5

def do_forward(m):
    begin(use_desired_cmd_vel=True)
    enableObstacleAvoidance(True)
    forward(m)
    enableObstacleAvoidance(False)
    end()

# main
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='forward gbn')
    parser.add_argument('GX', type=float, help='Goal X [m]')

    args = parser.parse_args()

    do_forward(args.GX)

