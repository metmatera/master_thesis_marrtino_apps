import sys,os
import argparse

sys.path.append(os.getenv("MARRTINO_APPS_HOME")+"/program")

import robot_cmd_ros
from robot_cmd_ros import *

robot_cmd_ros.use_audio = False
robot_cmd_ros.tv_good = 0.5

# main
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='forward gbn')
    parser.add_argument('GX', type=float, help='Goal X [m]')

    args = parser.parse_args()

    begin(use_desired_cmd_vel=True)

    forward(args.GX, obstacleAvoidance=True)

    end()

