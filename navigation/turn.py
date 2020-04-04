import sys,os
import argparse

sys.path.append(os.getenv("MARRTINO_APPS_HOME")+"/program")

import robot_cmd_ros
from robot_cmd_ros import *

robot_cmd_ros.use_audio = False

def do_turn(deg):
    begin()
    turn(deg)
    end()

# main
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='turn')
    parser.add_argument('GTH', type=float, help='Goal Theta [deg]')

    args = parser.parse_args()

    do_turn(args.GTH)


