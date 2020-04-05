import sys,os
import argparse

sys.path.append(os.getenv("MARRTINO_APPS_HOME")+"/program")

import robot_cmd_ros
from robot_cmd_ros import *

robot_cmd_ros.use_audio = False

def do_turn(deg, mode):
    begin()
    turn(deg, mode)
    end()

# main
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='turn')
    parser.add_argument('GTH', type=float, help='Goal Theta [deg]')
    parser.add_argument('mode', type=str, nargs='?', default='REL', help='REL or ABS target angle')

    args = parser.parse_args()

    do_turn(args.GTH, args.mode)


