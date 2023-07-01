import rosbag
import sys
import numpy as np

scenario = sys.argv[1]
filename = sys.argv[2]
x0_r = float(sys.argv[3])
y0_r = float(sys.argv[4])
x0_h = float(sys.argv[5])
y0_h = float(sys.argv[6])

robot_pose = '/base_pose_ground_truth'
robot_vel = '/cmd_vel'
human_pose = '/human1/base_pose_ground_truth'
human_vel = '/human1/cmd_vel'
topics = [robot_pose, human_pose]

robot = []
human = []

bagfile = f'experiments/scenario{scenario}/test/{filename}.bag'
with rosbag.Bag(bagfile, 'r') as bag:
    for (topic, msg, ts) in bag.read_messages(topics=topics):

        if topic == robot_pose:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            if (x != x0_r or y != y0_r):
                robot.append([x,y])

        elif topic == human_pose:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            if (x != x0_h or y != y0_h):
                human.append([x,y])

        else:
            assert False, 'Unknown message type'

t = 0.0
file = open(f'experiments/scenario{scenario}/test/{filename}.txt', 'w')
file.write(f'# Scenario: {scenario}\n')
file.write('# t,x_r,y_r,x_h,y_h\n')
file.write(f'{t},{x0_r},{y0_r},{x0_h},{y0_h}\n')

# FIX TO HAVE SAME LENGTH
robot_goal = robot[-1]
idx = robot.index(robot_goal)
robot = robot[:idx+1]
human = human[:len(robot)]
assert len(robot) == len(human)
timesteps = len(robot)

for i in range(timesteps):
    t += 0.1
    file.write(f'{t},{robot[i][0]},{robot[i][1]},{human[i][0]},{human[i][1]}\n')
