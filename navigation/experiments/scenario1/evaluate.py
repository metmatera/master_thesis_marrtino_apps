import math
from math import sqrt, acos, pi
import numpy as np
from numpy import dot
from numpy.linalg import norm

import sys

def eucl_dist(x, y):
    return norm(x-y)

def RAD(x):
    return x*pi/180.0

x0_h = float(sys.argv[1])           # initial x-position of the human
y0_h = float(sys.argv[2])           # initial y-position of the human
p0_h = np.array([x0_h, y0_h])       # initiail position (x0,y0) of the human
v_h = float(sys.argv[3])            # human velocity (constant)
r_h = float(sys.argv[4])            # human radius
r_r = float(sys.argv[5])            # robot radius
R = r_h + r_r
d_proxemics = float(sys.argv[6])    # proxemics distance
theta_fov = RAD(float(sys.argv[7])) # field of view angle
filename = sys.argv[8]              # test filename without extention

f = open(f'test/{filename}.txt','r')

robot = []
lines = f.readlines()
for line in lines:
    if '#' in line:
        continue
    t, x, y = line.strip().split(',')
    robot.append([float(x),float(y),float(t)])
robot = np.array(robot)

human = []
for elem in robot:
    t = elem[0]
    x = x0_h - v_h*t
    y = y0_h
    human.append([x,y,t])
human = np.array(human)

cost_danger = 0.0
cnt1 = 0
cost_passby = 0.0
cnt2 = 0
cost_visibility = 0.0
cnt3 = 0
for i in range(1, len(robot)):
    t0 = robot[i-1][2]
    t1 = robot[i][2]
    deltaT = t1 - t0
    # robot position
    x_r, y_r = robot[i][0], robot[i][1]
    p_r = np.array([x_r, y_r])
    # human position
    x_h, y_h = human[i][0], human[i][1]
    p_h = np.array([x_h, y_h])
    # distance vector between robot and human
    p_rh = p_r - p_h
    # distance vector between human and robot
    p_hr = -p_rh

    deltaX = x_r - robot[i-1][0]
    deltaY = y_r - robot[i-1][1]
    # robot velocity vector between thw two point
    vr = np.array([deltaX/deltaT, deltaY/deltaT])
    # human velocity vector: constant
    vh = np.array([-v_h, 0.0])
    # relative velocity of the robot wrt the human
    v_rel = vr - vh
    # effective distance between human and robot
    dhreff = eucl_dist(p_r, p_h) - R
    # TTC: Time To Collision
    TTC = dhreff / norm(v_rel)
    cost_danger += 1 / TTC
    cnt1 += 1

    # Compute cost_passby
    term1 = pow(norm(v_rel),2) * pow(norm(p_rh),2)
    term2 = pow(dot(p_rh,v_rel),2)
    term3 = norm(v_rel)*R
    if dot(p_rh,v_rel) > 0 and sqrt(term1-term2) > term3:
        cost_passby += (norm(v_rel) * abs(sqrt(term1-term2))) / (norm(p_rh) * (sqrt(term1-term2)-term3))
        cnt2 += 1

    # Compute cost_visibility
    alpha = 2 * d_proxemics / theta_fov
    u_h = np.array([-1.0,0.0])
    theta = acos(dot(u_h,p_hr)/norm(p_hr))
    cost_visibility += alpha * theta / dhreff
    cnt3 += 1

cost_danger /= cnt1
print(f'cost_danger: {cost_danger:.2f}')
try:
    cost_passby /= cnt2
    print(f'cost_passby: {cost_passby:.2f}')
except ZeroDivisionError:
    print('cost_passby: no data available.')
cost_visibility /= cnt3
print(f'cost_visibility: {cost_visibility:.2f}')
