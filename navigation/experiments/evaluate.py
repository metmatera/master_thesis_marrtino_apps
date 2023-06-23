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

def compute_navigation_metrics(t, xr, yr, xh, yh):
    # path length
    path_length = 0.0
    for i in range(len(xr)-1):
        p0 = np.array([xr[i],yr[i]])
        p1 = np.array([xr[i+1],yr[i+1]])
        path_length += eucl_dist(p0, p1)
    # path efficiency
    p0 = np.array([xr[0],yr[0]])
    p1 = np.array([xr[-1],yr[-1]])
    path_efficiency = eucl_dist(p0, p1) / path_length
    # time to reach goal
    ttrg = t[-1]
    return path_length, path_efficiency, ttrg

def compute_discomfort_metrics(t, xr, yr, xh, yh, R):
    d_proxemics = 1
    # 1) time spent in proxemics zones
    # 2) minimum distance to a human
    tspz = 0.0
    min_dist = 100.0
    for i in range(len(xr)):
        pr = np.array([xr[i],yr[i]])
        ph = np.array([xh[i],yh[i]])
        distance = eucl_dist(pr, ph)
        if distance <= d_proxemics:
            tspz += 0.1
        if distance < min_dist:
            min_dist = distance
    # time to collision (TTC)
    TTC = 0.0
    cost_danger = 0.0
    cnt1 = 0
    cost_passby = 0.0
    cnt2 = 0
    for i in range(len(xr)-1):
        # distance vector between robot and human P_rh
        pr = np.array([xr[i],yr[i]])
        ph = np.array([xh[i],yh[i]])
        p_rh = ph - pr
        # Relative velocity V_rel
        delta_xr = xr[i+1] - xr[i]
        delta_yr = yr[i+1] - yr[i]
        delta_xh = xh[i+1] - xh[i]
        delta_yh = yh[i+1] - yh[i]
        delta_t = 0.1
        vr = np.array([delta_xr/delta_t, delta_yr/delta_t])
        vh = np.array([delta_xh/delta_t, delta_yh/delta_t])
        v_rel = vr - vh
        # computation
        term1 = dot(p_rh,v_rel)
        term2 = pow(norm(v_rel),2) * (pow(norm(p_rh),2) - pow(R,2))
        if (term1 > 0) and (pow(term1,2) - term2 > 0):
            TTC += (term1 - sqrt(pow(term1,2) - term2)) / pow(norm(v_rel),2)
            cost_danger += pow(norm(v_rel),2) / (term1 - sqrt(pow(term1,2) - term2))
            cnt1 += 1

        term3 = pow(norm(v_rel),2) * pow(norm(p_rh),2) - pow(term1,2)
        if (term1 > 0) and (term3 > 0) and (sqrt(term3) > R*norm(v_rel)):
            cost_passby += (norm(v_rel) * abs(sqrt(term3))) / (norm(p_rh) * (sqrt(term3) - R*norm(v_rel)))
            cnt2 += 1

    if cnt1 > 0:
        TTC /= cnt1
        cost_danger /= cnt1
    else:
        TTC = 'N/A'
        cost_danger = 'N/A'
    if cnt2 > 0:
        cost_passby /= cnt2
    else:
        cost_passby = 'N/A'

    return tspz, min_dist, TTC, cost_danger, cost_passby

scenario = sys.argv[1]
filename = sys.argv[2]

human_radius = 0.3
robot_radius = 0.0
R = human_radius + robot_radius

file = open(f'scenario{scenario}/test/{filename}.txt','r')

times = []
x_r, y_r, v_r = [], [], []
x_h, y_h, v_h = [], [], []
lines = file.readlines()
for line in lines:
    if '#' in line:
        continue
    t, xr, yr, vr, xh, yh, vh = line.strip().split(',')
    times.append(float(t))
    x_r.append(float(xr))
    y_r.append(float(yr))
    v_r.append(float(vr))
    x_h.append(float(xh))
    y_h.append(float(yh))
    v_h.append(float(vh))
times = np.array(times)
x_r = np.array(x_r)
y_r = np.array(y_r)
v_r = np.array(v_r)
x_h = np.array(x_h)
y_h = np.array(y_h)
v_h = np.array(v_h)

path_length, path_efficiency, ttrg = compute_navigation_metrics(times, x_r, y_r, x_h, y_h)
tspz, min_dist, TTC, cost_danger, cost_passby = compute_discomfort_metrics(times, x_r, y_r, x_h, y_h, R)

print('Navigation Metrics:')
print(f'Path Length: {path_length:.2f}')
print(f'Path Efficiency: {path_efficiency:.2f}')
print(f'Time to reach goal: {ttrg:.2f}\n')
print('Discomfort Metrics:')
print(f'Time spent in proxemics zone: {tspz:.2f}')
print(f'Minimum distance to a human: {min_dist:.2f}')
print(f'Time To Collision (TTC): {TTC:.2f}')
print(f'Cost Danger: {cost_danger:.2f}')
print(f'Cost Pass-by: {cost_passby:.2f}')
