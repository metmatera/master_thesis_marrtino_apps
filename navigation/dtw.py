import sys, math

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

def eucl_dist(x, y):
    return np.linalg.norm(x-y)

def dtw(t1, t2):
    n1 = len(t1)
    n2 = len(t2)
    C = np.zeros((n1+1, n2+1))
    C[1:, 0] = float('inf')
    C[0, 1:] = float('inf')

    for i in np.arange(n1) + 1:
        for j in np.arange(n2) + 1:
            C[i,j] = eucl_dist(t1[i - 1], t2[j - 1]) + min(C[i, j - 1], C[i - 1, j - 1], C[i - 1, j])

    dtw = C[n1, n2]
    return dtw

def compute_avg_vel(trajectory, start, end):
    p0 = np.array([trajectory[start][0], trajectory[start][1]])
    p1 = np.array([trajectory[end][0], trajectory[end][1]])
    distance = eucl_dist(p0, p1)
    deltaT = trajectory[end][2] - trajectory[start][2]
    return distance / deltaT


# main
if (len(sys.argv) != 3):
    sys.exit(0)

filename = sys.argv[1]
type = sys.argv[2]

f1 = open("trajs/"+filename+".txt", "r")
f2 = open("trajs/"+filename+"_cohan_"+type+".txt", "r")

t1 = []
lines = f1.readlines()
for line in lines:
    if '#' in line:
        continue
    t, x, y = line.strip().split(",")
    t1.append([float(x),float(y),float(t)])
t1 = np.array(t1)

v_m = compute_avg_vel(t1, 0, len(t1)-1)

t2 = []
last_x, last_y, t = 0.0, 0.0, 0.0
lines = f2.readlines()
for line in lines:
    if '#' in line:
        continue
    x, y = line.strip().split(",")
    x = float(x)
    y = float(y)
    if len(t2) == 0:
        t2.append([x,y,t])
        last_x = x
        last_y = y
    else:
        p0 = np.array([last_x, last_y])
        p1 = np.array([x, y])
        deltaT = eucl_dist(p0, p1) / v_m
        t += deltaT
        t2.append([x,y,t])
        last_x = x
        last_y = y
t2 = np.array(t2)

score = dtw(t1, t2)
print(f"DTW score: {score:.3f}")
print(f"Total time (Human-controlled): {t1[-1][2]:.3f} s")
print(f"Total time (CoHAN-planned): {t2[-1][2]:.3f} s")
print(f"Time difference: {abs(t1[-1][2] - t2[-1][2]):.3f} s")

# Plot 3D trajectories [x,y,t]
mpl.rcParams['legend.fontsize'] = 10
fig = plt.figure(figsize=(7,7))
ax = plt.axes(projection='3d')
ax.set_title(f"DTW score: {score:.3f}")
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('t')

delta = 2
x_min = min([elem[0] for elem in t1] + [elem[0] for elem in t2])
x_max = max([elem[0] for elem in t1] + [elem[0] for elem in t2])
y_min = min([elem[1] for elem in t1] + [elem[1] for elem in t2])
y_max = max([elem[1] for elem in t1] + [elem[1] for elem in t2])
t_max = max([elem[2] for elem in t1] + [elem[2] for elem in t2])

ax.set_xlim((x_min-delta, x_max+delta))
ax.set_ylim((y_min-delta, y_max+delta))
ax.set_zlim((0, t_max+delta))
ax.set_aspect('equal')

x = [elem[0] for elem in t1]
y = [elem[1] for elem in t1]
t = [elem[2] for elem in t1]
ax.plot(x, y, t, 'green', label='Human-controlled trajectory')

x = [elem[0] for elem in t2]
y = [elem[1] for elem in t2]
t = [elem[2] for elem in t2]
ax.plot(x, y, t, 'red', label='CoHAN-planned trajectory')

ax.legend()
plt.show()
