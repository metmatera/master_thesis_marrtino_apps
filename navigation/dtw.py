import sys
import math

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

def plot_trajectories(T1, T2, distance):
    mpl.rcParams['legend.fontsize'] = 10
    fig = plt.figure(figsize=(7,7))
    ax = plt.axes(projection='3d')
    ax.set_title(f"Distance between trajectories (DTW): {distance:.3f}", fontsize=10)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('t')
    ax.set_xlim((-20,20))
    ax.set_ylim((-20,20))
    ax.set_zlim((0,30))
    ax.set_aspect('equal')

    x1, y1, t1 = [], [], []
    for elem in T1:
        x1.append(elem[0])
        y1.append(elem[1])
        t1.append(elem[2])

    ax.plot(x1,y1,t1,'red',label='CoHAN-planned trajectory')

    x2, y2, t2 = [], [], []
    for elem in T2:
        x2.append(elem[0])
        y2.append(elem[1])
        t2.append(elem[2])

    ax.plot(x2,y2,t2,'green',label='Human-controlled trajectory')

    ax.legend()
    plt.show()

# main
if (len(sys.argv) != 2):
	sys.exit(0)
	
filename = sys.argv[1]

f1 = open("trajs/"+filename+".txt", "r")
f2 = open("trajs/"+filename+"_cohan.txt", "r")

# Trajectories (x,y)
t1, t2 = [], []

# Trajectories (x,y,t)
T1, T2 = [], []

lines = f1.readlines()
for line in lines:
    if "#" in line:
        t1_name = line.strip().split(" ")[1]
        continue
    t, x, y = line.strip().split(",")
    t1.append([float(x),float(y)])
    T1.append([float(x),float(y),float(t)])
t1 = np.array(t1)
T1 = np.array(T1)

lines = f2.readlines()
for line in lines:
    if "#" in line:
        t2_name = line.strip().split(" ")[1]
        continue
    t, x, y = line.strip().split(",")
    t2.append([float(x),float(y)])
    T2.append([float(x),float(y),float(t)])
t2 = np.array(t2)
T2 = np.array(T2)

dtw = dtw(t1,t2)
print("DTW: " + str(dtw))
print("\nPress CTRL+C to close the window...")
plot_trajectories(T1,T2,dtw)
