import pandas as pd
import numpy as np
import sys

# Computation packages
from scipy.spatial.distance import euclidean
from fastdtw import fastdtw

def eucl_dist(x, y):
    return np.linalg.norm(x-y)

def compute_euclidean_distance_matrix(t1, t2) -> np.array:
    dist = np.zeros((len(t1), len(t2)))
    for i in range(len(t2)):
        for j in range(len(t1)):
            dist[i,j] = eucl_dist(t1[j], t2[i])
    return dist

def compute_accumulated_cost_matrix(t1, t2) -> np.array:

    distances = compute_euclidean_distance_matrix(t1, t2)

    # Initialization
    cost = np.zeros((len(t2), len(t1)))
    cost[0,0] = distances[0,0]

    for i in range(1, len(t2)):
        cost[i, 0] = distances[i, 0] + cost[i-1, 0]

    for j in range(1, len(t1)):
        cost[0, j] = distances[0, j] + cost[0, j-1]

    # Accumulated warp path cost
    for i in range(1, len(t2)):
        for j in range(1, len(t1)):
            cost[i, j] = min(
                cost[i-1, j],    # insertion
                cost[i, j-1],    # deletion
                cost[i-1, j-1]   # match
            ) + distances[i, j]

    return cost

scenario = sys.argv[1]
filename = sys.argv[2]

f1 = open(f'experiments/scenario{scenario}/test/{filename}.txt','r')
f2 = open(f'experiments/scenario{scenario}/target/{filename}.txt','r')

t1 = []
lines = f1.readlines()
for line in lines:
    if '#' in line:
        continue
    x, y = line.strip().split(',')[1:3]
    t1.append([float(x), float(y)])
t1 = np.array(t1)

t2 = []
lines = f2.readlines()
for line in lines:
    if '#' in line:
        continue
    x, y = line.strip().split(',')[1:3]
    t2.append([float(x), float(y)])
t2 = np.array(t2)

dtw_distance, warp_path = fastdtw(t1, t2, dist=euclidean)

print("DTW distance: ", dtw_distance)
