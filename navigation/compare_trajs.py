import sys
import math

import matplotlib.pyplot as plt

fn1 = sys.argv[1]
fn2 = sys.argv[2]

f1 = open(fn1, "r")
f2 = open(fn2, "r")

t1_x, t1_y, t2_x, t2_y = [], [], [], []

lines = f1.readlines()
for line in lines:
    if "#" in line:
        continue
    x, y = line.strip().split(",")
    t1_x.append(float(x))
    t1_y.append(float(y))

lines = f2.readlines()
for line in lines:
    if "#" in line:
        continue
    x, y = line.strip().split(",")
    t2_x.append(float(x))
    t2_y.append(float(y))

timesteps = min(len(t1_x), len(t2_x))
avg_distance = 0.0
for i in range(timesteps):
    p1 = [t1_x[i],t1_y[i]]
    p2 = [t2_x[i],t2_y[i]]
    avg_distance += math.dist(p1,p2)

avg_distance /= timesteps
print(f"Average distance: {avg_distance}")

plt.plot(t1_x, t1_y, 'r')
plt.plot(t2_x, t2_y, 'g')
plt.show()
