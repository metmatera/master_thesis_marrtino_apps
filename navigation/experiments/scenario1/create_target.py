import sys
import numpy as np
import math
from numpy.linalg import norm
import matplotlib.pyplot as plt

def eucl_dist(x, y):
    return norm(x-y)

filename = sys.argv[1]
v_h = float(sys.argv[2])
v_r = 0.6
d = 3

src_file = open(f'test/{filename}.txt','r')
lines = src_file.readlines()
x0_h = float(lines[2].strip().split(',')[3])
last_raw = lines[-1].strip().split(',')
xf = float(last_raw[1])
yf = float(last_raw[2])

x0, y0 = 0, 0
x1, y1 = v_r * (x0_h - d) / (v_r + v_h), 0
x2, y2 = x1 + 1, 1

t1 = (x0_h - d) / (v_r + v_h)
t2 = t1 + math.sqrt(2) / v_r

path_length = x1 + math.sqrt(2) + eucl_dist(np.array([x2,y2]), np.array([xf, yf]))
total_time = int(path_length * 10 / v_r) / 10

T = np.linspace(0, total_time, num=int(total_time*10 + 1))

X = []
Y = []

for t in T:
    if t < t1:
        X.append(v_r * t)
        Y.append(0.0)
    elif t < t2:
        x = x1 + (math.sqrt(2) * v_r * (t-t1) / 2)
        X.append(x)
        Y.append(x - x1)
    else:
        deltax = xf - x2
        deltay = yf - y2
        theta = math.atan2(deltay, deltax)
        x = x2 + (v_r * math.cos(theta) * (t-t2))
        y = (deltay / deltax) * (x - x2) + y2
        X.append(x)
        Y.append(y)

target_file = open(f'target/{filename}.txt','w')
assert len(X) == len(Y)
t = 0.0
for i in range(len(X)):
    target_file.write(f'{t},{X[i]},{Y[i]}\n')
    t += 0.1

target_file.close()

plt.plot(X,Y,'r')
plt.axis('equal')
plt.grid()
plt.show()
