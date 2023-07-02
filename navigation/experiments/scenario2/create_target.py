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
d1 = 1.5
d2 = 1

src_file = open(f'test/{filename}.txt','r')
lines = src_file.readlines()
x0_h = float(lines[2].strip().split(',')[3])
last_raw = lines[-1].strip().split(',')
xf = float(last_raw[1])
yf = float(last_raw[2])

# R0
x0 = 0.0
y0 = 0.0

t1 = (x0_h - d1) / (v_r - v_h)
# R1
x1 = v_r * t1
y1 = 0.0
# H1
x1_h = x0_h + v_h * t1

t12 = math.sqrt(2) / v_r
t2 = t1 + t12
# R2
x2 = x1 + 1.0
y2 = 1.0
# H2
x2_h = x1_h + v_h * t12

t23 = (d2 + x2_h - x2) / (v_r - v_h)
t3 = t2 + t23
# R3
x3 = x2 + v_r * t23
y3 = 1.0
# H3
x3_h = x2_h + v_h * t23

delta23 = eucl_dist(np.array([x2,y2]), np.array([x3, y3]))
delta3f = eucl_dist(np.array([x3,y3]), np.array([xf, yf]))
path_length = x1 + math.sqrt(2) + delta23 + delta3f
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
    elif t < t3:
        x = x2 + v_r * (t - t2)
        X.append(x)
        Y.append(1.0)
    else:
        deltax = xf - x3
        deltay = yf - y3
        theta = math.atan2(deltay, deltax)
        x = x3 + (v_r * math.cos(theta) * (t-t3))
        y = (deltay / deltax) * (x - x3) + y3
        X.append(x)
        Y.append(y)

target_file = open(f'target/{filename}.txt','w')
assert len(X) == len(Y)
t = 0.0
for i in range(len(X)):
    target_file.write(f'{t},{X[i]},{Y[i]}\n')
    t += 0.1

target_file.close()

#plt.plot(X,Y,'r')
#plt.axis('equal')
#plt.grid()
#plt.show()
