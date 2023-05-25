import sys
import numpy as np
import math
import matplotlib.pyplot as plt

x0_r = float(sys.argv[1])
y0_r = float(sys.argv[2])
v_r = float(sys.argv[3])

x0_h = float(sys.argv[4])
y0_h = float(sys.argv[5])
v_h = float(sys.argv[6])

l = float(sys.argv[7])
d = float(sys.argv[8])

xf_h = 17.0
xf_r = 15.0

t1 = (d + x0_h - x0_r) / (v_r - v_h)
t2 = t1 + math.sqrt(2) / v_h
t3 = t2 + l / v_h
t4 = t3 + math.sqrt(2) / v_h

x1_h = x0_h + v_h * t1
x2_h = x1_h + 1
x3_h = x2_h + l
x4_h = x3_h + 1

x1_r = x0_r + v_r * t1
x2_r = x0_r + v_r * t2
x3_r = x0_r + v_r * t3
x4_r = x0_r + v_r * t4
x5_r = x4_r + 1
x6_r = x5_r + (v_r / v_h) * (xf_r - x4_h)
x7_r = x6_r + 1

# create target trajectory for the human
ts_h = int(xf_h * 10 + 1)
X_h = np.linspace(x0_h, xf_h, num=ts_h)
Y_h = []
for x in X_h:
    if x <= x1_h or x > x4_h:
        Y_h.append(0.0)
    elif x <= x2_h:
        Y_h.append(x - x1_h)
    elif x <= x3_h:
        Y_h.append(1.0)
    else:
        Y_h.append(-x + x4_h)
Y_h = np.array(Y_h)

# create target trajectory for the robot
ts_r = int(xf_r * 10 + 1)
X_r = np.linspace(x0_r, xf_r, num=ts_r)
Y_r = []
for x in X_r:
    if x <= x4_r or x > x7_r:
        Y_r.append(0.0)
    elif x <= x5_r:
        Y_r.append(-x + x4_r)
    elif x <= x6_r:
        Y_r.append(-1.0)
    else:
        Y_r.append(x - x7_r)
Y_r = np.array(Y_r)

total_distance = 10 + 2*math.sqrt(2)

f = open(f'target/r{v_r}_h{v_h}.txt','w')
f.write(f'# Total distance: {total_distance}\n')
for i in range(len(X_r)):
    f.write(str(X_r[i]) + ',' + str(Y_r[i]) + '\n')

f.close()

plt.figure(figsize=(10,7))

plt.title(f'Scenario 2 - robot vel: {v_r} - human_vel: {v_h}')
plt.plot(X_h,Y_h,'g-', label='Human')
plt.plot(X_r,Y_r,'r-', label='Robot')

plt.scatter(x0_h, y0_h, s=30, c='green')
plt.text(x0_h-0.2, y0_h+0.5, 'H0')
plt.scatter(x0_r, y0_r, s=30, c='red')
plt.text(x0_r-0.2, y0_r-0.5, 'R0')

plt.scatter(x1_h, 0.0, s=30, c='green')
plt.text(x1_h-0.2, 0.5, 'H1')
plt.scatter(x1_r, 0.0, s=30, c='red')
plt.text(x1_r-0.2, -0.5, 'R1')

plt.scatter(x4_h, 0.0, s=30, c='green')
plt.text(x4_h-0.2, 0.5, 'H2')
plt.scatter(x4_r, 0.0, s=30, c='red')
plt.text(x4_r-0.2, -0.5, 'R2')

plt.scatter(xf_r, 0.0, s=30, c='green')
plt.text(xf_r-0.2, 0.5, 'H3')
plt.scatter(x6_r, -1.0, s=30, c='red')
plt.text(x6_r-0.2, -1.5, 'R3')

plt.axis('equal')
plt.grid()
plt.legend()
plt.show()
