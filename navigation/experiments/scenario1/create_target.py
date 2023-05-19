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

x1 = v_r / (v_r + v_h) * (x0_h - x0_r - 2.0)

xi, xf = 0.0, 12.0
timesteps = int(xf * 10 + 1)
total_distance = 10.0 + 2*math.sqrt(2)

X = np.linspace(xi, xf, num=timesteps)

Y1 = []
for x in X:
    if x <= x1 or x > x1 + 4.0:
        Y1.append(0.0)
    elif x <= x1 + 1.0:
        Y1.append(x-x1)
    elif x <= x1 + 3.0:
        Y1.append(1.0)
    else:
        Y1.append(-x+x1+4)

Y1 = np.array(Y1)

Y2 = []
for x in X:
    if x <= x1 or x > x1 + 4.0:
        Y2.append(0.0)
    elif x <= x1 + 1.0:
        Y2.append(-x+x1)
    elif x <= x1 + 3.0:
        Y2.append(-1.0)
    else:
        Y2.append(x-x1-4)

Y2 = np.array(Y2)

f1 = open(f'target/r{v_r}_h{v_h}.txt','w')
f1.write(f'# Total distance: {total_distance}\n')
#f2 = open(f'target/r{v_r}_h{v_h}_2.txt','w')
#f2.write(f'# Total distance: {total_distance}\n')
for i in range(len(X)):
    f1.write(str(X[i]) + ',' + str(Y1[i]) + '\n')
    #f2.write(str(X[i]) + ',' + str(Y2[i]) + '\n')

f1.close()
#f2.close()

plt.title(f'Scenario 1 - robot vel: {v_r} - human_vel: {v_h}')
plt.plot(X,Y2,'r--')
plt.plot(X,Y1,'g-')

plt.scatter(x0_r, y0_r, s=100, c='black')
plt.text(x0_r-0.3, y0_r-0.6, 'R_0')
plt.scatter(x1, 0.0, s=100, c='black')
plt.text(x1-0.3, -0.6, 'R_1')
plt.scatter(x0_h, y0_h, s=100, c='black')
plt.text(x0_h-0.3, y0_h-0.6, 'H_0')
if (v_h != 0.0):
    plt.scatter(x1+2.0, 0.0, s=100, c='black')
    plt.text(x1+1.7, -0.6, 'H_1')

plt.axis('equal')
plt.grid()
plt.show()
