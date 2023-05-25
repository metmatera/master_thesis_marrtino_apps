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

x1 = x0_r + v_r * ((d + x0_h - x0_r)/(v_r - v_h) + math.sqrt(2)/v_h)

xi, xf = x0_h, 14.0
timesteps = int(xf * 10 + 1)
total_distance = 10.0 + 2*math.sqrt(2)

X = np.linspace(xi, xf, num=timesteps)

Y = []
for x in X:
    if x <= x1 or x > x1 + 4.0:
        Y.append(0.0)
    elif x <= x1 + 1.0:
        Y.append(-x+x1)
    elif x <= x1 + 3.0:
        Y.append(-1.0)
    else:
        Y.append(x-x1-4)

Y = np.array(Y)

f = open(f'target/r{v_r}_h{v_h}.txt','w')
f.write(f'# Total distance: {total_distance}\n')
for i in range(len(X)):
    f.write(str(X[i]) + ',' + str(Y[i]) + '\n')

f.close()

plt.title(f'Scenario 1 - robot vel: {v_r} - human_vel: {v_h}')
plt.plot(X,Y,'g-')

plt.scatter(x0_r, y0_r, s=100, c='black')
plt.text(x0_r-0.3, y0_r-0.6, 'R_0')
plt.scatter(x0_h, y0_h, s=100, c='black')
plt.text(x0_h-0.3, y0_h-0.6, 'H_0')

plt.axis('equal')
plt.grid()
plt.show()
