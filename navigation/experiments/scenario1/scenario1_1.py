import numpy as np
import math
import matplotlib.pyplot as plt

robot = [0.0, 0.0]
human = [6.0, 0.0]

x_i, x_f = 0.0, 12.0
timesteps = int(x_f * 10 + 1)
total_distance = 10.0 + 2*math.sqrt(2)

X = np.linspace(x_i, x_f, num=timesteps)

Y1 = []
for x in X:
    if x <= 4.0 or x > 8.0:
        Y1.append(0.0)
    elif x <= 5.0:
        Y1.append(x-4.0)
    elif x <= 7:
        Y1.append(1.0)
    else:
        Y1.append(-x+8.0)

Y1 = np.array(Y1)

Y2 = []
for x in X:
    if x <= 4.0 or x > 8.0:
        Y2.append(0.0)
    elif x <= 5.0:
        Y2.append(-x+4.0)
    elif x <= 7:
        Y2.append(-1.0)
    else:
        Y2.append(x-8.0)

Y2 = np.array(Y2)

f1 = open('target/scenario1_1_1.txt','w')
f1.write("# Total distance: " + str(total_distance) + "\n")
f2 = open('target/scenario1_1_2.txt','w')
f2.write("# Total distance: " + str(total_distance) + "\n")
for i in range(len(X)):
    f1.write(str(X[i]) + ',' + str(Y1[i]) + '\n')
    f2.write(str(X[i]) + ',' + str(Y2[i]) + '\n')

f1.close()
f2.close()

plt.title('Scenario 1 with static human')
plt.plot(X,Y2,'r--')
plt.plot(X,Y1,'g-')
plt.scatter(robot[0], robot[1], s=100, c='black')
plt.text(robot[0]-0.5, robot[1]-0.6, 'robot')
plt.scatter(human[0], human[1], s=100, c='black')
plt.text(human[0]-0.5, human[1]-0.6, 'human')
plt.axis('equal')
plt.show()
