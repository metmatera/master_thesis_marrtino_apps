import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

# main
if len(sys.argv) != 3:
    print("Missing arguments: scenario (1|2|3) - filename (without extension)")
    sys.exit(0)

scenario = sys.argv[1]
filename = sys.argv[2]

file = open(f'scenario{scenario}/test/{filename}.txt', 'r')

times = []
x_r, y_r = [], []
x_h, y_h = [], []
lines = file.readlines()
for line in lines:
    if '#' in line:
        continue
    t, xr, yr, xh, yh = line.strip().split(',')
    times.append(float(t))
    x_r.append(float(xr))
    y_r.append(float(yr))
    x_h.append(float(xh))
    y_h.append(float(yh))
times = np.array(times)
x_r = np.array(x_r)
y_r = np.array(y_r)
x_h = np.array(x_h)
y_h = np.array(y_h)

fig, ax = plt.subplots(1, 1, figsize=(15,5))

points_r = np.array([x_r,y_r]).T.reshape(-1,1,2)
segments_r = np.concatenate([points_r[:-1], points_r[1:]], axis=1)

points_h = np.array([x_h,y_h]).T.reshape(-1,1,2)
segments_h = np.concatenate([points_h[:-1], points_h[1:]], axis=1)

norm = plt.Normalize(times.min(), times.max())
lc_r = LineCollection(segments_r, cmap='viridis', norm=norm)
lc_h = LineCollection(segments_h, cmap='viridis', norm=norm)
lc_r.set_array(times)
lc_h.set_array(times)
lc_r.set_linewidth(4)
lc_h.set_linewidth(4)
line_r = ax.add_collection(lc_r)
line_h = ax.add_collection(lc_h)
cbar = fig.colorbar(line_r, ax=ax)
cbar.set_label('Time [s]')
ax.grid()

x_min = min(x_r.min(), x_h.min())
y_min = min(y_r.min(), y_h.min())
x_max = max(x_r.max(), x_h.max())
y_max = max(y_r.max(), y_h.max())
ax.set_xlim(x_min-0.5, x_max+0.5)
ax.set_ylim(-2.5, 2.5)
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_aspect('equal')

# Draw Robot and Human
dist = 100.0
idx = 0
for i in range(len(x_r)):
    if abs(x_r[i]-x_h[i]) < dist:
        dist = abs(x_r[i]-x_h[i])
        idx = i

ax.plot(x_r[idx], y_r[idx], c='red', marker='>', markersize=20, label='Robot')
ax.plot(x_h[idx], y_h[idx], c='blue', marker='<', markersize=20, label='Human')
ax.legend()

plt.savefig(f'scenario{scenario}/plots/{filename}_path.png')
#plt.show()
