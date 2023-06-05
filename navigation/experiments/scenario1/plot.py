import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

def get_points(p_r, p_h):
    x_r, y_r = p_r[0], p_r[1]
    x_h, y_h = p_h[0], p_h[1]
    robot_x = [x_r-0.25, x_r+0.25, x_r-0.25, x_r-0.25]
    robot_y = [y_r+0.1, y_r, y_r-0.1, y_r+0.1]
    human_x = [x_h+0.25, x_h-0.25, x_h+0.25, x_h+0.25]
    human_y = [y_h+0.1, y_h, y_h-0.1, y_h+0.1]
    return robot_x, robot_y, human_x, human_y

def plot(file1, file2, vh, fig, axs, index):
    x_r = []
    y_r = []
    times = []
    lines = file1.readlines()
    for line in lines:
        if '#' in line:
            continue
        t, x, y = line.strip().split(',')
        x_r.append(float(x))
        y_r.append(float(y))
        times.append(float(t))
    x_r = np.array(x_r)
    y_r = np.array(y_r)
    times = np.array(times)

    # extract position to draw robot in plot
    idx = np.where(y_r == max(y_r.min(), y_r.max(), key=abs))[0][0]
    p_r = [x_r[idx], y_r[idx]]

    x_h = []
    y_h = []
    x0_h = 12.0
    y0_h = 0.0
    for t in times:
        x_h.append(x0_h - vh*t)
        y_h.append(0.0)
    x_h = np.array(x_h)
    y_h = np.array(y_h)

    # extract position to draw human in plot
    p_h = [x_h[idx], y_h[idx]]

    v_r = []
    v_h = []
    t_vel = []
    lines = file2.readlines()
    for line in lines:
        if '#' in line:
            continue
        t, v = line.strip().split(',')
        t_vel.append(float(t))
        v_r.append(float(v))
        v_h.append(vh)
    t_vel = np.array(t_vel)
    v_r = np.array(v_r)
    v_h = np.array(v_h)

    points_r = np.array([x_r,y_r]).T.reshape(-1,1,2)
    segments_r = np.concatenate([points_r[:-1], points_r[1:]], axis=1)

    points_h = np.array([x_h,y_h]).T.reshape(-1,1,2)
    segments_h = np.concatenate([points_h[:-1], points_h[1:]], axis=1)

    norm = plt.Normalize(times.min(), times.max())
    lc_r = LineCollection(segments_r, cmap='viridis', norm=norm)
    lc_h = LineCollection(segments_h, cmap='viridis', norm=norm)
    lc_r.set_array(times)
    lc_h.set_array(times)
    lc_r.set_linewidth(6)
    lc_h.set_linewidth(6)
    line_r = axs[index][0].add_collection(lc_r)
    line_h = axs[index][0].add_collection(lc_h)
    fig.colorbar(line_r, ax=axs[index][0])

    # plot human and robot
    robot_x, robot_y, human_x, human_y = get_points(p_r, p_h)
    axs[index][0].plot(robot_x, robot_y, 'black', linewidth=1.1, label='Robot')
    axs[index][0].plot(human_x, human_y, 'black', linewidth=2.2, label='Human')
    axs[index][0].legend()
    axs[index][0].grid()

    x_min = min(x_r.min(), x_h.min())
    y_min = min(y_r.min(), y_h.min())
    x_max = max(x_r.max(), x_h.max())
    y_max = max(y_r.max(), y_h.max())
    axs[index][0].set_xlim(x_min-0.5, x_max+0.5)
    axs[index][0].set_ylim(-1.5, 1.5)
    axs[index][0].set_xlabel('x [m]')
    axs[index][0].set_ylabel('y [m]')

    axs[index][1].plot(t_vel, v_r, 'red', label='Robot')
    axs[index][1].plot(t_vel, v_h, 'blue', label='Human')
    v_min = min(v_r.min(), v_h.min())
    v_max = max(v_r.max(), v_h.max())
    axs[index][1].set_xlim(0, t_vel.max())
    axs[index][1].set_ylim(0, 1)
    axs[index][1].legend()
    axs[index][1].grid()
    axs[index][1].set_xlabel('Time [s]')
    axs[index][1].set_ylabel('Speeds [m/s]')

if len(sys.argv) != 3:
    sys.exit(0)

filename = sys.argv[1]
vh = float(sys.argv[2])

# CoHAN
f1 = open(f'test/path_cohan{filename}.txt', 'r')
f2 = open(f'test/cmdvel_cohan{filename}.txt', 'r')
# Simple Move Base
f3 = open(f'test/path_smb{filename}.txt', 'r')
f4 = open(f'test/cmdvel_smb{filename}.txt', 'r')

fig, axs = plt.subplots(2, 2, figsize=(16,8))

plot(f1, f2, vh, fig, axs, 0)
plot(f3, f4, vh, fig, axs, 1)

plt.show()
