import matplotlib.pyplot as plt
import numpy as np
import math
from scipy import interpolate
import scipy.ndimage as ndimage
from pyparsing import stringEnd
import matplotlib.patches as patches

# plt.rcParams.update({
#     "text.usetex": True,
#     "font.family": "serif",
#     "font.serif": ["Computer Modern Roman"]})

plt.rcParams["figure.figsize"] = (7.5, 7.5)

ROOT_FOLDER = "ros_basics_ws"
DATA_PATH = ROOT_FOLDER+'/'+"data"
GRAPH_PATH = ROOT_FOLDER+'/'+"graphs"
OBS_LENGTH = 6.4e-2
OBS_WIDTH = 3.3e-2

# f = open('ros_basics_ws/best_run.txt', 'r')
def plot_path(filename, color, label, alpha):
    f = open(DATA_PATH+'/'+filename, 'r')

    poses = []
    for row in f:
        if ('wpt' not in row):
            pose = row.split(';')
            pose = [float(i) for i in pose]
            poses.append(pose[2:])

    poses = np.array(poses)

    xp = poses[:, 0]
    yp = poses[:, 1]

    jump = np.sqrt(np.diff(xp)**2 + np.diff(yp)**2) 
    smooth_jump = ndimage.gaussian_filter1d(jump, 5, mode='wrap')  # window of size 5 is arbitrary
    limit = 2*np.median(smooth_jump)    # factor 2 is arbitrary
    xn, yn = xp[:-1], yp[:-1]
    xn = xn[(jump > 0) & (smooth_jump < limit)]
    yn = yn[(jump > 0) & (smooth_jump < limit)]

    tck, u = interpolate.splprep([xn, yn], s=0.002)
    x_i, y_i = interpolate.splev(np.linspace(0, 1, 100), tck)
    plt.plot(x_i, y_i, color = color, label = label, alpha=alpha, linewidth=3)

    # Set marker on starting position
    plt.scatter(xp[0], yp[0], color = color, marker = 'x', linewidth = 20, s = 10)

    # plt.scatter(xp, yp, color = color, label = label, s=4*np.ones(len(xp)))
    dx = 0.001/2

    # for pose in poses:
    for i in range(0, len(poses), 10):
        pose = poses[i]
        plt.arrow(pose[0]-dx*math.cos(pose[2]), pose[1]-dx*math.sin(pose[2]), dx*math.cos(pose[2]), dx*math.sin(pose[2]), 
                  head_width = 0.015, color = color, alpha=alpha)

def plot_waypoints(filename, color):
    f = open(DATA_PATH+'/'+filename, 'r')
    waypoints = []
    i = 0
    dx = -0.01
    dy = 0.03
    for row in f:
        if ('wpt' in row):
            i = i + 1
            waypoint = row.split(';')
            waypoint = [float(i) for i in waypoint[1:3]]
            waypoints.append(waypoint)
            plt.text(waypoint[0] + dx, waypoint[1] + dy, str(i), color=color, fontsize = 15, fontweight='bold')
    waypoints = np.array(waypoints)
    plt.scatter(waypoints[:, 0], waypoints[:, 1], marker='o', color = color, s = 100*np.ones(len(waypoints[:, 0])), zorder=3, label="Waypoints")

def plot_obstacle(color):
    dx = -0.06
    dy = 0.03
    fig, ax = plt.subplots()
    rect = patches.Rectangle((0-OBS_LENGTH/2, 0-OBS_WIDTH/2), OBS_LENGTH, OBS_WIDTH, linewidth=2, edgecolor=color, facecolor='none')
    plt.text(dx, dy, "Obstacle", color=color, fontsize = 15)
    ax.add_patch(rect)

plot_obstacle('k')
plot_path("best_run.txt", 'r', "Real trajectory", .5)
plot_path("best_run_simu.txt", 'b', "Simulated trajectory", .5)
plot_waypoints("best_run.txt", 'k')

plt.xlim(-.5, .5)
plt.ylim(-.5, .5)
plt.xticks(np.arange(-.5, .6, step=0.1))
plt.yticks(np.arange(-.5, .6, step=0.1))
plt.gca().set_aspect('equal', adjustable='box')
plt.legend(loc="upper left")
plt.title("Real trajectory vs simulated trajectory")
plt.xlabel("x-coordinate [m]")
plt.ylabel("y-coordinate [m]")

plt.savefig(GRAPH_PATH+'/plot.pdf', bbox_inches='tight')
plt.show()
    