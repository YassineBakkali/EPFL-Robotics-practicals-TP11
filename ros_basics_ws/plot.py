import matplotlib.pyplot as plt
import numpy as np
import math

from pyparsing import stringEnd

# plt.rcParams.update({
#     "text.usetex": True,
#     "font.family": "serif",
#     "font.serif": ["Computer Modern Roman"]})

ROOT_FOLDER = "ros_basics_ws"

# f = open('ros_basics_ws/best_run.txt', 'r')
def plot_path(filename, color):
    f = open(ROOT_FOLDER+'/'+filename, 'r')

    poses = []
    for row in f:
        if ('wpt' not in row):
            pose = row.split(';')
            pose = [float(i) for i in pose]
            poses.append(pose[2:])
        # else:
        #     way
    poses = np.array(poses)

    print(np.max(np.abs(poses[:, 0:2])))
    plt.plot(poses[:, 0], poses[:, 1], color = color)

    dx = 0.001

    # for pose in poses:
    for i in range(0, len(poses), 5):
        pose = poses[i]
        plt.arrow(pose[0], pose[1], dx*math.cos(pose[2]), dx*math.sin(pose[2]), head_width = 0.01, color = color)

def plot_waypoints(filename, color):
    f = open('ros_basics_ws/'+filename, 'r')
    waypoints = []
    i = 0
    for row in f:
        if ('wpt' in row):
            i = i + 1
            waypoint = row.split(';')
            waypoint = [float(i) for i in waypoint[1:3]]
            waypoints.append(waypoint)
            plt.text(waypoint[0], waypoint[1], str(i), color=color, fontsize = 15)
    waypoints = np.array(waypoints)
    plt.scatter(waypoints[:, 0], waypoints[:, 1], marker='x', color = color, s = 100*np.ones(len(waypoints[:, 0])), zorder=3)

plot_path("best_run.txt", 'r')
plot_path("best_run_simu2.txt", 'b')
plot_waypoints("best_run.txt", 'g')


plt.xlabel("x-coordinate")
plt.ylabel("y-coordinate")
plt.show()
    
