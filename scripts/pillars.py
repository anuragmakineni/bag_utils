#!/usr/bin/env python
import rosbag
import matplotlib
matplotlib.use('Qt4Agg')
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt


#get bag data
robots = ['/pico03/', '/pico04/', '/pico05/']
start = ['S1', 'S2', 'S3']
goal = ['G1', 'G2', 'G3']
path = '/home/anuragmakineni/Day3/d3_forest_formation_3.bag'

start_index = 1500
end_index = 2300

j = 0
fig = plt.figure(figsize=[12,4.7])
for robot in robots:
    odom_x = np.empty((0))
    odom_y = np.empty((0))
    odom_z = np.empty((0))

    setpoint_x = np.empty((0))
    setpoint_y = np.empty((0))
    setpoint_z = np.empty((0))

    error = np.empty((0))

    time = np.empty((0))
    time_pos = np.empty((0))

    bag = rosbag.Bag(path)
    topic = robot + 'odom'

    for topic, msg, t in bag.read_messages(topics=[topic]):

        odom_x = np.append(odom_x, msg.pose.pose.position.x)
        odom_y = np.append(odom_y, msg.pose.pose.position.y)
        odom_z = np.append(odom_z, msg.pose.pose.position.z)

        time = np.append(time, msg.header.stamp.to_sec())

    topic = robot + 'position_cmd'

    for topic, msg, t in bag.read_messages(topics=[topic]):

        setpoint_x = np.append(setpoint_x, msg.position.x)
        setpoint_y = np.append(setpoint_y, msg.position.y)
        setpoint_z = np.append(setpoint_z, msg.position.z)

        time_pos = np.append(time_pos, msg.header.stamp.to_sec())

    i=0
    for t in time_pos:
        delta_t = abs(time - t)
        o_i = np.argmin(delta_t)

        error_x = setpoint_x[i] - odom_x[o_i]
        error_y = setpoint_y[i] - odom_y[o_i]
        error_z = setpoint_z[i] - odom_z[o_i]

        total_error = np.sqrt(error_x * error_x + error_y * error_y + error_z * error_z)
        error = np.append(error, total_error)

        i = i+1

    odom_x_cropped = odom_x[start_index:end_index]
    odom_y_cropped = odom_y[start_index:end_index]

    plt.plot(odom_x_cropped, odom_y_cropped, lw=2.0)
    plt.gca().text(odom_x_cropped[0], odom_y_cropped[0] + 0.05, start[j], fontsize=16)
    plt.gca().text(odom_x_cropped[-1], odom_y_cropped[-1] + 0.05, goal[j], fontsize=16)
    j = j + 1

plt.grid(1)
plt.legend(['Robot 1', 'Robot 2', 'Robot 3'])
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
pillar_radius = 0.075
robot_radius = 0.067
#pillar positions
positions = [(0.116, 0.157), (0.423, 0.207), (0.845, 0.239), (1.005, -0.075), (0.610, -0.079), (0.243, -0.164)]
ax = plt.gca()
for p in positions:
    cir = plt.Circle(p, pillar_radius, color='k', linewidth=5.0, fill=False, hatch='///')
    cir_bound = plt.Circle(p, pillar_radius + robot_radius, color='g', fill=False, linewidth=3.0, ls='dashed')
    ax.add_artist(cir)
    ax.add_artist(cir_bound)
#plt.axis('equal')
ax.set_aspect('equal', 'datalim')
ax.set_ylim([-0.5,0.5])
plt.show()
bag.close()
