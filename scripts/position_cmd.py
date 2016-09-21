#!/usr/bin/env python
import rosbag
import rospy
import math
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tf.transformations
import csv

#get bag data

odom_x = np.empty((0))
odom_y = np.empty((0))
odom_z = np.empty((0))

bag = rosbag.Bag('/home/anuragmakineni/Desktop/cc_1.bag')
topic = '/pico03/position_cmd'

for topic, msg, t in bag.read_messages(topics=[topic]):

    odom_x = np.append(odom_x, msg.position.x)
    odom_y = np.append(odom_y, msg.position.y)
    odom_z = np.append(odom_z, msg.position.z)


#odom
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(odom_x, odom_y, odom_z)

topic = '/pico04/position_cmd'

odom_x = np.empty((0))
odom_y = np.empty((0))
odom_z = np.empty((0))

for topic, msg, t in bag.read_messages(topics=[topic]):

    odom_x = np.append(odom_x, msg.position.x)
    odom_y = np.append(odom_y, msg.position.y)
    odom_z = np.append(odom_z, msg.position.z)

ax.plot(odom_x, odom_y, odom_z)

topic = '/pico05/position_cmd'

odom_x = np.empty((0))
odom_y = np.empty((0))
odom_z = np.empty((0))

for topic, msg, t in bag.read_messages(topics=[topic]):

    odom_x = np.append(odom_x, msg.position.x)
    odom_y = np.append(odom_y, msg.position.y)
    odom_z = np.append(odom_z, msg.position.z)

ax.plot(odom_x, odom_y, odom_z)

plt.show()
bag.close()
