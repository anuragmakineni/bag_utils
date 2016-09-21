#!/usr/bin/env python
import rosbag
import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

#get bag data
time = np.empty((0))
odom_z = np.empty((0))
vel_z = np.empty((0))
initial_time = 0.0

bag = rosbag.Bag('/home/anuragmakineni/run3.bag')
topic = '/vicon/am_bungee/odom'

for topic, msg, t in bag.read_messages(topics=[topic]):
    odom_z = np.append(odom_z, msg.pose.pose.position.z)
    vel_z = np.append(vel_z, msg.twist.twist.linear.z)

    if time.size == 0:
        time = np.append(time, initial_time)
        initial_time = msg.header.stamp.to_sec()
    else:
        time = np.append(time, msg.header.stamp.to_sec() - initial_time)

#pos
fig = plt.figure()
plt.plot(time, odom_z)
plt.grid(1)
plt.title('Z Position vs. Time')
plt.xlabel('Time (s)')
plt.ylabel('Height (m)')
plt.xlim([0.0, np.max(time)])

#vel
fig = plt.figure()
plt.plot(time, vel_z)
plt.grid(1)
plt.title('Z Velocity vs. Time')
plt.xlabel('Time (s)')
plt.ylabel('Vertical Velocity (m/s)')
plt.xlim([0.0, np.max(time)])

#acc
acc_z = np.divide(np.diff(vel_z), np.diff(time))/9.81 + 1.0
acc_t = time[:-1]

#filter
N = 10
fc = 25
fs = 100
h = signal.firwin(numtaps=N, cutoff=fc, nyq=fs/2.0)
y = signal.lfilter(h, 1.0, np.vstack((acc_t, acc_z)))

fig = plt.figure()
plt.plot(y[0,:], y[1,:])
plt.grid(1)
plt.title('Inerial Z Acceleration vs. Time')
plt.xlabel('Time (s)')
plt.ylabel('Z Acceleration (G)')
plt.xlim([0.0, np.max(time)])
plt.show()
bag.close()
