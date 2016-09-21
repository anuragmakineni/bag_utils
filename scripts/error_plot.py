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
path = '/home/anuragmakineni/Desktop/cc_1.bag'

start_index = 1433
end_index = 2604


fig = plt.figure()
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


    time_pos_cropped = time_pos[start_index:end_index]
    error_cropped = error[start_index:end_index]

    plt.plot(time_pos_cropped - time_pos_cropped[0], error_cropped)
    print('\n' + robot + " max Error: " + str(np.amax(error_cropped)))
    print(robot + " std dev: " + str(np.std(error_cropped)))

plt.title('Total Error vs. Time')
plt.grid(1)
plt.legend(robots)
plt.xlabel('Time (s)')
plt.ylabel('Error (m)')
plt.show()
bag.close()
