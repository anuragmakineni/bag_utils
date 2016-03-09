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

angular_velocity_x = np.empty((0))
angular_velocity_y = np.empty((0))
angular_velocity_z = np.empty((0))
linear_acceleration_x = np.empty((0))
linear_acceleration_y = np.empty((0))
linear_acceleration_z = np.empty((0))
roll = np.empty((0))
pitch = np.empty((0))
yaw = np.empty((0))
initial_time_imu = 0
time_imu = np.empty((0))


bag = rosbag.Bag('/home/anuragmakineni/Ubuntu_Share/high_speed.bag')
topic = '/fla1/mavros/imu/data'
#topic = '/imu_vn/imu'
#topic = '/sync/imu/imu'

for topic, msg, t in bag.read_messages(topics=[topic]):

    angular_velocity_x = np.append(angular_velocity_x, msg.angular_velocity.x)
    angular_velocity_y = np.append(angular_velocity_y, msg.angular_velocity.y)
    angular_velocity_z = np.append(angular_velocity_z, msg.angular_velocity.z)

    linear_acceleration_x = np.append(linear_acceleration_x, msg.linear_acceleration.x)
    linear_acceleration_y = np.append(linear_acceleration_y, msg.linear_acceleration.y)
    linear_acceleration_z = np.append(linear_acceleration_z, msg.linear_acceleration.z)

    quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    roll = np.append(roll, np.rad2deg(euler[0]))
    pitch = np.append(pitch, np.rad2deg(euler[1]))
    yaw = np.append(yaw, np.rad2deg(euler[2]))

    if time_imu.size == 0:
        time_imu = np.append(time_imu, initial_time_imu)
        initial_time_imu = msg.header.stamp.to_sec()
    else:
        time_imu = np.append(time_imu, msg.header.stamp.to_sec() - initial_time_imu)


#angular velocity
ang_vel = plt.figure()
plt.title('Angular Velocity ' + topic)
plt.ylabel('Velocity (rad/s)')
plt.xlabel('Time (s)')
plt.plot(time_imu, angular_velocity_x, color = 'r')
plt.plot(time_imu, angular_velocity_y, color = 'g')
plt.plot(time_imu, angular_velocity_z, color = 'b')

#linear acceleration
lin_acc = plt.figure()
plt.title('Linear Acceleration ' + topic)
plt.ylabel('Linear Acceleration (m/s^2)')
plt.xlabel('Time (s)')
plt.plot(time_imu, linear_acceleration_x, color = 'r')
plt.plot(time_imu, linear_acceleration_y, color = 'g')
plt.plot(time_imu, linear_acceleration_z, color = 'b')

#rpy
rpy = plt.figure()
plt.title('Roll, Pitch ' + topic)
plt.ylabel('Angle (deg)')
plt.xlabel('Time (s)')
max_pitch = np.max(pitch)
min_pitch = np.min(pitch)
plt.plot(time_imu, max_pitch * np.ones_like(time_imu), color='g', ls='--')
plt.plot(time_imu, min_pitch * np.ones_like(time_imu), color='g', ls='--')
plt.plot(time_imu,roll, color='r')
plt.plot(time_imu, pitch, color='g')
#plt.plot(time_imu, yaw, color='b')

#write csv
f = open("imu.csv","w")
out = csv.writer(f, quoting=csv.QUOTE_NONE)
out.writerow(time_imu.tolist())

out.writerow(linear_acceleration_x)
out.writerow(linear_acceleration_y)
out.writerow(linear_acceleration_z)

out.writerow(roll)
out.writerow(pitch)
out.writerow(yaw)

out.writerow(angular_velocity_x)
out.writerow(angular_velocity_y)
out.writerow(angular_velocity_z)

f.close()
del out


plt.rcParams.update({'font.size': 12})

plt.show()
bag.close()
