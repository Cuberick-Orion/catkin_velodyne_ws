#!/usr/bin/env python
import rospy
from roslib import message

import rosbag
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

def read_data():
	# pub = rospy.Publisher('velodyne_point_data', String, queue_size=10)

		rospy.init_node('read_data',anonymous=True)

	# while not rospy.is_shutdown():
		bag = rosbag.Bag("/home/cuberick/raw_data/kitti_2011_09_26_drive_0001_synced.bag")
		# bag = rosbag.Bag("kitti_2011_09_26_drive_0001_synced")
		topic = "/kitti/velo/pointcloud"
		msg = "sensor_msgs/PointCloud2"


		count = 1

		for topic, msg, t in bag.read_messages():
			if count < 1:
				break

			count -= 1

			# data_length = len(msg.data)
			# print (data_length)

			# a = type(msg)
			# print(a)

			# print(msg)

			total_digit_count = 0
			data_length = len(msg.data)
			raw_data = pc2.read_points(msg, skip_nans=False)

			all_points = np.empty([1,3],dtype=float)
			# a = all_points.shape
			# print(a)

			for point in raw_data:
				current_point = [point[0], point[1], point[2]]
				all_points = np.vstack([all_points , current_point])
				# print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
			print(all_points)
			a = all_points.shape
			print(a)
			# for point_count in range(data_length/4):
				# for digit_count in range(4):
					# first_dig = raw_data[total_digit_count]
					# second_dig = raw_data[total_digit_count + 1]
					# third_dig = raw_data[total_digit_count + 2]
					# fourth_dig = raw_data[total_digit_count + 3]


					# print(first_dig, second_dig, third_dig, fourth_dig)

					# raw_number = np.array([first_dig, second_dig, third_dig, fourth_dig], dtype = 'uint8')
					# float_number = raw_number.view('<f4')

					# total_digit_count += 4
					# print(float_number)

					# if point_count > 0:
					# 	break
		bag.close()





if __name__ == '__main__':
	try:
		read_data()
	except rospy.ROSInterruptException:
		pass