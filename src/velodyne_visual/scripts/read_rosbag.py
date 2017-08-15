#!/usr/bin/env python
import rospy
from roslib import message

import rosbag
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud



def read_data():
	# pub = rospy.Publisher('velodyne_point_data', String, queue_size=10)

		rospy.init_node('test_velodyne',anonymous=True)


		bag = rosbag.Bag("/home/cuberick/raw_data/kitti_2011_09_26_drive_0001_synced.bag")

		topic = "/kitti/velo/pointcloud"
		msg = "sensor_msgs/PointCloud2"


		count = 3
		all_points = np.empty([1,3],dtype=float)

		for topic, msg, t in bag.read_messages():
			print("start processing, count")
			print count
			with 
			print(msg)

# 			data_length = len(msg.data)
# ## msg is of type PointCloud2
# 			raw_data = pc2.read_points(msg)
	

# 			for point in raw_data:
# 				current_point = [point[0], point[1], point[2]]
# 				if point[0] > 4:
# 					all_points = np.vstack([all_points , current_point])
# 			print(all_points)
# 			a = all_points.shape
# 			print(a)



			# frameid = getFrameId (msg)
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
			
			# header = msg.header
			#header

			# processed_data = pc2.create_cloud(header, fields, all_points)

			
			# print(processed_data)
		bag.close()

		# print('generating processed data')
		# header = std_msgs.msg.Header()
		# header.stamp = rospy.Time.now()
		# header.frame_id = 'map'

		# fields = [PointField('x', 0, PointField.FLOAT32, 1),
  #                 PointField('y', 4, PointField.FLOAT32, 1),
  #                 PointField('z', 8, PointField.FLOAT32, 1),
  #                 PointField('i', 12, PointField.FLOAT32, 1)]


		# processed_data = pc2.create_cloud_xyz32(header, all_points)

		# pcl_pub = rospy.Publisher("/velodyne_pub", PointCloud2, queue_size = 10)
		# rospy.loginfo("info...")
		# rospy.sleep(1.)
		# rospy.loginfo("publishing sample pointcloud.. !")
		# pcl_pub.publish(processed_data)


		



if __name__ == '__main__':
	try:
		read_data()
	except rospy.ROSInterruptException:
		pass