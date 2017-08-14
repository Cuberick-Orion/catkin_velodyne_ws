#!/usr/bin/env python
import rospy


import rosbag
import sensor_msgs

def read_data():
	bag = rosbag.Bag("/home/cuberick/raw_data/kitti_2011_09_26_drive_0001_synced.bag")
	# bag = rosbag.Bag("kitti_2011_09_26_drive_0001_synced")
	topic = "/kitti/velo/pointcloud"
	msg = "sensor_msgs/PointCloud2"


	num_msgs = 1

	for topic, msg, t in bag.read_messages():
		if num_msgs < 1:
			break

		num_msgs -= 1
		data_length = len(msg.data)
		print (data_length)

	bag.close()



if __name__ == '__main__':
	read_data()