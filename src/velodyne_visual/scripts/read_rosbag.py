#!/usr/bin/env python
import rospy
from roslib import message

import rosbag
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
# require the installation of ros-kinetic-tf2_sensor_msgs
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import subprocess
import tf
import math
from numpy.linalg import inv

def process():
	# pub = rospy.Publisher('velodyne_point_data', String, queue_size=10)
		
		rospy.init_node('test_velodyne',anonymous=True)

		bag = rosbag.Bag("/home/cuberick/raw_data/kitti_2011_09_26_drive_0001_synced.bag")


# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Read IMU-to-Velodyne Transformation Matrix
		tcount = 1
		print(">>>reading tf info")
		print
		for topic, msg, t in bag.read_messages("/tf_static"):
			# if tcount < 1:
			# 	break
			# tcount -= 1

			
			# print count

			All_tfs = msg.transforms
# Extract T_imu_to_velo
			for transf in All_tfs:
				if transf.child_frame_id == "velo_link":
					T_imu_to_velo = transf.transform

# Transform between quaternion and Euler
			T_imu_to_velo_Quaternion_rotation = T_imu_to_velo.rotation
			T_imu_to_velo_translation = T_imu_to_velo.translation
			# print(T_imu_to_velo_Quaternion_rotation)
			quaternion = (
				T_imu_to_velo_Quaternion_rotation.x,
				T_imu_to_velo_Quaternion_rotation.y,
				T_imu_to_velo_Quaternion_rotation.z,
				T_imu_to_velo_Quaternion_rotation.w)

			T_imu_to_velo_Euler_rotaiton = tf.transformations.euler_from_quaternion(quaternion)
			# print(T_imu_to_velo_Euler_rotaiton)
			roll = T_imu_to_velo_Euler_rotaiton[0]
			pitch = T_imu_to_velo_Euler_rotaiton[1]
			yaw = T_imu_to_velo_Euler_rotaiton[2]

			T_imu_to_velo_homo = np.empty([4,4],dtype=float)
			T_imu_to_velo_homo = [[math.cos(yaw)*math.cos(pitch), 
							-math.cos(yaw)*math.sin(pitch)*math.sin(roll)+math.sin(yaw)*math.cos(roll), 
							-math.cos(yaw)*math.sin(pitch)*math.cos(roll)-math.sin(yaw)*math.sin(roll),
							-T_imu_to_velo_translation.x],
							[-math.sin(yaw)*math.cos(pitch),
							math.sin(yaw)*math.sin(pitch)*math.sin(roll)+math.cos(yaw)*math.cos(roll),
							-math.sin(yaw)*math.sin(pitch)*math.cos(roll)+math.cos(yaw)*math.sin(roll),
							-T_imu_to_velo_translation.y],
							[math.sin(pitch),
							-math.cos(pitch)*math.sin(roll),
							math.cos(pitch)*math.cos(roll),
							-T_imu_to_velo_translation.z],
							[0, 0, 0, 1] ]

			# print T_imu_to_velo_homo
		print ("   T_imu_to_velo obtained")
		print
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Read OXTS data
		OXTS_GPS_raw = np.empty([1,3],dtype=float)
		gcount = 1
		print (">>>Read OXTS GPS raw data")
		print

		for topic, msg, t in bag.read_messages("/kitti/oxts/gps/fix"):
			# if gcount < 1:
			# 	break
			# gcount -= 1

			current_GPS_data = [msg.latitude, msg.longitude, msg.altitude]
			OXTS_GPS_raw = np.vstack([OXTS_GPS_raw , current_GPS_data])

		OXTS_GPS_raw = np.delete(OXTS_GPS_raw, (0), axis=0)	
		print("   OSTX GPS raw data obtained")
		print
		# print(OXTS_GPS_raw)

		print (">>>Read OXTS IMU data")
		print

		OXTS_IMU_raw = np.empty([1,3],dtype=float)
		icount = 3

		for topic, msg, t in bag.read_messages("/kitti/oxts/imu"):
			# if icount < 1:
			# 	break
			# icount -= 1

			# print msg

			IMU_raw = msg.orientation
			quaternion_IMU = (
				IMU_raw.x,
				IMU_raw.y,
				IMU_raw.z,
				IMU_raw.w)

			IMU_data = tf.transformations.euler_from_quaternion(quaternion_IMU)
			IMU_roll = IMU_data[0]
			IMU_pitch = IMU_data[1]
			IMU_heading = IMU_data[2]
			OXTS_IMU_raw = np.vstack([OXTS_IMU_raw , [IMU_roll, IMU_pitch, IMU_heading]])

			# print IMU_data
		OXTS_IMU_raw = np.delete(OXTS_IMU_raw, (0), axis=0)
		# print OXTS_IMU_raw
		print ("   OXTS_IMU data obtained")
		print

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Read velodyne info
		print (">>>Read Velodyne point data")
		print

		all_points = np.empty([1,3],dtype=float)
		vcount = 1
		print("=========start processing, count==========")
		print

		for topic, msg, t in bag.read_messages("/kitti/velo/pointcloud"):

			if vcount < 1:
				break
			vcount -= 1

			
			print vcount
			print

			# print vcount

			data_length = len(msg.data)
## msg is of type PointCloud2
			raw_data = pc2.read_points(msg)
	

			for point in raw_data:
				current_point = [point[0], point[1], point[2]]
				if point[0] > 4:
					all_points = np.vstack([all_points , current_point])
			# print(all_points)
			# a = all_points.shape
			# print(a)


		print ("   Velodyne point data obtained")
		print

		all_points = np.delete(all_points, (0), axis=0)
		# print all_points
		bag.close()


#  everything completely read
#  proceed into processing
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



# ---->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# get pose (loadOxtsliteData and convertOxtsToPose)



# compute scale from first lat value
		oxts_first = OXTS_GPS_raw[0][0]
		scale = math.cos (oxts_first * math.pi / 180.00)
		# print scale

# OXTS_GPS_raw [1] [2] [3] and OXTS_IMU_raw [1] [2] [3]
		oxts = np.concatenate ((OXTS_GPS_raw, OXTS_IMU_raw), axis = 1)
		# print oxts
		lengh_of_oxts = np.shape(oxts)[0]
		# print lengh_of_oxts

		pose = [None] * lengh_of_oxts
		Tr_0_inv = np.zeros(shape = (4,4))
		isempty = np.zeros(shape = (4,4))
		# a = oxts[0]
		# print(a)

		i = 0
		for i in range(lengh_of_oxts-1):
			if oxts[i] == []:
				pose[i] = np.empty((3,3,)) * np.nan
				continue

			t = np.empty((3,1,))
			current_oxts_1 = oxts[i][0]
			current_oxts_2 = oxts[i][1]

			er = 6378137
			current_t_11 = scale * current_oxts_2 * math.pi * er / 180
			current_t_12 = scale * er * math.log(math.tan( (90+ current_oxts_1) * math.pi / 360 ))
			current_t_13 = oxts[i][2]
			t = [[current_t_11], [current_t_12], [current_t_13]]

			# print t
			# print
			# print i
			# print(oxts[i])
			rx = oxts[i][3]
			ry = oxts[i][4]
			rz = oxts[i][5]

			# print (rx)
			# print (ry)
			# print (rz)

			Rx = np.matrix([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
			Ry = np.matrix([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
			Rz = np.matrix([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
			R = np.empty((3,3,))
			R = np.dot(np.dot(Rz,Ry),Rx)

			# print (Rx)
			# print (Ry)
			# print (Rz)

			# print R
			# print

			current_matrix = np.zeros(shape = (4,4))
			first_three_row = np.concatenate ((R,t), axis =1)
			current_matrix = np.vstack([first_three_row, [0,0,0,1]])
			# print first_three_row
		
			if np.array_equal(Tr_0_inv,isempty):
				# print "enter if statement"
				# print i

				Tr_0_inv = inv(current_matrix)

			# if i == 0:
			# 	print Tr_0_inv
			# 	print four_rows
			current_pose = np.empty((4,4,))
			current_pose = Tr_0_inv.dot(current_matrix)
			pose[i] = current_pose


			# print i
			# print oxts[i]
			# print pose[i]

			# raw_input("press ehnter to continue")
# ---->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


# ---->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# start processing as of pose_T



# init pose
		# pose = 
		# subprocess.call(['spd-say','start publishing'])

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
		process()
	except rospy.ROSInterruptException:
		pass