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
import sys
import os


def process():
	# pub = rospy.Publisher('velodyne_point_data', String, queue_size=10)
	# while not rospy.is_shutdown():
		rospy.init_node('test_velodyne',anonymous=True)

		bag_name = "kitti_2011_09_26_drive_0001_synced"

		bag_dir = "/home/cuberick/raw_data/rosbag/%s.bag" % (bag_name)

		bag = rosbag.Bag(bag_dir)

		interval = 1
		density = 50

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Read IMU-to-Velodyne Transformation Matrix
		tcount = 1
		print ("===============================================")
		print ("|             ---PROGRAM START---             |")
		print ("|                                             |")
		print ("|                                             |")
		print ("|                                             |")
		print ("|                   Ver.INS                   |")
		print ("|            by Cuberick.YuukiAsuna           |")
		print ("===============================================")
		print
		print

		sys.stdout.write("KITTI sequence: %s" % bag_name)
		print
		sys.stdout.write("Frame interval: %d, Point density: %d" %  (interval, density) )
		# sys.stdout.flush()
		print
		print
		print ("Bag LOADED, NERvGear START")
		print


		sys.stdout.write("\r>>>Read tf info")
		sys.stdout.flush()
		# print
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
		sys.stdout.write("\r   T_imu_to_velo obtained")
		sys.stdout.flush()
		# print ("   T_imu_to_velo obtained")
		# print
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Read OXTS data
		OXTS_GPS_raw = np.empty([1,3],dtype=float)
		gcount = 1
		sys.stdout.write("\r>>>Read OXTS GPS raw data")
		sys.stdout.flush()
		# print

		for topic, msg, t in bag.read_messages("/kitti/oxts/gps/fix"):
			# if gcount < 1:
			# 	break
			# gcount -= 1

			current_GPS_data = [msg.latitude, msg.longitude, msg.altitude]
			OXTS_GPS_raw = np.vstack([OXTS_GPS_raw , current_GPS_data])

		OXTS_GPS_raw = np.delete(OXTS_GPS_raw, (0), axis=0)	
		sys.stdout.write("\r   OSTX GPS raw data obtained")
		# print
		# print(OXTS_GPS_raw)

		sys.stdout.write("\r>>>Read OXTS IMU data")
		sys.stdout.flush()
		# print

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
		sys.stdout.write("\r   OXTS_IMU data obtained")
		sys.stdout.flush()
		# print

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

		pose_T = [None] * lengh_of_oxts

		for i in range(lengh_of_oxts-1):
			transfer_pose = np.empty((4,4,))

			# print (T_imu_to_velo_homo)
			# print
			# print(pose[i])
			transfer_pose = np.dot(T_imu_to_velo_homo, pose[i])

			pose_T[i] = np.empty((4,4,))
			pose_T[i] = transfer_pose


		

		

		frame = 0
		frame_count = 0
		frame_counts = 0
		total_frames = 0
		frames_left = 0
		skipped_count = 0
		rejected_count = 0
		# for frame in range(0,interval,len(pose_T)):
		# print pose
		


# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Read velodyne info
		sys.stdout.write("\r>>>Read Velodyne point data")
		sys.stdout.flush()
		# print

		all_points = np.empty([1,3],dtype=float)
		current_point_set = np.empty((999999,3,)) * np.NaN
		vcount = 5

		bag_count = -1
		for topic, msg, t in bag.read_messages("/kitti/velo/pointcloud"):

			# transformed_points = np.empty((1,3,))
			transformed_points = np.empty((999999,3,)) * np.NaN


			bag_count += 1
			if (bag_count) % interval != 0:
				continue

			# if vcount < 1:
			# 	break
			# vcount -= 1

			# print("counting cycles")
			
			# print vcount

			frame_count += 1
			total_frames = len(pose_T) / interval
			total_frames = math.ceil(total_frames)
			frames_left = total_frames - frame_count + 1

			info_of_frame = "Processing scan No.%d , %d remaining" % (frame_count,frames_left)
			sys.stdout.write("\r%s" % info_of_frame)
			sys.stdout.flush()
			# sys.stdout.write("    ~~~~~~working hard     >.<      please wait!~~~~~~~")
			# print

			# print vcount

			data_length = len(msg.data)
## msg is of type PointCloud2
			raw_data = pc2.read_points(msg)
	
			point_count_raw = 0
			for point in raw_data:
				current_point = [point[0], point[1], point[2]]
				# if point[0] > 4:
				try:
				# print point_count_raw
				# print current_point
					current_point_set[point_count_raw] = current_point
					point_count_raw += 1
				except:
					# print ".^.   skip recording this point"
					skipped_count += 1
					continue

			current_point_set = np.delete(current_point_set, (0), axis=0)
			current_point_set = current_point_set[~np.isnan(current_point_set).any(axis=1)]
			

			velo = current_point_set

			if np.shape(velo)[0] < 2:
				continue

			j = 0
			point_count = -1
			for j in range(np.shape(velo)[0]):
				# try:/
					point_count += 1
					if (point_count + 1 ) % density != 0:
						continue

					pose_a = pose_T[bag_count]

					# print pose_a
					point = velo[j]
					# print point
					a = type(point)
					# print a
					# print point
					point_a = point[np.newaxis, :].T
					# print point_a
					point_b = np.vstack([point_a, [1]])
					
					# print point_b
					# print j
					# print (pose_a)
					# print (point_b)
					# print

					point_c = np.dot(pose_a, point_b)
					point_c = point_c[np.newaxis, :].T
					# print point_c

					point_c = np.delete(point_c, [3], axis=1)
					# print point_c
					# a = type(point_c)
					# print a
					# print point_c
					# print transformed_points[j]
					# raw_input("press ehnter to continue")
					if (point_c[0,2] > -6) and (point_c[0,2] < 6):
						transformed_points[j] = point_c
					else:
						rejected_count += 1
						# print " O_o   point rejected due to range limit"
				# except:
					# print "except!!!"
					# continue
			# print "Cumulated rejected and skipped points:"
			# print (rejected_count + skipped_count)
			# print

			# transformed_points = np.delete(transformed_points, [3], axis=1)
			transformed_points = transformed_points[~np.isnan(transformed_points).any(axis=1)]
			all_points = np.vstack([all_points, transformed_points])
			all_points = np.delete(all_points, (0), axis=0)

			# print(all_points)
			# a = all_points.shape
			# print(a)





		sys.stdout.write("\rVelodyne point data processing finished")
		sys.stdout.flush()
		# print

		all_points = np.delete(all_points, (0), axis=0)
		# print all_points
		


# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

# ---->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



# init pose
		# pose = 
		# subprocess.call(['spd-say','start publishing'])

		
		header = std_msgs.msg.Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'map'

		fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('i', 12, PointField.FLOAT32, 1)]



		# print all_points
		# print
		# print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
		# prsint(">>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
		sys.stdout.write("\rProcessing completed, generating system report")
		print
		# sys.stdout.flush()
		# print
		# a = type(all_points)
		b = np.shape(all_points)
		# print a 
		sys.stdout.write("	Total points:")
		print b[0]
		sys.stdout.write("	Skipped points:")
		print skipped_count
		sys.stdout.write("	Rejected points:")
		print rejected_count
		print
		# print
		print ("Start visualising...")

		all_points = all_points.tolist()

		# print all_points

		processed_data = pc2.create_cloud_xyz32(header, all_points)
		# [[1, 1, 1]]
		# a = [[1, 1, 1]]
		# b = type(a)
		# print b


		pcl_pub = rospy.Publisher("/gps_visu", PointCloud2, queue_size = 10)
		rospy.loginfo("Publisher started at: /velodyne_pub")
		rospy.sleep(1.)
		rospy.loginfo("Publishing...")
		pcl_pub.publish(processed_data)


		
		bag.close()


if __name__ == '__main__':
	os.system('cls' if os.name == 'nt' else 'clear')
	try:
		process()
	except rospy.ROSInterruptException:
		pass