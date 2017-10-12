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
# require the installation of transforms3d
import transforms3d
# require the installation of sympy
import sympy as sp
# from sympy import *
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
		
# bag_name should be the same for both rosbag and gicp_simplified_result
		bag_name = "kitti_2011_09_26_drive_0005_synced"

		bag_dir = "/home/cuberick/raw_data/rosbag/%s.bag" % (bag_name)
		gicp_output_dir = "/home/cuberick/raw_data/gicp_simplified_output/%s.txt" % (bag_name)

		bag = rosbag.Bag(bag_dir)

		interval = 1
		density = 50
		duration = rospy.Duration(0.1,0)
		number_of_frame = 152
		# lengh_of_oxts = number_of_frame
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Read IMU-to-Velodyne Transformation Matrix
		tcount = 1
		print ("===============================================")
		print ("|             ---PROGRAM START---             |")
		print ("|                                             |")
		print ("|                                             |")
		print ("|                                             |")
		print ("|             Ver.EKF_incremental             |")
		print ("===============================================")
		print
		print

		sys.stdout.write("KITTI sequence: %s" % bag_name)
		print
		sys.stdout.write("Frame interval: %d, Point density: %d" %  (interval, density) )
		# sys.stdout.flush()
		print
		print
		print ("Bag LOADED")
		print ("Please launch rviz")
		print
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


# ---->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Obtain gicp results
		sys.stdout.write("\r>>>Read GICP raw data")
		sys.stdout.flush()
		gicp_raw = np.loadtxt(gicp_output_dir)

		gicp_row_length = np.shape(gicp_raw)[0]
		pose_icp = [None] * number_of_frame
		pose_icp[0] = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

		pose_icp_incr = [None] * number_of_frame
		pose_icp_incr[0] = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

		i = 0
		current_starting_row = 0
		current_ending_row = 0
		accumulate_tf = pose_icp[0]

		for i in range(number_of_frame - 1):
			
			current_starting_row = i * 4 + 0
			current_ending_row = i * 4 + 3
			current_pose_row_0 = gicp_raw[current_starting_row , :]
			current_pose_row_1 = gicp_raw[current_starting_row + 1 , :]
			current_pose_row_2 = gicp_raw[current_starting_row + 2 , :]
			current_pose_row_3 = gicp_raw[current_starting_row + 3 , :]
			current_pose = np.zeros(shape = (4,4))
			current = np.matrix([current_pose_row_0,current_pose_row_1,current_pose_row_2,current_pose_row_3])
			accumulate_tf = current.dot(accumulate_tf)

			pose_icp[i+1] = accumulate_tf
			pose_icp_incr[i+1] = current

			# print current_starting_row
		# print i

# ---->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# include Tr matrix

		pose_icp_T = [None] * number_of_frame
		pose_icp_incr_T = [None] * number_of_frame
		for i in range(number_of_frame-1):
			transfer_pose = np.empty((4,4,))

			# print (T_imu_to_velo_homo)
			# print
			# print(pose[i])
			transfer_pose = np.dot(T_imu_to_velo_homo, pose_icp[i])
			transfer_pose_incr = np.dot(T_imu_to_velo_homo, pose_icp_incr[i])

			pose_icp_T[i] = np.empty((4,4,))
			pose_icp_T[i] = transfer_pose
			pose_icp_incr_T[i] = np.empty((4,4,))
			pose_icp_incr_T[i] = transfer_pose_incr

		sys.stdout.write("\r   Pose_GICP data obtained")
		sys.stdout.flush()

		# rospy.sleep(0.5) # Sleeps for 1 sec
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Read OXTS data
		OXTS_GPS_raw = np.empty([1,3],dtype=float)
		gcount = 1
		sys.stdout.write("\r>>>Read OXTS GPS raw data")
		sys.stdout.flush()
		# print
		# rospy.sleep(0.5) # Sleeps for 1 sec

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



# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	# Calculate pose (loadOxtsliteData and convertOxtsToPose)

	# compute scale from first lat value
		oxts_first = OXTS_GPS_raw[0][0]
		scale = math.cos (oxts_first * math.pi / 180.00)
		# print scale

	# OXTS_GPS_raw [1] [2] [3] and OXTS_IMU_raw [1] [2] [3]
		oxts = np.concatenate ((OXTS_GPS_raw, OXTS_IMU_raw), axis = 1)
		# print oxts
		lengh_of_oxts = np.shape(oxts)[0]
		# print lengh_of_oxts

		pose_gps = [None] * lengh_of_oxts
		Tr_0_inv = np.zeros(shape = (4,4))
		isempty = np.zeros(shape = (4,4))
		# a = oxts[0]
		# print(a)

		i = 0
		for i in range(lengh_of_oxts-1):
			if oxts[i] == []:
				pose_gps[i] = np.empty((3,3,)) * np.nan
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
			pose_gps[i] = current_pose


			# print i
			# print oxts[i]
			# print pose[i]

			# raw_input("press ehnter to continue")
# ---->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# include Tr matrix

		pose_gps_T = [None] * lengh_of_oxts

		for i in range(lengh_of_oxts-1):
			transfer_pose = np.empty((4,4,))

			# print (T_imu_to_velo_homo)
			# print
			# print(pose[i])
			transfer_pose = np.dot(T_imu_to_velo_homo, pose_gps[i])

			pose_gps_T[i] = np.empty((4,4,))
			pose_gps_T[i] = transfer_pose


		
		sys.stdout.write("\r   Pose_GPS data obtained")
		sys.stdout.flush()
		

		frame = 0
		frame_count = 0
		frame_counts = 0
		total_frames = 0
		frames_left = 0
		skipped_count = 0
		rejected_count = 0
		# for frame in range(0,interval,len(pose_T)):

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Data summary:
    # pose_icp_T - gicp pose data with Tr transform
	# pose_icp_incr_T - incremental pose data
    # pose_gps_T - gps pose data with Tr transform
    # pose_T the original variable used, now it is modified
    # pose_T should be the ekf pose result
# Compare data size
		if len(pose_icp_T) < len(pose_gps_T):
			frameNo = len(pose_icp_T)
		else:
			frameNo = len(pose_gps_T)
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

# obtain z_k from gps information
		z = [None] * frameNo
		j  =  0
		for j in range(frameNo - 1):
			# print; print current_pose
			current_pose = pose_gps_T[j]
			current_rotation_matrix = current_pose[0:3,0:3]
			current_rotation_euler = transforms3d.euler.mat2euler(current_rotation_matrix)
			# z[j] = np.asmatrix(current_rotation_euler)
			z[j] = np.matrix([  [current_pose[0,3]],[current_pose[1,3]],[current_pose[2,3]]  ])
			# print current_pose

# obtain x_k from icp incr
		x = [None] * frameNo
		j = 0
		for j in range(frameNo-1):
			current_pose = pose_icp_incr[j]
			current_rotation_matrix = current_pose[0:3,0:3]
			current_rotation_euler = transforms3d.euler.mat2euler(current_rotation_matrix)
			current_rotation_euler_m = np.asmatrix(current_rotation_euler)
			x[j] = np.matrix([  [current_pose[0,3]],[current_pose[1,3]],[current_pose[2,3]],
								[current_rotation_euler_m[0,2]],[current_rotation_euler_m[0,1]],[current_rotation_euler_m[0,0]]  ])

		sys.stdout.write("\r>>>start EKF")
		sys.stdout.flush()
# EKF implementation
	
		P = [None] * frameNo
		G = [None] * frameNo
		F = [None] * frameNo
		S = [None] * frameNo
		V = [None] * frameNo
		X = [None] * frameNo

		H = np.matrix([ [1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0] ])
		# H = np.asmatrix(H)
		Q = np.diag([10**2, 10**2, 10**2, 0.7**2, 0.7**2, 0.7**2])
		Q = np.asmatrix(Q)
		Q = Q * 0.1

		R = np.diag([1, 1, 1])
		R = np.asmatrix(R)

		rx,ry,rz,delta_x,delta_y,delta_z = sp.symbols('rx,ry,rz,delta_x,delta_y,delta_z')
		# d_R11, d_R12, d_R13, d_R21, d_R22, d_R23, d_R31, d_R32, d_R33

		Rx = sp.Matrix([[1,0,0], [0,sp.cos(rx),-sp.sin(rx)], [0,sp.sin(rx),sp.cos(rx)]])
		Ry = sp.Matrix([[sp.cos(ry),0,sp.sin(ry)], [0,1,0], [-sp.sin(ry),0,sp.cos(ry)]])
		Rz = sp.Matrix([[sp.cos(rz),-sp.sin(rz),0], [sp.sin(rz),sp.cos(rz),0], [0,0,1]])
		R_matrix = Rz*Ry*Rx

		A = sp.Matrix([[delta_x], [delta_y], [delta_z]])
		F_top_right_de_ja = (R_matrix * A).T
		F_top_right = F_top_right_de_ja.jacobian([rz,ry,rx])

		F_template = sp.Matrix([[1,0,0,F_top_right[0,0],F_top_right[0,1],F_top_right[0,2]],
								[0,1,0,F_top_right[1,0],F_top_right[1,1],F_top_right[1,2]],
								[0,0,1,F_top_right[2,0],F_top_right[2,1],F_top_right[2,2]],
								[0,0,0,0,0,0],
								[0,0,0,0,0,0],
								[0,0,0,0,0,0]])

		F[0] = np.asmatrix(np.eye(6))

		P[0] = np.asmatrix(np.eye(6))
		# P[0] = np.asmatrix(P[0])

		I = np.eye(6)
		I = np.asmatrix(I)
		accumulated_R = np.asmatrix(np.eye(3))
		X[0] = x[0]

		k = 1
		for k in range(1,frameNo-1):
    			# in MATLAB, inverse is ^(-1), transpose is .'
				# matrix multiplication is easily implemented as *
				# as long as the variables are of type np.matrix
			# print
			# print k
			# print X[k-1]

			# print; print x[k-1]
			x_down_three = X[k-1][3:6,0] + x[k][3:6,0]
			x_up_three = pose_icp[k][0:3,3]
			# print;print x_up_three
			X[k] = np.concatenate((x_up_three,x_down_three), axis=0)
			X[k] = np.asmatrix(X[k])
			# print X[k]
			P[k] = F[k-1] * P[k-1] * F[k-1].T + Q
			# print P[k]
			# print; print temp_transpose
			S[k] = H * P[k] * H.T + R
			G[k] = P[k] * H.T * (S[k])**(-1)

			V[k] = z[k] - H * X[k]
			X[k] = X[k] + G[k]*V[k]
			# print X[k]
			P[k] = (I - G[k] * H) * P[k]
			# P[k+1] = P[k] + Q
			# print;print P[k]
			delta_x_u = x[k][0,0]
			delta_y_u = x[k][1,0]
			delta_z_u = x[k][2,0]
			rz_u = X[k][3,0]
			ry_u = X[k][4,0]
			rx_u = X[k][5,0]

			F[k] = F_template.subs({rx:rx_u, ry:ry_u, rz:rz_u, delta_x:delta_x_u, delta_y:delta_y_u, delta_z:delta_z_u})
			# print F[k]
		# print P

		sys.stdout.write("\r   EKF processed")
		sys.stdout.flush()

# calculate pose for ekf
		pose_ekf = [None] * frameNo
		j = 0
		for j in range(frameNo-1):
			temp_r = transforms3d.euler.euler2mat(X[j][5,0],X[j][4,0],X[j][3,0])
			pose_ekf[j] = np.asmatrix( np.concatenate( (temp_r,X[j][0:3,0]), axis=1)  )
			a = np.matrix([[0,0,0,1]])
			pose_ekf[j] = np.concatenate( (pose_ekf[j],a), axis=0)
			# print; print pose_ekf[j]
			# pose_ekf[j] = np.matrix([ [temp_kf[3,0],temp_kf[4,0],temp_kf[5,0],temp_kf[0,0]],
			# 	[temp_kf[6,0],temp_kf[7,0],temp_kf[8,0],temp_kf[1,0]],
			# 	[temp_kf[9,0],temp_kf[10,0],temp_kf[11,0],temp_kf[2,0]],
			# 	[0,0,0,1] ])

		# print pose_ekf
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Read velodyne info
		sys.stdout.write("\r>>>Read Velodyne point data")
		sys.stdout.flush()
		print;print

		# all_points = np.empty([1,3],dtype=float)
		current_point_set = np.empty((999999,3,)) * np.NaN
		vcount = 5

		bag_count = -1
		total_msg_no = 0
		for topic, msg, t in bag.read_messages("/kitti/velo/pointcloud"):
			# bag_count += 1
			# if (bag_count) % interval != 0:
			# 	continue
			total_msg_no += 1

		all_points = [np.empty([1,4],dtype=float)] * total_msg_no

		for topic, msg, t in bag.read_messages("/kitti/velo/pointcloud"):
    
			# transformed_points = np.empty((1,3,))
			transformed_points = np.empty((999999,3,)) * np.NaN


			bag_count += 1
			# if (bag_count) % interval != 0:
			# 	continue

			# if vcount < 1:
			# 	break
			# vcount -= 1

			# print("counting cycles")
			
			# print vcount

			frame_count += 1
			total_frames = len(pose_ekf) / interval
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
				try:
					point_count += 1
					if (point_count + 1 ) % density != 0:
						continue
					# print;print pose_ekf
					pose_a = pose_ekf[bag_count]

					point = velo[j]

					point_a = point[np.newaxis, :].T
					# print point_a
					point_b = np.vstack([point_a, [1]])
					
					point_c = np.dot(pose_a, point_b)
					point_c = point_c[np.newaxis, :].T

					point_c = np.delete(point_c, [3], axis=1)
					# print; print point_c
					transformed_points[j] = point_c

				except:
					# print;print "except"
					continue
			
			transformed_points = transformed_points[~np.isnan(transformed_points).any(axis=1)]
			# print; print transformed_points
			try:
				transformed_points = np.delete(transformed_points, (0), axis=0)
			except:
				continue

			all_points[frame_count-1] = transformed_points
			all_points[frame_count-1] = np.delete(all_points[frame_count-1], (0), axis=0)
			# all_points = np.vstack([all_points, transformed_points])
			# all_points = np.delete(all_points, (0), axis=0)

			# print(all_points)
			# a = all_points.shape
			# print(a)
			# print frame_count




		sys.stdout.write("\rVelodyne point data processing finished")
		sys.stdout.flush()

		# bag.close()


# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

# ---->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



		# pose = 
		# subprocess.call(['spd-say','start publishing'])

		# print all_points
		# print
		# print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
		# prsint(">>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
		sys.stdout.write("\rProcessing completed, generating system report")
		# print
		# sys.stdout.flush()
		# print
		# a = type(all_points)
		b = np.shape(all_points)
		# print;print a 
		sys.stdout.write("	Total frames:")
		print b[0]
		sys.stdout.write("	Skipped points:")
		print skipped_count
		sys.stdout.write("	Rejected points:")
		print rejected_count
		print
		# print
		# print ("Start visualising...")

		pcl_pub = rospy.Publisher("/ekf_visu", PointCloud2, queue_size = 10)
		rospy.loginfo("Publisher started at: /ekf_visu")
		rospy.sleep(1.)
		rospy.loginfo("Publishing...")
		print
		print
		print

		bag.close()

		current_visual_set = np.empty([1,3])

		while (1):
			raw_input("\r... waiting for instruction")
			# sys.stdout.write("Start visualising...")
			sys.stdout.flush()

			current_visual_set = np.empty([1,3])

			k = 0
			for k in range(total_msg_no):

				sys.stdout.write("\rVisualising frame %d" %k)
				sys.stdout.flush()

				header = std_msgs.msg.Header()
				header.stamp = rospy.Time.now()
				header.frame_id = 'map'

				fields = [PointField('x', 0, PointField.FLOAT32, 1),
		                  PointField('y', 4, PointField.FLOAT32, 1),
		                  PointField('z', 8, PointField.FLOAT32, 1),
		                  PointField('i', 12, PointField.FLOAT32, 1)]
				try:
					current_visual_set = np.concatenate((current_visual_set, all_points[k]))
					# a = type(current_visual_set)
					# print;print a 
				except:
					continue
				current_visual_set_list = current_visual_set.tolist()

				# print all_points

				processed_data = pc2.create_cloud_xyz32(header, current_visual_set_list)
				rospy.sleep(duration)
				# [[1, 1, 1]]
				# a = [[1, 1, 1]]
				# b = type(a)
				# print b


				
				pcl_pub.publish(processed_data)

			sys.stdout.write("\rVisualisation complete")
			sys.stdout.flush()
			# print
			# print


		
		

# def publish_odom(self,rtom_list):
	

if __name__ == '__main__':
	os.system('cls' if os.name == 'nt' else 'clear')
	try:
		process()
	except rospy.ROSInterruptException:
		pass