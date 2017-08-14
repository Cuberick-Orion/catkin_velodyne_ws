import rosbag
import sensor_msgs

def read_data (bag, topic, inputFileName):
	bag = rosbag.Bag('~/raw_data/kitti_2011_09_26_drive_0001_synced.bag')

	topic = "/kitti/velo/pointcloud"
	msg = "sensor_msgs/PointCloud2"

	for topic, msg, t in bag.read_messages():
		print msg

	bag.close()



if __name__ == '__main__':
	read_data ()