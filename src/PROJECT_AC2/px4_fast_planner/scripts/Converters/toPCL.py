import rospy
import rosbag
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from tqdm import tqdm

def bag_to_ply(bag_file_path, ply_file_path):
    point_clouds = []
    total_points = 0

    # Read the bag file and count total points
    with rosbag.Bag(bag_file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/parsed_point_cloud']):
            total_points += 1

        # Write point cloud data to PLY file
        print("Writing PLY file:", ply_file_path)
        with open(ply_file_path, 'w') as ply_file:
            ply_file.write("ply\n")
            ply_file.write("format ascii 1.0\n")
            ply_file.write("element vertex {}\n".format(total_points))
            ply_file.write("property float x\n")
            ply_file.write("property float y\n")
            ply_file.write("property float z\n")
            ply_file.write("end_header\n")

            # Convert point cloud messages to PLY format
            progress_bar = tqdm(total=total_points, desc="Converting to PLY", unit=" points")
            for topic, msg, t in bag.read_messages(topics=['/parsed_point_cloud']):
                for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                    ply_file.write("{} {} {}\n".format(point[0], point[1], point[2]))
                    progress_bar.update(1)
            progress_bar.close()

# Usage
bag_file_path = '/home/hurkan/RAT_ws/src/PROJECT_AC2/px4_fast_planner/scripts/slam_sc2_fr3_1.bag'
ply_file_path = '/home/hurkan/RAT_ws/src/PROJECT_AC2/px4_fast_planner/scripts/slam_sc2_fr3_1.ply'
bag_to_ply(bag_file_path, ply_file_path)
