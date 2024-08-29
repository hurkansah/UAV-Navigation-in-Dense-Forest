import struct
import rosbag
from sensor_msgs.msg import PointCloud2, PointField
import rospy
import sys
from tqdm import tqdm

def parse_osa_file(file_path):
    point_clouds = []

    with open(file_path, 'rb') as f:
        while True:
            # Read a block of binary data from the file
            data = f.read(28)  # Assuming each record is 28 bytes

            # Break the loop if no more data is read
            if not data:
                break

            # Unpack the binary data into floats
            try:
                x, y, z = struct.unpack('fff', data[:12])
                # Other data parsing logic can be added here based on the actual structure
            except struct.error:
                print("Error unpacking data:", data)
                continue

            # Create a point cloud tuple and append it to the list
            point_clouds.append((x, y, z))

    return point_clouds

def save_to_bag(point_clouds, bag_file_path):
    with rosbag.Bag(bag_file_path, 'w') as bag:
        for i, point_cloud in enumerate(tqdm(point_clouds, desc='Writing ROS Bag')):
            header = rospy.Header(frame_id="map")
            fields = [
                PointField(name="x", offset=0, datatype=7, count=1),
                PointField(name="y", offset=4, datatype=7, count=1),
                PointField(name="z", offset=8, datatype=7, count=1)
            ]
            msg = PointCloud2(
                header=header,
                height=1,
                width=1,
                fields=fields,
                is_bigendian=False,
                point_step=12,
                row_step=12,
                data=struct.pack('fff', *point_cloud)
            )
            bag.write('/parsed_point_cloud', msg)

# Usage
osa_file_path = '/home/hurkan/RAT_ws/src/PROJECT_AC2/px4_fast_planner/scripts/slam_sc2_fr3_1.osa' 
bag_file_path = '/home/hurkan/RAT_ws/src/PROJECT_AC2/px4_fast_planner/scripts/slam_sc2_fr3_1.bag'
point_clouds = parse_osa_file(osa_file_path)
save_to_bag(point_clouds, bag_file_path)



# osa_file_path = '/home/hurkan/RAT_ws/src/PROJECT_AC2/px4_fast_planner/scripts/slam_sc2_fr3_1.osa'


   #osa_file_path = '/home/hurkan/RAT_ws/src/PROJECT_AC2/px4_fast_planner/scripts/slam_sc2_fr3_1.osa'  # Replace with the actual path to your .osa file
    #bag_file_path = '/home/hurkan/RAT_ws/src/PROJECT_AC2/px4_fast_planner/scripts/slam_sc2_fr3_1.bag'   # Replace with the desired path to save the bag file
    