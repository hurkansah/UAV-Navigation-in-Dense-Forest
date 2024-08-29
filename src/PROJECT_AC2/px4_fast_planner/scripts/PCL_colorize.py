#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import open3d as o3d
from sensor_msgs import point_cloud2
import struct

def colorize_point_cloud():
    rospy.init_node('colorize_point_cloud', anonymous=True)
    
    # Publisher to publish the colored point cloud
    pub = rospy.Publisher('/orb_slam3/all_points_colored', PointCloud2, queue_size=10)
    
    # Subscriber to listen to the input point cloud
    def callback(data):
        # Convert PointCloud2 message to open3d point cloud
        points = list(point_cloud2.read_points(data))
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        
        # Colorize the point cloud
        colors = [[255, 0, 0] for _ in range(len(points))]  # Red color
        cloud.colors = o3d.utility.Vector3dVector(colors)
        
        # Convert back to PointCloud2 message
        header = Header()
        header.frame_id = data.header.frame_id
        
        # Create the binary data for the PointCloud2 message
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgb', 12, PointField.FLOAT32, 1)]
        data_rgb = []
        for p, c in zip(points, colors):
            packed_rgb = struct.pack('I', c[2] | (c[1] << 8) | (c[0] << 16))
            data_rgb.extend(struct.pack('ffffI', *p, struct.unpack('f', packed_rgb)[0]))
        
        pc2_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=16,
            row_step=len(points) * 16,
            data=bytes(data_rgb)
        )
        
        # Publish the colored point cloud
        pub.publish(pc2_msg)
    
    rospy.Subscriber('/orb_slam3/all_points', PointCloud2, callback)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    colorize_point_cloud()
