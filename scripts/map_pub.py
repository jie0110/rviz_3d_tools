#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import open3d as o3d

class MapPublisher:
    def __init__(self):
        rospy.init_node('map_publisher')
        self.pub = rospy.Publisher('/map', PointCloud2, queue_size=1)

    def map_pub(self, pcd_file_path):
        """读取PCD文件并发布点云消息"""
        try:
            # 使用Open3D读取PCD文件
            pcd = o3d.io.read_point_cloud(pcd_file_path)
            points = np.asarray(pcd.points).astype(np.float32)
            
            # 创建点云消息
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"  # 可根据需要修改frame_id
            
            # 定义字段
            fields = [
                pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1)
            ]
            
            # 创建PointCloud2消息
            pointcloud_msg = pc2.create_cloud(header, fields, points)
            
            # 发布消息
            rate = rospy.Rate(1)  # 1Hz
            while not rospy.is_shutdown():
                self.pub.publish(pointcloud_msg)
                rospy.loginfo("Published point cloud with %d points" % len(points))
                rate.sleep()
                
        except Exception as e:
            rospy.logerr("Error reading PCD file or publishing: %s" % str(e))

if __name__ == '__main__':
    map_publisher = MapPublisher()
    
    # 指定PCD文件路径
    pcd_file_path = "/home/jiewang/catkin_ws/src/rviz_3d_tools/map/output.pcd"  # 替换为实际的PCD文件路径
    
    try:
        map_publisher.map_pub(pcd_file_path)
    except rospy.ROSInterruptException:
        pass
