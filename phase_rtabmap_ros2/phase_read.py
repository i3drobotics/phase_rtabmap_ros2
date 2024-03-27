#!/usr/bin/env python3
"""!
 @authors KinYip Chan (kinyip@i3drobotics.com)
 @date 2023-02-21
 @copyright Copyright (c) I3D Robotics Ltd, 2021
 @file phase_pub.py
 @brief ROS2 humble phase stereo image publish
"""
# cd dev_ws
# colcon build --packages-select phase_rtabmap_ros2
# . install/setup.bash
# ros2 run phase_rtabmap_ros2 phase_camera 
# ros2 run phase_rtabmap_ros2 phase_camera  left_serial:=40266661 right_serial:=40298125 camera_name:=746974616e24324 device_type:=titania interface_type:= usb exposure:=10000

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2, PointField
from cv_bridge import CvBridge

import phase.pyphase as phase

import os
import numpy as np
import yaml
import argparse
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber
import message_filters

# PhaseCameraNode class for camera data capture
class PhaseCameraNode(Node):

    def __init__(self):
        super().__init__("phase_read")
        self.count_ = 0
        self.node = rclpy.create_node("phase_read")

        self.cv_bridge = CvBridge()

        script_path = os.path.dirname(os.path.realpath(__file__))
        home_path = os.path.expanduser('~')
        package_name = "phase_rtabmap_ros2"

        left_yaml_name = "left.yaml"
        right_yaml_name = "right.yaml"

        # Define calibration files
        cal_folder = os.path.join(home_path, "ros2_ws", "install", package_name, "share", package_name, "cal")
        left_yaml = os.path.join(cal_folder, left_yaml_name)
        right_yaml = os.path.join(cal_folder, right_yaml_name)
        # self.imageL = message_filters.Subscriber("/left/camera_info", CameraInfo)
        # self.imageR = message_filters.Subscriber("/right/camera_info", CameraInfo)
        
        # Check for I3DRSGM license
        license_valid = phase.stereomatcher.StereoI3DRSGM().isLicenseValid()
        if license_valid:
            print("I3DRSGM license accepted")
            stereo_params = phase.stereomatcher.StereoParams(
                phase.stereomatcher.StereoMatcherType.STEREO_MATCHER_I3DRSGM,
                9, 0, 49, False
            )
        else:
            print("Missing or invalid I3DRSGM license. Will use StereoBM")
            stereo_params = phase.stereomatcher.StereoParams(
                phase.stereomatcher.StereoMatcherType.STEREO_MATCHER_BM,
                17, 0, 48, True
            )

        # Load calibration
        self.calibration_ = phase.calib.StereoCameraCalibration.calibrationFromYAML(
            left_yaml, right_yaml)

        # Create stereo matcher
        self.matcher_ = phase.stereomatcher.createStereoMatcher(stereo_params)

        self.left_camerainfo_ = self.yaml_to_camerainfo(left_yaml)
        self.right_camerainfo_ = self.yaml_to_camerainfo(right_yaml)

        # self.imageL = self.create_subscription(Image, "camera2/left/image_raw", self.read_frame,10)
        # self.imageR = self.create_subscription(Image, "camera2/right/image_raw", self.read_frame,10)

        self.pub_caminfo_left_ = self.create_publisher(CameraInfo, 'left/camera_info', 100)
        self.pub_caminfo_right_ = self.create_publisher(CameraInfo, 'right/camera_info', 100)
        self.pub_img_left_ = self.create_publisher(Image, 'left/image_rect', 100)
        self.pub_img_right_ = self.create_publisher(Image, 'right/image_rect', 100)
        self.pub_depth_ = self.create_publisher(Image, 'depth/image', 100)

        self.pub_caminfo_left_ = self.create_publisher(CameraInfo, 'left/camera_info', 100)
        self.pub_caminfo_right_ = self.create_publisher(CameraInfo, 'right/camera_info', 100)

        imageL = Subscriber(self, Image, "left/image_raw")
        imageR = Subscriber(self, Image, "right/image_raw")

        # ts = ApproximateTimeSynchronizer([imageL,imageR], queue_size=10, slop=0.5, allow_headerless=True)
        # # # Synchronize images
        ts = TimeSynchronizer([imageL, imageR], 10)
        ts.registerCallback(self.read_frame)
        
        # # Publish camera name message every 500ms
        # self.timer_read = self.create_timer(0.1, self.read_frame)

    def yaml_to_camerainfo(self, yaml_file_path):
        # Load data from file
        with open(yaml_file_path, "r") as file_handle:
            calib_data = yaml.load(file_handle, Loader=yaml.FullLoader)
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.k = self.array_to_float(calib_data["camera_matrix"]["data"])
        camera_info_msg.d = self.array_to_float(calib_data["distortion_coefficients"]["data"])
        camera_info_msg.r = self.array_to_float(calib_data["rectification_matrix"]["data"])
        camera_info_msg.p = self.array_to_float(calib_data["projection_matrix"]["data"])
        camera_info_msg.distortion_model = calib_data["distortion_model"]

        return camera_info_msg

    def array_to_float(self, arr):
        arr_temp = []
        for item in arr:
            arr_temp.append(float(item))
        return arr_temp

    def point_cloud(self, points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx3 array of xyz positions.
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        Code source:
            https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
        References:
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
            http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
        """
        # In a PointCloud2 message, the point cloud is stored as an byte 
        # array. In order to unpack it, we also include some parameters 
        # which desribes the size of each individual point.
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        # fields = [PointField(
        #     name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        #     for i, n in enumerate('xyz')]
        fields = [PointField(name='x', offset=0, datatype=ros_dtype, count=1),
            PointField(name='y', offset=4, datatype=ros_dtype, count=1),
            PointField(name='z', offset=8, datatype=ros_dtype, count=1),
            PointField(name='r', offset=12, datatype=ros_dtype, count=1),
            PointField(name='g', offset=16, datatype=ros_dtype, count=1),
            PointField(name='b', offset=20, datatype=ros_dtype, count=1)]

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = Header(frame_id=parent_frame)

        return PointCloud2(
            header=header,
            height=1, 
            width=len(points),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 6), # Every point consists of three float32s.
            row_step=(itemsize * 6 * points.shape[0]),
            data=data
        )

    def read_frame(self, imageL, imageR):
        br = CvBridge()
        imageLeft = br.imgmsg_to_cv2(imageL)
        imageRight = br.imgmsg_to_cv2(imageR)
        self.get_logger().info("Receiving Frames")
        rect_image_pair = self.calibration_.rectify(imageLeft, imageRight)
        rect_img_left = rect_image_pair.left
        rect_img_right = rect_image_pair.right

        match_result = self.matcher_.compute(rect_img_left, rect_img_right)

        # Check compute is valid
        if not match_result.valid:
            print("Failed to compute match")

        # Find the disparity from matcher
        disparity = match_result.disparity
        depth = phase.disparity2depth(disparity, self.calibration_.getQ())

        header = Header()
        header.frame_id = "camera_link"
        header.stamp = self.get_clock().now().to_msg()

        # header_pc = Header()
        # #header_pc.frame_id = "map"
        # header_pc.stamp = header.stamp

        # self.pc_msg = self.point_cloud(points, 'map')

        self.left_camerainfo_.header = header
        self.right_camerainfo_.header = header
        #self.pc_msg.header = header_pc

        # self.publish_image(self.pub_img_rawleft_, imageLeft, header = header)
        # self.publish_image(self.pub_img_rawright_, imageRight.right, header = header)
        self.publish_image(self.pub_img_left_, rect_img_left, header = header)
        self.publish_image(self.pub_img_right_, rect_img_right, header = header)
        self.publish_image(self.pub_depth_, depth, encoding = "32FC1", header = header)
        # self.pub_pointcloud_.publish(self.pc_msg)
        self.pub_caminfo_left_.publish(self.left_camerainfo_)
        self.pub_caminfo_right_.publish(self.right_camerainfo_)


    def publish_image(self,
            image_pub, image,
            encoding = "bgr8", header = Header):
        msg = self.cv_bridge.cv2_to_imgmsg(image, encoding=encoding, header=header)
        image_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    phase_camera_node = PhaseCameraNode()

    rclpy.spin(phase_camera_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    phase_camera_node.destroy_node()
    phase_camera_node.cam_.disconnect()
    rclpy.shutdown()

    # os.system("rtabmap-export --images --poses_camera --poses_format 11 ~/.ros/rtabmap.db")


if __name__ == '__main__':
    main()