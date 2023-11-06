#!/usr/bin/env python3
"""!
 @authors KinYip Chan (kinyip@i3drobotics.com)
 @date 2023-02-21
 @copyright Copyright (c) I3D Robotics Ltd, 2021
 @file phase_pub.py
 @brief ROS2 foxy phase stereo image publish
"""
# cd dev_ws
# colcon build --packages-select phase_rtabmap_ros2
# . install/setup.bash
# ros2 run phase_rtabmap_ros2 phase_camera_record

### With optional parameters ###
# ros2 run phase_rtabmap_ros2 phase_camera_record  left_serial:=40266661 right_serial:=40298125 camera_name:=746974616e24324 device_type:=titania interface_type:=usb exposure:=10000

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

# PhaseCameraNode class for camera data capture
class PhaseCameraNode(Node):

    def __init__(self):
        super().__init__("phase_camera_record")
        self.count_ = 0

        print ("Numpy version: " + str(np.__version__))
        print ("Numpy path: " + str(np.__path__))

        parser = argparse.ArgumentParser()
        parser.add_argument('--left_serial', type=str, default="40098270", help="Left Serial of Camera")
        parser.add_argument('--right_serial', type=str, default="40098281", help="Right Serial of Camera")
        parser.add_argument('--camera_name', type=str, default="746974616e24316", help="Camera Name of Camera")
        parser.add_argument('--device_type', type=str, default="titania", help="titania or phobos")
        parser.add_argument('--interface_type', type=str, default="usb", help="usb or gige")
        parser.add_argument('--exposure', type=int, default=25000, help="Exposure value")
        args, unknown = parser.parse_known_args()
        
        self.left_serial_ = args.left_serial
        self.right_serial_ = args.right_serial
        self.camera_name_ = args.camera_name

        if args.device_type == 'titania':
            self.device_type_ = phase.stereocamera.CameraDeviceType.DEVICE_TYPE_TITANIA
        elif args.device_type == 'phobos':
            self.device_type_ = phase.stereocamera.CameraDeviceType.DEVICE_TYPE_PHOBOS
        else:
            self.device_type_ = phase.stereocamera.CameraDeviceType.DEVICE_TYPE_GENERIC_PYLON
            
        if args.interface_type == 'usb':
            self.interface_type_ = phase.stereocamera.CameraInterfaceType.INTERFACE_TYPE_USB
        elif args.interface_type == 'gige':
            self.interface_type_ = phase.stereocamera.CameraInterfaceType.INTERFACE_TYPE_GIGE
        else:
            self.interface_type_ = phase.stereocamera.CameraInterfaceType.INTERFACE_TYPE_VIRTUAL

        self.cv_bridge = CvBridge()

        script_path = os.path.dirname(os.path.realpath(__file__))
        package_name = "phase_rtabmap_ros2"

        # Define calibration files
        
        # Define parameters for read process
        self.exposure_value_ = args.exposure

        device_info = phase.stereocamera.CameraDeviceInfo(
            self.left_serial_, self.right_serial_, self.camera_name_,
            self.device_type_, self.interface_type_
        )

        if args.device_type == 'titania':
            self.cam_ = phase.stereocamera.TitaniaStereoCamera(device_info)
        elif args.device_type == 'phobos':
            self.cam_ = phase.stereocamera.PhobosStereoCamera(device_info)

        ret = self.cam_.connect()
        if (ret):
            self.cam_.startCapture()

        self.pub_img_rawleft_ = self.create_publisher(Image, 'left/image_raw', 100)
        self.pub_img_rawright_ = self.create_publisher(Image, 'right/image_raw', 100)
        
        # Publish camera name message every 500ms
        self.timer_read = self.create_timer(0.1, self.read_frame)

    def read_frame(self):
        self.cam_.setExposure(self.exposure_value_)
        read_result = self.cam_.read()
        if (read_result.valid):
            self.get_logger().info("Stereo result received")

            header = Header()
            header.frame_id = "camera_link"
            header.stamp = self.get_clock().now().to_msg()

            self.publish_image(self.pub_img_rawleft_, read_result.left, header = header)
            self.publish_image(self.pub_img_rawright_, read_result.right, header = header)

        else:
            self.get_logger().warn("Failed to read stereo result")

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
    rclpy.shutdown()
    phase_camera_node.cam_.disconnect()


if __name__ == '__main__':
    main()