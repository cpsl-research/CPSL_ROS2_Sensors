#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import numpy as np

class CameraInfoPrinter(Node):
    def __init__(self):
        super().__init__('camera_info_printer')

        # Change this to the topic you want (color, depth, aligned depth)
        self.camera_info_topic = '/cpsl_human_movement/camera/cpsl_realsense/color/camera_info'

        self.subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def camera_info_callback(self, msg: CameraInfo):
        # Print basic info
        self.get_logger().info(f'Camera Info from topic: {self.camera_info_topic}')
        self.get_logger().info(f'Image size: {msg.width} x {msg.height} pixels')
        self.get_logger().info(f'Distortion model: {msg.distortion_model}')
        self.get_logger().info(f'Distortion coefficients: {msg.d}')

        # Convert arrays to matrices
        K = np.array(msg.k).reshape(3, 3)
        R = np.array(msg.r).reshape(3, 3)
        P = np.array(msg.p).reshape(3, 4)

        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]

        self.get_logger().info(f'\nIntrinsic matrix K:\n{K}')
        self.get_logger().info(f'Focal lengths: fx={fx}, fy={fy} pixels')
        self.get_logger().info(f'Principal point: cx={cx}, cy={cy} pixels')

        self.get_logger().info(f'\nRectification matrix R:\n{R}')
        self.get_logger().info(f'\nProjection matrix P:\n{P}')

        self.get_logger().info(f'\nBinning: x={msg.binning_x}, y={msg.binning_y}')
        roi = msg.roi
        self.get_logger().info(f'ROI: x_offset={roi.x_offset}, y_offset={roi.y_offset}, '
                               f'width={roi.width}, height={roi.height}, do_rectify={roi.do_rectify}')

        # Optional: unsubscribe after first message
        self.subscription.destroy()
        self.get_logger().info('All camera info printed. Shutting down node.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPrinter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
