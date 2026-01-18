#!/usr/bin/env python3

"""
LeRobot ROS 2 Inference Node
"""

import rclpy
from pai_bringup.lerobot_infer import LeRobotInfer


def main(args=None):
    rclpy.init(args=args)
    node = LeRobotInfer("lerobot_infer")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
