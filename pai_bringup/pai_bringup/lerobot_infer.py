#!/usr/bin/env python3

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from copy import copy
from contextlib import nullcontext
import cv2
import numpy as np
from numpy.typing import NDArray
import torch

class LeRobotInfer(Node):
    """
    LeRobot Infer Node
    It uses the inference of LeRobot VLA models
    """
    def __init__(self, node_name: str):
        """
        Creates the ros2 node for lerobot inference
        """
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter('policy_path', 'outputs/train/act_move_to_cube/checkpoints/040000/pretrained_model')
        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('command_topic', '/forward_position_controller/commands')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('use_amp', False)
        self.declare_parameter('task', 'Pick up the blue cube')
        self.declare_parameter('robot_type', 'so101_follower')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('use_degrees_training', False)  # Was the model trained with use_degrees=True?

        # Get parameters
        self.policy_path = self.get_parameter('policy_path').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.device = torch.device(self.get_parameter('device').get_parameter_value().string_value)
        self.use_amp = self.get_parameter('use_amp').get_parameter_value().bool_value
        self.task = self.get_parameter('task').get_parameter_value().string_value
        self.robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.camera_width = self.get_parameter('camera_width').get_parameter_value().integer_value
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().integer_value
        self.use_degrees_training = self.get_parameter('use_degrees_training').get_parameter_value().bool_value

        self.get_logger().info(f'Loading LeRobot policy from: {self.policy_path}')

        # Initialize CV bridge for image conversion
        self.cv_bridge = CvBridge()

        # Create subscribers and publishers
        self.camera_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            10
        )

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            self.command_topic,
            10
        )

        self.action_features = [
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_pitch_joint', 'wrist_roll_joint', 'jaw_joint'
            ]

        # Control rate limiting
        self.last_inference_time = self.get_clock().now()
        self.inference_period = 1.0 / self.fps  # seconds

        self.get_logger().info('LeRobot inference node initialized')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        self.get_logger().info(f'Command topic: {self.command_topic}')
        self.get_logger().info(f'Target FPS: {self.fps}')
        self.get_logger().info(f'Training mode - use_degrees: {self.use_degrees_training}')

        # Important calibration warning
        self.get_logger().warn('IMPORTANT: Joint limits are hardcoded approximations!')
        self.get_logger().warn('Please verify these match your robot\'s actual calibrated ranges.')
        self.get_logger().warn('Incorrect limits may cause unsafe robot movements!')
    
    def camera_callback(self, msg: Image):
        """Callback function for camera data."""
        current_time = self.get_clock().now()

        # Rate limiting based on target FPS
        time_since_last = (current_time - self.last_inference_time).nanoseconds / 1e9
        if time_since_last < self.inference_period:
            return

        self.last_inference_time = current_time

        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Resize image to expected dimensions if needed
            if cv_image.shape[:2] != (self.camera_height, self.camera_width):
                cv_image = cv2.resize(cv_image, (self.camera_width, self.camera_height))

            # Run inference
            # joint_positions = self._run_inference(cv_image)

            # Publish command
            # self._publish_command(joint_positions)

        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {str(e)}')

    def _publish_command(self, joint_positions: NDArray):
        """Publish joint position commands."""
        try:
            # Convert LeRobot action values to radians
            radians_positions = self._convert_lerobot_to_radians(joint_positions)

            # Create Float64MultiArray message
            cmd_msg = Float64MultiArray()
            cmd_msg.data = radians_positions.tolist()

            # Publish the command
            self.command_pub.publish(cmd_msg)

            # Log the published values (for debugging)
            joint_values_lerobot = ", ".join([f"{name}: {val:.3f}" for name, val in zip(self.action_features, joint_positions)])
            joint_values_radians = ", ".join([f"{name}: {val:.3f}" for name, val in zip(self.action_features, radians_positions)])
            self.get_logger().info(f'LeRobot values: {joint_values_lerobot}')
            self.get_logger().info(f'Published radians: {joint_values_radians}')

        except Exception as e:
            self.get_logger().error(f'Error publishing command: {str(e)}')

    def _convert_lerobot_to_radians(self, lerobot_actions: np.ndarray) -> np.ndarray:
        """
        Convert LeRobot action values to radians for ROS 2 joint commands.

        LeRobot SO101 has two normalization modes:
        - MotorNormMode.RANGE_M100_100 (default, use_degrees=False): actions in [-100, 100] range
        - MotorNormMode.DEGREES (use_degrees=True): actions directly in degrees

        ROS 2 joint_position_controller expects radians.

        IMPORTANT: These joint limits should match your robot's actual calibrated ranges!
        You may need to adjust these based on your specific robot calibration.
        """
        # Typical SO101 joint limits in degrees
        # WARNING: These are approximate! Adjust based on your robot's calibration
        joint_limits_deg = {
            'shoulder_pan_joint': (-180, 180),      # Full rotation
            'shoulder_lift_joint': (-90, 90),       # Shoulder lift range
            'elbow_joint': (-135, 135),              # Elbow range
            'wrist_pitch_joint': (-90, 90),         # Wrist pitch
            'wrist_roll_joint': (-180, 180),        # Wrist roll full rotation
            'jaw_joint': (0, 100)                   # Gripper 0-100% (special case)
        }

        radians_output = np.zeros_like(lerobot_actions)

        for i, joint_name in enumerate(self.action_features):
            lerobot_value = lerobot_actions[i]

            if joint_name == 'jaw_joint':
                # Gripper handling - always normalize to [0, 1] range for ROS 2
                if self.use_degrees_training:
                    # If trained with degrees mode, gripper was in [0, 100] range
                    normalized_gripper = lerobot_value / 100.0
                else:
                    # If trained with [-100, 100] range, convert to [0, 1]
                    normalized_gripper = (lerobot_value + 100) / 200.0
                radians_output[i] = np.clip(normalized_gripper, 0.0, 1.0)
            else:
                # Regular joints
                min_deg, max_deg = joint_limits_deg[joint_name]

                if self.use_degrees_training:
                    # Model was trained with degrees directly
                    degrees_value = lerobot_value
                else:
                    # Model was trained with [-100, 100] normalized range
                    # Convert from [-100, 100] to [min_deg, max_deg]
                    degrees_value = ((lerobot_value + 100) / 200.0) * (max_deg - min_deg) + min_deg

                # Convert degrees to radians
                radians_output[i] = np.radians(degrees_value)

        return radians_output
    
    def _predict_action(self, observation: dict[str, np.ndarray]) -> np.ndarray:
        """
        Predict action using the loaded policy.
        Based on lerobot.utils.control_utils.predict_action
        """
        observation = copy(observation)

        with (
            torch.inference_mode(),
            torch.autocast(device_type=self.device.type) if self.device.type == "cuda" and self.use_amp else nullcontext(),
        ):
            # Convert to pytorch format: channel first and float32 in [0,1] with batch dimension
            for name in observation:
                observation[name] = torch.from_numpy(observation[name])
                if "image" in name or "images" in name:  # Handle camera data
                    observation[name] = observation[name].type(torch.float32) / 255
                    observation[name] = observation[name].permute(2, 0, 1).contiguous()
                observation[name] = observation[name].unsqueeze(0)
                observation[name] = observation[name].to(self.device)

            # Compute the next action with the policy
            action = self.policy.select_action(observation)

            # Remove batch dimension and convert to numpy
            action = action.squeeze(0).cpu().numpy()

            return action
