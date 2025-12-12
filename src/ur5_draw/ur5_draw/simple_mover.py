#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SimpleMover(Node):
    def __init__(self):
        super().__init__("simple_mover")

        # Connect to the simulation controller
        self.pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        # Wait a moment for connection
        self.timer = self.create_timer(2.0, self.move_robot)
        self.get_logger().info("Simple Mover Node Started. Waiting to send command...")

    def move_robot(self):
        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Create a waypoint: "Ready Pose"
        # This lifts the robot UP and points the wrist DOWN
        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        point.time_from_start.sec = 4  # Take 4 seconds to move smoothly

        traj.points.append(point)

        self.pub.publish(traj)
        self.get_logger().info("Command SENT! Watch the robot move.")

        # Stop the timer so we only send it once
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
