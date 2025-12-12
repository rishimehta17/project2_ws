#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import Pose
import math
import time


class DirectDrawer(Node):
    def __init__(self):
        super().__init__("direct_drawer")

        # 1. Publisher: Sends commands directly to the robot (SimpleMover style)
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        # 2. Client: Asks MoveIt for IK (Cartesian -> Joints)
        self.ik_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")

        self.get_logger().info("Waiting for MoveIt IK service...")
        self.ik_client.wait_for_service()
        self.get_logger().info("Ready! Starting sequence...")

        # Start the sequence
        self.timer = self.create_timer(1.0, self.run_sequence)
        self.step = 0

    def run_sequence(self):
        # STEP 1: RESET (SimpleMover Logic)
        if self.step == 0:
            self.get_logger().info("--- STEP 1: Moving to Ready Pose ---")
            self.move_to_joints([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], duration=4.0)
            self.step = 1
            self.wait_ticks = 5  # Wait 5 seconds

        # STEP 2: DRAW SQUARE
        elif self.step == 2:
            self.get_logger().info("--- STEP 2: Drawing Square ---")
            # Define Square in 3D (Meters)
            # Z=0.25 is the safe height above table
            points = [
                (0.4, 0.2, 0.25),  # Top Left
                (0.6, 0.2, 0.25),  # Bottom Left
                (0.6, -0.2, 0.25),  # Bottom Right
                (0.4, -0.2, 0.25),  # Top Right
                (0.4, 0.2, 0.25),  # Close Loop
            ]
            self.execute_cartesian_path(points)
            self.step = 3
            self.wait_ticks = 8

        # STEP 3: DRAW CIRCLE
        elif self.step == 4:
            self.get_logger().info("--- STEP 3: Drawing Circle ---")
            points = []
            cx, cy, z = 0.5, 0.0, 0.25
            radius = 0.15
            for i in range(50):
                angle = i * (2 * math.pi / 49)
                px = cx + radius * math.cos(angle)
                py = cy + radius * math.sin(angle)
                points.append((px, py, z))

            self.execute_cartesian_path(points)
            self.step = 5
            self.wait_ticks = 10

        # STEP 4: FINISH
        elif self.step == 6:
            self.get_logger().info("Done! Shutting down.")
            raise SystemExit

        # WAITING LOGIC
        elif self.step % 2 != 0:
            if self.wait_ticks > 0:
                self.wait_ticks -= 1
            else:
                self.step += 1

    # --- FUNCTION A: MOVE DIRECTLY (Like SimpleMover) ---
    def move_to_joints(self, positions, duration=4.0):
        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        traj.points.append(point)
        self.traj_pub.publish(traj)

    # --- FUNCTION B: CALCULATE PATH & MOVE ---
    def execute_cartesian_path(self, waypoints_list):
        # 1. Prepare Request for MoveIt
        req = GetCartesianPath.Request()
        req.group_name = "ur_manipulator"
        req.link_name = "tool0"
        req.max_step = 0.01
        req.jump_threshold = 0.0

        # Convert (x,y,z) tuples to Pose messages
        for x, y, z in waypoints_list:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            # Orientation: Pointing DOWN (180 deg around X)
            pose.orientation.x = 1.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0
            req.waypoints.append(pose)

        # 2. Call Service (Sync call for simplicity in this logic)
        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_response_callback)

    def ik_response_callback(self, future):
        try:
            response = future.result()
            if response.fraction < 0.5:
                self.get_logger().warn(
                    f"Plan failed! Only computed {response.fraction * 100}%"
                )
                return

            # 3. DIRECT PUBLISH (The 'SimpleMover' Trick)
            # The service returns a trajectory. We just forward it to the controller topic!
            joint_traj = response.solution.joint_trajectory

            # IMPORTANT: Re-time the trajectory so it doesn't move instantly
            # We stretch it out to take 5 seconds total
            total_points = len(joint_traj.points)
            for i, p in enumerate(joint_traj.points):
                time_offset = (i + 1) * (5.0 / total_points)  # 5 seconds total duration
                p.time_from_start.sec = int(time_offset)
                p.time_from_start.nanosec = int((time_offset - int(time_offset)) * 1e9)

            self.traj_pub.publish(joint_traj)
            self.get_logger().info("Path Calculated & Sent to Controller!")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DirectDrawer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
