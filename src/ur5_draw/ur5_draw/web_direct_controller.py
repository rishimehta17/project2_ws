#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import asyncio
import websockets
import json
import threading
import ikpy.chain
import time

# === CONFIGURATION ===
URDF_PATH = "/home/rishimehta/ur5_imitation/src/ur_description/urdf/ur5_robot.urdf"

# "Ready" pose (Joints in Radians)
HOME_JOINTS = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]


class WebDirectController(Node):
    def __init__(self):
        super().__init__("web_direct_controller")

        # 1. Setup Publisher
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        # 2. Load Kinematics Chain
        self.get_logger().info(f"Loading URDF from: {URDF_PATH}")
        try:
            # Mask for 10 links: 0=Base, 1-6=Joints, 7-9=Tool parts
            self.chain = ikpy.chain.Chain.from_urdf_file(
                URDF_PATH,
                base_elements=["base_link"],
                active_links_mask=[
                    False,
                    True,
                    True,
                    True,
                    True,
                    True,
                    True,
                    False,
                    False,
                    False,
                ],
            )
            self.get_logger().info(
                f"IK Chain Loaded Successfully! Total links: {len(self.chain.links)}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to load URDF: {e}")
            raise e

        # 3. Move to Home Position
        self.move_to_joints(HOME_JOINTS, duration=4.0)

    def move_to_joints(self, positions, duration=2.0):
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
        point.positions = list(positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points.append(point)
        self.traj_pub.publish(traj)
        self.get_logger().info("Published Move Command")

    def execute_stroke(self, points_3d):
        self.get_logger().info(f"Processing Stroke: {len(points_3d)} points")

        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        current_joints = HOME_JOINTS

        # Tool Orientation: Pointing Down
        target_orientation = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
        duration_per_point = 0.1

        n_links = len(self.chain.links)  # Should be 10

        for i, (x, y, z) in enumerate(points_3d):
            try:
                # Prepare seed with correct length
                current_seed_full = [0.0] * n_links
                current_seed_full[1:7] = current_joints

                ik_solution = self.chain.inverse_kinematics(
                    target_position=[x, y, z],
                    target_orientation=target_orientation,
                    initial_position=current_seed_full,
                )

                clean_joints = ik_solution[1:7]

                point = JointTrajectoryPoint()
                point.positions = list(clean_joints)

                time_offset = (i + 1) * duration_per_point
                point.time_from_start.sec = int(time_offset)
                point.time_from_start.nanosec = int(
                    (time_offset - int(time_offset)) * 1e9
                )

                traj.points.append(point)
                current_joints = clean_joints

            except Exception as e:
                self.get_logger().warn(f"IK Failed for point {i}: {e}")

        if len(traj.points) > 0:
            self.traj_pub.publish(traj)
            self.get_logger().info("Trajectory Sent to Robot!")


# === PIXEL_TO_3D FUNCTION ===
def pixel_to_3d(points):
    """Maps Screen Points (u, v, [z]) to Robot Coordinates (x, y, z)"""
    out = []
    for point in points:
        # 1. Extract U and V (Screen coordinates)
        u = point[0]
        v = point[1]

        # 2. Extract Z if provided (Height), else default to 0.25
        # The frontend now sends [u, v, z]
        if len(point) > 2:
            z = float(point[2])
        else:
            z = 0.25

        # 3. Scale X and Y
        # Scale X: Screen Left(0) -> 0.2m, Screen Right(1280) -> 0.8m
        x = 0.2 + (u / 1280.0) * 0.6
        # Scale Y: Screen Top(0) -> 0.25m, Screen Bottom(720) -> -0.25m
        y = -0.25 + ((720.0 - v) / 720.0) * 0.5

        out.append([x, y, z])
    return out


# === WEBSOCKET & MAIN LOGIC ===
async def ws_handler(websocket, node):
    node.get_logger().info("Client connected!")
    try:
        async for message in websocket:
            data = json.loads(message)
            # We look for "stroke" type which matches index.html
            if data.get("type") == "stroke":
                raw_points = data.get("points", [])
                if len(raw_points) < 1:
                    continue
                # Convert raw points to Robot 3D points
                points_3d = pixel_to_3d(raw_points)
                node.execute_stroke(points_3d)
                await websocket.send(json.dumps({"status": "ok"}))
    except Exception:
        pass


def ros_spin_thread(node):
    rclpy.spin(node)


async def run_server(node):
    print("WebSocket Server Started on ws://0.0.0.0:8765")
    async with websockets.serve(lambda ws: ws_handler(ws, node), "0.0.0.0", 8765):
        await asyncio.Future()


def main():
    rclpy.init()
    node = WebDirectController()

    spinner = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spinner.start()

    print("ROS 2 Node Started. Waiting for strokes...")

    try:
        asyncio.run(run_server(node))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
