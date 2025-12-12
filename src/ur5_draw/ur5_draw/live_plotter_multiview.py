#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class LivePlotterMultiView(Node):
    def __init__(self):
        super().__init__("live_plotter_multiview")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.x_data, self.y_data, self.z_data = [], [], []

        plt.ion()
        self.fig = plt.figure(figsize=(12, 10))

        # 1. Top Left: XY Plane (Robot Table) - Matches "XY Plane (Table / Top View)" in HTML
        self.ax_xy = self.fig.add_subplot(2, 2, 1)

        # 2. Top Right: YZ Plane (Side View)
        self.ax_yz = self.fig.add_subplot(2, 2, 2)

        # 3. Bottom Left: XZ Plane (Robot Vertical Front) - Matches "XZ Plane (Vertical / Front View)" in HTML
        self.ax_xz = self.fig.add_subplot(2, 2, 3)

        # 4. Bottom Right: 3D View
        self.ax_3d = self.fig.add_subplot(2, 2, 4, projection="3d")

        self.timer = self.create_timer(0.1, self.update_plot)
        self.get_logger().info("Plotter Started...")

    def setup_axes(self):
        # 1. XY Plane (Table)
        self.ax_xy.set_title("XY Plane (Table / Top View)")
        self.ax_xy.set_xlabel("X (Forward/Back) [m]")
        self.ax_xy.set_ylabel("Y (Left/Right) [m]")
        self.ax_xy.grid(True)
        self.ax_xy.axis("equal")
        self.ax_xy.invert_xaxis()
        self.ax_xy.invert_yaxis()

        # 2. YZ Plane (Side)
        self.ax_yz.set_title("YZ Plane (Side View)")
        self.ax_yz.set_xlabel("Y [m]")
        self.ax_yz.set_ylabel("Z (Height) [m]")
        self.ax_yz.grid(True)
        self.ax_yz.axis("equal")

        # 3. XZ Plane (Vertical Front)
        self.ax_xz.set_title("XZ Plane (Vertical / Front View)")
        self.ax_xz.set_xlabel("X [m]")
        self.ax_xz.set_ylabel("Z (Height) [m]")
        self.ax_xz.grid(True)
        self.ax_xz.axis("equal")

        # 4. 3D
        self.ax_3d.set_title("3D Isometric View")
        self.ax_3d.set_xlabel("X")
        self.ax_3d.set_ylabel("Y")
        self.ax_3d.set_zlabel("Z")

    def update_plot(self):
        try:
            t = self.tf_buffer.lookup_transform("base_link", "tool0", rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            if not self.x_data or (abs(x - self.x_data[-1]) > 0.001):
                self.x_data.append(x)
                self.y_data.append(y)
                self.z_data.append(z)

                self.ax_xy.clear()
                self.ax_yz.clear()
                self.ax_xz.clear()
                self.ax_3d.clear()
                self.setup_axes()

                # PLOT 1: X vs Y (Table)
                self.ax_xy.plot(self.x_data, self.y_data, "b-")
                self.ax_xy.plot(x, y, "ro")

                # PLOT 2: Y vs Z (Side)
                self.ax_yz.plot(self.y_data, self.z_data, "g-")
                self.ax_yz.plot(y, z, "ro")

                # PLOT 3: X vs Z (Vertical Front)
                self.ax_xz.plot(self.x_data, self.z_data, "m-")
                self.ax_xz.plot(x, z, "ro")

                # PLOT 4: 3D
                self.ax_3d.plot(self.x_data, self.y_data, self.z_data, "k-")
                self.ax_3d.scatter(x, y, z, color="red")

                plt.draw()
                plt.pause(0.001)

        except TransformException:
            pass


def main():
    rclpy.init()
    node = LivePlotterMultiView()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
