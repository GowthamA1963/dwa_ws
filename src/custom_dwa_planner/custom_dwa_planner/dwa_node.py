#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
import random

class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Timer callback for planning
        self.timer = self.create_timer(0.1, self.plan)

        # Robot State
        self.pose = [0, 0, 0]  # x, y, yaw
        self.vel = [0.0, 0.0]  # linear x, angular z
        self.scan = None

        # DWA Parameters
        self.v_res = 0.02  # Finer resolution for linear speed sampling
        self.w_res = 0.05  # Finer resolution for angular speed sampling
        self.max_v = 0.4
        self.min_v = 0.0
        self.max_w = 1.5
        self.min_w = -1.5
        self.dt = 0.1
        self.predict_time = 1.0

        # Cost Weights
        self.goal_cost_weight = 1.0  # Encourage moving toward the goal
        self.obstacle_cost_weight = 0.5  # Balance obstacle avoidance
        self.smoothness_cost_weight = 0.1  # Smoothness, lesser weight

        # Goal Position (static for now)
        self.goal = [2.0, 0.0]

    def odom_callback(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.pose[2] = euler_from_quaternion([
            orientation_q.x, orientation_q.y,
            orientation_q.z, orientation_q.w
        ])
        self.vel[0] = msg.twist.twist.linear.x
        self.vel[1] = msg.twist.twist.angular.z

    def scan_callback(self, msg):
        self.scan = msg

    def plan(self):
        if self.scan is None:
            return

        best_u = None
        min_cost = float('inf')
        all_trajectories = []

        for v in np.arange(self.min_v, self.max_v + self.v_res, self.v_res):
            for w in np.arange(self.min_w, self.max_w + self.w_res, self.w_res):
                traj = self.predict_trajectory(v, w)
                all_trajectories.append(traj)

                to_goal_cost = self.calc_to_goal_cost(traj)
                obstacle_cost = self.calc_obstacle_cost(traj)
                smoothness_cost = abs(w)

                if obstacle_cost == float('inf'):
                    continue

                total_cost = (
                    self.goal_cost_weight * to_goal_cost +
                    self.obstacle_cost_weight * obstacle_cost +
                    self.smoothness_cost_weight * smoothness_cost
                )

                self.get_logger().debug(
                    f"✔ v={v:.2f}, w={w:.2f} → to_goal={to_goal_cost:.2f}, obs={obstacle_cost:.2f}, total={total_cost:.2f}"
                )

                if total_cost < min_cost:
                    min_cost = total_cost
                    best_u = [v, w]

        if best_u and (best_u[0] > 0.01 or abs(best_u[1]) > 0.01):
            cmd = Twist()
            cmd.linear.x = best_u[0]
            cmd.angular.z = best_u[1]
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'✅ Best cmd: v={best_u[0]:.2f}, w={best_u[1]:.2f}')
        else:
            self.get_logger().warn("⚠️ No valid trajectory. Trying slow forward motion.")
            cmd = Twist()
            cmd.linear.x = 0.05
            cmd.angular.z = 0.2
            self.cmd_pub.publish(cmd)

        self.publish_markers(all_trajectories)

    def predict_trajectory(self, v, w):
        x, y, theta = self.pose
        traj = []
        time = 0.0

        while time <= self.predict_time:
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += w * self.dt
            traj.append((x, y))
            time += self.dt

        return traj

    def calc_to_goal_cost(self, traj):
        goal_x, goal_y = self.goal
        last_x, last_y = traj[-1]
        return math.hypot(goal_x - last_x, goal_y - last_y)

    def calc_obstacle_cost(self, traj):
        if not self.scan:
            return 0.0

        ranges = np.array(self.scan.ranges)
        angles = np.linspace(self.scan.angle_min, self.scan.angle_max, len(ranges))

        valid = np.isfinite(ranges) & (ranges > self.scan.range_min) & (ranges < self.scan.range_max)
        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) == 0:
            return float('inf')

        ox = self.pose[0] + ranges * np.cos(self.pose[2] + angles)
        oy = self.pose[1] + ranges * np.sin(self.pose[2] + angles)

        min_dist = float('inf')
        for x, y in traj:
            dists = np.hypot(ox - x, oy - y)
            min_dist = min(min_dist, np.min(dists))

        self.get_logger().info(f'📏 Obstacle min_dist: {min_dist:.2f}')

        if min_dist < 0.05:  # Relaxed threshold for obstacle proximity
            return float('inf')
        return 1.0 / min_dist

    def publish_markers(self, trajectories):
        marker_array = MarkerArray()

        for i, traj in enumerate(trajectories):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.02
            marker.color.a = 0.8
            marker.color.r = random.uniform(0.0, 1.0)
            marker.color.g = random.uniform(0.0, 1.0)
            marker.color.b = random.uniform(0.0, 1.0)
            marker.pose.orientation.w = 1.0

            for x, y in traj:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.0
                marker.points.append(p)

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
