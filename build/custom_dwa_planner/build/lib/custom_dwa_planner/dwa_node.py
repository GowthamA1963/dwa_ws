import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np

class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.goal = None
        self.pose = None
        self.scan = None
        self.robot_radius = 0.2
        self.max_vel = 0.5
        self.max_yawrate = 1.0
        self.predict_time = 2.0
        self.dt = 0.1

    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"New goal set: {self.goal}")

    def odom_callback(self, msg):
        self.pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        )
        self.twist = msg.twist.twist

    def scan_callback(self, msg):
        self.scan = msg

    def control_loop(self):
        if self.pose is None or self.scan is None or self.goal is None:
            return

        best_score = float('inf')
        best_cmd = Twist()
        best_trajectory = None
        all_trajectories = []

        for v in np.arange(0, self.max_vel, 0.1):
            for w in np.arange(-self.max_yawrate, self.max_yawrate, 0.2):
                traj = self.predict_trajectory(v, w)
                all_trajectories.append(traj)
                score = self.evaluate_trajectory(traj)
                if score < best_score:
                    best_score = score
                    best_cmd.linear.x = v
                    best_cmd.angular.z = w
                    best_trajectory = traj

        self.publish_trajectories(all_trajectories)
        self.cmd_pub.publish(best_cmd)

    def predict_trajectory(self, v, w):
        x, y, theta = self.pose
        traj = []
        for _ in np.arange(0, self.predict_time, self.dt):
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += w * self.dt
            traj.append((x, y, theta))
        return traj

    def evaluate_trajectory(self, traj):
        goal_dist = math.hypot(self.goal[0] - traj[-1][0], self.goal[1] - traj[-1][1])

        min_dist = float('inf')
        angle_min = self.scan.angle_min
        angle_increment = self.scan.angle_increment
        ranges = np.array(self.scan.ranges)

        for (x, y, _) in traj:
            dx = x - self.pose[0]
            dy = y - self.pose[1]
            dist = math.hypot(dx, dy)
            angle = math.atan2(dy, dx)
            index = int((angle - angle_min) / angle_increment)
            if 0 <= index < len(ranges):
                scan_dist = ranges[index]
                if scan_dist < min_dist:
                    min_dist = scan_dist

        if min_dist < self.robot_radius:
            self.get_logger().warn(f"Trajectory in collision! Min dist: {min_dist:.2f}")
            return float('inf')

        return goal_dist + (1.0 / (min_dist + 1e-5))

    def publish_trajectories(self, trajectories):
        marker_array = MarkerArray()
        for i, traj in enumerate(trajectories):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dwa_trajectories"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.02
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 0.7
            marker.pose.orientation.w = 1.0

            for pose in traj:
                p = Point()
                p.x, p.y = pose[0], pose[1]
                p.z = 0.0
                marker.points.append(p)

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = DWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
