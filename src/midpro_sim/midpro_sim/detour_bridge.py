#!/usr/bin/env python3
"""detour_bridge.py — replaces the ROS1 eband path source.

The midpro FSM controller publishes the goal it wants a detour for on /midpro/detour_request
while it is in the `detour` path state. This node calls the second planner_server
(namespace /detour, Smac Hybrid-A* on a rolling local costmap) and republishes the resulting
path on /midpro/detour_path at `replan_rate` Hz for as long as requests keep arriving.

Topics
  sub  /midpro/detour_request   geometry_msgs/PoseStamped  (map frame goal)
  pub  /midpro/detour_path      nav_msgs/Path
Action client
  /detour/compute_path_to_pose  nav2_msgs/action/ComputePathToPose  (planner_id configurable)
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node


class DetourBridge(Node):
    def __init__(self):
        super().__init__('detour_bridge')
        self.declare_parameter('action_name', '/detour/compute_path_to_pose')
        self.declare_parameter('planner_id', 'DetourPlanner')
        self.declare_parameter('replan_rate', 2.0)         # Hz while a request is active
        self.declare_parameter('request_timeout', 1.5)     # s, stop replanning after last request
        self.declare_parameter('use_start', False)         # False -> planner uses current robot pose

        self.action_name = self.get_parameter('action_name').value
        self.planner_id = self.get_parameter('planner_id').value
        self.replan_rate = float(self.get_parameter('replan_rate').value)
        self.request_timeout = float(self.get_parameter('request_timeout').value)
        self.use_start = bool(self.get_parameter('use_start').value)

        self.client = ActionClient(self, ComputePathToPose, self.action_name)
        self.path_pub = self.create_publisher(Path, '/midpro/detour_path', 1)
        self.create_subscription(PoseStamped, '/midpro/detour_request', self.on_request, 1)

        self.goal = None
        self.last_request_time = None
        self.in_flight = False
        self.timer = self.create_timer(1.0 / self.replan_rate, self.tick)
        self.get_logger().info(f'detour bridge -> {self.action_name} ({self.planner_id})')

    def on_request(self, msg: PoseStamped):
        self.goal = msg
        self.last_request_time = self.get_clock().now()

    def tick(self):
        if self.goal is None or self.last_request_time is None:
            return
        age = (self.get_clock().now() - self.last_request_time).nanoseconds * 1e-9
        if age > self.request_timeout:
            self.goal = None  # controller left the detour state
            return
        if self.in_flight:
            return
        if not self.client.server_is_ready():
            self.get_logger().warn('detour planner action server not ready', throttle_duration_sec=2.0)
            return
        req = ComputePathToPose.Goal()
        req.goal = self.goal
        req.planner_id = self.planner_id
        req.use_start = self.use_start
        self.in_flight = True
        fut = self.client.send_goal_async(req)
        fut.add_done_callback(self.on_goal_response)

    def on_goal_response(self, fut):
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.in_flight = False
            self.get_logger().warn('detour goal rejected')
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        self.in_flight = False
        try:
            result = fut.result().result
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'detour planning failed: {e}')
            return
        if result is None or len(result.path.poses) == 0:
            self.get_logger().warn('detour planner returned an empty path', throttle_duration_sec=2.0)
            return
        path = result.path
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = DetourBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
