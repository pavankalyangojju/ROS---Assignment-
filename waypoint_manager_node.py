#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import math
import time

def quaternion_from_yaw(yaw: float):
    # returns (x,y,z,w)
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        # parameters (could be ROS params)
        self.declare_parameter('waypoint_file', 'config/waypoints.yaml')
        wp_file = self.get_parameter('waypoint_file').get_parameter_value().string_value

        # load waypoints
        with open(wp_file, 'r') as f:
            self.waypoints = yaml.safe_load(f)

        self.get_logger().info(f'Loaded waypoints: {list(self.waypoints.keys())}')

        # action client for NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # publishers/subscribers
        self.status_pub = self.create_publisher(String, 'waypoint_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', 10)
        self.seq_sub = self.create_subscription(String, 'waypoint_sequence', self.sequence_cb, 10)
        self.cancel_sub = self.create_subscription(Bool, 'cancel_nav', self.cancel_cb, 10)

        # internal
        self._current_goal_handle = None
        self._cancel_requested = False

        # publish markers once
        self.publish_waypoint_markers()

    def publish_waypoint_markers(self):
        ma = MarkerArray()
        idx = 0
        for name, val in self.waypoints.items():
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'waypoints'
            m.id = idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(val['x'])
            m.pose.position.y = float(val['y'])
            m.pose.position.z = 0.05
            # orientation default
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.02
            # color: vary slightly
            m.color.r = 0.0
            m.color.g = 0.7
            m.color.b = 0.2
            m.color.a = 0.8
            # add a text label marker
            ma.markers.append(m)

            t = Marker()
            t.header = m.header
            t.ns = 'waypoint_labels'
            t.id = 1000 + idx
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = m.pose.position.x
            t.pose.position.y = m.pose.position.y
            t.pose.position.z = 0.35
            t.scale.z = 0.15
            t.text = name
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 1.0
            ma.markers.append(t)

            idx += 1

        self.marker_pub.publish(ma)
        self.get_logger().info('Published waypoint markers')

    def sequence_cb(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        names = [s.strip() for s in text.split(',') if s.strip()]
        self.get_logger().info(f'Received waypoint sequence: {names}')
        # start navigation in background
        self.get_logger().info('Starting waypoint navigation...')
        self._cancel_requested = False
        self.navigate_sequence(names)

    def cancel_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Cancel requested by GUI')
            self._cancel_requested = True
            if self._current_goal_handle:
                fut = self._current_goal_handle.cancel_goal_async()
                fut.add_done_callback(lambda fut: self.get_logger().info('Cancel request sent to action server'))

    def send_status(self, s: str):
        m = String()
        m.data = s
        self.status_pub.publish(m)
        self.get_logger().info(f'STATUS: {s}')

    def make_pose_stamped(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        q = quaternion_from_yaw(float(yaw))
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        return ps

    def wait_for_nav_action(self, timeout=10.0):
        # wait for action server available
        if not self._action_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('NavigateToPose action server not available')
            return False
        return True

    def navigate_sequence(self, names):
        # run synchronously (but not blocking callbacks)
        if not self.wait_for_nav_action(timeout=15.0):
            self.send_status('Nav2 action server not available')
            return

        # ensure Home will be appended at the end
        seq = list(names)
        home_name = 'home'
        if home_name not in seq:
            seq.append(home_name)

        for idx, name in enumerate(seq):
            if self._cancel_requested:
                self.send_status('Cancelled')
                return
            if name not in self.waypoints:
                self.send_status(f'Failed: unknown waypoint {name}')
                continue

            wp = self.waypoints[name]
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.make_pose_stamped(wp['x'], wp['y'], wp.get('yaw', 0.0))

            self.send_status(f'Navigating to {name}')
            send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
            r = send_goal_future.result()
            if not r.accepted:
                self.send_status(f'Goal to {name} rejected')
                continue

            self._current_goal_handle = r
            get_result_future = r.get_result_async()
            # wait for result, but allow cancel to be processed
            while not get_result_future.done():
                if self._cancel_requested:
                    # cancel the goal
                    self.send_status('Cancelling current goal...')
                    cancel_future = self._current_goal_handle.cancel_goal_async()
                    cancel_future.result()  # wait brief
                    self.send_status('Cancelled')
                    return
                rclpy.spin_once(self, timeout_sec=0.1)

            result = get_result_future.result().result
            status = get_result_future.result().status

            # status values: 4 = SUCCEEDED (action_msgs)
            if status == 4:
                self.send_status(f'Reached {name}')
            else:
                self.send_status(f'Failed {name} (status {status})')
                # decide whether to continue or return home — we will continue to next waypoint but
                # at end will go home anyway (home already appended).
            self._current_goal_handle = None

        # finished sequence (home appended), confirm at end
        self.send_status('All goals processed; returned to Home')

    def feedback_cb(self, feedback_msg):
        # feedback_msg is GoalHandle.feedback (type NavigateToPose.Feedback)
        fb = feedback_msg.feedback
        # nav2 frequently returns progress info in fb.estimate or fb.current_pose - print succinctly
        if hasattr(fb, 'distance_to_goal'):
            s = f"Distance to goal: {fb.distance_to_goal:.2f} m"
        else:
            s = "Navigating..."
        # publish shorter updates (not too spammy)
        self.send_status(s)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
