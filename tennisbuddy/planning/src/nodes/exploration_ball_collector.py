#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Exploration Ball Collector
integrate exploration and ball collection functionality:
- when no ball is detected: use Nav2 NavigateThroughPoses for exploration
- when a ball is detected: insert ball position into waypoint queue, prioritize picking up the ball
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert yaw angle to quaternion"""
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class ExplorationBallCollector(Node):
    """
    State machine:
    - EXPLORING: exploration mode, move according to waypoint sequence
    - SCANNING: scanning mode, pause at waypoint to detect balls
    - COLLECTING_BALL: ball collection mode, prioritize picking up the ball
    """
    
    STATE_EXPLORING = 'EXPLORING'
    STATE_SCANNING = 'SCANNING'
    STATE_COLLECTING_BALL = 'COLLECTING_BALL'
    
    def __init__(self):
        super().__init__('exploration_ball_collector')
        
        # Parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('ball_positions_topic', '/ball_positions')
        self.declare_parameter('exploration_waypoints_topic', '/exploration_waypoints')
        self.declare_parameter('pickup_distance', 0.1)
        self.declare_parameter('ball_detection_timeout', 15.0)  # seconds
        self.declare_parameter('waypoint_visit_threshold', 0.5)  # meters
        self.declare_parameter('use_navigate_through_poses', True)
        self.declare_parameter('scan_at_waypoint', True)
        self.declare_parameter('scan_duration', 2.0)  # seconds
        self.declare_parameter('waypoint_arrival_threshold', 0.3)  # meters
        self.declare_parameter('immediate_ball_response', True)  # cancel exploration immediately on ball detection
        
        self.frame_id = self.get_parameter('frame_id').value
        self.pickup_d = float(self.get_parameter('pickup_distance').value)
        self.ball_timeout = float(self.get_parameter('ball_detection_timeout').value)
        self.visit_threshold = float(self.get_parameter('waypoint_visit_threshold').value)
        self.use_navigate_through_poses = bool(self.get_parameter('use_navigate_through_poses').value)
        self.scan_at_waypoint = bool(self.get_parameter('scan_at_waypoint').value)
        self.scan_duration = float(self.get_parameter('scan_duration').value)
        self.waypoint_arrival_threshold = float(self.get_parameter('waypoint_arrival_threshold').value)
        self.immediate_ball_response = bool(self.get_parameter('immediate_ball_response').value)
        
        # QoS
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.sub_balls = self.create_subscription(
            PoseArray,
            self.get_parameter('ball_positions_topic').value,
            self.on_balls,
            qos_sensor
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.on_odom,
            qos_rel
        )
        self.sub_waypoints = self.create_subscription(
            PoseArray,
            self.get_parameter('exploration_waypoints_topic').value,
            self.on_waypoints,
            qos_rel
        )
        
        # Action Clients
        if self.use_navigate_through_poses:
            self.ac_navigate_through_poses = ActionClient(
                self, NavigateThroughPoses, 'navigate_through_poses'
            )
        else:
            self.ac_navigate_through_poses = None
        
        self.ac_navigate_to_pose = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        
        # State
        self.state = self.STATE_EXPLORING
        self.robot_xy = None
        self.robot_yaw = None
        self.balls = []  # List of [x, y]
        self.last_ball_time = None
        self.exploration_waypoints = []  # List of Pose
        self.visited_waypoints = []  # List of [x, y]
        self.current_goal_handle = None
        self.current_waypoint_index = 0
        self.scan_start_time = None
        self.current_waypoint_position = None
        self.last_sent_waypoints = []
        
        # Wait for action servers
        if self.use_navigate_through_poses:
            self.get_logger().info('[exploration_ball_collector] Waiting for navigate_through_poses server...')
            self.ac_navigate_through_poses.wait_for_server()
        
        self.get_logger().info('[exploration_ball_collector] Waiting for navigate_to_pose server...')
        self.ac_navigate_to_pose.wait_for_server()
        
        self.get_logger().info('[exploration_ball_collector] Ready.')
        
        # Timer for state management
        self.create_timer(0.5, self.tick)
    
    def on_odom(self, msg: Odometry):
        """Update robot position"""
        self.robot_xy = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ], dtype=float)
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
    
    def on_balls(self, msg: PoseArray):
        """
        Handle ball detections with immediate response
        Strategy 3: Cancel exploration immediately when ball is detected
        """
        now = self.get_clock().now()
        
        if len(msg.poses) > 0:
            # Ball detected - immediate response
            new_balls = []
            for p in msg.poses:
                new_balls.append([p.position.x, p.position.y])
            
            # Strategy 3: Immediately cancel exploration if ball detected during exploration
            if self.immediate_ball_response:
                if (self.state == self.STATE_EXPLORING or self.state == self.STATE_SCANNING):
                    if self.current_goal_handle is not None:
                        self.get_logger().info(
                            '[exploration_ball_collector] Ball detected during exploration/scanning, '
                            'canceling current goal immediately'
                        )
                        self.cancel_current_goal()
                        # Exit scanning state if in scanning
                        if self.state == self.STATE_SCANNING:
                            self.scan_start_time = None
                            self.current_waypoint_position = None
            
            self.balls = new_balls
            self.last_ball_time = now
            self.get_logger().info(
                f'[exploration_ball_collector] Detected {len(self.balls)} balls at positions: '
                f'{[(b[0]:.2f, b[1]:.2f) for b in self.balls]}'
            )
        else:
            # No ball detection - check timeout
            if self.last_ball_time is not None:
                elapsed = (now - self.last_ball_time).nanoseconds / 1e9
                
                # Dynamic timeout based on distance and state
                if self.state == self.STATE_COLLECTING_BALL and self.robot_xy is not None:
                    if self.balls and len(self.balls) > 0:
                        distances = [np.linalg.norm(np.array(b) - self.robot_xy) for b in self.balls]
                        min_dist = min(distances) if distances else float('inf')
                        
                        # Extended timeout when close to ball
                        extended_timeout = self.ball_timeout
                        if min_dist < 2.0:
                            extended_timeout = self.ball_timeout * 3
                        elif min_dist < 1.0:
                            extended_timeout = self.ball_timeout * 5
                        
                        if elapsed > extended_timeout:
                            self.balls = []
                            self.last_ball_time = None
                            self.get_logger().info(
                                f'[exploration_ball_collector] Ball timeout (extended: {extended_timeout:.1f}s, '
                                f'distance: {min_dist:.2f}m)'
                            )
                    else:
                        if elapsed > self.ball_timeout:
                            self.balls = []
                            self.last_ball_time = None
                else:
                    # Exploration/scanning mode, use normal timeout
                    if elapsed > self.ball_timeout:
                        self.balls = []
                        self.last_ball_time = None
                        self.get_logger().debug(
                            f'[exploration_ball_collector] Ball detection timeout after {elapsed:.1f}s'
                        )
    
    def on_waypoints(self, msg: PoseArray):
        """Handle exploration waypoints"""
        self.exploration_waypoints = list(msg.poses)
        self.get_logger().info(
            f'[exploration_ball_collector] Received {len(self.exploration_waypoints)} exploration waypoints'
        )
    
    def compute_pickup_pose(self, robot_xy, ball_xy):
        """compute the target position for ball collection (in front of the ball a certain distance)"""
        vec = ball_xy - robot_xy
        dist = np.linalg.norm(vec)
        
        if dist <= 1e-6:
            theta = 0.0
        else:
            theta = math.atan2(vec[1], vec[0])
        
        # compute the target position (in front of the ball a certain distance)
        desired_offset = max(0.0, min(self.pickup_d, dist - 0.1))
        gx = ball_xy[0] - desired_offset * math.cos(theta)
        gy = ball_xy[1] - desired_offset * math.sin(theta)
        
        return gx, gy, theta
    
    def cancel_current_goal(self):
        """cancel the current goal"""
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
                self.get_logger().info('[exploration_ball_collector] Cancelled current goal')
            except Exception as e:
                self.get_logger().warn(f'[exploration_ball_collector] Failed to cancel goal: {e}')
            self.current_goal_handle = None
    
    def send_ball_goal(self, ball_xy):
        """send the goal to pick up the ball"""
        if self.robot_xy is None:
            return
        
        gx, gy, yaw = self.compute_pickup_pose(self.robot_xy, np.array(ball_xy))
        
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = float(gx)
        ps.pose.position.y = float(gy)
        ps.pose.orientation = yaw_to_quat(yaw)
        goal.pose = ps
        
        self.get_logger().info(
            f'[exploration_ball_collector] Sending ball goal -> ({gx:.2f}, {gy:.2f})'
        )
        
        future = self.ac_navigate_to_pose.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)
    
    def send_exploration_waypoints(self, waypoints):
        """send exploration waypoint sequence"""
        if not waypoints:
            self.get_logger().warn('[exploration_ball_collector] No waypoints to send')
            return

        self.last_sent_waypoints = waypoints
        
        if self.use_navigate_through_poses and self.ac_navigate_through_poses is not None:
            # use NavigateThroughPoses
            goal = NavigateThroughPoses.Goal()
            goal.poses = []
            
            for wp in waypoints:
                ps = PoseStamped()
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.header.frame_id = self.frame_id
                ps.pose = wp
                goal.poses.append(ps)
            
            self.get_logger().info(
                f'[exploration_ball_collector] Sending {len(goal.poses)} exploration waypoints'
            )
            
            future = self.ac_navigate_through_poses.send_goal_async(
                goal, feedback_callback=self.on_feedback
            )
            future.add_done_callback(self.on_goal_response)
        else:
            # use NavigateToPose (only send the first waypoint)
            if waypoints:
                wp = waypoints[0]
                goal = NavigateToPose.Goal()
                ps = PoseStamped()
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.header.frame_id = self.frame_id
                ps.pose = wp
                goal.pose = ps
                
                self.get_logger().info(
                    f'[exploration_ball_collector] Sending first exploration waypoint -> '
                    f'({wp.position.x:.2f}, {wp.position.y:.2f})'
                )
                
                future = self.ac_navigate_to_pose.send_goal_async(
                    goal, feedback_callback=self.on_feedback
                )
                future.add_done_callback(self.on_goal_response)
    
    def on_goal_response(self, future):
        """handle the goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[exploration_ball_collector] Goal rejected')
            self.current_goal_handle = None
            return
        
        self.get_logger().info('[exploration_ball_collector] Goal accepted')
        self.current_goal_handle = goal_handle
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)
    
    def on_feedback(self, feedback):
        """handle the feedback"""
        # can check the progress here, update visited_waypoints etc.
        pass
    
    def on_result(self, future):
        """handle the result"""
        result = future.result().result
        self.get_logger().info(
            f'[exploration_ball_collector] Goal finished. State: {result}'
        )
        self.current_goal_handle = None
        
        # Mark all waypoints in the last sent sequence as visited
        if hasattr(self, 'last_sent_waypoints') and self.last_sent_waypoints:
            for wp in self.last_sent_waypoints:
                wp_xy = [wp.position.x, wp.position.y]
                # Check if already marked
                already_visited = False
                for vp in self.visited_waypoints:
                    if np.linalg.norm(np.array(wp_xy) - np.array(vp)) < self.visit_threshold:
                        already_visited = True
                        break
                if not already_visited:
                    self.visited_waypoints.append(wp_xy)
                    self.get_logger().debug(
                        f'[exploration_ball_collector] Marked waypoint ({wp_xy[0]:.2f}, {wp_xy[1]:.2f}) as visited'
                    )
    
    def tick(self):
        """
        Main state machine loop with optimized ball collection logic
        Strategy: Continuous detection during movement, immediate response, scanning at waypoints
        """
        if self.robot_xy is None:
            return
        
        now = self.get_clock().now()
        
        # Strategy 5: Handle scanning state
        if self.state == self.STATE_SCANNING:
            if self.scan_start_time is None:
                self.scan_start_time = now
                self.get_logger().info(
                    f'[exploration_ball_collector] Starting scan at waypoint '
                    f'({self.robot_xy[0]:.2f}, {self.robot_xy[1]:.2f})'
                )
            
            elapsed = (now - self.scan_start_time).nanoseconds / 1e9
            
            # Check for ball during scan - immediate response
            if len(self.balls) > 0:
                self.get_logger().info('[exploration_ball_collector] Ball detected during scan!')
                self.state = self.STATE_COLLECTING_BALL
                self.scan_start_time = None
                self.current_waypoint_position = None
                self.cancel_current_goal()
            elif elapsed >= self.scan_duration:
                # Scan completed, continue exploration
                self.get_logger().info(
                    f'[exploration_ball_collector] Scan completed after {elapsed:.1f}s, continuing exploration'
                )
                self.state = self.STATE_EXPLORING
                self.scan_start_time = None
                self.current_waypoint_position = None
            else:
                # Still scanning, don't process other logic
                return
        
        # Check if there is a ball (consider timeout and active navigation)
        has_ball = len(self.balls) > 0
        
        # If in collecting mode, be more lenient about switching back
        if self.state == self.STATE_COLLECTING_BALL:
            # If we have an active goal to a ball, keep in collecting mode
            if self.current_goal_handle is not None:
                # Keep in collecting mode while navigating to ball
                has_ball = True
                self.get_logger().debug(
                    '[exploration_ball_collector] Navigating to ball, staying in COLLECTING_BALL mode'
                )
            elif not has_ball and self.last_ball_time is not None:
                # No current detection, but check timeout
                elapsed = (now - self.last_ball_time).nanoseconds / 1e9
                if elapsed < self.ball_timeout:
                    # Still within timeout, keep trying
                    has_ball = True
                    self.get_logger().debug(
                        f'[exploration_ball_collector] No ball detection but within timeout '
                        f'({elapsed:.1f}s / {self.ball_timeout:.1f}s), staying in COLLECTING_BALL mode'
                    )
                else:
                    # Timeout reached, clear ball info and switch to exploration
                    self.balls = []
                    self.last_ball_time = None
                    has_ball = False
                    self.get_logger().info(
                        f'[exploration_ball_collector] Ball detection timeout after {elapsed:.1f}s, '
                        f'switching to EXPLORING mode'
                    )
        
        if has_ball:
            # Ball detected: switch to ball collection mode
            if self.state != self.STATE_COLLECTING_BALL:
                self.get_logger().info('[exploration_ball_collector] Switching to COLLECTING_BALL mode')
                self.state = self.STATE_COLLECTING_BALL
                self.scan_start_time = None  # Cancel any ongoing scan
                self.current_waypoint_position = None
                self.cancel_current_goal()
            
            # Send ball goal
            if self.current_goal_handle is None:
                if self.balls:
                    # Select the nearest ball
                    distances = [np.linalg.norm(np.array(b) - self.robot_xy) for b in self.balls]
                    nearest_idx = np.argmin(distances)
                    self.send_ball_goal(self.balls[nearest_idx])
                elif self.last_ball_time is not None:
                    # No current balls but within timeout - wait for detection
                    self.get_logger().debug(
                        '[exploration_ball_collector] Waiting for ball detection or timeout...'
                    )
        else:
            # No ball: exploration mode
            # Strategy 5: Check if arrived at waypoint for scanning
            if self.scan_at_waypoint and self.state == self.STATE_EXPLORING:
                if self.current_goal_handle is None and self.exploration_waypoints:
                    # Check if we're at a waypoint
                    for wp in self.exploration_waypoints:
                        wp_xy = np.array([wp.position.x, wp.position.y])
                        dist = np.linalg.norm(wp_xy - self.robot_xy)
                        
                        # Check if already visited
                        is_visited = False
                        for vp in self.visited_waypoints:
                            if np.linalg.norm(np.array(vp) - wp_xy) < self.visit_threshold:
                                is_visited = True
                                break
                        
                        # If arrived at unvisited waypoint, start scanning
                        if not is_visited and dist < self.waypoint_arrival_threshold:
                            self.state = self.STATE_SCANNING
                            self.scan_start_time = None  # Will be set in next tick
                            self.current_waypoint_position = wp_xy
                            self.get_logger().info(
                                f'[exploration_ball_collector] Arrived at waypoint '
                                f'({wp_xy[0]:.2f}, {wp_xy[1]:.2f}), entering scan mode'
                            )
                            return  # Wait for scan to complete
            
            # Switch to exploration mode
            if self.state != self.STATE_EXPLORING and self.state != self.STATE_SCANNING:
                if self.current_goal_handle is None:
                    self.get_logger().info('[exploration_ball_collector] Switching to EXPLORING mode')
                    self.state = self.STATE_EXPLORING
                    self.cancel_current_goal()
            
            # Send exploration waypoint
            if self.current_goal_handle is None and self.state == self.STATE_EXPLORING and self.exploration_waypoints:
                # Filter visited waypoints
                unvisited = []
                for wp in self.exploration_waypoints:
                    wp_xy = [wp.position.x, wp.position.y]
                    is_visited = False
                    for vp in self.visited_waypoints:
                        if np.linalg.norm(np.array(wp_xy) - np.array(vp)) < self.visit_threshold:
                            is_visited = True
                            break
                    if not is_visited:
                        unvisited.append(wp)
                
                if unvisited:
                    # Select the nearest unvisited waypoint
                    distances = [
                        np.linalg.norm(np.array([wp.position.x, wp.position.y]) - self.robot_xy)
                        for wp in unvisited
                    ]
                    nearest_idx = np.argmin(distances)
                    nearest_wp = unvisited[nearest_idx]
                    
                    # Send sequence starting from nearest waypoint
                    start_idx = unvisited.index(nearest_wp)
                    waypoints_to_send = unvisited[start_idx:]
                    
                    self.send_exploration_waypoints(waypoints_to_send)
                else:
                    self.get_logger().info(
                        '[exploration_ball_collector] All waypoints visited, regenerating...'
                    )
                    self.visited_waypoints = []


def main():
    rclpy.init()
    node = ExplorationBallCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()