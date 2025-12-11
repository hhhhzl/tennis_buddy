#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert yaw angle to quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class DeterministicCollector(Node):
    STATE_EXPLORING  = 'EXPLORING'
    STATE_SCANNING   = 'SCANNING'
    STATE_COLLECTING = 'COLLECTING'

    def __init__(self):
        super().__init__('deterministic_ball_collector')

        # Frame + thresholds
        self.frame_id = 'map'

        # Geometric arrival threshold for ball pickup (meters)
        self.pickup_d = 0.30

        # Distance below which two ball detections are treated as duplicates
        self.dup_thresh = 0.25

        # Scan settings
        self.scan_duration = 2.0   # seconds to spend scanning at each waypoint
        self.last_nav_status = None
        # State
        self.state = self.STATE_EXPLORING

        self.robot_xy = None
        self.robot_yaw = None

        self.target_ball = None          # np.array([x, y]) for current target
        self.ball_queue = []             # list of np.array([x, y])

        self.exploration_waypoints = []  # list of Pose
        self.explore_idx = 0             # current exploration waypoint index

        self.current_goal_handle = None  # Nav2 goal handle
        self.scan_start_time = None

        # Subscribers
        self.create_subscription(PoseArray, "/ball_positions", self.on_balls, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_odom, 10)
        self.create_subscription(PoseArray, "/exploration_waypoints", self.on_wps, 10)

        # Action client
        self.ac_pose = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for navigate_to_pose server...")
        self.ac_pose.wait_for_server()

        # Main loop timer
        self.create_timer(0.2, self.tick)

        self.get_logger().info("Deterministic scanning ball collector READY")

    # ======================================================================
    #  Basic updates
    # ======================================================================
    def on_odom(self, msg: Odometry):
        self.robot_xy = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y],
            dtype=float
        )
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def on_wps(self, msg: PoseArray):
        new_wps = list(msg.poses)
        first_time = len(self.exploration_waypoints) == 0
        self.exploration_waypoints = new_wps
        if first_time:
            self.explore_idx = 0
            self.get_logger().info(
                f"Received {len(self.exploration_waypoints)} exploration waypoints."
            )

    # ======================================================================
    #  Duplicate ball check
    # ======================================================================
    def is_duplicate_ball(self, ball_xy) -> bool:
        """Return True if ball_xy is effectively same as current target or queued."""
        b = np.array(ball_xy, dtype=float)

        # Compare with current target
        if self.target_ball is not None:
            if np.linalg.norm(b - self.target_ball) < self.dup_thresh:
                return True

        # Compare with queued balls
        for qb in self.ball_queue:
            if np.linalg.norm(b - qb) < self.dup_thresh:
                return True

        return False

    # ======================================================================
    #  Ball detection callback (dedupe + queue + locking)
    # ======================================================================
    def on_balls(self, msg: PoseArray):
        if len(msg.poses) == 0:
            return

        detections = [np.array([p.position.x, p.position.y], dtype=float)
                      for p in msg.poses]

        # Filter out duplicates w.r.t. current target + queue
        deduped = []
        for d in detections:
            if not self.is_duplicate_ball(d):
                deduped.append(d)

        if len(deduped) == 0:
            return

        # If exploring or scanning → choose nearest ball and lock it
        if self.state in [self.STATE_EXPLORING, self.STATE_SCANNING]:
            if self.robot_xy is None:
                return

            dists = [np.linalg.norm(b - self.robot_xy) for b in deduped]
            primary_idx = int(np.argmin(dists))
            primary = deduped[primary_idx]
            extras = [deduped[i] for i in range(len(deduped)) if i != primary_idx]

            self.target_ball = primary
            for b in extras:
                if not self.is_duplicate_ball(b):
                    self.ball_queue.append(b)

            self.get_logger().info(
                f"LOCKED target ball [{primary[0]:.3f}, {primary[1]:.3f}], "
                f"queued {len(extras)} extra(s)."
            )

            # Cancel current exploration goal and switch to collecting
            self.cancel_goal()
            self.state = self.STATE_COLLECTING
            # Kick off collection towards this ball
            self.start_collecting_ball(self.target_ball)
            return

        # If already collecting, just queue non-duplicate detections
        if self.state == self.STATE_COLLECTING:
            for d in deduped:
                if not self.is_duplicate_ball(d):
                    self.ball_queue.append(d)

    # ======================================================================
    #  Navigation helpers
    # ======================================================================
    def cancel_goal(self):
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None

    def send_pose_goal(self, x: float, y: float, yaw: float):
        self.last_nav_status = None
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation = yaw_to_quat(yaw)
        goal.pose = ps

        self.get_logger().info(
            f"Sending NavigateToPose goal to ({x:.2f}, {y:.2f})"
        )

        send_future = self.ac_pose.send_goal_async(goal)
        send_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected by Nav2.")
            self.current_goal_handle = None
            return

        self.get_logger().info("Goal accepted by Nav2.")
        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        """Called when Nav2 finishes a goal (success, canceled, or failure)."""
        result_wrapper = future.result()
        status = result_wrapper.status
        self.last_nav_status = status
        _result = result_wrapper.result
        self.current_goal_handle = None

        # Collecting a ball
        if self.state == self.STATE_COLLECTING:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    "[COLLECTING] Nav2 reports ball goal succeeded, checking pickup..."
                )
                self.finish_or_retry_collection(force_success=True)
            else:
                self.get_logger().warn(
                    f"[COLLECTING] Ball goal ended with status={status}, "
                    f"checking distance / possible retry."
                )
                self.finish_or_retry_collection(force_success=False)
            return

        # Exploration waypoint goals
        if self.state == self.STATE_EXPLORING and self.target_ball is None:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Reached exploration waypoint, entering SCANNING.")
                self.state = self.STATE_SCANNING
                self.scan_start_time = None
            else:
                self.get_logger().warn(
                    f"Nav2 goal to exploration waypoint did not succeed (status={status}). "
                    "Skipping scan and moving to next waypoint."
                )
                # Move on to the next waypoint instead of scanning
                self.state = self.STATE_EXPLORING
                self.scan_start_time = None
                self.explore_idx += 1
                self.send_next_waypoint()

    # ======================================================================
    #  Waypoint helpers
    # ======================================================================
    def send_next_waypoint(self):
        """Send a goal to the next exploration waypoint."""
        if not self.exploration_waypoints:
            return

        if self.explore_idx >= len(self.exploration_waypoints):
            self.explore_idx = 0  # loop patrol

        wp = self.exploration_waypoints[self.explore_idx]
        gx = wp.position.x
        gy = wp.position.y

        if self.robot_xy is not None:
            yaw = math.atan2(gy - self.robot_xy[1], gx - self.robot_xy[0])
        else:
            yaw = 0.0

        self.get_logger().info(
            f"Exploring: going to waypoint [{self.explore_idx}] at ({gx:.2f}, {gy:.2f})"
        )
        self.send_pose_goal(gx, gy, yaw)

    def nearest_waypoint_index(self):
        """Return index of the waypoint nearest to current robot position."""
        if not self.exploration_waypoints or self.robot_xy is None:
            return 0

        dists = []
        for i, wp in enumerate(self.exploration_waypoints):
            xy = np.array([wp.position.x, wp.position.y], dtype=float)
            d = np.linalg.norm(xy - self.robot_xy)
            dists.append((d, i))

        dists.sort(key=lambda x: x[0])
        return dists[0][1]

    # ======================================================================
    #  Ball queue helpers
    # ======================================================================
    def pop_next_ball(self):
        if len(self.ball_queue) == 0:
            return None
        return self.ball_queue.pop(0)

    def start_collecting_ball(self, ball_xy):
        self.target_ball = np.array(ball_xy, dtype=float)
        self.state = self.STATE_COLLECTING
        self.cancel_goal()
        bx, by = self.target_ball
        self.get_logger().info(
            f"Starting collection for ball at ({bx:.2f}, {by:.2f})"
        )

        # Send a goal toward the ball
        if self.robot_xy is not None:
            yaw = math.atan2(by - self.robot_xy[1], bx - self.robot_xy[0])
        else:
            yaw = 0.0
        self.send_pose_goal(bx, by, yaw)

    def finish_or_retry_collection(self, force_success: bool) -> None:
        """Decide if the ball is collected or needs another attempt."""
        if self.target_ball is None or self.robot_xy is None:
            # Nothing to do, fall back to exploration
            self.state = self.STATE_EXPLORING
            return

        tb = self.target_ball
        dist = np.linalg.norm(tb - self.robot_xy)

        # If Nav2 says success, or we're within pickup_d, treat as collected
        if force_success or (self.last_nav_status == GoalStatus.STATUS_SUCCEEDED):
            self.get_logger().info(
                f"Collected ball at ({tb[0]:.2f}, {tb[1]:.2f}). "
                f"dist={dist:.2f}, force_success={force_success}"
            )
            self.target_ball = None
            self.current_goal_handle = None

            # If more balls in queue → collect next
            next_ball = self.pop_next_ball()
            if next_ball is not None:
                self.start_collecting_ball(next_ball)
                return

            # No more balls → go back to nearest waypoint and resume scanning loop
            idx = self.nearest_waypoint_index()
            self.explore_idx = idx
            wp = self.exploration_waypoints[idx] if self.exploration_waypoints else None
            if wp is not None:
                gx, gy = wp.position.x, wp.position.y
                yaw = math.atan2(gy - self.robot_xy[1], gx - self.robot_xy[0])
                self.get_logger().info(
                    f"Returning to nearest waypoint [{idx}] at ({gx:.2f}, {gy:.2f})"
                )
                self.state = self.STATE_EXPLORING
                self.send_pose_goal(gx, gy, yaw)
            else:
                # No waypoints, just idle in exploring
                self.state = self.STATE_EXPLORING
            return

        # Not close enough and not forced → try again
        self.get_logger().info(
            f"Ball not yet within pickup distance (dist={dist:.2f}), NAV2."
        )
        return
    # ======================================================================
    #  Main tick: state machine
    # ======================================================================
    def tick(self):
        if self.robot_xy is None:
            return

        # --------------------------------------------------------------
        # STATE: COLLECTING
        # --------------------------------------------------------------
        if self.state == self.STATE_COLLECTING:
            # Backup: if for some reason we never got a Nav2 result, still
            # allow geometric completion here.
            # self.finish_or_retry_collection(force_success=False)
            return

        # --------------------------------------------------------------
        # STATE: SCANNING
        # --------------------------------------------------------------
        if self.state == self.STATE_SCANNING:
            if self.scan_start_time is None:
                self.scan_start_time = self.get_clock().now()
                self.get_logger().info("Scanning at waypoint...")
                return

            elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9

            # If during scanning we see a ball, on_balls() will have already
            # switched us to COLLECTING. In that case, do nothing here.
            if self.state != self.STATE_SCANNING:
                return

            if elapsed >= self.scan_duration:
                # Done scanning this waypoint → go to next
                self.get_logger().info(
                    f"Scan complete after {elapsed:.1f}s, resuming exploration."
                )
                self.state = self.STATE_EXPLORING
                self.scan_start_time = None
                self.explore_idx += 1
                self.send_next_waypoint()
            return

        # --------------------------------------------------------------
        # STATE: EXPLORING
        # --------------------------------------------------------------
        if self.state == self.STATE_EXPLORING:
            # If no active goal, it means either:
            # - just started, or
            # - finished returning to a waypoint, or
            # - finished previous waypoint and scan is over.
            if self.current_goal_handle is None and self.exploration_waypoints:
                self.send_next_waypoint()
            return


def main():
    rclpy.init()
    node = DeterministicCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

