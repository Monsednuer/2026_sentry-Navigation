#!/usr/bin/env python3

from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PathToFollowPathActionBridge(Node):
    def __init__(self) -> None:
        super().__init__("path_to_follow_path_bridge")

        self.path_topic = self.declare_parameter("path_topic", "/sPath").value
        self.action_name = self.declare_parameter("action_name", "/follow_path").value
        self.controller_id = self.declare_parameter("controller_id", "FollowPath").value
        self.goal_checker_id = self.declare_parameter(
            "goal_checker_id", "general_goal_checker"
        ).value
        self.min_points = int(self.declare_parameter("min_points", 2).value)
        self.min_send_interval_sec = float(
            self.declare_parameter("min_send_interval_sec", 0.3).value
        )
        self.republish_if_same = bool(
            self.declare_parameter("republish_if_same", False).value
        )
        self.signature_decimals = int(
            self.declare_parameter("signature_decimals", 2).value
        )

        self._action_client = ActionClient(self, FollowPath, self.action_name)
        self._path_sub = self.create_subscription(Path, self.path_topic, self._path_cb, 10)
        self._timer = self.create_timer(0.1, self._try_send_goal)

        self._latest_path: Optional[Path] = None
        self._latest_sig: Optional[str] = None
        self._last_sent_sig: Optional[str] = None
        self._last_send_time = self.get_clock().now() - Duration(seconds=3600.0)
        self._goal_seq = 0
        self._latest_goal_seq = 0
        self._waiting_server_logged = False

        self.get_logger().info(
            f"Bridge started: path_topic={self.path_topic}, action={self.action_name}, "
            f"controller_id={self.controller_id}, goal_checker_id={self.goal_checker_id}"
        )

    def _path_cb(self, msg: Path) -> None:
        if len(msg.poses) < self.min_points:
            self.get_logger().warn(
                f"Ignore path with too few poses: {len(msg.poses)} < {self.min_points}"
            )
            return

        self._latest_path = msg
        self._latest_sig = self._path_signature(msg)

    def _path_signature(self, path: Path) -> str:
        n = len(path.poses)
        if n == 0:
            return "empty"

        d = self.signature_decimals
        sample_count = min(20, n)
        step = max(1, n // sample_count)
        chunks = [str(n)]
        for i in range(0, n, step):
            p = path.poses[i].pose.position
            chunks.append(f"{round(p.x, d)}:{round(p.y, d)}")
        p_last = path.poses[-1].pose.position
        chunks.append(f"end={round(p_last.x, d)}:{round(p_last.y, d)}")
        return "|".join(chunks)

    def _try_send_goal(self) -> None:
        if self._latest_path is None or self._latest_sig is None:
            return

        if (not self.republish_if_same) and (self._latest_sig == self._last_sent_sig):
            return

        if not self._action_client.wait_for_server(timeout_sec=0.0):
            if not self._waiting_server_logged:
                self.get_logger().warn(
                    f"Waiting for FollowPath action server: {self.action_name}"
                )
                self._waiting_server_logged = True
            return
        self._waiting_server_logged = False

        now = self.get_clock().now()
        if (now - self._last_send_time).nanoseconds < int(
            self.min_send_interval_sec * 1e9
        ):
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = self._latest_path
        goal_msg.controller_id = self.controller_id
        goal_msg.goal_checker_id = self.goal_checker_id

        self._goal_seq += 1
        goal_seq = self._goal_seq
        self._latest_goal_seq = goal_seq
        self._last_send_time = now

        self.get_logger().info(
            f"Send FollowPath goal #{goal_seq}, poses={len(goal_msg.path.poses)}"
        )
        send_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        send_future.add_done_callback(
            lambda fut, seq=goal_seq, sig=self._latest_sig: self._goal_response_cb(
                fut, seq, sig
            )
        )

    def _goal_response_cb(self, future, goal_seq: int, path_sig: str) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"FollowPath goal #{goal_seq} rejected")
            return

        self.get_logger().info(f"FollowPath goal #{goal_seq} accepted")
        self._last_sent_sig = path_sig
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, seq=goal_seq: self._result_cb(fut, seq)
        )

    def _feedback_cb(self, _feedback_msg) -> None:
        # Keep quiet to avoid high-frequency logs.
        return

    def _result_cb(self, future, goal_seq: int) -> None:
        result_wrapper = future.result()
        status = result_wrapper.status if result_wrapper is not None else None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"FollowPath goal #{goal_seq} succeeded")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f"FollowPath goal #{goal_seq} aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"FollowPath goal #{goal_seq} canceled")
        elif status is not None:
            self.get_logger().warn(
                f"FollowPath goal #{goal_seq} finished with status={status}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathToFollowPathActionBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
