#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32, String

import py_trees
from py_trees.common import Access, Status

from bt_demo.bt_tree import (
    BlackboardKeys,
    Topics,
    build_tree,
    get_active_top_branch_name,
)


class BTExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("bt_executor")

        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("low_battery_threshold", 0.2)

        tick_hz = float(self.get_parameter("tick_hz").value)
        low_bat = float(self.get_parameter("low_battery_threshold").value)

        self.tree: py_trees.trees.BehaviourTree = build_tree(low_battery_threshold=low_bat)

        self.bb = py_trees.blackboard.Client(name="bt_executor_bb")
        self.bb.register_key(key=BlackboardKeys.ESTOP, access=Access.WRITE)
        self.bb.register_key(key=BlackboardKeys.BATTERY_LEVEL, access=Access.WRITE)
        self.bb.register_key(key=BlackboardKeys.HAS_TASK, access=Access.WRITE)

        self._estop = False
        self._battery = 1.0
        self._has_task = False
        self._last_active_top: str | None = None

        self._last_status: dict[str, Status] = {}

        self.pub_active = self.create_publisher(String, Topics.active_top, 10)

        self.create_subscription(Bool, Topics.estop, self._on_estop, 10)
        self.create_subscription(Float32, Topics.battery_level, self._on_battery, 10)
        self.create_subscription(Bool, Topics.has_task, self._on_has_task, 10)

        self.timer = self.create_timer(1.0 / tick_hz, self._on_tick)

        self.get_logger().info(
            f"bt_executor started: tick_hz={tick_hz}, low_battery_threshold={low_bat}"
        )
        self.get_logger().info(
            f"publish: {Topics.active_top} | subscribe: {Topics.estop}, {Topics.battery_level}, {Topics.has_task}"
        )

    def _on_estop(self, msg: Bool) -> None:
        v = bool(msg.data)
        if v != self._estop:
            self._estop = v
            self.get_logger().info(f"[bb] estop -> {self._estop}")
        setattr(self.bb, BlackboardKeys.ESTOP, v)

    def _on_battery(self, msg: Float32) -> None:
        v = float(msg.data)
        if abs(v - self._battery) > 1e-6:
            self._battery = v
            self.get_logger().info(f"[bb] battery -> {self._battery:.3f}")
        setattr(self.bb, BlackboardKeys.BATTERY_LEVEL, v)

    def _on_has_task(self, msg: Bool) -> None:
        v = bool(msg.data)
        if v != self._has_task:
            self._has_task = v
            self.get_logger().info(f"[bb] has_task -> {self._has_task}")
        setattr(self.bb, BlackboardKeys.HAS_TASK, v)

    def _iter_tree_nodes(self, root: py_trees.behaviour.Behaviour):
        stack = [root]
        while stack:
            node = stack.pop()
            yield node
            for child in getattr(node, "children", [])[::-1]:
                stack.append(child)

    def _log_status_changes(self):
        changes = []
        for node in self._iter_tree_nodes(self.tree.root):
            name = node.name
            st = node.status
            last = self._last_status.get(name, None)
            if last is None:
                self._last_status[name] = st
                continue
            if st != last:
                self._last_status[name] = st
                changes.append((name, last, st))

        if changes:
            for (name, last, st) in changes:
                self.get_logger().info(f"[bt] {name}: {last.name} -> {st.name}")

    def _on_tick(self) -> None:
        self.tree.tick()

        active = get_active_top_branch_name(self.tree)
        out = String()
        out.data = active
        self.pub_active.publish(out)

        if active != self._last_active_top:
            self.get_logger().info(
                f"[bt] active_top: {self._last_active_top} -> {active} | estop={self._estop} battery={self._battery:.3f} has_task={self._has_task}"
            )
            self._last_active_top = active

        self._log_status_changes()


def main() -> None:
    rclpy.init()
    node = BTExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

