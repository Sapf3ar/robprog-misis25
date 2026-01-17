import threading
import time
import unittest

import launch
import launch_ros
import launch_testing

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, Float32, String


def generate_test_description():
    uut = launch_ros.actions.Node(
        package="bt_demo",
        executable="bt_executor_node",
        name="bt_executor",
        output="screen",
        parameters=[{"tick_hz": 20.0}, {"low_battery_threshold": 0.2}],
    )

    return (
        launch.LaunchDescription(
            [
                uut,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"uut": uut},
    )


class TestBTIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("bt_test_client")
        cls.exec = SingleThreadedExecutor()
        cls.exec.add_node(cls.node)

        cls._active_msgs = []
        cls.node.create_subscription(
            String,
            "/bt_demo/debug/active_top",
            lambda m: cls._active_msgs.append(m.data),
            10,
        )

        cls.pub_estop = cls.node.create_publisher(Bool, "/bt_demo/safety/estop", 10)
        cls.pub_bat = cls.node.create_publisher(Float32, "/bt_demo/battery/level", 10)
        cls.pub_task = cls.node.create_publisher(Bool, "/bt_demo/task/has_task", 10)

        cls._spin_thread = threading.Thread(target=cls.exec.spin, daemon=True)
        cls._spin_thread.start()

        time.sleep(0.5)

    @classmethod
    def tearDownClass(cls):
        cls.exec.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for_active(self, expected: str, timeout_s: float = 5.0):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if any(m == expected for m in self._active_msgs[-50:]):
                return
            time.sleep(0.05)
        self.fail(f"timeout waiting for active_top == {expected}, last={self._active_msgs[-10:]}")

    def test_order_then_charge_then_safety(self):
        self._active_msgs.clear()

        msg = Bool(); msg.data = False
        self.pub_estop.publish(msg)

        msg = Float32(); msg.data = 0.6
        self.pub_bat.publish(msg)

        msg = Bool(); msg.data = True
        self.pub_task.publish(msg)

        self._wait_for_active("Seq: Order")

        msg = Float32(); msg.data = 0.1
        self.pub_bat.publish(msg)

        self._wait_for_active("Seq: Charge")

        msg = Bool(); msg.data = True
        self.pub_estop.publish(msg)

        self._wait_for_active("Seq: Safety")

