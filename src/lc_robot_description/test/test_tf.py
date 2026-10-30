import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException


ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / "src/pb2025_sentry_nav/pb2025_nav_bringup/launch/robot_state_publisher_launch.py"


class LiveTfTest(unittest.TestCase):
    def test_default_and_namespaced_model_with_real_joint_input(self):
        rclpy.init()
        try:
            for namespace in ("", "lc_model_test"):
                with self.subTest(namespace=namespace):
                    self.check_model(namespace)
        finally:
            rclpy.shutdown()

    def check_model(self, namespace):
        prefix = "/" + namespace if namespace else ""
        with tempfile.TemporaryFile(mode="w+") as log:
            process = subprocess.Popen([
                "ros2", "launch", str(WRAPPER), "robot_name:=lc_sentry2027",
                f"namespace:={namespace}", "use_sim_time:=false",
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            node = rclpy.create_node("model_tf_test", cli_args=[
                "--ros-args", "-r", f"/tf:={prefix}/tf", "-r", f"/tf_static:={prefix}/tf_static",
            ])
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            pub = node.create_publisher(JointState, f"{prefix}/serial/gimbal_joint_state", 10)
            try:
                deadline = time.monotonic() + 20
                received = False
                while time.monotonic() < deadline and process.poll() is None:
                    msg = JointState()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.name = ["yaw_joint"]
                    msg.position = [.35]
                    pub.publish(msg)
                    rclpy.spin_once(node, timeout_sec=.05)
                    try:
                        transform = buffer.lookup_transform("base_link", "yaw_link", rclpy.time.Time())
                        q = transform.transform.rotation
                        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                        buffer.lookup_transform("base_footprint", "mid360_link", rclpy.time.Time())
                        imu = buffer.lookup_transform("mid360_link", "mid360_imu", rclpy.time.Time())
                        if abs(yaw - (.35 - .043174)) < 1e-5:
                            self.assertAlmostEqual(imu.transform.translation.x, .011)
                            self.assertAlmostEqual(imu.transform.translation.y, .02329)
                            self.assertAlmostEqual(imu.transform.translation.z, -.04412)
                            received = True
                            break
                    except TransformException:
                        pass
                log.seek(0)
                self.assertTrue(received, log.read())
            finally:
                listener.unregister()
                node.destroy_node()
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
