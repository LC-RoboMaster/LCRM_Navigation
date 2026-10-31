import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException


ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / "src/pb2025_sentry_nav/pb2025_nav_bringup/launch/robot_state_publisher_launch.py"


class LiveTfTest(unittest.TestCase):
    def test_default_and_namespaced_model_with_joint_input_and_fake_velocity_tf(self):
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
            command = [
                "ros2", "launch", str(WRAPPER),
                "use_sim_time:=false",
            ]
            if namespace:
                command.append(f"namespace:={namespace}")
            processes = [subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)]
            fake_command = [
                "ros2", "run", "fake_vel_transform", "fake_vel_transform_node", "--ros-args",
                "-r", f"__ns:={prefix or '/'}", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static",
            ]
            for parameter in (
                "use_sim_time:=false", "robot_base_frame:=gimbal_yaw",
                "fake_robot_base_frame:=gimbal_yaw_fake", "odom_topic:=odometry",
                "input_cmd_vel_topic:=cmd_vel_nav2_result", "output_cmd_vel_topic:=cmd_vel",
                "init_spin_speed:=0.0",
            ):
                fake_command.extend(["-p", parameter])
            processes.append(subprocess.Popen(fake_command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            node = rclpy.create_node("model_tf_test", cli_args=[
                "--ros-args", "-r", f"/tf:={prefix}/tf", "-r", f"/tf_static:={prefix}/tf_static",
            ])
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            broadcaster = TransformBroadcaster(node)
            pub = node.create_publisher(JointState, f"{prefix}/serial/gimbal_joint_state", 10)
            odom_pub = node.create_publisher(Odometry, f"{prefix}/odometry", 10)
            try:
                for angle in (.35, -.2):
                    deadline = time.monotonic() + 20
                    received = False
                    while time.monotonic() < deadline and all(p.poll() is None for p in processes):
                        stamp = node.get_clock().now().to_msg()
                        msg = JointState()
                        msg.header.stamp = stamp
                        msg.name = ["yaw_joint"]
                        msg.position = [angle]
                        pub.publish(msg)
                        # Provide only the odometry edge normally produced by sensor_scan_generation.
                        odom_tf = TransformStamped()
                        odom_tf.header.stamp = stamp
                        odom_tf.header.frame_id = "odom"
                        odom_tf.child_frame_id = "base_footprint"
                        odom_tf.transform.rotation.w = 1.
                        broadcaster.sendTransform(odom_tf)
                        odom = Odometry()
                        odom.header.stamp = stamp
                        odom.header.frame_id = "odom"
                        odom.child_frame_id = "gimbal_yaw"
                        odom.pose.pose.orientation.z = math.sin((angle - .043174) / 2)
                        odom.pose.pose.orientation.w = math.cos((angle - .043174) / 2)
                        odom_pub.publish(odom)
                        rclpy.spin_once(node, timeout_sec=.05)
                        try:
                            transform = buffer.lookup_transform("base_link", "gimbal_yaw", rclpy.time.Time())
                            q = transform.transform.rotation
                            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                            for frame in ("front_mid360", "mid360_link", "mid360_imu", "gimbal_yaw_fake"):
                                buffer.lookup_transform("odom", frame, rclpy.time.Time())
                            fake = buffer.lookup_transform("odom", "gimbal_yaw_fake", rclpy.time.Time())
                            if abs(yaw - (angle - .043174)) < 1e-5 and abs(fake.transform.rotation.z) < 1e-5:
                                for parent, child in (("yaw_link", "gimbal_yaw"), ("mid360_link", "front_mid360")):
                                    alias = buffer.lookup_transform(parent, child, rclpy.time.Time()).transform
                                    for value in (alias.translation.x, alias.translation.y, alias.translation.z,
                                                  alias.rotation.x, alias.rotation.y, alias.rotation.z):
                                        self.assertAlmostEqual(value, 0.)
                                    self.assertAlmostEqual(alias.rotation.w, 1.)
                                imu = buffer.lookup_transform("mid360_link", "mid360_imu", rclpy.time.Time())
                                self.assertAlmostEqual(imu.transform.translation.x, .011)
                                self.assertAlmostEqual(imu.transform.translation.y, .02329)
                                self.assertAlmostEqual(imu.transform.translation.z, -.04412)
                                received = True
                                break
                        except TransformException:
                            pass
                    log.seek(0)
                    self.assertTrue(received, f"namespace={namespace}, yaw={angle}\n{log.read()}")
            finally:
                listener.unregister()
                node.destroy_node()
                for process in processes:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
