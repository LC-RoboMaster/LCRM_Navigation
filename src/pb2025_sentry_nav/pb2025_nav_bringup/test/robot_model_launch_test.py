import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
import yaml


BRINGUP = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("reality_model_launch", BRINGUP / "launch/rm_navigation_reality_launch.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ModelLaunchTest(unittest.TestCase):
    def test_legacy_model_keeps_user_params(self):
        context = LaunchContext()
        context.launch_configurations.update(robot_name="lc2025_sentry_robot", params_file="custom.yaml")
        self.assertEqual(MODULE.resolve_robot_model_params(context), [])
        self.assertEqual(context.launch_configurations["params_file"], "custom.yaml")

    def test_model_overrides_all_consumers_and_preserves_tuning(self):
        source = BRINGUP / "config/reality/nav2_params.yaml"
        original = source.read_bytes()
        context = LaunchContext()
        context.launch_configurations.update(
            robot_name="lc_sentry2027", params_file=str(source),
            rviz_config_file=str(BRINGUP / "rviz/nav2_default_view.rviz"),
        )
        with patch.object(MODULE, "get_package_share_directory", return_value=str(BRINGUP)):
            for action in MODULE.resolve_robot_model_params(context):
                action.execute(context)
        output = Path(context.launch_configurations["params_file"])
        try:
            config = yaml.safe_load(output.read_text())
            old = yaml.safe_load(source.read_text())
            self.assertEqual(config["livox_ros_driver2"]["ros__parameters"]["frame_id"], "mid360_link")
            for name in ("loam_interface", "sensor_scan_generation", "small_gicp_relocalization"):
                self.assertEqual(config[name]["ros__parameters"]["lidar_frame"], "mid360_imu")
            self.assertEqual(config["terrain_analysis"]["ros__parameters"]["sensor_frame"], "mid360_imu")
            self.assertEqual(config["local_costmap"]["local_costmap"]["ros__parameters"]["robot_base_frame"], "yaw_link_fake")
            self.assertEqual(config["global_costmap"]["global_costmap"]["ros__parameters"]["robot_base_frame"], "yaw_link_fake")
            self.assertEqual(config["fake_vel_transform"]["ros__parameters"]["robot_base_frame"], "yaw_link")
            self.assertNotIn("front_mid360", output.read_text())
            self.assertNotIn("gimbal_yaw", output.read_text())
            self.assertEqual(config["controller_server"], old["controller_server"])
            self.assertEqual(config["point_lio"], old["point_lio"])
            self.assertEqual(source.read_bytes(), original)
            rviz = Path(context.launch_configurations["rviz_config_file"])
            self.assertNotIn("gimbal_yaw", rviz.read_text())
            self.assertIn("yaw_link_fake", rviz.read_text())
        finally:
            output.unlink()
            Path(context.launch_configurations["rviz_config_file"]).unlink()

    def test_new_driver_keeps_network_but_removes_old_mount_correction(self):
        directory = BRINGUP / "config/reality"
        old = json.loads((directory / "mid360_user_config.json").read_text())
        new = json.loads((directory / "lc_sentry2027_mid360.json").read_text())
        self.assertEqual(new["MID360"], old["MID360"])
        for lidar in new["lidar_configs"]:
            self.assertEqual(set(lidar["extrinsic_parameter"].values()), {0.})


if __name__ == "__main__":
    unittest.main()
