from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "src/lc_robot_description"


class ModelTest(unittest.TestCase):
    def setUp(self):
        self.model = ET.parse(PACKAGE / "urdf/lc_sentry2027.urdf").getroot()

    def test_original_geometry_and_joints_preserved(self):
        original = ET.parse(ROOT / "urdf_temp/sentry/urdf/sentry.urdf").getroot()
        for element in original:
            migrated = self.model.find(f"{element.tag}[@name='{element.attrib['name']}']")
            self.assertIsNotNone(migrated)
            element.tail = migrated.tail = None
            expected = ET.tostring(element).replace(
                b"package://sentry/meshes/",
                b"package://lc_robot_description/meshes/lc_sentry2027/",
            )
            self.assertEqual(ET.tostring(migrated), expected)

    def test_one_tree_and_no_odometry_or_fake_frame(self):
        links = {link.attrib["name"] for link in self.model.findall("link")}
        children = [joint.find("child").attrib["link"] for joint in self.model.findall("joint")]
        self.assertEqual(len(children), len(set(children)))
        self.assertEqual(links - set(children), {"base_footprint"})
        self.assertNotIn("odom", links)
        self.assertNotIn("yaw_link_fake", links)
        self.assertNotIn("gimbal_yaw_fake", links)
        self.assertEqual(self.model.attrib["name"], "lc_sentry2027")

    def test_navigation_aliases_preserve_sensor_and_gimbal_origins(self):
        for parent, child in (("yaw_link", "gimbal_yaw"), ("mid360_link", "front_mid360")):
            joint = next(j for j in self.model.findall("joint") if j.find("child").attrib["link"] == child)
            self.assertEqual(joint.attrib["type"], "fixed")
            self.assertEqual(joint.find("parent").attrib["link"], parent)
            for key in ("xyz", "rpy"):
                self.assertEqual([float(x) for x in joint.find("origin").attrib[key].split()], [0., 0., 0.])

    def test_imu_frame_matches_point_lio_internal_extrinsic(self):
        params = yaml.safe_load((ROOT / "src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml").read_text())
        mapping = params["point_lio"]["ros__parameters"]["mapping"]
        joint = self.model.find("joint[@name='mid360_to_imu']")
        xyz = [float(x) for x in joint.find("origin").attrib["xyz"].split()]
        self.assertEqual(xyz, [-x for x in mapping["extrinsic_T"]])
        self.assertEqual(mapping["extrinsic_R"], [1., 0., 0., 0., 1., 0., 0., 0., 1.])
        self.assertFalse(mapping["extrinsic_est_en"])

    def test_mesh_resources_installed_under_real_package(self):
        filenames = set()
        for mesh in self.model.iter("mesh"):
            uri = mesh.attrib["filename"]
            self.assertTrue(uri.startswith("package://lc_robot_description/"))
            path = PACKAGE / uri.removeprefix("package://lc_robot_description/")
            self.assertTrue(path.is_file(), str(path))
            original = ROOT / "urdf_temp/sentry/meshes" / path.name
            self.assertEqual(path.read_bytes(), original.read_bytes())
            filenames.add(path.name)
        self.assertEqual(len(filenames), 10)


if __name__ == "__main__":
    unittest.main()
