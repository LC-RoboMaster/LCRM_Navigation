"""Exercise the real launch resolver; no GUI or simulated sensor is required."""
import json
from pathlib import Path
import runpy
import sys
import tempfile
import unittest

from launch import LaunchContext

source = Path(sys.argv.pop(1))
launch = runpy.run_path(str(source))
validate = launch["validate_passage_map_reference"]
resolve = launch["resolve_passage_file"]


class MapReferenceTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.map = self.root / "field.yaml"
        self.map.write_text("image: field.pgm\nresolution: 0.05\norigin: [-3.58, -9.44, 0]\n")
        self.channel = self.root / "field.passages.json"
        self.reference = {"origin": [-3.58, -9.44], "yaw": 0, "resolution": 0.05}

    def tearDown(self):
        self.directory.cleanup()

    def context(self, passage="auto", slam="false"):
        context = LaunchContext()
        context.launch_configurations.update(passage_file=passage, map=str(self.map), slam=slam)
        return context

    def test_matching_reference_and_auto_sidecar(self):
        self.channel.write_text(json.dumps({"map_reference": self.reference}))
        validate(self.channel, self.map)
        context = self.context()
        for action in resolve(context):
            action.execute(context)
        self.assertEqual(context.launch_configurations["passage_file"], str(self.channel))

    def test_legacy_image_origin_is_rejected(self):
        self.reference["origin"] = [0, 0]
        self.channel.write_text(json.dumps({"map": self.reference}))
        with self.assertRaisesRegex(RuntimeError, "reference mismatch"):
            resolve(self.context())

    def test_online_mapping_and_missing_auto_sidecar_stay_compatible(self):
        for context in (self.context(), self.context(slam="true")):
            for action in resolve(context):
                action.execute(context)
            self.assertEqual(context.launch_configurations["passage_file"], "")

    def test_old_semantic_file_without_reference_stays_compatible(self):
        self.channel.write_text(json.dumps({"schema_version": 1, "passages": []}))
        validate(self.channel, self.map)


if __name__ == "__main__":
    unittest.main()
