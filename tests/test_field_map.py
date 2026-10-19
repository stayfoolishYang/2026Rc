"""Field coordinates to existing serial modes, without opening a real port."""
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from upper_host_v86.MoveControl import MoveControl
from upper_host_v86.field_map import HOME_INNER_EDGE_MM, ESTIMATED_MARKERS_MM, relative_move_m


class FakeSerial:
    def __init__(self):
        self.frames = []

    def write(self, frame):
        self.frames.append(frame)

    def close(self):
        pass


class FieldMapTests(unittest.TestCase):
    def test_home_to_marker_uses_y_then_x_and_keeps_centimeters(self):
        control = MoveControl.__new__(MoveControl)
        control.serial = FakeSerial()
        control.send_buffer = [0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0xFE]
        control.last_buffer = list(control.send_buffer)
        control._MoveControl__wait_for_movement_done = lambda: True
        delta = relative_move_m(HOME_INNER_EDGE_MM, ESTIMATED_MARKERS_MM["场内左上"])
        self.assertEqual(delta, {"x": 1.09, "y": -0.64})
        with patch.object(time, "sleep"):
            control.set_distance(**delta)
            control.set_distance(x=0.21)
        self.assertEqual(control.serial.frames, [
            bytes.fromhex("FF 02 FF C0 00 00 00 00 00 FE"),
            bytes.fromhex("FF 01 00 6D 00 00 00 00 00 FE"),
            bytes.fromhex("FF 01 00 15 00 00 00 00 00 FE"),
        ])


if __name__ == "__main__":
    unittest.main()
