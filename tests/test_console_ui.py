"""Offline UI regression checks for navigation/filtering and no hardware effects."""
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import stm32_hardware_tuning_gui as gui


class ConsoleUITests(unittest.TestCase):
    def test_speed_values_outside_old_ranges_and_reversed_order_can_be_saved(self):
        with tempfile.TemporaryDirectory(prefix="rc_speed_limits_") as folder:
            source = Path(folder) / "controller.c"
            source.write_bytes(gui.DEFAULT_SOURCE.read_bytes())
            with patch.object(gui, "SerialWorker") as worker, \
                 patch.object(gui.list_ports, "comports", return_value=[]), \
                 patch.object(gui.messagebox, "askyesno", return_value=True), \
                 patch.object(gui.messagebox, "showinfo"), \
                 patch.object(gui.messagebox, "showerror") as errors:
                app = gui.HardwareTuningApp(source)
                app.withdraw()
                try:
                    changed = {
                        "CHASSIS_STEP_HALF_PERIOD_US": 20,
                        "CHASSIS_SHORT_STEP_HALF_PERIOD_US": 10,
                        "CHASSIS_RAMP_START_HALF_PERIOD_US": 10,
                        "CHASSIS_TASK2_LONG_STEP_HALF_PERIOD_US": 900,
                        "CHASSIS_SLOW_SHORT_STEP_HALF_PERIOD_US": 800,
                        "CHASSIS_ROTATE_SLOW_STEP_HALF_PERIOD_US": 1200,
                        "CHASSIS_TASK2_STOP_HALF_PERIOD_US": 100,
                        "LIFT_STEP_HALF_PERIOD_US": 5,
                        "LIFT_GROUND_SLOW_STEP_HALF_PERIOD_US": 3,
                        "CHASSIS_FINE_STEP_HALF_PERIOD_US": 2000,
                    }
                    for define, value in changed.items():
                        app.value_vars[define].set(str(value))
                    self.assertIsNotNone(app._validated_values())
                    self.assertTrue(app._save_source())
                    errors.assert_not_called()
                    saved, _ = gui.parse_source_parameters(source)
                    for define, value in changed.items():
                        self.assertEqual(saved[define], value)
                    app.value_vars["CHASSIS_STEP_HALF_PERIOD_US"].set("bad")
                    self.assertIsNone(app._validated_values())
                    app.value_vars["CHASSIS_STEP_HALF_PERIOD_US"].set("0")
                    self.assertIsNone(app._validated_values())
                    worker.return_value.request.assert_not_called()
                finally:
                    app.destroy()

    def test_navigation_and_filter_keep_input_and_safety_state(self):
        with patch.object(gui, "SerialWorker") as worker, patch.object(gui.list_ports, "comports", return_value=[]):
            app = gui.HardwareTuningApp(gui.DEFAULT_SOURCE)
            app.withdraw()
            try:
                app.value_vars["SERVO1_DISC_ALIGN_PULSE_US"].set("2440")
                app.action_armed_var.set(True)
                for index, button in enumerate(app.console_nav_buttons):
                    button.invoke()
                    app.update_idletasks()
                    self.assertEqual(app.notebook.index(app.notebook.select()), index)
                app.parameter_search_var.set("SERVO1_DISC_ALIGN_PULSE_US")
                app._filter_parameters()
                self.assertEqual(app.parameter_count_var.get(), "1 项")
                app.parameter_search_var.set("没有这个参数")
                app._filter_parameters()
                self.assertEqual(app.parameter_count_var.get(), "0 项")
                app.parameter_search_var.set("")
                app.parameter_group_var.set("Servo1 伸缩")
                self.assertEqual(app.parameter_count_var.get(), "2 项")
                self.assertEqual(app.value_vars["SERVO1_DISC_ALIGN_PULSE_US"].get(), "2440")
                self.assertTrue(app.action_armed_var.get())
                self.assertEqual(len(app.value_vars), len(gui.PARAMETERS))
                app._show_timing_operation(gui.TIMING_OPERATIONS[0][0])
                app.value_vars["SERVO1_DISC_ALIGN_PULSE_US"].set("2400")
                self.assertIn("2400", app.task_relationship_var.get())
                app._load_source(show_message=False)
                self.assertIn(str(app.loaded_values["SERVO1_DISC_ALIGN_PULSE_US"]), app.task_relationship_var.get())
                worker.return_value.request.assert_not_called()
            finally:
                app.destroy()


if __name__ == "__main__":
    unittest.main()
