"""Check UI timing meanings against executed C routines, without hardware I/O."""
import json
import re
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import stm32_hardware_tuning_gui as gui
import tuning_time_catalog as catalog


SOURCE = gui.DEFAULT_SOURCE.read_text(encoding="utf-8")
CONSTANTS = catalog.source_constants(SOURCE)


def function_body(name):
    start = SOURCE.index("{", re.search(r"static\s+\w+\s+" + name + r"\s*\(", SOURCE).start())
    level = 1
    end = start + 1
    while level:
        level += (SOURCE[end] == "{") - (SOURCE[end] == "}")
        end += 1
    return SOURCE[start + 1:end - 1]


class TimingSourceTests(unittest.TestCase):
    def test_all_task_fields_match_initializer_and_executed_order(self):
        functions = {"grub": "Arm_RunGrubToDisc", "put": "Arm_RunPutFromDisc", "place": "Arm_RunTask2Place"}
        for _, prefix, _, specs in gui.TIMING_OPERATIONS:
            with self.subTest(task=prefix):
                family, roles = catalog.task_family(prefix)
                initializer = re.search(r"\b" + prefix.lower() + r"_timing\s*=\s*\{([^}]+)\}", SOURCE)[1]
                actual_macros = re.findall(r"\b\w+_MS\b", initializer)
                expected = [prefix + "_" + suffix + "_MS" for suffix in roles]
                self.assertEqual([spec.define for spec in specs], actual_macros)
                self.assertEqual(expected, actual_macros)
                body = function_body(functions[family])
                self.assertEqual(re.findall(r"timing->(\w+)", body), [suffix.lower() + "_ms" for suffix in roles])
                shared_calls = body.count("Arm_RestoreDiscAlignment();") + body.count("Arm_Servo1Move(")
                shared = dict((name, count) for name, count, _ in catalog.shared_waits(prefix))
                self.assertEqual(shared["SERVO1_SETTLE_MS"], shared_calls)
                self.assertEqual(shared.get("SERVO2_SETTLE_MS", 0), body.count("SERVO2_SETTLE_MS"))

    def test_inactive_fields_removed_and_active_fields_present(self):
        clean = re.sub(r"/\*.*?\*/|//[^\n]*", "", SOURCE, flags=re.DOTALL)
        for name in catalog.LEGACY_TIME_DEFINES:
            self.assertEqual(len(re.findall(r"\b" + name + r"\b", clean)), 1)
            self.assertNotIn(name, gui.PARAMETER_BY_DEFINE)
        for name in catalog.NEW_TIME_DEFINES:
            self.assertGreater(len(re.findall(r"\b" + name + r"\b", clean)), 1)
            self.assertIn(name, gui.PARAMETER_BY_DEFINE)
        self.assertEqual(len(gui.PARAMETERS), 182)
        inventory = catalog.audit_inventory(SOURCE, gui.PARAMETERS)
        for row in inventory:
            if row["editable"]:
                self.assertGreater(row["active_reference_count"], 0, row["define"])

    def test_lift_phase_and_release_sum_descriptions_are_correct(self):
        body = function_body("Arm_RunGrubToDisc")
        self.assertLess(body.index("Lift_MoveToHeight(transfer_height_mm)"), body.index("timing->clamp_hold_ms"))
        put = function_body("Arm_RunPutFromDisc")
        self.assertLess(put.index("timing->clamp_hold_ms"), put.index("Lift_MoveToHeight(LIFT_TRANSFER_HEIGHT_MM)", put.index("timing->clamp_hold_ms")))
        self.assertRegex(put, r"Arm_Servo2MoveWait\(SERVO2_BOTTOM_OPEN_PULSE_US, SERVO2_SETTLE_MS\);\s*delay_ms\(timing->stage_interval_ms\);\s*delay_ms\(timing->final_open_hold_ms\)")
        values = dict(CONSTANTS, SERVO2_SETTLE_MS=380, PUT2_STAGE_INTERVAL_MS=100, PUT2_FINAL_OPEN_HOLD_MS=200)
        self.assertIn("380 + 100 + 200 = 680 ms", catalog.wait_summary("PUT2", values))
        self.assertIn("SERVO2_SETTLE_MS", function_body("Arm_Servo2Move"))
        self.assertIn("SERVO1_SETTLE_MS", function_body("Arm_RestoreDiscAlignment"))

    def test_save_new_waits_preserves_inactive_defines_and_all_other_values(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "controller.c"
            path.write_bytes(gui.DEFAULT_SOURCE.read_bytes())
            values, digest = gui.parse_source_parameters(path)
            for name, value in zip(catalog.NEW_TIME_DEFINES, (510, 25, 30)):
                values[name] = value
            gui.replace_source_parameters(path, digest, values)
            changed = catalog.source_constants(path.read_text(encoding="utf-8"))
            for name, original in CONSTANTS.items():
                self.assertEqual(changed[name], values[name] if name in catalog.NEW_TIME_DEFINES else original)


class TimingUITests(unittest.TestCase):
    def setUp(self):
        self.worker_patch = patch.object(gui, "SerialWorker")
        self.ports_patch = patch.object(gui.list_ports, "comports", return_value=[])
        self.worker = self.worker_patch.start()
        self.ports_patch.start()
        self.app = gui.HardwareTuningApp(gui.DEFAULT_SOURCE)
        self.app.withdraw()

    def tearDown(self):
        self.app.destroy()
        self.worker_patch.stop()
        self.ports_patch.stop()

    def test_shared_and_private_waits_refresh_live_without_motion(self):
        app = self.app
        label, prefix, *_ = next(item for item in gui.TIMING_OPERATIONS if item[1] == "PUT2")
        app._show_timing_operation(label)
        app.value_vars["PUT2_STAGE_INTERVAL_MS"].set("100")
        app.value_vars["PUT2_FINAL_OPEN_HOLD_MS"].set("200")
        app.value_vars["SERVO2_SETTLE_MS"].set("400")
        self.assertIn("400 + 100 + 200 = 700 ms", app.task_wait_summary_var.get())
        self.assertIn("mode71/81 夹爪单项：400 ms", app.command_wait_summary_var.get())
        app.value_vars["PUT2_HOME_WAIT_MS"].set("60000")
        self.assertIn("60秒超时", app.task_wait_summary_var.get())
        app._load_source(show_message=False)
        self.assertIn("380", app.command_wait_summary_var.get())
        self.worker.return_value.request.assert_not_called()

    def test_old_profile_keeps_new_waits_and_ignores_inactive_fields(self):
        app = self.app
        old = {name: int(var.get()) for name, var in app.value_vars.items() if name not in catalog.NEW_TIME_DEFINES + catalog.PROFILE_FINE_DEFINES}
        old.update({name: 999 for name in catalog.LEGACY_TIME_DEFINES})
        app.value_vars["SERVO_POWER_ON_SETTLE_MS"].set("515")
        with patch.object(gui.filedialog, "askopenfilename", return_value="old.json"), \
             patch.object(gui.Path, "read_text", return_value=json.dumps({"parameters": old})), \
             patch.object(gui.messagebox, "showerror") as errors:
            app._import_profile()
        errors.assert_not_called()
        self.assertEqual(app.value_vars["SERVO_POWER_ON_SETTLE_MS"].get(), "515")
        self.assertEqual(app.value_vars["CHASSIS_TASK2_FINE_STEP_HALF_PERIOD_US"].get(), "400")
        self.assertNotIn(catalog.LEGACY_TIME_DEFINES[0], app.value_vars)
        self.worker.return_value.request.assert_not_called()

    def test_all_sections_fit_small_window_and_keep_inputs(self):
        app = self.app
        app.deiconify()
        app.geometry("1000x680")
        app.update()
        app.value_vars["SERVO1_SETTLE_MS"].set("210")
        for label in [catalog.OVERVIEW] + [row[0] for row in gui.TIMING_OPERATIONS] + [catalog.SYSTEM, catalog.LEGACY]:
            app._show_timing_operation(label)
            app.notebook.select(1)
            app.update()
            region = app.timing_canvas.bbox("all")
            self.assertIsNotNone(region)
            self.assertLessEqual(region[2] - region[0], app.timing_canvas.winfo_width() + 2, label)
            self.assertEqual(app.value_vars["SERVO1_SETTLE_MS"].get(), "210")
            self.assertGreater(app.timing_detail.winfo_height(), 100)
        self.worker.return_value.request.assert_not_called()


if __name__ == "__main__":
    unittest.main()
