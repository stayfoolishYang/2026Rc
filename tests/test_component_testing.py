"""Offline protocol/serial/UI tests. Never opens a real serial port."""
import json
import queue
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import stm32_hardware_tuning_gui as gui


class ProtocolTests(unittest.TestCase):
    def test_wire_format(self):
        self.assertEqual(gui.make_wheel_test_frame(3, 1, 3200, 5000),
                         bytes.fromhex("FF 5A 03 01 0C 80 13 88 00 FE"))
        self.assertEqual(gui.make_lift_test_frame(280, 1000),
                         bytes.fromhex("FF 5C 01 18 03 E8 00 00 00 FE"))
        self.assertEqual(gui.make_lift_test_frame(360, 1000),
                         bytes.fromhex("FF 5C 01 68 03 E8 00 00 00 FE"))
        self.assertEqual(gui.make_servo_test_frame(2, 765),
                         bytes.fromhex("FF 5E 02 02 FD 00 00 00 00 FE"))
        self.assertEqual(gui.make_chassis_linear_test_frame(5),
                         bytes.fromhex("FF 01 00 05 00 00 00 00 00 FE"))
        self.assertEqual(gui.make_chassis_linear_test_frame(-5),
                         bytes.fromhex("FF 01 FF FB 00 00 00 00 00 FE"))
        self.assertEqual(gui.make_chassis_rotate_test_frame(-10),
                         bytes.fromhex("FF 03 00 00 F6 00 00 00 00 FE"))

    def test_invalid_values(self):
        for args in ((1, 0, 200, 1000), (3, 2, 200, 1000), (3, 0, 0, 1000),
                     (3, 0, 3201, 1000), (3, 0, 200, 0), (3, 0, 200, 65536)):
            with self.assertRaises(ValueError):
                gui.make_wheel_test_frame(*args)
        for args in ((-1, 1000), (361, 1000), (5, 0), (5, 65536)):
            with self.assertRaises(ValueError):
                gui.make_lift_test_frame(*args)
        self.assertEqual(gui.make_lift_test_frame(360, 5000)[1], gui.TEST_MODE_LIFT)
        self.assertEqual(gui.make_wheel_test_frame(3, 0, 1, 65535)[6:8], bytes([0xFF, 0xFF]))
        for args in ((4, 1500), (1, 499), (1, 2501)):
            with self.assertRaises(ValueError):
                gui.make_servo_test_frame(*args)
        for value in (0, -21, 21):
            with self.assertRaises(ValueError):
                gui.make_chassis_linear_test_frame(value)
        for value in (0, -91, 91):
            with self.assertRaises(ValueError):
                gui.make_chassis_rotate_test_frame(value)

    def test_split_capability_and_button_responses(self):
        worker = gui.SerialWorker.__new__(gui.SerialWorker)
        worker.events = queue.Queue()
        pending = {"started": time.monotonic(), "terminal_statuses": (0, 7)}
        buffer = bytearray(b"\xff\x07")
        self.assertIs(worker._parse_status(buffer, pending), pending)
        buffer.append(0xFE)
        self.assertIsNone(worker._parse_status(buffer, pending))
        self.assertEqual(worker.events.get()["status"], 7)
        self.assertEqual(worker.events.get()["event"], "operation_done")
        for status in (5, 6):
            pending = {"started": time.monotonic(), "terminal_statuses": (0, 5, 6)}
            self.assertIsNone(worker._parse_status(bytearray([255, status, 254]), pending))

    def test_original_ack_does_not_finish_stop(self):
        worker = gui.SerialWorker.__new__(gui.SerialWorker)
        worker.events = queue.Queue()
        pending = {"started": time.monotonic(), "terminal_statuses": (0, 4)}
        self.assertIs(worker._parse_status(bytearray.fromhex("FF 01 FE"), pending), pending)
        self.assertIsNone(worker._parse_status(bytearray.fromhex("FF 04 FE"), pending))
        events = list(worker.events.queue)
        self.assertEqual(sum(e["event"] == "operation_done" for e in events), 1)

    def test_stop_bypasses_pending_send(self):
        class FakePort:
            def __init__(self, *args, **kwargs):
                self.writes = []
                self.rx = bytearray()
                self.port = None
                self.dtr = self.rts = True
                self.open_states = []
            def open(self):
                self.open_states.append((self.port, self.dtr, self.rts))
            def reset_input_buffer(self): pass
            def reset_output_buffer(self): pass
            def write(self, frame):
                self.writes.append(frame)
                if frame[1] == gui.TEST_MODE_STOP:
                    self.rx.extend(bytes.fromhex("FF 01 FE FF 04 FE"))
                return len(frame)
            def flush(self): pass
            @property
            def in_waiting(self): return len(self.rx)
            def read(self, count):
                result = bytes(self.rx[:count])
                del self.rx[:count]
                return result
            def close(self): pass

        port = FakePort()
        with patch.object(gui.serial, "Serial", return_value=port) as serial_class:
            worker = gui.SerialWorker(queue.Queue())
            try:
                worker.request("connect")
                worker.request("send", (gui.make_wheel_test_frame(3, 0, 200, 1000), "wheel", 5))
                worker.request("hardware_stop", (gui.make_frame(gui.TEST_MODE_STOP), "stop", 3))
                deadline = time.monotonic() + 2
                events = []
                while time.monotonic() < deadline:
                    event = worker.events.get(timeout=2)
                    events.append(event)
                    if event["event"] == "operation_done": break
                serial_class.assert_called_once_with(port=None, baudrate=gui.SERIAL_BAUD, timeout=0)
                self.assertEqual(port.open_states, [(gui.SERIAL_PORT, False, False)])
                self.assertEqual([f[1] for f in port.writes], [90, 91])
                self.assertFalse(any(e["event"] == "serial_error" for e in events))
                self.assertEqual([e["status"] for e in events if e["event"] == "rx"], [1, 4])
            finally:
                worker.request("stop")
                worker.thread.join(timeout=2)
                self.assertFalse(worker.thread.is_alive())


class UITests(unittest.TestCase):
    def test_four_claw_states_and_live_task_relationships(self):
        with patch.object(gui, "SerialWorker") as worker_class:
            app = gui.HardwareTuningApp(gui.DEFAULT_SOURCE)
            app.withdraw()
            try:
                states = [spec.define for spec in gui.PARAMETERS if spec.group == "Servo2 夹爪"]
                self.assertEqual(set(states), {"SERVO2_CLAMP_PULSE_US", "SERVO2_DISC_PICK_OPEN_PULSE_US",
                                               "SERVO2_DISC_RELEASE_PULSE_US", "SERVO2_BOTTOM_OPEN_PULSE_US"})
                cube_states = [spec.define for spec in gui.PARAMETERS if spec.group == "Servo2 正方体（舵盘2/4）"]
                self.assertEqual(cube_states, ["SERVO2_CUBE_GROUND_CLAMP_PULSE_US",
                                               "SERVO2_CUBE_DISC_OPEN_PULSE_US",
                                               "SERVO2_CUBE_DISC_CLAMP_PULSE_US"])
                modes = {mode for _, actions in gui.TASK_GROUPS for _, mode in actions}
                self.assertNotIn(21, modes)
                self.assertNotIn(22, modes)
                for label, prefix, _, _ in gui.TIMING_OPERATIONS:
                    app._show_timing_operation(label)
                    text = app.task_relationship_var.get()
                    expected_clamp = app.loaded_values[
                        "SERVO2_CUBE_GROUND_CLAMP_PULSE_US" if prefix in {"GRUB2", "GRUB4"} else
                        "SERVO2_CUBE_DISC_CLAMP_PULSE_US" if prefix in {"PUT2", "PUT4"} else
                        "SERVO2_CLAMP_PULSE_US"]
                    if prefix.startswith("PUT") or prefix.startswith("TASK2_PLACE"):
                        if prefix in {"PUT2", "PUT4"}:
                            self.assertIn(f"舵盘抓取松开 {app.loaded_values['SERVO2_CUBE_DISC_OPEN_PULSE_US']} us", text)
                        else:
                            self.assertIn(f"舵盘高位抓取张开 {app.loaded_values['SERVO2_DISC_PICK_OPEN_PULSE_US']} us", text)
                    else:
                        self.assertIn(f"舵盘低位放块张开 {app.loaded_values['SERVO2_DISC_RELEASE_PULSE_US']} us", text)
                    self.assertIn(f"夹紧 {expected_clamp} us", text)
                    if prefix in {"GRUB2", "GRUB4", "PUT2", "PUT4"}:
                        self.assertIn("本任务为2/4号舵盘正方体", text)
                app._show_timing_operation(next(item[0] for item in gui.TIMING_OPERATIONS if item[1] == "PUT3"))
                self.assertIn(
                    f"舵盘3（{app.value_vars['SERVO3_DISC3_PULSE_US'].get()} us）",
                    app.task_relationship_var.get(),
                )
                app.value_vars["SERVO2_DISC_PICK_OPEN_PULSE_US"].set("1397")
                app._show_timing_operation(next(item[0] for item in gui.TIMING_OPERATIONS if item[1] == "PUT1"))
                self.assertIn("舵盘高位抓取张开 1397 us", app.task_relationship_var.get())
                self.assertEqual(
                    app.value_vars["SERVO2_DISC_RELEASE_PULSE_US"].get(),
                    str(app.loaded_values["SERVO2_DISC_RELEASE_PULSE_US"]),
                )
                legacy = dict(app._validated_values())
                for define in gui.CUBE_SERVO2_PARAMETER_DEFINES + gui.CUBE_STACK_HEIGHT_PARAMETER_DEFINES:
                    legacy.pop(define)
                legacy.update(SERVO2_DISC3_PICK_OPEN_PULSE_US=1215,
                              SERVO2_TASK2_RELEASE_PULSE_US=1173, SERVO2_GROUND_RELEASE_STAGE_PULSE_US=1018)
                with patch.object(gui.filedialog, "askopenfilename", return_value="legacy.json"), \
                     patch.object(gui.Path, "read_text", return_value=json.dumps({"parameters": legacy})), \
                     patch.object(gui.messagebox, "showerror") as error:
                    app._import_profile()
                    error.assert_not_called()
                self.assertEqual(app.value_vars["SERVO2_DISC_PICK_OPEN_PULSE_US"].get(), "1397")
                for define in gui.CUBE_SERVO2_PARAMETER_DEFINES + gui.CUBE_STACK_HEIGHT_PARAMETER_DEFINES:
                    self.assertEqual(app.value_vars[define].get(), str(app.loaded_values[define]))
                app.update_idletasks()
                worker_class.return_value.request.assert_not_called()
            finally:
                app.destroy()

    def test_disc24_stack_heights_are_independent_and_leave_storage_height(self):
        with patch.object(gui, "SerialWorker") as worker_class:
            app = gui.HardwareTuningApp(gui.DEFAULT_SOURCE)
            app.withdraw()
            try:
                storage = app.value_vars["LIFT_PLACE_HEIGHT_MM"].get()
                ground = app.value_vars["LIFT_GROUND_PLACE_HEIGHT_MM"].get()
                app.value_vars["LIFT_DISC2_STACK_PLACE_HEIGHT_MM"].set("51")
                app.value_vars["LIFT_DISC4_STACK_PLACE_HEIGHT_MM"].set("55")
                def show(prefix):
                    app._show_timing_operation(next(item[0] for item in gui.TIMING_OPERATIONS if item[1] == prefix))
                    return app.task_relationship_var.get()
                text = show("PUT2")
                self.assertIn("缓降至圆柱顶部 51 mm", text)
                self.assertIn(f"缓降至 {storage} mm", text)
                app.value_vars["LIFT_DISC2_STACK_PLACE_HEIGHT_MM"].set("52")
                self.assertIn("缓降至圆柱顶部 52 mm", app.task_relationship_var.get())
                self.assertIn("缓降至圆柱顶部 55 mm", show("PUT4"))
                for prefix in ("PUT1", "PUT3", "PUT5"):
                    self.assertIn(f"缓降至地面 {ground} mm", show(prefix))
                for prefix in ("GRUB2", "GRUB4"):
                    self.assertIn(f"缓降至舵盘 {storage} mm", show(prefix))
                self.assertEqual(app.value_vars["LIFT_PLACE_HEIGHT_MM"].get(), storage)
                self.assertEqual(app.value_vars["LIFT_GROUND_PLACE_HEIGHT_MM"].get(), ground)
                legacy = dict(app._validated_values())
                for define in gui.CUBE_STACK_HEIGHT_PARAMETER_DEFINES:
                    legacy.pop(define)
                with patch.object(gui.filedialog, "askopenfilename", return_value="legacy.json"), \
                     patch.object(gui.Path, "read_text", return_value=json.dumps({"parameters": legacy})), \
                     patch.object(gui.messagebox, "showerror") as error:
                    app._import_profile()
                    error.assert_not_called()
                self.assertEqual(app.value_vars["LIFT_DISC2_STACK_PLACE_HEIGHT_MM"].get(), "52")
                self.assertEqual(app.value_vars["LIFT_DISC4_STACK_PLACE_HEIGHT_MM"].get(), "55")
                with patch.object(gui.messagebox, "showerror"):
                    app.value_vars["LIFT_DISC2_STACK_PLACE_HEIGHT_MM"].set("361")
                    self.assertIsNone(app._validated_values())
                app.update_idletasks()
                worker_class.return_value.request.assert_not_called()
            finally:
                app.destroy()

    def test_task2_height_roles_and_field_extension(self):
        with patch.object(gui, "SerialWorker") as worker_class:
            app = gui.HardwareTuningApp(gui.DEFAULT_SOURCE)
            app.withdraw()
            try:
                values = dict(app.loaded_values)
                self.assertEqual(values["LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM"], 55)
                self.assertEqual(values["LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM"], 37)
                self.assertTrue(0 <= values["LIFT_CAMERA_TARGET_LOW_HEIGHT_MM"] <
                                values["LIFT_CAMERA_TARGET_HIGH_HEIGHT_MM"] <= gui.LIFT_TARGET_MAX_MM)
                self.assertEqual(gui.PARAMETER_BY_DEFINE["LIFT_CAMERA_TARGET_HIGH_HEIGHT_MM"].group, "摄像头打靶高度")
                app.value_vars["LIFT_TASK2_DISC_PLACE_HEIGHT_MM"].set("190")
                app.value_vars["LIFT_TASK2_TRANSFER_HEIGHT_MM"].set("260")
                self.assertEqual(gui.PARAMETER_BY_DEFINE["LIFT_TASK2_DISC_PLACE_HEIGHT_MM"].group, "任务2升降高度")
                def show(prefix):
                    app._show_timing_operation(next(item[0] for item in gui.TIMING_OPERATIONS if item[1] == prefix))
                    return app.task_relationship_var.get()
                for prefix in ("TASK2_GRUB1", "TASK2_GRUB3", "TASK2_GRUB5"):
                    text = show(prefix)
                    self.assertIn("升至 260 mm", text)
                    self.assertIn("缓降至舵盘 190 mm", text)
                for prefix in ("GRUB1", "GRUB2", "GRUB3", "GRUB4", "GRUB5"):
                    text = show(prefix)
                    self.assertIn(f"升至 {values['LIFT_TRANSFER_HEIGHT_MM']} mm", text)
                    self.assertIn(f"缓降至舵盘 {values['LIFT_PLACE_HEIGHT_MM']} mm", text)
                text = show("TASK2_PLACE3")
                self.assertIn("升至 260 mm", text)
                self.assertIn(f"降至 {values['LIFT_TASK2_DISC_HEIGHT_MM']} mm", text)
                self.assertIn(f"升至 {values['LIFT_TASK2_RETURN_CLEARANCE_MM']} mm", text)
                self.assertIn("缓降至领奖台 55 mm", show("TASK2_PLACE1"))
                self.assertIn("缓降至领奖台 37 mm", show("TASK2_PLACE2"))
                for prefix in ("PUT1", "PUT2", "PUT3", "PUT4", "PUT5"):
                    self.assertIn(f"保持伸缩对位 {values['SERVO1_DISC_ALIGN_PULSE_US']} us", show(prefix))
                legacy = dict(app._validated_values())
                for define in gui.TASK2_NEW_HEIGHT_PARAMETER_DEFINES:
                    legacy.pop(define)
                with patch.object(gui.filedialog, "askopenfilename", return_value="legacy.json"), \
                     patch.object(gui.Path, "read_text", return_value=json.dumps({"parameters": legacy})), \
                     patch.object(gui.messagebox, "showerror") as error:
                    app._import_profile()
                    error.assert_not_called()
                self.assertEqual(app.value_vars["LIFT_TASK2_DISC_PLACE_HEIGHT_MM"].get(), "190")
                self.assertEqual(app.value_vars["LIFT_TASK2_TRANSFER_HEIGHT_MM"].get(), "260")
                legacy_podium = dict(app._validated_values())
                legacy_podium["LIFT_TASK2_CHAMPION_HEIGHT_MM"] = legacy_podium.pop("LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM")
                legacy_podium["LIFT_TASK2_RUNNER_UP_HEIGHT_MM"] = legacy_podium.pop("LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM")
                for define in gui.CAMERA_TARGET_HEIGHT_PARAMETER_DEFINES:
                    legacy_podium.pop(define)
                with patch.object(gui.filedialog, "askopenfilename", return_value="legacy-podium.json"), \
                     patch.object(gui.Path, "read_text", return_value=json.dumps({"parameters": legacy_podium})), \
                     patch.object(gui.messagebox, "showerror") as error:
                    app._import_profile()
                    error.assert_not_called()
                self.assertEqual(app.value_vars["LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM"].get(), "55")
                self.assertEqual(app.value_vars["LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM"].get(), "37")
                for define in gui.CAMERA_TARGET_HEIGHT_PARAMETER_DEFINES:
                    self.assertEqual(app.value_vars[define].get(), str(values[define]))
                with patch.object(gui.messagebox, "showerror"):
                    for define, invalid in (
                        ("LIFT_TASK2_DISC_PLACE_HEIGHT_MM", 260),
                        ("LIFT_TASK2_TRANSFER_HEIGHT_MM", values["LIFT_TASK2_DISC_HEIGHT_MM"]),
                        ("LIFT_TASK2_RETURN_CLEARANCE_MM", values["LIFT_TASK2_DISC_HEIGHT_MM"]),
                        ("LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM", values["LIFT_TASK2_RETURN_CLEARANCE_MM"] + 1),
                    ):
                        original = app.value_vars[define].get()
                        app.value_vars[define].set(str(invalid))
                        self.assertIsNone(app._validated_values())
                        app.value_vars[define].set(original)
                app.update_idletasks()
                worker_class.return_value.request.assert_not_called()
            finally:
                app.destroy()

    def test_default_retract_ignores_removed_clearance_profile_field(self):
        with patch.object(gui, "SerialWorker") as worker_class:
            app = gui.HardwareTuningApp(gui.DEFAULT_SOURCE)
            app.withdraw()
            try:
                values = app._validated_values()
                self.assertNotIn("SERVO1_RETRACT_PULSE_US", values)
                disc = values["SERVO1_DISC_ALIGN_PULSE_US"]
                changed_disc = disc - 5 if disc > gui.SERVO_TUNING_MIN_PULSE_US + 5 else disc + 5
                app.value_vars["SERVO1_DISC_ALIGN_PULSE_US"].set(str(changed_disc))
                self.assertNotIn("SERVO1_RETRACT_PULSE_US", app._validated_values())
                legacy = dict(values)
                del legacy["SERVO1_DISC_ALIGN_PULSE_US"]
                legacy["SERVO1_RETRACT_PULSE_US"] = 2350
                with patch.object(gui.filedialog, "askopenfilename", return_value="legacy.json"), \
                     patch.object(gui.Path, "read_text", return_value=json.dumps({"parameters": legacy})), \
                     patch.object(gui.messagebox, "showerror") as error:
                    app._import_profile()
                    error.assert_not_called()
                self.assertNotIn("SERVO1_RETRACT_PULSE_US", app._validated_values())
                self.assertEqual(app._validated_values()["SERVO1_DISC_ALIGN_PULSE_US"], changed_disc)
                app.update_idletasks()
                worker_class.return_value.request.assert_not_called()
            finally:
                app.destroy()

    def test_ready_repeats_preserve_confirmation_and_probe_allows_motion(self):
        with patch.object(gui, "SerialWorker") as worker_class, \
             patch.object(gui.messagebox, "askyesno", return_value=True):
            app = gui.HardwareTuningApp(gui.DEFAULT_SOURCE)
            app.withdraw()
            confirmations = (app.action_armed_var, app.lift_synced_var, app.turntable_safe_var)
            def status(code):
                app.events.put({"event": "rx", "frame": bytes([255, code, 254]),
                                "status": code, "elapsed": None})
            try:
                app.events.put({"event": "connected"})
                app._poll_events()
                for value in confirmations:
                    value.set(True)
                status(2)
                status(2)
                app._poll_events()
                self.assertTrue(all(value.get() for value in confirmations))
                self.assertFalse(app.component_supported)
                worker_class.return_value.request.assert_not_called()

                app._component_read(gui.TEST_MODE_PROBE, "probe")
                status(2)  # A final queued READY may arrive before the probe reply.
                status(7)
                app.events.put({"event": "operation_done"})
                app._poll_events()
                self.assertTrue(app.component_supported)
                self.assertTrue(all(value.get() for value in confirmations))
                app._test_component_wheel(3, 0, "wheel")
                self.assertEqual(worker_class.return_value.request.call_args.args[1][0][1], 90)

                status(1)
                app.events.put({"event": "operation_done"})
                app._poll_events()
                status(2)  # READY resumes after command replies: controller restarted.
                app._poll_events()
                self.assertFalse(app.component_supported)
                self.assertFalse(any(value.get() for value in confirmations))
                for value in confirmations:
                    value.set(True)
                status(2)
                status(2)
                app._poll_events()
                self.assertTrue(all(value.get() for value in confirmations))
                app._component_read(gui.TEST_MODE_PROBE, "probe after restart")
                status(7)
                app.events.put({"event": "operation_done"})
                app._poll_events()
                self.assertTrue(app.component_supported)
                self.assertTrue(all(value.get() for value in confirmations))
            finally:
                app.destroy()

    def test_new_connection_disconnect_and_flash_clear_motion_confirmation(self):
        with patch.object(gui, "SerialWorker") as worker_class, \
             patch.object(gui.messagebox, "showinfo"):
            app = gui.HardwareTuningApp(gui.DEFAULT_SOURCE)
            app.withdraw()
            try:
                confirmations = (app.action_armed_var, app.lift_synced_var, app.turntable_safe_var)
                for event in ({"event": "disconnected"}, {"event": "connected"},
                              {"event": "flash_verified", "bin_path": "offline-test.bin"}):
                    with self.subTest(event=event["event"]):
                        app.component_supported = True
                        for value in confirmations:
                            value.set(True)
                        app.events.put(event)
                        app._poll_events()
                        self.assertFalse(app.component_supported)
                        self.assertFalse(any(value.get() for value in confirmations))
                worker_class.return_value.request.assert_not_called()
            finally:
                app.destroy()

    def test_page_and_controls_without_serial(self):
        with patch.object(gui, "SerialWorker") as worker_class:
            app = gui.HardwareTuningApp(gui.DEFAULT_SOURCE)
            app.withdraw()
            try:
                self.assertIn("元器件测试", [app.notebook.tab(t, "text") for t in app.notebook.tabs()])
                app.update_idletasks()
                worker_class.return_value.request.assert_not_called()
                app.connected = True
                app._component_read(gui.TEST_MODE_PROBE, "probe")
                self.assertEqual(worker_class.return_value.request.call_args.args[1][0][1], 93)
                app.component_supported = True
                app._stop_component_test()
                self.assertEqual(worker_class.return_value.request.call_args.args[0], "hardware_stop")
                app.events.put({"event": "rx", "frame": bytes.fromhex("FF 07 FE"), "status": 7, "elapsed": None})
                app.events.put({"event": "operation_done"})
                app._poll_events()
                self.assertTrue(app.component_supported)
                self.assertFalse(app.busy)
                self.assertIn("固件支持", app.component_result_var.get())
            finally:
                app.destroy()

    def test_chassis_trial_controls_build_legacy_frames(self):
        with patch.object(gui, "SerialWorker") as worker_class, \
             patch.object(gui.messagebox, "askyesno", return_value=True):
            app = gui.HardwareTuningApp(gui.DEFAULT_SOURCE)
            app.withdraw()
            try:
                app.connected = True
                app.action_armed_var.set(True)
                app.test_chassis_distance_var.set("5")
                app._test_chassis_linear()
                linear_frame = worker_class.return_value.request.call_args.args[1][0]
                self.assertEqual(linear_frame, bytes.fromhex("FF 01 00 05 00 00 00 00 00 FE"))
                app.busy = False
                app.test_chassis_angle_var.set("-10")
                app._test_chassis_rotate()
                rotate_frame = worker_class.return_value.request.call_args.args[1][0]
                self.assertEqual(rotate_frame, bytes.fromhex("FF 03 00 00 F6 00 00 00 00 FE"))
            finally:
                app.destroy()


if __name__ == "__main__":
    unittest.main()
