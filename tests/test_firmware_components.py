"""Compile the actual component-test firmware functions with mock I/O."""
import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class FirmwareTests(unittest.TestCase):
    def test_lift_isolation_and_default_retract_task_order(self):
        compiler = shutil.which("gcc")
        if not compiler:
            self.skipTest("gcc unavailable; Keil build remains the real firmware check")
        source = (ROOT / "USER/chassis_controller_main.c").read_text(encoding="utf-8")
        names = ["Arm_RestoreDiscAlignment", "Arm_PWM_Init", "Arm_SetPowerOnTurntableHome",
                 "Arm_SetPowerOnReadyPose", "Lift_MoveToHeightRawAtHalfPeriod",
                 "Lift_MoveToHeightAtHalfPeriod", "Lift_MoveToHeight", "Lift_MoveToPlaceHeightSlow",
                 "Arm_RunSoftwareInit", "Arm_Servo1Move", "Arm_Servo1MoveWait",
                  "Arm_Servo2MoveWait", "Arm_Servo3MoveWait", "Arm_DiscPulseFromMode",
                  "Arm_Task2ParamsFromMode", "Arm_Task2GrubPulseFromMode", "Arm_Task2HeightsValid",
                  "Arm_IsCubeDiscMode", "Arm_IsPutMode", "Arm_FieldPlaceHeightFromMode",
                  "Arm_GrubTimingFromMode", "Arm_PutTimingFromMode",
                  "Arm_Task2GrubTimingFromMode", "Arm_Task2PlaceTimingFromMode", "Arm_RunTask2Place",
                 "Arm_RunGrubToDisc", "Arm_RunPutFromDisc"]
        functions = []
        for name in names:
            start = re.search(r"^static [^\n]+\b" + name + r"\(", source, re.MULTILINE).start()
            end = source.index("\n}\n", start) + 3
            functions.append(source[start:end])
        defines = "\n".join(re.findall(
            r"^#define (?:SERVO\w+|LIFT_\w+|RPI_CMD_\w+|GRUB\w+|PUT\w+|TASK2_\w+)\s+[^\r\n]+",
            source, re.MULTILINE))
        timings = source[source.index("typedef struct {"):source.index("#define RPI_FRAME_PAYLOAD_SIZE")]
        handler = source[source.index("static uint8_t Chassis_HandleCommand("):]
        # Execute the actual task cases, leaving unrelated chassis/serial paths out of the mock.
        task_cases = handler[handler.index("    case RPI_CMD_GRUB_DISC1:"):
                             handler.index("    case RPI_CMD_RESET_ANGLE:")]
        task_handler = ("\nstatic uint8_t test_handle_arm_mode(uint8_t mode)\n{\n"
                        "    const uint8_t payload[] = {mode};\n"
                        "    uint16_t disc_pulse_us, podium_height_mm, ground_clamp_pulse_us;\n"
                        "    uint16_t disc_open_pulse_us, disc_clamp_pulse_us, field_place_height_mm;\n"
                        "    switch (payload[0]) {\n" + task_cases +
                        "    default: return 0u;\n    }\n}\n")
        with tempfile.TemporaryDirectory(prefix="rc_lift_clearance_") as directory:
            temp = Path(directory)
            exe = temp / ("lift_clearance.exe" if os.name == "nt" else "lift_clearance")
            variants = [
                ("current calibration", {}, False),
                ("independent task2 and stack heights", {
                    "LIFT_TASK2_DISC_PLACE_HEIGHT_MM": 190,
                    "LIFT_TASK2_TRANSFER_HEIGHT_MM": 260,
                    "LIFT_DISC2_STACK_PLACE_HEIGHT_MM": 51,
                    "LIFT_DISC4_STACK_PLACE_HEIGHT_MM": 55,
                }, False),
                ("stack releases above 60 mm", {
                    "LIFT_DISC2_STACK_PLACE_HEIGHT_MM": 61,
                    "LIFT_DISC4_STACK_PLACE_HEIGHT_MM": 75,
                }, False),
            ]
            for define in ("LIFT_TASK2_TRANSFER_HEIGHT_MM", "LIFT_TASK2_DISC_PLACE_HEIGHT_MM",
                           "LIFT_TASK2_DISC_HEIGHT_MM", "LIFT_TASK2_RETURN_CLEARANCE_MM"):
                variants.append((f"invalid range {define}", {define: 361}, True))
            pickup = int(re.search(r"#define LIFT_TASK2_DISC_HEIGHT_MM\s+(\d+)u", source).group(1))
            for label, overrides in (
                ("transfer equals placement", {"LIFT_TASK2_TRANSFER_HEIGHT_MM": 190,
                                               "LIFT_TASK2_DISC_PLACE_HEIGHT_MM": 190}),
                ("transfer below pickup", {"LIFT_TASK2_TRANSFER_HEIGHT_MM": pickup - 1}),
                ("return equals pickup", {"LIFT_TASK2_RETURN_CLEARANCE_MM": pickup}),
                ("return below champion", {"LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM": 222}),
                ("return below runner-up", {"LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM": 222}),
                ("return below third-place", {"LIFT_TASK2_THIRD_PLACE_HEIGHT_MM": 222}),
            ):
                variants.append((f"invalid order {label}", overrides, True))
            for variant, values, invalid in variants:
                with self.subTest(variant=variant):
                    overrides = "".join(f"\n#undef {name}\n#define {name} {value}u\n"
                                        for name, value in values.items())
                    if invalid:
                        overrides += "\n#define EXPECT_INVALID_TASK2_HEIGHTS 1\n"
                    (temp / "lift_clearance_under_test.inc").write_text(
                        defines + overrides + "\n" + timings + "\n" + "\n".join(functions) + task_handler,
                        encoding="utf-8")
                    result = subprocess.run(
                        [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-I", str(temp),
                         str(ROOT / "tests/firmware_lift_clearance_harness.c"), "-o", str(exe)],
                        text=True, capture_output=True,
                    )
                    self.assertEqual(result.returncode, 0, result.stderr)
                    result = subprocess.run([str(exe)], text=True, capture_output=True)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    expected = "Invalid task2 heights rejected" if invalid else "Fixed retract simulation OK"
                    self.assertIn(expected, result.stdout)

    def test_normal_firmware_does_not_rotate_at_power_on(self):
        source = (ROOT / "USER/chassis_controller_main.c").read_text(encoding="utf-8")
        main = source[source.index("int main(void)"):]
        startup = main[:main.index("for (;;) {")]
        self.assertNotIn("POWER_ON_ROTATION_TEST_DEG", source)
        self.assertNotRegex(startup, r"Chassis_(?:Move|Pulse\w*|AlignRotate|RunPowerOnRotationTest)\s*\(")
        self.assertIn("Chassis_HandleCommand(payload)", main)

    def test_camera_modes_and_task2_podium_heights_are_separate(self):
        source = (ROOT / "USER/chassis_controller_main.c").read_text(encoding="utf-8")
        task2_params = source[source.index("static uint8_t Arm_Task2ParamsFromMode("):source.index("static uint8_t Arm_Task2GrubPulseFromMode(")]
        handler = source[source.index("static uint8_t Chassis_HandleCommand("):]
        self.assertRegex(source, r"#define LIFT_MAX_HEIGHT_MM\s+360u")
        self.assertRegex(source, r"#define LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM\s+55u")
        self.assertRegex(source, r"#define LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM\s+37u")
        camera_high = int(re.search(r"#define LIFT_CAMERA_TARGET_HIGH_HEIGHT_MM\s+(\d+)u", source)[1])
        camera_low = int(re.search(r"#define LIFT_CAMERA_TARGET_LOW_HEIGHT_MM\s+(\d+)u", source)[1])
        self.assertTrue(0 <= camera_low < camera_high <= 360)
        self.assertIn("*podium_height_mm = LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM;", task2_params)
        self.assertIn("*podium_height_mm = LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM;", task2_params)
        self.assertIn("case RPI_CMD_CAMERA_TARGET_HIGH:\n        Lift_MoveToHeight(LIFT_CAMERA_TARGET_HIGH_HEIGHT_MM);", handler)
        self.assertIn("case RPI_CMD_CAMERA_TARGET_LOW:\n        Lift_MoveToHeight(LIFT_CAMERA_TARGET_LOW_HEIGHT_MM);", handler)

    def test_alignment_converges_with_observed_rotation_sign(self):
        compiler = shutil.which("gcc")
        if not compiler:
            self.skipTest("gcc unavailable; Keil build remains the real firmware check")
        source = (ROOT / "USER/chassis_controller_main.c").read_text(encoding="utf-8")
        names = ["Chassis_AbsInt32", "Chassis_LinearPulses", "Chassis_RotationPulses",
                 "Chassis_HeadingError", "Chassis_SetDirection", "Chassis_Move",
                 "Chassis_AlignRotate", "Chassis_MotorAlign"]
        functions = []
        for name in names:
            start = re.search(r"^static [^\n]+\b" + name + r"\(", source, re.MULTILINE).start()
            end = source.index("\n}\n", start) + 3
            functions.append(source[start:end])
        defines = "\n".join(re.findall(
            r"^#define (?:CHASSIS_\w+|ALIGN_\w+)\s+[^\r\n]+", source, re.MULTILINE))
        enum_end = source.index("} ChassisVector;") + len("} ChassisVector;")
        enum_start = source.rfind("typedef enum {", 0, enum_end)
        with tempfile.TemporaryDirectory(prefix="rc_alignment_test_") as directory:
            temp = Path(directory)
            (temp / "alignment_under_test.inc").write_text(
                defines + "\n" + source[enum_start:enum_end] + "\n" + "\n".join(functions), encoding="utf-8")
            exe = temp / ("alignment_test.exe" if os.name == "nt" else "alignment_test")
            subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-I", str(temp),
                            str(ROOT / "tests/firmware_alignment_harness.c"), "-o", str(exe)], check=True, capture_output=True)
            result = subprocess.run([str(exe)], text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("Alignment simulation OK", result.stdout)

    def test_power_on_turntable_pwm_disabled(self):
        source = (ROOT / "USER/chassis_controller_main.c").read_text(encoding="utf-8")
        init = source[source.index("static void Arm_PWM_Init("):source.index("static void Arm_SetPowerOnTurntableHome(")]
        servo = (ROOT / "HARDWARE/SERVO/servo.c").read_text(encoding="gbk")
        self.assertRegex(source, r"#define FIRMWARE_DIAGNOSTIC_VERSION\s+181u")
        self.assertIn("TIM_SetCompare4(TIM5, 0u);", init)
        self.assertNotIn("TIM_SetCompare4(TIM5, SERVO3_HOME_PULSE_US);", init)
        self.assertRegex(servo, r"#define SERVO_TIM5_CH4_INIT_PULSE_US\s+0u")
        self.assertIn("TIM_SetCompare4(TIM5, SERVO_TIM5_CH4_INIT_PULSE_US);", servo)
        self.assertIn("TIM_SetCompare4(TIM5, SERVO3_HOME_PULSE_US);", source)

    def test_component_functions_on_mock_pins(self):
        compiler = shutil.which("gcc")
        if not compiler:
            self.skipTest("gcc unavailable; Keil build remains the real firmware check")
        source = (ROOT / "USER/chassis_controller_main.c").read_text(encoding="utf-8")
        functions = source[source.index("static void HardwareTest_Stop("):source.index("static void Chassis_SetDirection(")]
        defines = "\n".join(re.findall(
            r"^#define (?:RPI_CMD_\w+|COMMAND_RESULT_\w+|TEST_\w+|LIFT_MAX_HEIGHT_MM|LIFT_PULSES_PER_MM|RPI_FRAME_PAYLOAD_SIZE|CHASSIS_STEP_PINS)\s+[^\r\n]+", source, re.MULTILINE))
        with tempfile.TemporaryDirectory(prefix="rc_component_test_") as directory:
            temp = Path(directory)
            (temp / "component_under_test.inc").write_text(defines + "\n" + functions, encoding="utf-8")
            exe = temp / ("firmware_test.exe" if os.name == "nt" else "firmware_test")
            compile_result = subprocess.run(
                [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-I", str(temp),
                 str(ROOT / "tests/firmware_component_harness.c"), "-o", str(exe)],
                text=True, capture_output=True,
            )
            self.assertEqual(compile_result.returncode, 0, compile_result.stderr)
            result = subprocess.run([str(exe)], text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("Firmware component simulation OK", result.stdout)


if __name__ == "__main__":
    unittest.main()
