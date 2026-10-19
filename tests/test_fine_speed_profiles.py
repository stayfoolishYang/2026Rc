"""Execute the real C speed-selection function with simulated pulse output."""
import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class FineSpeedProfileTests(unittest.TestCase):
    def test_modes_5_and_6_follow_87_88_even_with_small_distance_threshold(self):
        gcc = shutil.which("gcc")
        if not gcc:
            self.skipTest("gcc unavailable")
        source = (ROOT / "USER/chassis_controller_main.c").read_text(encoding="utf-8")

        def function(name):
            match = re.search(r"^static [^\n]+\b" + name + r"\([^;]*?\)\s*\{", source, re.MULTILINE)
            end = source.index("\n}", match.end()) + 2
            return source[match.start():end]

        defines = "\n".join(re.findall(r"^#define (?:CHASSIS_\w+|RPI_CMD_\w+)\s+\d+u[^\r\n]*", source, re.MULTILINE))
        functions = "\n".join(function(name) for name in ("Chassis_AbsInt32", "Chassis_LinearPulses", "Chassis_RotationPulses", "Chassis_Move"))
        mm_start = source.index("    case RPI_CMD_X_MM:")
        mm_cases = source[mm_start:source.index("    case RPI_CMD_GRUB_DISC1:", mm_start)]
        profile_cases = source[source.index("    case RPI_CMD_TASK2_CHASSIS_PROFILE:"):source.index("    case RPI_CMD_ARM_START_BUTTON:")]
        stubs = r'''
#include <stdint.h>
#include <assert.h>
typedef int ChassisVector;
enum { CHASSIS_VECTOR_FORWARD_BACK, CHASSIS_VECTOR_LEFT_RIGHT, CHASSIS_VECTOR_ROTATE };
#define CHASSIS_STEP_PINS 0u
#define GPIOC 0
#define COMMAND_RESULT_DONE 1u
static uint8_t chassis_task2_profile_enabled = 0u;
static uint32_t period, ramp_start;
static int route, direction;
static void Chassis_SetDirection(ChassisVector vector, uint8_t sign) { (void)vector; direction=sign; }
static void delay_ms(uint32_t ms) { (void)ms; }
static void GPIO_ResetBits(int port, uint32_t pins) { (void)port; (void)pins; }
static void Chassis_Pulse(uint32_t n) { (void)n; route=3; }
static void Chassis_PulseAtHalfPeriod(uint32_t n, uint32_t p) { (void)n; period=p; route=1; }
static void Chassis_PulseWithRamp(uint32_t n, uint32_t p, uint32_t start, uint32_t ramp) {
    (void)n; (void)ramp; period=p; ramp_start=start; route=2;
}
static int8_t Chassis_ReadInt8(uint8_t v) { return (int8_t)v; }
'''
        handler = "\nstatic uint8_t handle(const uint8_t *payload) { switch(payload[0]) {\n" + mm_cases + profile_cases + "default: return 0u; } }\n"
        checks = r'''
int main(void) {
    const uint8_t slow[] = {87}, fast[] = {88};
    const int distances[] = {1, 127, -1, -128};
    uint8_t frame[3] = {0};
    assert(chassis_task2_profile_enabled == 0u);
    for (unsigned profile=0; profile<4; profile++) {
        if (profile == 1 || profile == 3) assert(handle(slow) == 1u);
        if (profile == 2) assert(handle(fast) == 1u);
        for (unsigned mode=5; mode<=6; mode++) {
            for (unsigned index=0; index<4; index++) {
                frame[0]=(uint8_t)mode; frame[1]=0; frame[2]=0;
                frame[mode-4]=(uint8_t)distances[index];
                assert(handle(frame)==1u);
                assert(route==1);
                assert(period==((profile==1 || profile==3) ?
                    CHASSIS_TASK2_FINE_STEP_HALF_PERIOD_US :
                    CHASSIS_FINE_STEP_HALF_PERIOD_US));
                assert(direction==(distances[index]<0));
            }
        }
        Chassis_Move(CHASSIS_VECTOR_FORWARD_BACK, 125, 0u, 10u);
        assert(route==2);
        assert(period==(chassis_task2_profile_enabled ? CHASSIS_TASK2_LONG_STEP_HALF_PERIOD_US : CHASSIS_STEP_HALF_PERIOD_US));
        Chassis_Move(CHASSIS_VECTOR_ROTATE, 20, 1u, 0u);
        assert(route==3);
    }
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix="rc_fine_profiles_") as folder:
            temp = Path(folder)
            for threshold in (500, 1):
                with self.subTest(long_distance_threshold=threshold):
                    variant = re.sub(r"(#define CHASSIS_RAMP_MIN_DISTANCE_MM\s+)\d+u", rf"\g<1>{threshold}u", defines)
                    cfile = temp / "fine.c"
                    cfile.write_text(variant + stubs + functions + handler + checks, encoding="utf-8")
                    exe = temp / ("fine.exe" if os.name == "nt" else "fine")
                    compile_result = subprocess.run([gcc, "-std=c99", "-Wall", "-Wextra", "-Werror", str(cfile), "-o", str(exe)], capture_output=True, text=True)
                    self.assertEqual(compile_result.returncode, 0, compile_result.stderr)
                    result = subprocess.run([str(exe)], capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
