"""Compile actual mode 52 case with mock PWM; never accesses real hardware."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Servo1ReturnTests(unittest.TestCase):
    def test_mode52_sets_only_servo1_to_disc_alignment_then_waits(self):
        compiler = shutil.which("gcc")
        if compiler is None:
            self.skipTest("gcc unavailable")
        source = (ROOT / "USER/chassis_controller_main.c").read_text(encoding="utf-8")
        start = source.index("    case RPI_CMD_SERVO1_RETRACT:")
        end = source.index("    case RPI_CMD_LIFT_MID:", start)
        defines = "\n".join(re.findall(
            r"^#define (?:RPI_CMD_SERVO1_RETRACT|SERVO1_DISC_ALIGN_PULSE_US|SERVO_POWER_ON_SETTLE_MS)\s+[^\r\n]+",
            source, re.MULTILINE))
        harness = """
#include <assert.h>
#include <stdint.h>
#define TIM5 5
static unsigned writes, waited;
static unsigned compares[] = {0, 0, 946, 1460, 2004};
static void TIM_SetCompare2(int timer, uint16_t value) {
    assert(timer == TIM5); compares[2] = value; ++writes;
}
static void delay_ms(unsigned ms) {
    assert(writes == 1 && compares[2] == 2455); waited += ms;
}
""" + defines + "\nstatic uint8_t handle(uint8_t mode) { switch(mode) {\n" + source[start:end] + """
default: return 0; } }
int main(void) {
    assert(RPI_CMD_SERVO1_RETRACT == 52);
    assert(handle(52) == 1);
    assert(writes == 1 && compares[2] == 2455);
    assert(compares[3] == 1460 && compares[4] == 2004);
    assert(waited == SERVO_POWER_ON_SETTLE_MS);
    return 0;
}
"""
        with tempfile.TemporaryDirectory(prefix="rc_servo1_return_") as directory:
            folder = Path(directory)
            file = folder / "test.c"
            exe = folder / "test.exe"
            file.write_text(harness, encoding="utf-8")
            build = subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", str(file), "-o", str(exe)],
                                   capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            run = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)


if __name__ == "__main__":
    unittest.main()
