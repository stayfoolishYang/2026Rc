#!/usr/bin/env python3
"""STM32F103ZE mecanum chassis and arm hardware tuning GUI.

The application never sends a command automatically.  Connecting, editing a
field, loading a profile, or saving source parameters cannot move hardware.
Mechanical movement happens only after the operator explicitly arms the UI,
clicks a test/task button, and accepts the confirmation dialog.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import queue
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Callable, Iterable
import tuning_console_ui as console_ui
import tuning_time_catalog as time_catalog

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - depends on the workstation
    serial = None
    list_ports = None


APP_TITLE = "STM32 麦轮小车与机械臂硬件调参台"
DEFAULT_ROOT = Path(__file__).resolve().parent
DEFAULT_SOURCE = DEFAULT_ROOT / "USER" / "chassis_controller_main.c"
PROJECT_PATH = DEFAULT_ROOT / "USER" / "Template.uvprojx"
OUTPUT_DIR = DEFAULT_ROOT / "OBJ"
LISTING_DIR = PROJECT_PATH.parent / "Listings"
BUILD_LOG_PATH = OUTPUT_DIR / "Template.build_log.htm"
AXF_PATH = OUTPUT_DIR / "Template.axf"


def _configured_path(name: str) -> Path | None:
    value = os.environ.get(name, "").strip()
    if not value:
        return None
    return Path(os.path.expandvars(os.path.expanduser(value)))


def _find_tool(env_name: str, executable: str, relative_to_keil_root: str) -> Path | None:
    configured = _configured_path(env_name)
    if configured is not None:
        return configured

    roots: list[Path] = []
    configured_root = _configured_path("KEIL_ROOT")
    if configured_root is not None:
        roots.append(configured_root)

    for variable in ("ProgramFiles", "ProgramFiles(x86)"):
        value = os.environ.get(variable, "").strip()
        if value:
            roots.append(Path(value) / "Keil_v5")

    roots.extend(
        Path(value)
        for value in (r"C:\Keil_v5", r"D:\Keil_v5", r"C:\Keil", r"D:\Keil")
    )
    for root in roots:
        candidate = root / relative_to_keil_root
        if candidate.exists():
            return candidate

    discovered = shutil.which(executable)
    return Path(discovered) if discovered else None


KEIL_PATH = _find_tool("KEIL_UV4", "UV4.exe", "UV4\\UV4.exe")
FROMELF_PATH = _find_tool("KEIL_FROMELF", "fromelf.exe", "ARM\\ARMCC\\bin\\fromelf.exe")
PYTHON_PATH = Path(sys.executable)
SERIAL_PORT = os.environ.get("STM32_SERIAL_PORT", "COM14").strip().upper() or "COM14"
try:
    SERIAL_BAUD = int(os.environ.get("STM32_SERIAL_BAUD", "115200"))
except ValueError:
    SERIAL_BAUD = 115200

FRAME_HEAD = 0xFF
FRAME_TAIL = 0xFE
PAYLOAD_SIZE = 8
STATUS_NAMES = {
    0x00: "命令无效",
    0x01: "动作完成",
    0x02: "控制器就绪",
    0x03: "PE6 启动按键已按下",
    0x04: "元器件测试已停止；舵机信号已关闭",
    0x05: "PE6 输入为低电平（松开）",
    0x06: "PE6 输入为高电平（按下）",
    0x07: "固件支持元器件测试",
}

TUNING_MODE_SERVO1 = 80
TUNING_MODE_SERVO2 = 81
TUNING_MODE_SERVO3 = 82
TUNING_MODE_LIFT = 83
TEST_MODE_WHEEL = 90
TEST_MODE_STOP = 91
TEST_MODE_LIFT = 92
TEST_MODE_PROBE = 93
TEST_MODE_SERVO = 94
TEST_MODE_BUTTON = 95
CHASSIS_MODE_LINEAR_DISTANCE = 1
CHASSIS_MODE_ROTATE = 3
CHASSIS_LINEAR_TEST_MAX_CM = 20
CHASSIS_ROTATE_TEST_MAX_DEG = 90
TEST_WHEELS = (
    ("左前轮 / MOTOR7", 7, "STEP PC8 / DIR PA8"),
    ("右前轮 / MOTOR4", 4, "STEP PC7 / DIR PG8"),
    ("左后轮 / MOTOR8", 8, "STEP PC9 / DIR PC1"),
    ("右后轮 / MOTOR3", 3, "STEP PC6 / DIR PG7"),
)
SERVO_TUNING_MIN_PULSE_US = 500
SERVO_TUNING_MAX_PULSE_US = 2500
# The current delay_us uses a 24-bit SysTick at 72 MHz.
STEP_PERIOD_MIN_US = 1
STEP_PERIOD_MAX_US = 233015
LIFT_TARGET_MAX_MM = 360


@dataclass(frozen=True)
class ParameterSpec:
    define: str
    label: str
    group: str
    unit: str
    minimum: int
    maximum: int
    step: int
    actuator: str | None
    description: str


PARAMETERS = (
    ParameterSpec("SERVO1_DISC_ALIGN_PULSE_US", "默认收回 / 舵盘对位位置", "Servo1 伸缩", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo1", "上电、初始化、任务收回与mode52使用同一默认位置，当前2455 us；升降不按高度联动伸缩"),
    ParameterSpec("SERVO1_TASK2_EXTEND_PULSE_US", "任务2前伸位置", "Servo1 伸缩", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo1", "奖杯放到讲台前的伸出位置，当前约 60°"),
    ParameterSpec("SERVO2_CLAMP_PULSE_US", "夹紧 / 转运保持", "Servo2 夹爪", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "任务1/2从地面或舵盘抓住物块后共用；保持夹紧再升降、转台"),
    ParameterSpec("SERVO2_DISC_PICK_OPEN_PULSE_US", "舵盘高位抓取张开", "Servo2 夹爪", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "任务1全部舵盘（含舵盘3）及任务2：转台到目标舵盘后，在安全高位张开再下降夹取"),
    ParameterSpec("SERVO2_DISC_RELEASE_PULSE_US", "舵盘低位放块张开", "Servo2 夹爪", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "任务1/2从地面放到舵盘：下降至舵盘放置高度后松开"),
    ParameterSpec("SERVO2_BOTTOM_OPEN_PULSE_US", "地面抓取 / 放置全张开", "Servo2 夹爪", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "地面抓取前充分张开，减少夹爪遮挡圆形识别；地面/领奖台放置、任务结束及mode70也使用"),
    ParameterSpec("SERVO2_CUBE_GROUND_CLAMP_PULSE_US", "正方体地面抓取夹紧", "Servo2 正方体（舵盘2/4）", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "仅任务1正方体从地面抓取时使用；地面抓取前的松开仍使用地面全张开值"),
    ParameterSpec("SERVO2_CUBE_DISC_OPEN_PULSE_US", "正方体舵盘抓取松开", "Servo2 正方体（舵盘2/4）", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "仅任务1正方体从舵盘2/4抓取前，在高位张开后再下降"),
    ParameterSpec("SERVO2_CUBE_DISC_CLAMP_PULSE_US", "正方体舵盘抓取夹紧", "Servo2 正方体（舵盘2/4）", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "仅任务1正方体从舵盘2/4抓取时，在舵盘高度夹紧"),
    ParameterSpec("SERVO3_HOME_PULSE_US", "回正位置", "Servo3 底部旋转", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo3", "机械臂正前方基准位置"),
    ParameterSpec("SERVO3_DISC1_PULSE_US", "舵盘1位置", "Servo3 底部旋转", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo3", "任务1与任务2共用"),
    ParameterSpec("SERVO3_DISC2_PULSE_US", "舵盘2位置", "Servo3 底部旋转", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo3", "任务1使用"),
    ParameterSpec("SERVO3_DISC3_PULSE_US", "舵盘3位置", "Servo3 底部旋转", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo3", "任务1与任务2共用"),
    ParameterSpec("SERVO3_PUT_DISC4_PULSE_US", "舵盘4位置", "Servo3 底部旋转", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo3", "任务1抓取与放置共用"),
    ParameterSpec("SERVO3_DISC5_PULSE_US", "舵盘5位置", "Servo3 底部旋转", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo3", "任务1与任务2共用"),
    ParameterSpec("LIFT_GROUND_PICK_HEIGHT_MM", "地面抓取高度", "升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "从地面夹取物块或奖杯"),
    ParameterSpec("LIFT_GROUND_PLACE_HEIGHT_MM", "普通物块地面放置高度", "升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "任务1舵盘1/3/5放回赛场地面；舵盘2/4堆叠有各自独立高度"),
    ParameterSpec("LIFT_DISC2_STACK_PLACE_HEIGHT_MM", "舵盘2正方体堆叠放置高度", "升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "仅mode15末段：从舵盘2取出后放到赛场圆柱上；绝对软件高度，非加高量；车载取盘高度独立"),
    ParameterSpec("LIFT_DISC4_STACK_PLACE_HEIGHT_MM", "舵盘4正方体堆叠放置高度", "升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "仅mode17末段：从舵盘4取出后放到赛场圆柱上；绝对软件高度，非加高量；车载取盘高度独立"),
    ParameterSpec("LIFT_GROUND_SLOW_APPROACH_DISTANCE_MM", "末段缓降距离", "升降高度", "mm", 1, 270, 5, None, "任务1/2放到舵盘、任务1从舵盘抓取、赛场地面/堆叠和领奖台放置共用；最后该距离减速"),
    ParameterSpec("LIFT_PICK_HEIGHT_MM", "待机结束高度", "升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "软件初始化和任务结束位置"),
    ParameterSpec("LIFT_PLACE_HEIGHT_MM", "任务1舵盘高度", "升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "仅任务1车载舵盘抓取和放置；任务2放杯、取杯高度独立"),
    ParameterSpec("LIFT_TRANSFER_HEIGHT_MM", "转台旋转安全高度", "升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "任务1转台旋转前的高度"),
    ParameterSpec("LIFT_MID_HEIGHT_MM", "中位试动高度", "升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "独立升降调参位置"),
    ParameterSpec("LIFT_178_HEIGHT_MM", "178mm 试动高度", "升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "独立升降调参位置"),
    ParameterSpec("LIFT_186_HEIGHT_MM", "186mm 试动高度", "升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "独立升降调参位置"),
    ParameterSpec("LIFT_TASK2_DISC_PLACE_HEIGHT_MM", "任务2奖杯放入舵盘高度", "任务2升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "mode42/43/44：从地面抓杯后放到舵盘1/3/5，下降到该高度再松开；与从盘取杯高度独立"),
    ParameterSpec("LIFT_TASK2_TRANSFER_HEIGHT_MM", "任务2转台旋转安全高度", "任务2升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "携杯转到舵盘及放杯后回正、去盘取杯前均使用；若旋转时扫到已存奖杯，调这一项"),
    ParameterSpec("LIFT_TASK2_DISC_HEIGHT_MM", "任务2从舵盘取杯高度", "任务2升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "仅mode33/34/35：从舵盘夹取奖杯时使用，不影响mode42/43/44放杯"),
    ParameterSpec("LIFT_TASK2_RETURN_CLEARANCE_MM", "任务2取杯后前伸/回正高度", "任务2升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "mode33/34/35夹紧后先升到该高度，再前伸并回正"),
    ParameterSpec("LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM", "冠军领奖台放置高度", "任务2升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "仅mode33最后放置奖杯时使用；保留原55mm标定"),
    ParameterSpec("LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM", "亚军领奖台放置高度", "任务2升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "仅mode34最后放置奖杯时使用；保留原37mm标定"),
    ParameterSpec("LIFT_TASK2_THIRD_PLACE_HEIGHT_MM", "季军领奖台放置高度", "任务2升降高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "仅mode35最后放置奖杯时使用；保留25mm标定"),
    ParameterSpec("LIFT_CAMERA_TARGET_HIGH_HEIGHT_MM", "摄像头打靶高位（mode84）", "摄像头打靶高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "mode84移动到此软件目标高度，单位mm；独立于领奖台放置高度"),
    ParameterSpec("LIFT_CAMERA_TARGET_LOW_HEIGHT_MM", "摄像头打靶低位（mode85）", "摄像头打靶高度", "mm", 0, LIFT_TARGET_MAX_MM, 1, "lift", "mode85移动到此软件目标高度，单位mm；独立于领奖台放置高度"),
    ParameterSpec("CHASSIS_STEP_HALF_PERIOD_US", "任务1长距速度", "速度与延时", "us", STEP_PERIOD_MIN_US, STEP_PERIOD_MAX_US, 5, None, "mode 88 下任务1长距离巡航速度；数值越小越快"),
    ParameterSpec("CHASSIS_SHORT_STEP_HALF_PERIOD_US", "任务1短距直线速度", "速度与延时", "us", STEP_PERIOD_MIN_US, STEP_PERIOD_MAX_US, 10, None, "mode 88 下小于长距阈值的厘米级直线移动；数值越小越快"),
    ParameterSpec("CHASSIS_RAMP_START_HALF_PERIOD_US", "任务1长距起步停车速度", "速度与延时", "us", STEP_PERIOD_MIN_US, STEP_PERIOD_MAX_US, 10, None, "任务1长距离的起步和停车半周期；不大于巡航值时固件使用固定速度"),
    ParameterSpec("CHASSIS_RAMP_PULSES", "任务1长距加减速脉冲数", "速度与延时", "pulse", 2, 10000, 50, None, "任务1长距离起步和刹停各使用该脉冲数"),
    ParameterSpec("CHASSIS_TASK2_LONG_STEP_HALF_PERIOD_US", "任务2长距巡航速度", "速度与延时", "us", STEP_PERIOD_MIN_US, STEP_PERIOD_MAX_US, 10, None, "mode 87 下长距离移动的独立巡航速度；数值越小越快"),
    ParameterSpec("CHASSIS_SLOW_SHORT_STEP_HALF_PERIOD_US", "任务2短距巡航速度", "速度与延时", "us", STEP_PERIOD_MIN_US, STEP_PERIOD_MAX_US, 10, None, "mode 87 下厘米级短距巡航；起停值大于巡航值时使用加减速"),
    ParameterSpec("CHASSIS_ROTATE_SLOW_STEP_HALF_PERIOD_US", "旋转/纠偏巡航速度", "速度与延时", "us", STEP_PERIOD_MIN_US, STEP_PERIOD_MAX_US, 10, None, "所有 mode 3 旋转和 mode 20 航向纠偏共用；起停值大于巡航值时使用加减速"),
    ParameterSpec("CHASSIS_TASK2_STOP_HALF_PERIOD_US", "任务2移动/全部旋转起停速度", "速度与延时", "us", STEP_PERIOD_MIN_US, STEP_PERIOD_MAX_US, 10, None, "起停半周期；不大于巡航值时固件使用固定速度，不启用该段加减速"),
    ParameterSpec("CHASSIS_TASK2_RAMP_PULSES", "任务2移动/全部旋转加减速脉冲数", "速度与延时", "pulse", 2, 10000, 50, None, "任务2直线移动以及所有旋转/纠偏的起步与停车两端共用；短动作会自动缩短"),
    ParameterSpec("CHASSIS_ROTATE_180_PULSES", "旋转180°脉冲数", "速度与延时", "pulse", 1000, 20000, 5, None, "暂用历史1/16细分标定7790；当前轮组响应非线性且有漂移，先修硬件响应再标定"),
    ParameterSpec("CHASSIS_FINE_STEP_HALF_PERIOD_US", "毫米微调速度", "速度与延时", "us", STEP_PERIOD_MIN_US, STEP_PERIOD_MAX_US, 10, None, "毫米级微调专用固定速度；数值越小越快"),
    ParameterSpec("CHASSIS_TASK2_FINE_STEP_HALF_PERIOD_US", "任务2毫米微调速度", "旋转与微调", "us", STEP_PERIOD_MIN_US, STEP_PERIOD_MAX_US, 10, None, "mode87下的mode5/6固定半周期；与mode88的微调速度独立"),
    ParameterSpec("CHASSIS_RAMP_MIN_DISTANCE_MM", "长距离判定阈值", "速度与延时", "mm", 1, 2000, 10, None, "任务1和任务2达到该距离时使用各自的长距加减速配置"),
    ParameterSpec("LIFT_STEP_HALF_PERIOD_US", "普通升降速度", "速度与延时", "us", STEP_PERIOD_MIN_US, STEP_PERIOD_MAX_US, 5, None, "常规升降速度；数值越小，升降越快"),
    ParameterSpec("LIFT_GROUND_SLOW_STEP_HALF_PERIOD_US", "末段缓降速度", "速度与延时", "us", STEP_PERIOD_MIN_US, STEP_PERIOD_MAX_US, 10, None, "舵盘放物、任务1从盘取物、赛场地面/堆叠和领奖台放置的最后一段共用；数值越大越慢"),
    ParameterSpec("SERVO1_SETTLE_MS", "伸缩动作等待", "速度与延时", "ms", 0, 5000, 50, None, "每次 Servo1 调整后的等待时间"),
    ParameterSpec("SERVO2_SETTLE_MS", "夹爪动作等待", "速度与延时", "ms", 0, 5000, 50, None, "每次夹爪调整后的等待时间"),
    ParameterSpec("SERVO3_SETTLE_MS", "转台动作等待", "速度与延时", "ms", 0, 5000, 50, None, "每次转台调整后的等待时间"),
    ParameterSpec("SERVO_POWER_ON_SETTLE_MS", "独立命令等待", "公共舵机等待", "ms", 0, 5000, 10, None, "mode36/52/70及初始化收尾"),
    ParameterSpec("CHASSIS_DIRECTION_SETTLE_MS", "底盘换向等待", "电机换向等待", "ms", 0, 5000, 1, None, "设好DIR到首个STEP之间"),
    ParameterSpec("LIFT_DIRECTION_SETTLE_MS", "升降换向等待", "电机换向等待", "ms", 0, 5000, 1, None, "普通升降DIR到STEP之间"),
    ParameterSpec("SERVO3_HOME_BEFORE_CLAW_OPEN_MS", "回正后张爪等待", "速度与延时", "ms", 0, 5000, 50, None, "任务1/2从地面抓取并放到舵盘后，转台回正到夹爪张开的专用等待；不影响其他转台动作"),
    ParameterSpec("ARM_ACTION_HOLD_MS", "抓放保持时间", "速度与延时", "ms", 0, 5000, 50, None, "夹紧或松开后的额外保持"),
    ParameterSpec("GROUND_RELEASE_STAGE_INTERVAL_MS", "地面分段张开间隔", "速度与延时", "ms", 0, 1000, 10, None, "中间松开到完全张开之间的时间"),
    ParameterSpec("GRUB1_OPEN_WAIT_MS", "抓取1 张开等待", "任务时序", "ms", 0, 5000, 10, None, "抓取开始时夹爪全张开后的等待"),
    ParameterSpec("GRUB1_CLAMP_WAIT_MS", "抓取1 地面夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "地面夹紧物块后的舵机等待"),
    ParameterSpec("GRUB1_CLAMP_HOLD_MS", "抓取1 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "夹紧并升到转运高度后的保持时间"),
    ParameterSpec("GRUB1_TURNTABLE_WAIT_MS", "抓取1 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到目标舵盘后的等待"),
    ParameterSpec("GRUB1_RELEASE_WAIT_MS", "抓取1 松开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开夹爪后的等待"),
    ParameterSpec("GRUB1_RELEASE_HOLD_MS", "抓取1 松开后保持", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开后的保持时间"),
    ParameterSpec("GRUB1_HOME_WAIT_MS", "抓取1 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("GRUB1_OPEN_AFTER_HOME_WAIT_MS", "抓取1 回正后张开等待", "任务时序", "ms", 0, 5000, 10, None, "回正后夹爪张开的等待"),
    ParameterSpec("GRUB2_OPEN_WAIT_MS", "抓取2 张开等待", "任务时序", "ms", 0, 5000, 10, None, "抓取开始时夹爪全张开后的等待"),
    ParameterSpec("GRUB2_CLAMP_WAIT_MS", "抓取2 地面夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "地面夹紧物块后的舵机等待"),
    ParameterSpec("GRUB2_CLAMP_HOLD_MS", "抓取2 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "夹紧并升到转运高度后的保持时间"),
    ParameterSpec("GRUB2_TURNTABLE_WAIT_MS", "抓取2 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到目标舵盘后的等待"),
    ParameterSpec("GRUB2_RELEASE_WAIT_MS", "抓取2 松开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开夹爪后的等待"),
    ParameterSpec("GRUB2_RELEASE_HOLD_MS", "抓取2 松开后保持", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开后的保持时间"),
    ParameterSpec("GRUB2_HOME_WAIT_MS", "抓取2 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("GRUB2_OPEN_AFTER_HOME_WAIT_MS", "抓取2 回正后张开等待", "任务时序", "ms", 0, 5000, 10, None, "回正后夹爪张开的等待"),
    ParameterSpec("GRUB3_OPEN_WAIT_MS", "抓取3 张开等待", "任务时序", "ms", 0, 5000, 10, None, "抓取开始时夹爪全张开后的等待"),
    ParameterSpec("GRUB3_CLAMP_WAIT_MS", "抓取3 地面夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "地面夹紧物块后的舵机等待"),
    ParameterSpec("GRUB3_CLAMP_HOLD_MS", "抓取3 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "夹紧并升到转运高度后的保持时间"),
    ParameterSpec("GRUB3_TURNTABLE_WAIT_MS", "抓取3 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到目标舵盘后的等待"),
    ParameterSpec("GRUB3_RELEASE_WAIT_MS", "抓取3 松开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开夹爪后的等待"),
    ParameterSpec("GRUB3_RELEASE_HOLD_MS", "抓取3 松开后保持", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开后的保持时间"),
    ParameterSpec("GRUB3_HOME_WAIT_MS", "抓取3 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("GRUB3_OPEN_AFTER_HOME_WAIT_MS", "抓取3 回正后张开等待", "任务时序", "ms", 0, 5000, 10, None, "回正后夹爪张开的等待"),
    ParameterSpec("GRUB4_OPEN_WAIT_MS", "抓取4 张开等待", "任务时序", "ms", 0, 5000, 10, None, "抓取开始时夹爪全张开后的等待"),
    ParameterSpec("GRUB4_CLAMP_WAIT_MS", "抓取4 地面夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "地面夹紧物块后的舵机等待"),
    ParameterSpec("GRUB4_CLAMP_HOLD_MS", "抓取4 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "夹紧并升到转运高度后的保持时间"),
    ParameterSpec("GRUB4_TURNTABLE_WAIT_MS", "抓取4 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到目标舵盘后的等待"),
    ParameterSpec("GRUB4_RELEASE_WAIT_MS", "抓取4 松开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开夹爪后的等待"),
    ParameterSpec("GRUB4_RELEASE_HOLD_MS", "抓取4 松开后保持", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开后的保持时间"),
    ParameterSpec("GRUB4_HOME_WAIT_MS", "抓取4 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("GRUB4_OPEN_AFTER_HOME_WAIT_MS", "抓取4 回正后张开等待", "任务时序", "ms", 0, 5000, 10, None, "回正后夹爪张开的等待"),
    ParameterSpec("GRUB5_OPEN_WAIT_MS", "抓取5 张开等待", "任务时序", "ms", 0, 5000, 10, None, "抓取开始时夹爪全张开后的等待"),
    ParameterSpec("GRUB5_CLAMP_WAIT_MS", "抓取5 地面夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "地面夹紧物块后的舵机等待"),
    ParameterSpec("GRUB5_CLAMP_HOLD_MS", "抓取5 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "夹紧并升到转运高度后的保持时间"),
    ParameterSpec("GRUB5_TURNTABLE_WAIT_MS", "抓取5 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到目标舵盘后的等待"),
    ParameterSpec("GRUB5_RELEASE_WAIT_MS", "抓取5 松开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开夹爪后的等待"),
    ParameterSpec("GRUB5_RELEASE_HOLD_MS", "抓取5 松开后保持", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开后的保持时间"),
    ParameterSpec("GRUB5_HOME_WAIT_MS", "抓取5 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("GRUB5_OPEN_AFTER_HOME_WAIT_MS", "抓取5 回正后张开等待", "任务时序", "ms", 0, 5000, 10, None, "回正后夹爪张开的等待"),
    ParameterSpec("TASK2_GRUB1_OPEN_WAIT_MS", "任务2抓取1 张开等待", "任务时序", "ms", 0, 5000, 10, None, "抓取开始时夹爪全张开后的等待"),
    ParameterSpec("TASK2_GRUB1_CLAMP_WAIT_MS", "任务2抓取1 地面夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "地面夹紧物块后的舵机等待"),
    ParameterSpec("TASK2_GRUB1_CLAMP_HOLD_MS", "任务2抓取1 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "夹紧并升到转运高度后的保持时间"),
    ParameterSpec("TASK2_GRUB1_TURNTABLE_WAIT_MS", "任务2抓取1 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到目标舵盘后的等待"),
    ParameterSpec("TASK2_GRUB1_RELEASE_WAIT_MS", "任务2抓取1 松开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开夹爪后的等待"),
    ParameterSpec("TASK2_GRUB1_RELEASE_HOLD_MS", "任务2抓取1 松开后保持", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开后的保持时间"),
    ParameterSpec("TASK2_GRUB1_HOME_WAIT_MS", "任务2抓取1 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("TASK2_GRUB1_OPEN_AFTER_HOME_WAIT_MS", "任务2抓取1 回正后张开等待", "任务时序", "ms", 0, 5000, 10, None, "回正后夹爪张开的等待"),
    ParameterSpec("TASK2_GRUB3_OPEN_WAIT_MS", "任务2抓取2 张开等待", "任务时序", "ms", 0, 5000, 10, None, "抓取开始时夹爪全张开后的等待"),
    ParameterSpec("TASK2_GRUB3_CLAMP_WAIT_MS", "任务2抓取2 地面夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "地面夹紧物块后的舵机等待"),
    ParameterSpec("TASK2_GRUB3_CLAMP_HOLD_MS", "任务2抓取2 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "夹紧并升到转运高度后的保持时间"),
    ParameterSpec("TASK2_GRUB3_TURNTABLE_WAIT_MS", "任务2抓取2 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到目标舵盘后的等待"),
    ParameterSpec("TASK2_GRUB3_RELEASE_WAIT_MS", "任务2抓取2 松开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开夹爪后的等待"),
    ParameterSpec("TASK2_GRUB3_RELEASE_HOLD_MS", "任务2抓取2 松开后保持", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开后的保持时间"),
    ParameterSpec("TASK2_GRUB3_HOME_WAIT_MS", "任务2抓取2 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("TASK2_GRUB3_OPEN_AFTER_HOME_WAIT_MS", "任务2抓取2 回正后张开等待", "任务时序", "ms", 0, 5000, 10, None, "回正后夹爪张开的等待"),
    ParameterSpec("TASK2_GRUB5_OPEN_WAIT_MS", "任务2抓取3 张开等待", "任务时序", "ms", 0, 5000, 10, None, "抓取开始时夹爪全张开后的等待"),
    ParameterSpec("TASK2_GRUB5_CLAMP_WAIT_MS", "任务2抓取3 地面夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "地面夹紧物块后的舵机等待"),
    ParameterSpec("TASK2_GRUB5_CLAMP_HOLD_MS", "任务2抓取3 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "夹紧并升到转运高度后的保持时间"),
    ParameterSpec("TASK2_GRUB5_TURNTABLE_WAIT_MS", "任务2抓取3 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到目标舵盘后的等待"),
    ParameterSpec("TASK2_GRUB5_RELEASE_WAIT_MS", "任务2抓取3 松开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开夹爪后的等待"),
    ParameterSpec("TASK2_GRUB5_RELEASE_HOLD_MS", "任务2抓取3 松开后保持", "任务时序", "ms", 0, 5000, 10, None, "舵盘上松开后的保持时间"),
    ParameterSpec("TASK2_GRUB5_HOME_WAIT_MS", "任务2抓取3 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("TASK2_GRUB5_OPEN_AFTER_HOME_WAIT_MS", "任务2抓取3 回正后张开等待", "任务时序", "ms", 0, 5000, 10, None, "回正后夹爪张开的等待"),
    ParameterSpec("PUT1_TURNTABLE_WAIT_MS", "放置1 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到舵盘后的等待"),
    ParameterSpec("PUT1_PICK_OPEN_WAIT_MS", "放置1 张开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘侧夹爪张开后的等待"),
    ParameterSpec("PUT1_CLAMP_WAIT_MS", "放置1 舵盘夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "从舵盘夹紧物块后的等待"),
    ParameterSpec("PUT1_CLAMP_HOLD_MS", "放置1 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "从舵盘夹紧并升到转运高度后的保持"),
    ParameterSpec("PUT1_HOME_WAIT_MS", "放置1 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("PUT1_STAGE_INTERVAL_MS", "放置1 地面张开额外保持", "任务时序", "ms", 0, 5000, 10, None, "直接全张开并等待Servo2到位后，再额外保持该时长"),
    ParameterSpec("PUT1_FINAL_OPEN_HOLD_MS", "放置1 最后张开保持", "任务时序", "ms", 0, 5000, 10, None, "地面放置最后完全张开后的额外保持（默认 0）"),
    ParameterSpec("PUT2_TURNTABLE_WAIT_MS", "放置2 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到舵盘后的等待"),
    ParameterSpec("PUT2_PICK_OPEN_WAIT_MS", "放置2 张开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘侧夹爪张开后的等待"),
    ParameterSpec("PUT2_CLAMP_WAIT_MS", "放置2 舵盘夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "从舵盘夹紧物块后的等待"),
    ParameterSpec("PUT2_CLAMP_HOLD_MS", "放置2 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "从舵盘夹紧并升到转运高度后的保持"),
    ParameterSpec("PUT2_HOME_WAIT_MS", "放置2 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("PUT2_STAGE_INTERVAL_MS", "放置2 地面张开额外保持", "任务时序", "ms", 0, 5000, 10, None, "直接全张开并等待Servo2到位后，再额外保持该时长"),
    ParameterSpec("PUT2_FINAL_OPEN_HOLD_MS", "放置2 最后张开保持", "任务时序", "ms", 0, 5000, 10, None, "地面放置最后完全张开后的额外保持（默认 0）"),
    ParameterSpec("PUT3_TURNTABLE_WAIT_MS", "放置3 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到舵盘后的等待"),
    ParameterSpec("PUT3_PICK_OPEN_WAIT_MS", "放置3 张开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘侧夹爪张开后的等待"),
    ParameterSpec("PUT3_CLAMP_WAIT_MS", "放置3 舵盘夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "从舵盘夹紧物块后的等待"),
    ParameterSpec("PUT3_CLAMP_HOLD_MS", "放置3 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "从舵盘夹紧并升到转运高度后的保持"),
    ParameterSpec("PUT3_HOME_WAIT_MS", "放置3 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("PUT3_STAGE_INTERVAL_MS", "放置3 地面张开额外保持", "任务时序", "ms", 0, 5000, 10, None, "直接全张开并等待Servo2到位后，再额外保持该时长"),
    ParameterSpec("PUT3_FINAL_OPEN_HOLD_MS", "放置3 最后张开保持", "任务时序", "ms", 0, 5000, 10, None, "地面放置最后完全张开后的额外保持（默认 0）"),
    ParameterSpec("PUT4_TURNTABLE_WAIT_MS", "放置4 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到舵盘后的等待"),
    ParameterSpec("PUT4_PICK_OPEN_WAIT_MS", "放置4 张开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘侧夹爪张开后的等待"),
    ParameterSpec("PUT4_CLAMP_WAIT_MS", "放置4 舵盘夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "从舵盘夹紧物块后的等待"),
    ParameterSpec("PUT4_CLAMP_HOLD_MS", "放置4 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "从舵盘夹紧并升到转运高度后的保持"),
    ParameterSpec("PUT4_HOME_WAIT_MS", "放置4 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("PUT4_STAGE_INTERVAL_MS", "放置4 地面张开额外保持", "任务时序", "ms", 0, 5000, 10, None, "直接全张开并等待Servo2到位后，再额外保持该时长"),
    ParameterSpec("PUT4_FINAL_OPEN_HOLD_MS", "放置4 最后张开保持", "任务时序", "ms", 0, 5000, 10, None, "地面放置最后完全张开后的额外保持（默认 0）"),
    ParameterSpec("PUT5_TURNTABLE_WAIT_MS", "放置5 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到舵盘后的等待"),
    ParameterSpec("PUT5_PICK_OPEN_WAIT_MS", "放置5 张开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘侧夹爪张开后的等待"),
    ParameterSpec("PUT5_CLAMP_WAIT_MS", "放置5 舵盘夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "从舵盘夹紧物块后的等待"),
    ParameterSpec("PUT5_CLAMP_HOLD_MS", "放置5 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "从舵盘夹紧并升到转运高度后的保持"),
    ParameterSpec("PUT5_HOME_WAIT_MS", "放置5 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("PUT5_STAGE_INTERVAL_MS", "放置5 地面张开额外保持", "任务时序", "ms", 0, 5000, 10, None, "直接全张开并等待Servo2到位后，再额外保持该时长"),
    ParameterSpec("PUT5_FINAL_OPEN_HOLD_MS", "放置5 最后张开保持", "任务时序", "ms", 0, 5000, 10, None, "地面放置最后完全张开后的额外保持（默认 0）"),
    ParameterSpec("TASK2_PLACE1_RETRACT_WAIT_MS", "任务2放置1（冠军） 收回等待", "任务时序", "ms", 0, 5000, 10, None, "机械臂收回后的等待"),
    ParameterSpec("TASK2_PLACE1_TURNTABLE_WAIT_MS", "任务2放置1（冠军） 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到舵盘后的等待"),
    ParameterSpec("TASK2_PLACE1_RELEASE_WAIT_MS", "任务2放置1（冠军） 张开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘侧张开后的等待"),
    ParameterSpec("TASK2_PLACE1_CLAMP_WAIT_MS", "任务2放置1（冠军） 舵盘夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "从舵盘夹紧奖杯后的等待"),
    ParameterSpec("TASK2_PLACE1_CLAMP_HOLD_MS", "任务2放置1（冠军） 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "夹紧后升到回正安全高度的保持"),
    ParameterSpec("TASK2_PLACE1_EXTEND_WAIT_MS", "任务2放置1（冠军） 前伸等待", "任务时序", "ms", 0, 5000, 10, None, "前伸到位后的等待"),
    ParameterSpec("TASK2_PLACE1_HOME_WAIT_MS", "任务2放置1（冠军） 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("TASK2_PLACE1_OPEN_WAIT_MS", "任务2放置1（冠军） 领奖台张开等待", "任务时序", "ms", 0, 5000, 10, None, "领奖台上张开夹爪后的等待"),
    ParameterSpec("TASK2_PLACE1_OPEN_HOLD_MS", "任务2放置1（冠军） 领奖台张开保持", "任务时序", "ms", 0, 5000, 10, None, "领奖台张开后的保持"),
    ParameterSpec("TASK2_PLACE2_RETRACT_WAIT_MS", "任务2放置2（亚军） 收回等待", "任务时序", "ms", 0, 5000, 10, None, "机械臂收回后的等待"),
    ParameterSpec("TASK2_PLACE2_TURNTABLE_WAIT_MS", "任务2放置2（亚军） 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到舵盘后的等待"),
    ParameterSpec("TASK2_PLACE2_RELEASE_WAIT_MS", "任务2放置2（亚军） 张开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘侧张开后的等待"),
    ParameterSpec("TASK2_PLACE2_CLAMP_WAIT_MS", "任务2放置2（亚军） 舵盘夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "从舵盘夹紧奖杯后的等待"),
    ParameterSpec("TASK2_PLACE2_CLAMP_HOLD_MS", "任务2放置2（亚军） 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "夹紧后升到回正安全高度的保持"),
    ParameterSpec("TASK2_PLACE2_EXTEND_WAIT_MS", "任务2放置2（亚军） 前伸等待", "任务时序", "ms", 0, 5000, 10, None, "前伸到位后的等待"),
    ParameterSpec("TASK2_PLACE2_HOME_WAIT_MS", "任务2放置2（亚军） 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("TASK2_PLACE2_OPEN_WAIT_MS", "任务2放置2（亚军） 领奖台张开等待", "任务时序", "ms", 0, 5000, 10, None, "领奖台上张开夹爪后的等待"),
    ParameterSpec("TASK2_PLACE2_OPEN_HOLD_MS", "任务2放置2（亚军） 领奖台张开保持", "任务时序", "ms", 0, 5000, 10, None, "领奖台张开后的保持"),
    ParameterSpec("TASK2_PLACE3_RETRACT_WAIT_MS", "任务2放置3（季军） 收回等待", "任务时序", "ms", 0, 5000, 10, None, "机械臂收回后的等待"),
    ParameterSpec("TASK2_PLACE3_TURNTABLE_WAIT_MS", "任务2放置3（季军） 转台到位等待", "任务时序", "ms", 0, 5000, 10, None, "转台转到舵盘后的等待"),
    ParameterSpec("TASK2_PLACE3_RELEASE_WAIT_MS", "任务2放置3（季军） 张开等待", "任务时序", "ms", 0, 5000, 10, None, "舵盘侧张开后的等待"),
    ParameterSpec("TASK2_PLACE3_CLAMP_WAIT_MS", "任务2放置3（季军） 舵盘夹紧等待", "任务时序", "ms", 0, 5000, 10, None, "从舵盘夹紧奖杯后的等待"),
    ParameterSpec("TASK2_PLACE3_CLAMP_HOLD_MS", "任务2放置3（季军） 夹紧后保持", "任务时序", "ms", 0, 5000, 10, None, "夹紧后升到回正安全高度的保持"),
    ParameterSpec("TASK2_PLACE3_EXTEND_WAIT_MS", "任务2放置3（季军） 前伸等待", "任务时序", "ms", 0, 5000, 10, None, "前伸到位后的等待"),
    ParameterSpec("TASK2_PLACE3_HOME_WAIT_MS", "任务2放置3（季军） 回正等待", "任务时序", "ms", 0, 5000, 10, None, "转台回正后的等待"),
    ParameterSpec("TASK2_PLACE3_OPEN_WAIT_MS", "任务2放置3（季军） 领奖台张开等待", "任务时序", "ms", 0, 5000, 10, None, "领奖台上张开夹爪后的等待"),
    ParameterSpec("TASK2_PLACE3_OPEN_HOLD_MS", "任务2放置3（季军） 领奖台张开保持", "任务时序", "ms", 0, 5000, 10, None, "领奖台张开后的保持"),
)

PARAMETERS = time_catalog.organize_specs(PARAMETERS)
PARAMETER_BY_DEFINE = {item.define: item for item in PARAMETERS}
CUBE_SERVO2_PARAMETER_DEFINES = (
    "SERVO2_CUBE_GROUND_CLAMP_PULSE_US",
    "SERVO2_CUBE_DISC_OPEN_PULSE_US",
    "SERVO2_CUBE_DISC_CLAMP_PULSE_US",
)
CUBE_STACK_HEIGHT_PARAMETER_DEFINES = (
    "LIFT_DISC2_STACK_PLACE_HEIGHT_MM",
    "LIFT_DISC4_STACK_PLACE_HEIGHT_MM",
)
TASK2_NEW_HEIGHT_PARAMETER_DEFINES = (
    "LIFT_TASK2_DISC_PLACE_HEIGHT_MM",
    "LIFT_TASK2_TRANSFER_HEIGHT_MM",
)
TASK2_PODIUM_HEIGHT_PARAMETER_DEFINES = (
    "LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM",
    "LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM",
    "LIFT_TASK2_THIRD_PLACE_HEIGHT_MM",
)
CAMERA_TARGET_HEIGHT_PARAMETER_DEFINES = (
    "LIFT_CAMERA_TARGET_HIGH_HEIGHT_MM",
    "LIFT_CAMERA_TARGET_LOW_HEIGHT_MM",
)
LEGACY_TASK2_PODIUM_PROFILE_FIELDS = {
    "LIFT_TASK2_CHAMPION_HEIGHT_MM": "LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM",
    "LIFT_TASK2_RUNNER_UP_HEIGHT_MM": "LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM",
}
GROUPS = ("Servo1 伸缩", "Servo2 夹爪", "Servo2 正方体（舵盘2/4）", "Servo3 底部旋转",
          "升降高度", "任务2升降高度", "摄像头打靶高度") + time_catalog.TIME_GROUPS


def _timing_specs(prefix: str) -> tuple[ParameterSpec, ...]:
    return tuple(spec for spec in PARAMETERS if spec.define.startswith(prefix))


TIMING_OPERATIONS = (
    ("任务1 抓取 → 舵盘1", "GRUB1", "mode 7", _timing_specs("GRUB1_")),
    ("任务1 正方体抓取 → 舵盘2", "GRUB2", "mode 8", _timing_specs("GRUB2_")),
    ("任务1 抓取 → 舵盘3", "GRUB3", "mode 9", _timing_specs("GRUB3_")),
    ("任务1 正方体抓取 → 舵盘4", "GRUB4", "mode 10", _timing_specs("GRUB4_")),
    ("任务1 抓取 → 舵盘5", "GRUB5", "mode 12", _timing_specs("GRUB5_")),
    ("任务2 抓取1 → 舵盘1", "TASK2_GRUB1", "mode 42", _timing_specs("TASK2_GRUB1_")),
    ("任务2 抓取2 → 舵盘3", "TASK2_GRUB3", "mode 43", _timing_specs("TASK2_GRUB3_")),
    ("任务2 抓取3 → 舵盘5", "TASK2_GRUB5", "mode 44", _timing_specs("TASK2_GRUB5_")),
    ("任务1 放置 舵盘1 → 地面", "PUT1", "mode 14", _timing_specs("PUT1_")),
    ("任务1 正方体放置 舵盘2 → 堆叠", "PUT2", "mode 15", _timing_specs("PUT2_")),
    ("任务1 放置 舵盘3 → 地面", "PUT3", "mode 16", _timing_specs("PUT3_")),
    ("任务1 正方体放置 舵盘4 → 堆叠", "PUT4", "mode 17", _timing_specs("PUT4_")),
    ("任务1 放置 舵盘5 → 地面", "PUT5", "mode 18", _timing_specs("PUT5_")),
    ("任务2 放置1 舵盘1→冠军", "TASK2_PLACE1", "mode 33", _timing_specs("TASK2_PLACE1_")),
    ("任务2 放置2 舵盘3→亚军", "TASK2_PLACE2", "mode 34", _timing_specs("TASK2_PLACE2_")),
    ("任务2 放置3 舵盘5→季军", "TASK2_PLACE3", "mode 35", _timing_specs("TASK2_PLACE3_")),
)


TASK_GROUPS = (
    (
        "任务1：圆形物块 地面 → 舵盘",
        (("舵盘1", 7), ("舵盘3", 9), ("舵盘5", 12)),
    ),
    (
        "任务1：正方体 地面 → 舵盘2/4",
        (("正方体→舵盘2", 8), ("正方体→舵盘4", 10)),
    ),
    (
        "任务1：圆形物块 舵盘 → 地面",
        (("舵盘1", 14), ("舵盘3", 16), ("舵盘5", 18)),
    ),
    (
        "任务1：正方体 舵盘2/4 → 地面",
        (("正方体 舵盘2→地面", 15), ("正方体 舵盘4→地面", 17)),
    ),
    (
        "任务2：地面奖杯 → 舵盘",
        (("奖杯1→舵盘1", 42), ("奖杯2→舵盘3", 43), ("奖杯3→舵盘5", 44)),
    ),
    (
        "任务2：舵盘 → 讲台",
        (("舵盘1→冠军", 33), ("舵盘3→亚军", 34), ("舵盘5→季军", 35)),
    ),
)

MANUAL_ACTIONS = (
    ("软件初始化", 72, True),
    ("Servo1 伸到任务2位置", 36, False),
    ("Servo1 收回", 52, False),
    ("夹爪地面全张开", 70, False),
    ("夹爪单独闭合", 71, False),
    ("升到最高行程（360 mm）", 48, True),
    ("降到 0 mm", 49, True),
    ("升到 mode 53 高度", 53, True),
    ("升到 178 mm", 58, True),
    ("升到 186 mm", 59, True),
    ("摄像头打靶高位（mode84配置高度）", 84, True),
    ("摄像头打靶低位（mode85配置高度）", 85, True),
    ("底部旋转舵机回 24°", 86, False),
    ("启用任务2底盘配置", 87, False),
    ("恢复任务1底盘配置", 88, False),
    ("启用 PE6 启动按键", 89, False),
    ("记录陀螺仪基准", 19, False),
    ("底盘航向纠偏", 20, False),
)


def task_relationship_text(prefix: str, values: dict[str, str]) -> str:
    """Describe task phases using the current calibration inputs; sends nothing."""
    if not prefix:
        return ""
    disc = int(prefix[-1])
    if prefix.startswith("TASK2_PLACE"):
        disc = (1, 3, 5)[disc - 1]
    disc_define = "SERVO3_PUT_DISC4_PULSE_US" if disc == 4 else f"SERVO3_DISC{disc}_PULSE_US"
    v = lambda define: values.get(define, "—")
    target = f"舵盘{disc}（{v(disc_define)} us）"
    ground = f"地面全张开 {v('SERVO2_BOTTOM_OPEN_PULSE_US')} us"
    clamp = f"夹紧 {v('SERVO2_CLAMP_PULSE_US')} us"
    high_open = f"舵盘高位抓取张开 {v('SERVO2_DISC_PICK_OPEN_PULSE_US')} us"
    disc_open = f"舵盘低位放块张开 {v('SERVO2_DISC_RELEASE_PULSE_US')} us"
    is_cube = prefix in {"GRUB2", "GRUB4", "PUT2", "PUT4"}
    ground_clamp = f"夹紧 {v('SERVO2_CUBE_GROUND_CLAMP_PULSE_US')} us" if is_cube else clamp
    disc_pick_open = f"舵盘抓取松开 {v('SERVO2_CUBE_DISC_OPEN_PULSE_US')} us" if is_cube else high_open
    disc_clamp = f"夹紧 {v('SERVO2_CUBE_DISC_CLAMP_PULSE_US')} us" if is_cube else clamp
    reach = f"伸缩对位 {v('SERVO1_DISC_ALIGN_PULSE_US')} us"
    transfer = f"{v('LIFT_TASK2_TRANSFER_HEIGHT_MM' if prefix.startswith('TASK2_') else 'LIFT_TRANSFER_HEIGHT_MM')} mm"
    disc_place = f"{v('LIFT_TASK2_DISC_PLACE_HEIGHT_MM' if prefix.startswith('TASK2_GRUB') else 'LIFT_PLACE_HEIGHT_MM')} mm"
    home = f"转台回正 {v('SERVO3_HOME_PULSE_US')} us"
    idle = f"{v('LIFT_PICK_HEIGHT_MM')} mm"
    if prefix.startswith("TASK2_PLACE"):
        podium_define = ("LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM", "LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM",
                        "LIFT_TASK2_THIRD_PLACE_HEIGHT_MM")[int(prefix[-1]) - 1]
        steps = (
            f"1. 伸缩收回 → 升至 {transfer} → {reach} → 转台到{target} → {high_open}。",
            f"2. 降至 {v('LIFT_TASK2_DISC_HEIGHT_MM')} mm → {clamp}；基础等待＋夹杯额外停留后才上升。",
            f"3. 升至 {v('LIFT_TASK2_RETURN_CLEARANCE_MM')} mm → 伸缩前伸 {v('SERVO1_TASK2_EXTEND_PULSE_US')} us → {home}。",
            f"4. 缓降至领奖台 {v(podium_define)} mm → {ground}；松爪等待＋额外停留后回升。",
            f"5. 升至 {idle}；保持领奖台前伸和全张开。",
        )
    elif prefix.startswith("PUT"):
        field_height_define = {"PUT2": "LIFT_DISC2_STACK_PLACE_HEIGHT_MM",
                               "PUT4": "LIFT_DISC4_STACK_PLACE_HEIGHT_MM"}.get(prefix, "LIFT_GROUND_PLACE_HEIGHT_MM")
        destination = "圆柱顶部" if is_cube else "地面"
        steps = (
            f"1. 升至 {transfer} → {reach} → 转台到{target} → {disc_pick_open}。",
            f"2. 缓降至 {v('LIFT_PLACE_HEIGHT_MM')} mm → {disc_clamp}；基础等待＋夹紧额外停留后才上升。",
            f"3. 升至 {transfer} → {home}；保持{reach}。",
            f"4. 保持对位，缓降至{destination} {v(field_height_define)} mm → {ground}；公共Servo2等待＋额外A＋额外B后回升。",
            f"5. 升至 {idle}；保持{reach}和全张开。",
        )
    else:
        steps = (
            f"1. {ground} → {reach} → 降至地面 {v('LIFT_GROUND_PICK_HEIGHT_MM')} mm。",
            f"2. {ground_clamp} → 夹紧等待 → 升至 {transfer} → 夹紧额外停留；保持{reach}。",
            f"3. 明确{reach} → 转台到{target}。",
            f"4. 缓降至舵盘 {disc_place} → {disc_open}；松爪等待＋额外停留后回升。",
            f"5. 升至 {transfer} → {home} → {ground} → 降至 {idle}。",
        )
    if prefix.startswith("TASK2_GRUB"):
        steps = ("任务2放杯流程：转盘安全高度与放入舵盘高度独立；“从舵盘取杯高度”只用于取杯任务。",) + steps
    if is_cube:
        steps = ("本任务为2/4号舵盘正方体，使用单独的夹爪参数：" +
                 f"地面夹紧 {v('SERVO2_CUBE_GROUND_CLAMP_PULSE_US')} us；" +
                 f"舵盘松开/夹紧 {v('SERVO2_CUBE_DISC_OPEN_PULSE_US')} / " +
                 f"{v('SERVO2_CUBE_DISC_CLAMP_PULSE_US')} us。",) + steps
    return "\n".join(steps)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def make_frame(mode: int, value: int | None = None) -> bytes:
    if not 0 <= mode <= 255:
        raise ValueError("mode 必须在 0–255 范围内")
    payload = [0] * PAYLOAD_SIZE
    payload[0] = mode
    if value is not None:
        if not 0 <= value <= 65535:
            raise ValueError("参数必须在 0–65535 范围内")
        payload[1] = (value >> 8) & 0xFF
        payload[2] = value & 0xFF
    return bytes([FRAME_HEAD, *payload, FRAME_TAIL])


def make_wheel_test_frame(motor: int, direction: int, pulses: int, half_period_us: int) -> bytes:
    if motor not in (3, 4, 7, 8) or direction not in (0, 1):
        raise ValueError("请选择 MOTOR3/4/7/8，方向为 0 或 1")
    if not 1 <= pulses <= 3200 or not 1 <= half_period_us <= 65535:
        raise ValueError("脉冲数应为 1–3200，半周期应为 1–65535 μs")
    return bytes([FRAME_HEAD, TEST_MODE_WHEEL, motor, direction,
                  pulses >> 8, pulses & 255, half_period_us >> 8, half_period_us & 255, 0, FRAME_TAIL])


def make_lift_test_frame(target_mm: int, half_period_us: int) -> bytes:
    if not 0 <= target_mm <= LIFT_TARGET_MAX_MM or not 1 <= half_period_us <= 65535:
        raise ValueError(f"升降目标应为 0–{LIFT_TARGET_MAX_MM} mm，半周期应为 1–65535 μs")
    return bytes([FRAME_HEAD, TEST_MODE_LIFT, target_mm >> 8, target_mm & 255,
                  half_period_us >> 8, half_period_us & 255, 0, 0, 0, FRAME_TAIL])


def make_servo_test_frame(servo: int, pulse_us: int) -> bytes:
    if servo not in (1, 2, 3) or not SERVO_TUNING_MIN_PULSE_US <= pulse_us <= SERVO_TUNING_MAX_PULSE_US:
        raise ValueError("舵机编号应为 1–3，脉宽应为 500–2500 μs")
    return bytes([FRAME_HEAD, TEST_MODE_SERVO, servo, pulse_us >> 8, pulse_us & 255,
                  0, 0, 0, 0, FRAME_TAIL])


def make_chassis_linear_test_frame(distance_cm: int) -> bytes:
    if distance_cm == 0 or not -CHASSIS_LINEAR_TEST_MAX_CM <= distance_cm <= CHASSIS_LINEAR_TEST_MAX_CM:
        raise ValueError(f"底盘直线试跑距离应为 ±1–{CHASSIS_LINEAR_TEST_MAX_CM} cm")
    value = distance_cm & 0xFFFF
    return bytes([FRAME_HEAD, CHASSIS_MODE_LINEAR_DISTANCE, value >> 8, value & 255,
                  0, 0, 0, 0, 0, FRAME_TAIL])


def make_chassis_rotate_test_frame(angle_deg: int) -> bytes:
    if angle_deg == 0 or not -CHASSIS_ROTATE_TEST_MAX_DEG <= angle_deg <= CHASSIS_ROTATE_TEST_MAX_DEG:
        raise ValueError(f"底盘旋转试跑角度应为 ±1–{CHASSIS_ROTATE_TEST_MAX_DEG}°")
    return bytes([FRAME_HEAD, CHASSIS_MODE_ROTATE, 0, 0, angle_deg & 255,
                  0, 0, 0, 0, FRAME_TAIL])


def response_statuses(mode: int) -> tuple[int, ...]:
    if mode == TEST_MODE_STOP:
        return (0, 4)
    if mode == TEST_MODE_PROBE:
        return (0, 7)
    if mode == TEST_MODE_BUTTON:
        return (0, 5, 6)
    return (0, 1, 4)


def hex_bytes(data: Iterable[int]) -> str:
    return " ".join(f"{value:02X}" for value in data)


def parse_source_parameters(source_path: Path) -> tuple[dict[str, int], str]:
    raw = source_path.read_bytes()
    values: dict[str, int] = {}
    for spec in PARAMETERS:
        pattern = re.compile(
            rb"(?m)^\s*#define\s+" + re.escape(spec.define.encode("ascii")) + rb"\s+(\d+)u\b"
        )
        match = pattern.search(raw)
        if match is None:
            raise ValueError(
                f"调参台参数列表与源码不匹配：{spec.define}\n"
                f"当前源码：{source_path}\n\n"
                "请关闭旧调参台窗口，再从项目目录的“启动硬件调参台.bat”重新打开。"
                "重新读取源码不会更新已运行的程序代码。"
            )
        values[spec.define] = int(match.group(1))
    return values, sha256_bytes(raw)


def read_firmware_version(source_path: Path) -> int:
    raw = source_path.read_bytes()
    match = re.search(
        rb"(?m)^\s*#define\s+FIRMWARE_DIAGNOSTIC_VERSION\s+(\d+)u\b", raw
    )
    if match is None:
        raise ValueError("源码中找不到 FIRMWARE_DIAGNOSTIC_VERSION")
    return int(match.group(1))


def replace_source_parameters(
    source_path: Path, expected_hash: str, values: dict[str, int]
) -> str:
    raw = source_path.read_bytes()
    if sha256_bytes(raw) != expected_hash:
        raise RuntimeError("源码已被其他程序修改。为避免覆盖现有改动，请先重新读取源码。")

    updated = raw
    for spec in PARAMETERS:
        value = values[spec.define]
        pattern = re.compile(
            rb"(?m)^(\s*#define\s+"
            + re.escape(spec.define.encode("ascii"))
            + rb"\s+)(\d+)(u\b)"
        )
        replacement = lambda match, new=str(value).encode("ascii"): (
            match.group(1) + new + match.group(3)
        )
        updated, count = pattern.subn(replacement, updated, count=1)
        if count != 1:
            raise RuntimeError(f"参数替换失败：{spec.define}")

    if updated == raw:
        return expected_hash

    temp_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", prefix=source_path.name + ".", suffix=".tmp", dir=source_path.parent, delete=False
        ) as handle:
            handle.write(updated)
            handle.flush()
            os.fsync(handle.fileno())
            temp_name = handle.name
        os.replace(temp_name, source_path)
        temp_name = None
    finally:
        if temp_name is not None:
            try:
                os.unlink(temp_name)
            except OSError:
                pass
    return sha256_bytes(updated)


def approximate_degrees(actuator: str, pulse_us: int) -> float:
    zero = 426.0 if actuator == "servo2" else 500.0
    return (pulse_us - zero) * 270.0 / 2000.0


class SerialWorker:
    """Own the configured serial port in a worker thread."""

    def __init__(self, events: queue.Queue[dict[str, object]]) -> None:
        self.events = events
        self.requests: queue.Queue[tuple[str, object]] = queue.Queue()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def request(self, action: str, data: object = None) -> None:
        self.requests.put((action, data))

    def _emit(self, event: str, **data: object) -> None:
        self.events.put({"event": event, **data})

    def _run(self) -> None:
        port = None
        rx_buffer = bytearray()
        pending: dict[str, object] | None = None
        running = True

        while running:
            try:
                while True:
                    action, data = self.requests.get_nowait()
                    if action == "connect":
                        if port is not None:
                            port.close()
                        try:
                            port = serial.Serial(port=None, baudrate=SERIAL_BAUD, timeout=0)
                            # Match the modem-line states of the verified COM14 runtime probe.
                            # Set them before opening instead of asserting pyserial's defaults.
                            port.dtr = False
                            port.rts = False
                            port.port = SERIAL_PORT
                            port.open()
                            port.reset_input_buffer()
                            port.reset_output_buffer()
                            rx_buffer.clear()
                            pending = None
                            self._emit("connected")
                        except Exception as exc:
                            port = None
                            self._emit("serial_error", message=f"打开 {SERIAL_PORT} 失败：{exc}")
                    elif action == "disconnect":
                        if port is not None:
                            port.close()
                        port = None
                        pending = None
                        rx_buffer.clear()
                        self._emit("disconnected")
                    elif action in ("send", "hardware_stop"):
                        frame, label, timeout_s = data  # type: ignore[misc]
                        if port is None:
                            self._emit("serial_error", message=f"请先连接 {SERIAL_PORT}。")
                            self._emit("operation_done")
                            continue
                        if pending is not None and action != "hardware_stop":
                            self._emit("serial_error", message="上一条命令仍在等待返回。")
                            self._emit("operation_done")
                            continue
                        try:
                            port.write(frame)
                            port.flush()
                            now = time.monotonic()
                            pending = {
                                "label": label,
                                "started": now,
                                "deadline": now + float(timeout_s),
                                "terminal_statuses": response_statuses(frame[1]),
                            }
                            self._emit("tx", frame=frame, label=label)
                        except Exception as exc:
                            self._emit("serial_error", message=f"发送失败：{exc}")
                            self._emit("operation_done")
                    elif action == "stop":
                        running = False
                        break
            except queue.Empty:
                pass
            except Exception as exc:
                self._emit("serial_error", message=f"串口线程异常：{exc}")

            if not running:
                break

            if port is not None:
                try:
                    waiting = port.in_waiting
                    if waiting:
                        rx_buffer.extend(port.read(waiting))
                    pending = self._parse_status(rx_buffer, pending)
                except Exception as exc:
                    try:
                        port.close()
                    except Exception:
                        pass
                    port = None
                    pending = None
                    self._emit("serial_error", message=f"{SERIAL_PORT} 连接中断：{exc}")
                    self._emit("disconnected")

            if pending is not None and time.monotonic() >= float(pending["deadline"]):
                elapsed = time.monotonic() - float(pending["started"])
                self._emit("timeout", label=pending["label"], elapsed=elapsed)
                pending = None
                self._emit("operation_done")

            time.sleep(0.01)

        if port is not None:
            port.close()

    def _parse_status(
        self, rx_buffer: bytearray, pending: dict[str, object] | None
    ) -> dict[str, object] | None:
        while len(rx_buffer) >= 3:
            try:
                head = rx_buffer.index(FRAME_HEAD)
            except ValueError:
                self._emit("noise", data=bytes(rx_buffer))
                rx_buffer.clear()
                return pending
            if head:
                self._emit("noise", data=bytes(rx_buffer[:head]))
                del rx_buffer[:head]
            if len(rx_buffer) < 3:
                break
            if rx_buffer[2] != FRAME_TAIL:
                self._emit("noise", data=bytes([rx_buffer[0]]))
                del rx_buffer[0]
                continue
            frame = bytes(rx_buffer[:3])
            del rx_buffer[:3]
            status = frame[1]
            elapsed = None
            if pending is not None:
                elapsed = time.monotonic() - float(pending["started"])
            self._emit("rx", frame=frame, status=status, elapsed=elapsed)
            if pending is not None and status in pending.get("terminal_statuses", (0x00, 0x01)):
                pending = None
                self._emit("operation_done")
        return pending


class HardwareTuningApp(tk.Tk):
    def __init__(self, source_path: Path) -> None:
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1240x820")
        self.minsize(1000, 680)

        self.source_path = source_path.resolve()
        self.events: queue.Queue[dict[str, object]] = queue.Queue()
        self.worker = SerialWorker(self.events) if serial is not None else None
        self.connected = False
        self.busy = False
        self.build_busy = False
        self.component_busy = False
        self.component_supported = False
        self._controller_responded = False
        self.component_result_var = tk.StringVar(value="先连接串口，再点击“检测测试固件”。")
        self.test_pulses_var = tk.StringVar(value="200")
        self.test_period_var = tk.StringVar(value="1000")
        self.test_lift_target_var = tk.StringVar(value="5")
        self.test_lift_period_var = tk.StringVar(value="1000")
        self.test_servo_vars = {index: tk.StringVar(value="1500") for index in (1, 2, 3)}
        self.test_chassis_distance_var = tk.StringVar(value="5")
        self.test_chassis_angle_var = tk.StringVar(value="10")
        self.loaded_hash = ""
        self.loaded_values: dict[str, int] = {}
        self.value_vars = {spec.define: tk.StringVar() for spec in PARAMETERS}
        self.task_relationship_var = tk.StringVar()
        self.task_wait_summary_var = tk.StringVar()
        self.command_wait_summary_var = tk.StringVar()
        self.task_relationship_prefix = ""
        self.time_source_constants = {}
        self.degree_vars: dict[str, tk.StringVar] = {}
        self._timing_traced: set[str] = set()
        self.servo2_manual_var = tk.StringVar(value="")
        self.servo2_manual_degree_var = tk.StringVar(value="")
        self.servo2_manual_var.trace_add("write", lambda *_args: self._update_servo2_manual_degree())
        self.action_armed_var = tk.BooleanVar(value=False)
        self.lift_synced_var = tk.BooleanVar(value=False)
        self.turntable_safe_var = tk.BooleanVar(value=False)
        self.connection_var = tk.StringVar(value="未连接")
        self.port_scan_var = tk.StringVar(value="尚未扫描")
        self.status_var = tk.StringVar(value="等待操作")
        self.dirty_var = tk.StringVar(value="")
        self.bin_name_var = tk.StringVar(value="")
        self.action_buttons: list[ttk.Button] = []
        self._loading_parameters = False
        for variable in self.value_vars.values():
            variable.trace_add("write", lambda *_args: self._on_parameter_changed())

        self._configure_style()
        self._build_ui()
        self._load_source(show_message=False)
        self._refresh_ports()
        self.after(50, self._poll_events)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        if serial is None:
            self.after(
                100,
                lambda: messagebox.showerror(
                    "缺少组件", "未安装 pyserial。请在仓库根目录运行：\npython -m pip install -r requirements.txt"
                ),
            )

    def _configure_style(self) -> None:
        console_ui.configure_style(self)

    def _build_ui(self) -> None:
        self._console_pages = console_ui.build_shell(self, APP_TITLE, SERIAL_PORT, SERIAL_BAUD)
        self._console_builders = (self._build_parameter_tab, self._build_timing_tab, self._build_task_tab,
                                  self._build_component_tab, self._build_build_tab, self._build_log_tab)
        self._console_loaded_pages = set()
        self._ensure_console_page(5)  # Retain the complete log from startup onward.
        self._ensure_console_page(0)
        self.notebook.bind("<<NotebookTabChanged>>", lambda _e: self._ensure_console_page(
            self.notebook.index(self.notebook.select())), add="+")
        self.bind("<Control-f>", self._focus_parameter_search)
        self.bind("<Destroy>", self._cancel_console_filter, add="+")

    def _cancel_console_filter(self, event) -> None:
        if event.widget is self and getattr(self, "_parameter_filter_job", None) is not None:
            self.after_cancel(self._parameter_filter_job)
            self._parameter_filter_job = None

    def _ensure_console_page(self, index: int) -> None:
        if index in self._console_loaded_pages:
            return
        self._console_loaded_pages.add(index)
        page = self._console_pages[index]
        try:
            self._console_builders[index](page)
        except Exception:
            self._console_loaded_pages.discard(index)
            raise
        console_ui.finish_widgets(self, page)

    def _build_parameter_tab(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="位置PWM（μs）决定舵机位置；半周期（μs）决定电机速度；等待（ms）决定下一步何时开始。",
                  style="Muted.TLabel", wraplength=650).pack(anchor="w", pady=(0, 10))
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill="x", pady=(0, 12))
        for column, (label, callback) in enumerate((
            ("重新读取源码", self._load_source), ("预览修改", self._preview_changes),
            ("导出参数方案", self._export_profile), ("导入参数方案", self._import_profile),
        )):
            ttk.Button(toolbar, text=label, command=callback).grid(row=0, column=column, padx=(0, 8))

        filters = ttk.Frame(parent)
        filters.pack(fill="x", pady=(0, 12))
        self.parameter_group_var = tk.StringVar(value=GROUPS[0])
        self.parameter_search_var = tk.StringVar()
        self.parameter_count_var = tk.StringVar()
        ttk.Label(filters, text="分组", style="Muted.TLabel").pack(side="left", padx=(0, 7))
        ttk.Combobox(filters, textvariable=self.parameter_group_var, state="readonly",
                     values=("全部器件",) + GROUPS, width=22).pack(side="left", padx=(0, 14))
        ttk.Label(filters, text="搜索", style="Muted.TLabel").pack(side="left", padx=(0, 7))
        self.parameter_search_entry = ttk.Entry(filters, textvariable=self.parameter_search_var, width=22)
        self.parameter_search_entry.pack(side="left", fill="x", expand=True)
        ttk.Label(filters, textvariable=self.parameter_count_var, style="Muted.TLabel").pack(side="right", padx=10)
        self.parameter_canvas, body = console_ui.scroll_surface(parent)
        self.parameter_sections = []
        self._parameter_rows = {}
        self._parameter_filter_job = None
        for group in GROUPS:
            frame = ttk.LabelFrame(body, text=group, padding=12)
            self.parameter_sections.append((group, frame, []))
        self.parameter_empty = ttk.Label(body, text="没有匹配的参数。试试器件名称、用途或参数宏名。",
                                         style="Muted.TLabel", padding=20)
        self.parameter_group_var.trace_add("write", lambda *_: self._filter_parameters())
        self.parameter_search_var.trace_add("write", lambda *_: self._queue_parameter_filter())
        self._filter_parameters()

        manual = ttk.LabelFrame(body, text="夹爪自由试动 · 独立于任务标定", padding=12)
        self.parameter_manual = manual
        manual.pack(fill="x", pady=(0, 12))
        ttk.Label(manual, text="手动脉宽").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(manual, textvariable=self.servo2_manual_var, from_=SERVO_TUNING_MIN_PULSE_US,
                    to=SERVO_TUNING_MAX_PULSE_US, increment=1, width=8).grid(row=0, column=1, padx=8)
        ttk.Label(manual, textvariable=self.servo2_manual_degree_var, width=16,
                  style="Muted.TLabel").grid(row=0, column=2)
        adjust = ttk.Frame(manual)
        adjust.grid(row=0, column=3, padx=5)
        for delta in (-10, -1, 1, 10):
            ttk.Button(adjust, text=f"{delta:+}", width=4, style="Small.TButton",
                       command=lambda amount=delta: self._nudge_servo2_manual(amount)).pack(side="left", padx=2)
        button = ttk.Button(manual, text="试动夹爪", command=self._test_servo2_manual)
        button.grid(row=0, column=4, padx=(8, 0))
        self.action_buttons.append(button)
        ttk.Label(manual, text=f"范围 {SERVO_TUNING_MIN_PULSE_US}–{SERVO_TUNING_MAX_PULSE_US} μs；"
                  "初值来自当前夹紧参数，请小步试动。", style="Muted.TLabel", wraplength=650).grid(
                      row=1, column=0, columnspan=5, sticky="w", pady=(10, 0))

    def _focus_parameter_search(self, _event=None) -> str:
        self.notebook.select(0)
        self.parameter_search_entry.focus_set()
        self.parameter_search_entry.selection_range(0, "end")
        return "break"

    def _filter_parameters(self) -> None:
        if self._parameter_filter_job is not None:
            self.after_cancel(self._parameter_filter_job)
            self._parameter_filter_job = None
        query = self.parameter_search_var.get().strip().lower()
        selected = self.parameter_group_var.get()
        if hasattr(self, "parameter_manual"):
            self.parameter_manual.pack_forget()
        count = 0
        for group, frame, rows in self.parameter_sections:
            frame.pack_forget()
            for spec in PARAMETERS:
                matches_query = query and query in f"{group} {spec.label} {spec.define} {spec.description}".lower()
                if spec.group == group and spec.define not in self._parameter_rows and (
                    matches_query or (not query and selected in ("全部器件", group))):
                    row = self._parameter_row(frame, len(rows), spec)
                    self._parameter_rows[spec.define] = row
                    rows.append((spec, row))
            rows.sort(key=lambda pair: PARAMETERS.index(pair[0]))
            matches = 0
            for spec, row in rows:
                row.pack_forget()
                visible = (bool(query) or selected == "全部器件" or selected == group) and (
                    not query or query in f"{group} {spec.label} {spec.define} {spec.description}".lower())
                if visible:
                    row.pack(fill="x", pady=4)
                    matches += 1
                else:
                    row.pack_forget()
            if matches:
                frame.pack(fill="x", pady=(0, 14))
            else:
                frame.pack_forget()
            count += matches
        self.parameter_count_var.set(f"{count} 项")
        if count:
            self.parameter_empty.pack_forget()
        else:
            self.parameter_empty.pack(fill="x")
        if hasattr(self, "parameter_manual"):
            self.parameter_manual.pack(fill="x", pady=(0, 12))
        self.parameter_canvas.yview_moveto(0)
        self._update_degree_labels()
        console_ui.finish_widgets(self, self.parameter_canvas)

    def _queue_parameter_filter(self) -> None:
        if self._parameter_filter_job is not None:
            self.after_cancel(self._parameter_filter_job)
        self._parameter_filter_job = self.after(150, self._filter_parameters)

    def _parameter_row(self, parent: ttk.LabelFrame, row: int, spec: ParameterSpec) -> ttk.Frame:
        frame = ttk.Frame(parent, padding=(0, 5))
        frame.pack(fill="x", pady=4)
        frame.columnconfigure(0, weight=1)
        variable = self.value_vars[spec.define]
        ttk.Label(frame, text=spec.label, style="Status.TLabel", wraplength=200).grid(
            row=0, column=0, sticky="w", padx=(0, 12))
        ttk.Spinbox(frame, textvariable=variable, from_=spec.minimum, to=spec.maximum,
                    increment=spec.step, width=8).grid(row=0, column=1, padx=4)
        if spec.actuator and spec.actuator.startswith("servo"):
            degree_var = tk.StringVar(value="")
            self.degree_vars[spec.define] = degree_var
            ttk.Label(frame, textvariable=degree_var, width=16, style="Muted.TLabel").grid(row=0, column=2, padx=7)
        else:
            ttk.Label(frame, text=spec.unit, width=16, style="Muted.TLabel").grid(row=0, column=2, padx=7)
        adjust = ttk.Frame(frame)
        adjust.grid(row=0, column=3, padx=4)
        for sign, amount in (("−", -spec.step), ("+", spec.step)):
            ttk.Button(adjust, text=f"{sign}{spec.step}", width=4, style="Small.TButton",
                       command=lambda s=spec, d=amount: self._nudge(s, d)).pack(side="left", padx=2)
        if spec.actuator is not None:
            button = ttk.Button(frame, text="试动此值", command=lambda s=spec: self._test_parameter(s),
                                width=9, style="Small.TButton")
            button.grid(row=0, column=4, padx=(8, 0))
            self.action_buttons.append(button)
        else:
            scope = "本任务专用" if spec.group == "任务时序" else "跨动作共享" if spec.unit == "ms" else "保存后生效"
            ttk.Label(frame, text=scope, width=11, style="Muted.TLabel").grid(row=0, column=4, padx=(8, 0))
        description = ttk.Label(frame, text=spec.description, style="Muted.TLabel", wraplength=600,
                                 font=("Microsoft YaHei UI", 9))
        description.grid(row=1, column=0, columnspan=5, sticky="w", pady=(6, 0))
        ttk.Separator(frame).grid(row=2, column=0, columnspan=5, sticky="ew", pady=(10, 0))
        return frame

    def _build_timing_tab(self, parent: ttk.Frame) -> None:
        selector = ttk.Frame(parent, padding=(0, 0, 0, 12))
        selector.pack(fill="x")
        ttk.Label(selector, text="查看时间用途", style="Status.TLabel").pack(side="left", padx=(0, 12))
        choices = [time_catalog.OVERVIEW] + [item[0] for item in TIMING_OPERATIONS] + [time_catalog.SYSTEM, time_catalog.LEGACY]
        self.timing_selection_var = tk.StringVar(value=time_catalog.OVERVIEW)
        choice = ttk.Combobox(selector, textvariable=self.timing_selection_var, state="readonly", values=choices, width=33)
        choice.pack(side="left", fill="x", expand=True)
        choice.bind("<<ComboboxSelected>>", lambda _e: self._show_timing_operation(self.timing_selection_var.get()))
        self.timing_op_buttons = []
        self.timing_canvas, self.timing_detail = console_ui.scroll_surface(parent)
        self._show_timing_operation(time_catalog.OVERVIEW)

    def _show_timing_operation(self, operation_name: str) -> None:
        if not hasattr(self, "timing_detail"):
            self._ensure_console_page(1)
        self.timing_view_name = operation_name
        self.timing_selection_var.set(operation_name)
        for child in self.timing_detail.winfo_children():
            child.destroy()
        self.timing_canvas.yview_moveto(0)
        self.task_relationship_prefix = ""
        if operation_name == time_catalog.OVERVIEW:
            self._show_common_waits()
        elif operation_name in (time_catalog.SYSTEM, time_catalog.LEGACY):
            self._show_readonly_times(operation_name)
        else:
            label, prefix, mode_desc, specs = next(item for item in TIMING_OPERATIONS if item[0] == operation_name)
            self.task_relationship_prefix = prefix
            self._update_task_relationship()
            relationship = ttk.LabelFrame(self.timing_detail, text=f"{label} · {mode_desc} · 真实动作顺序", padding=12)
            relationship.pack(fill="x", pady=(0, 12))
            ttk.Label(relationship, textvariable=self.task_relationship_var, wraplength=600).pack(anchor="w")
            ttk.Label(relationship, textvariable=self.task_wait_summary_var, style="Status.TLabel", wraplength=600).pack(
                anchor="w", pady=(10, 0))
            dedicated = ttk.LabelFrame(self.timing_detail, text="本任务专用等待 · ms · 按执行顺序", padding=12)
            dedicated.pack(fill="x", pady=(0, 12))
            for row, spec in enumerate(specs):
                self._parameter_row(dedicated, row, spec)
            common = ttk.LabelFrame(self.timing_detail, text="本任务还使用的公共等待 · 修改会影响其他动作", padding=12)
            common.pack(fill="x", pady=(0, 12))
            for row, (define, count, context) in enumerate(time_catalog.shared_waits(prefix)):
                spec = PARAMETER_BY_DEFINE[define]
                ttk.Label(common, text=f"本任务使用 {count} 次：{context}", style="Warn.TLabel", wraplength=600).pack(anchor="w")
                self._parameter_row(common, row, spec)
        ttk.Label(self.timing_detail, text="输入值是待保存配置；保存、编译并烧录后才影响板上完整任务。等待时间不代表舵机已机械到位。",
                  style="Muted.TLabel", wraplength=600).pack(anchor="w", pady=(5, 10))
        console_ui.finish_widgets(self, self.timing_detail)

    def _show_common_waits(self) -> None:
        self._update_task_relationship()
        ttk.Label(self.timing_detail, text="任务专用等待不会自动继承下面的值。请先看具体mode，再调整它实际使用的字段。",
                  style="Warn.TLabel", wraplength=600).pack(anchor="w", pady=(0, 12))
        commands = ttk.LabelFrame(self.timing_detail, text="独立命令与初始化 · 当前固定等待", padding=12)
        commands.pack(fill="x", pady=(0, 12))
        ttk.Label(commands, textvariable=self.command_wait_summary_var, wraplength=600).pack(anchor="w")
        for group in ("公共舵机等待", "电机换向等待"):
            frame = ttk.LabelFrame(self.timing_detail, text=group + " · ms", padding=12)
            frame.pack(fill="x", pady=(0, 12))
            for row, spec in enumerate(spec for spec in PARAMETERS if spec.group == group):
                self._parameter_row(frame, row, spec)

    def _show_readonly_times(self, section: str) -> None:
        rows = []
        if section == time_catalog.LEGACY:
            ttk.Label(self.timing_detail, text="以下三项在当前正式入口只定义、不参与执行，已从可编辑参数中移除。修改它们不会改变任何任务。",
                      style="Warn.TLabel", wraplength=600).pack(anchor="w", pady=(0, 12))
            for define in time_catalog.LEGACY_TIME_DEFINES:
                rows.append((define, str(self.time_source_constants.get(define, "—")) + " ms", "历史默认参考；实际用每个任务的专用字段。"))
        else:
            ttk.Label(self.timing_detail, text="这些是通信、诊断与软件刷新时间，只读参考；不是舵机到位等待，也不是电机速度参数。",
                      style="Muted.TLabel", wraplength=600).pack(anchor="w", pady=(0, 12))
            for define, (label, unit, effect) in time_catalog.SYSTEM_INFO.items():
                rows.append((label, str(self.time_source_constants.get(define, "—")) + " " + unit, effect))
            rows.extend(time_catalog.GUI_TIME_INFO)
            for define, formula, effect in time_catalog.DERIVED_INFO:
                rows.append((define, formula, effect))
            rows.extend((
                ("元器件测试中停止指令轮询", "每段最多100 μs", "HardwareTest_Delay分片轮询串口；仅新增元器件测试使用。"),
                ("陀螺仪完整诊断名义采样时间", "约24 s＋初始化／通信开销", "mode126：四路各2秒原始电平探测，随后四路×八种波特率×500ms有效帧接收窗口。"),
                ("舵机PWM周期", "20 ms / 50 Hz", "底层TIM5固定周期；不是每次动作后等待。位置脉宽500–2500μs在位置标定页。"),
                ("微秒延时表示范围", "0–233015 μs", "来自SYSTEM/delay；STEP半周期界面要求正整数，此前取消的速度软限制继续保持。"),
                ("旧motor.c定时器初始化", "正式入口未调用", "TIM2/3/4的旧ARR/PSC/CCR不控制当前正式入口的GPIO步进速度。"),
                ("旧TIM6计数器", "正式入口未调用", "HARDWARE/COUNT的计数初始化不是当前任务等待。"),
            ))
        for label, value, effect in rows:
            frame = ttk.Frame(self.timing_detail, padding=(10, 8))
            frame.pack(fill="x", pady=3)
            ttk.Label(frame, text=label, style="Status.TLabel", wraplength=270).pack(anchor="w")
            ttk.Label(frame, text=value, foreground="#2463EB", font=("Consolas", 11)).pack(anchor="w", pady=3)
            ttk.Label(frame, text=effect, style="Muted.TLabel", wraplength=600).pack(anchor="w")

    def _build_task_tab(self, parent: ttk.Frame) -> None:
        canvas, body = console_ui.scroll_surface(parent)

        warning = ttk.LabelFrame(body, text="操作前确认", padding=10)
        warning.pack(fill="x", pady=(0, 10))
        ttk.Label(
            warning,
            text="任务动作是阻塞执行；收到完成 ACK 前界面不会允许发送下一条。执行任务前必须先人工将升降机构放到最低点并确认软件高度同步。",
            style="Warn.TLabel",
            wraplength=650,
        ).pack(anchor="w")

        for title, actions in TASK_GROUPS:
            frame = ttk.LabelFrame(body, text=title, padding=10)
            frame.pack(fill="x", pady=6)
            for index, (label, mode) in enumerate(actions):
                button = ttk.Button(
                    frame,
                    text=f"{label}\nmode {mode}",
                    command=lambda m=mode, text=label: self._send_mode_action(m, text, needs_lift=True),
                    width=13,
                )
                button.grid(row=0, column=index, sticky="ew", padx=5, pady=3)
                frame.columnconfigure(index, weight=1)
                self.action_buttons.append(button)

        manual = ttk.LabelFrame(body, text="独立动作与初始化", padding=10)
        manual.pack(fill="x", pady=6)
        for index, (label, mode, needs_lift) in enumerate(MANUAL_ACTIONS):
            button = ttk.Button(
                manual,
                text=f"{label}\nmode {mode}",
                command=lambda m=mode, text=label, lift=needs_lift: self._send_mode_action(m, text, needs_lift=lift),
                width=13,
            )
            button.grid(row=index // 4, column=index % 4, sticky="ew", padx=5, pady=5)
            manual.columnconfigure(index % 4, weight=1)
            self.action_buttons.append(button)

    def _build_component_tab(self, parent: ttk.Frame) -> None:
        header = ttk.Frame(parent)
        header.pack(fill="x", pady=(0, 10))
        ttk.Button(header, text="检测测试固件", style="Accent.TButton",
                   command=lambda: self._component_read(TEST_MODE_PROBE, "检测测试固件")).pack(side="left")
        ttk.Button(header, text="停止元器件测试 / 关闭舵机信号", style="Danger.TButton",
                   command=self._stop_component_test).pack(side="right")
        ttk.Label(parent, textvariable=self.component_result_var, style="Status.TLabel",
                  wraplength=650).pack(anchor="w", pady=(0, 12))
        canvas, body = console_ui.scroll_surface(parent)
        ttk.Label(body, text="ACK 表示指令已处理；器件是否正常，需要观察运动或测量信号。车轮建议架空测试。",
                  wraplength=650, style="Muted.TLabel").pack(anchor="w", pady=(0, 14))

        chassis = ttk.LabelFrame(body, text="底盘试跑  /  整车移动", padding=14)
        chassis.pack(fill="x", pady=(0, 16))
        for row, (label, variable, limit, unit, button, callback) in enumerate((
            ("前后直线", self.test_chassis_distance_var, CHASSIS_LINEAR_TEST_MAX_CM, "cm", "直线试跑", self._test_chassis_linear),
            ("旋转角度", self.test_chassis_angle_var, CHASSIS_ROTATE_TEST_MAX_DEG, "°", "旋转试跑", self._test_chassis_rotate),
        )):
            ttk.Label(chassis, text=label, style="Status.TLabel", width=14).grid(row=row, column=0, sticky="w", pady=5)
            ttk.Spinbox(chassis, from_=-limit, to=limit, textvariable=variable, width=8).grid(row=row, column=1, padx=8)
            ttk.Label(chassis, text=f"{unit}   /   ±1–{limit}", style="Muted.TLabel", width=15).grid(row=row, column=2, sticky="w")
            ttk.Button(chassis, text=button, command=callback, width=12).grid(row=row, column=3, padx=(16, 0), sticky="e")
        chassis.columnconfigure(3, weight=1)
        ttk.Label(chassis, text="直线：正值前进、负值后退（mode 1）；旋转使用 mode 3，正负方向以底盘定义为准。",
                  wraplength=650, style="Muted.TLabel").grid(row=2, column=0, columnspan=4, sticky="w", pady=(10, 0))
        ttk.Label(chassis, text="这两项为阻塞命令，元器件停止按钮不能中断。请先选小范围并确认周围无障碍。",
                  wraplength=650, style="Warn.TLabel").grid(row=3, column=0, columnspan=4, sticky="w", pady=(5, 0))

        wheels = ttk.LabelFrame(body, text="车轮驱动  /  一次只转一个", padding=14)
        wheels.pack(fill="x", pady=(0, 16))
        settings = ttk.Frame(wheels)
        settings.pack(fill="x", pady=(0, 12))
        ttk.Label(settings, text="脉冲数").pack(side="left")
        ttk.Spinbox(settings, from_=1, to=3200, textvariable=self.test_pulses_var, width=8).pack(side="left", padx=(8, 20))
        ttk.Label(settings, text="半周期 μs").pack(side="left")
        ttk.Spinbox(settings, from_=1, to=65535, increment=100, textvariable=self.test_period_var, width=8).pack(side="left", padx=8)
        ttk.Label(settings, text="越大越慢", style="Muted.TLabel").pack(side="left")
        for label, motor, pins in TEST_WHEELS:
            line = ttk.Frame(wheels, padding=(0, 6))
            line.pack(fill="x")
            identity = ttk.Frame(line)
            identity.pack(side="left")
            ttk.Label(identity, text=label, style="Status.TLabel").pack(anchor="w")
            ttk.Label(identity, text=pins, style="Muted.TLabel", font=("Microsoft YaHei UI", 9)).pack(anchor="w", pady=(3, 0))
            actions = ttk.Frame(line)
            actions.pack(side="right")
            for direction, text in ((0, "正转 / DIR 0"), (1, "反转 / DIR 1")):
                ttk.Button(actions, text=text, command=lambda m=motor, d=direction, name=label:
                           self._test_component_wheel(m, d, name)).pack(side="left", padx=(7, 0))
        ttk.Label(wheels, text="1–3200 脉冲 · 半周期 1–65535 μs；DIR 电平对应的实际转向取决于接线。",
                  wraplength=650, style="Muted.TLabel").pack(anchor="w", pady=(8, 0))

        lift = ttk.LabelFrame(body, text="升降电机  /  STEP PB10 · DIR PA4", padding=14)
        lift.pack(fill="x", pady=(0, 16))
        ttk.Label(lift, text="目标高度 mm").grid(row=0, column=0)
        ttk.Spinbox(lift, from_=0, to=LIFT_TARGET_MAX_MM, textvariable=self.test_lift_target_var, width=8).grid(row=0, column=1, padx=8)
        ttk.Label(lift, text="半周期 μs").grid(row=0, column=2, padx=(12, 0))
        ttk.Spinbox(lift, from_=1, to=65535, increment=100, textvariable=self.test_lift_period_var, width=8).grid(row=0, column=3, padx=8)
        ttk.Button(lift, text="只测试升降", command=self._test_component_lift).grid(row=0, column=4, sticky="e")
        lift.columnconfigure(4, weight=1)
        ttk.Label(lift, text=f"0–{LIFT_TARGET_MAX_MM} mm。先人工确认底部并复位，使软件高度同步为 0。此处只驱动升降，"
                  "中途停止后，先完成一次整数高度测试再恢复任务。",
                  wraplength=650, style="Muted.TLabel").grid(row=1, column=0, columnspan=5, sticky="w", pady=(12, 0))

        servos = ttk.LabelFrame(body, text="舵机 PWM  /  三路独立设置", padding=14)
        servos.pack(fill="x", pady=(0, 16))
        for row, (servo, label, pin, define) in enumerate((
            (1, "Servo1 伸缩", "PA1 / TIM5_CH2", "SERVO1_DISC_ALIGN_PULSE_US"),
            (2, "Servo2 夹爪", "PA2 / TIM5_CH3", "SERVO2_BOTTOM_OPEN_PULSE_US"),
            (3, "Servo3 转台", "PA3 / TIM5_CH4", "SERVO3_HOME_PULSE_US"),
        )):
            identity = ttk.Frame(servos)
            identity.grid(row=row, column=0, sticky="w", pady=7)
            ttk.Label(identity, text=label, style="Status.TLabel").pack(anchor="w")
            ttk.Label(identity, text=pin, style="Muted.TLabel", font=("Microsoft YaHei UI", 9)).pack(anchor="w")
            ttk.Spinbox(servos, from_=500, to=2500, increment=10, textvariable=self.test_servo_vars[servo], width=8).grid(row=row, column=1, padx=10)
            ttk.Label(servos, text="μs", style="Muted.TLabel").grid(row=row, column=2)
            ttk.Button(servos, text="发送脉宽", command=lambda s=servo: self._test_component_servo(s)).grid(row=row, column=3, padx=8)
            ttk.Button(servos, text="使用当前标定值", command=lambda s=servo, key=define:
                       self.test_servo_vars[s].set(self.value_vars[key].get())).grid(row=row, column=4, sticky="e")
        servos.columnconfigure(4, weight=1)
        ttk.Label(servos, text="500–2500 μs。发送后持续输出；停止会关闭三路 PWM，舵机可能失去保持力。"
                  "Servo4 与 WS2812 共用 PA11，此处不启用。", wraplength=650, style="Muted.TLabel").grid(
                      row=3, column=0, columnspan=5, sticky="w", pady=(10, 0))

        inputs = ttk.LabelFrame(body, text="输入与通信", padding=14)
        inputs.pack(fill="x", pady=(0, 14))
        ttk.Button(inputs, text="读取 PE6 按键电平", command=lambda: self._component_read(TEST_MODE_BUTTON, "读取 PE6")).pack(anchor="w")
        ttk.Label(inputs, text="按下与松开后分别读取；不会启用比赛启动或锁定舵机。", style="Muted.TLabel",
                  wraplength=650).pack(anchor="w", pady=(8, 0))
        ttk.Label(body, text="本页 TX / RX", style="Status.TLabel").pack(anchor="w", pady=(0, 8))
        self.component_log = tk.Text(body, height=7, state="disabled", wrap="word", font=("Consolas", 10),
                                     background="#152239", foreground="#DDE7F5", borderwidth=0, padx=12, pady=10)
        self.component_log.pack(fill="x")

    def _component_read(self, mode: int, label: str) -> None:
        if not self.connected or self.busy or self.build_busy:
            messagebox.showinfo("无法读取", "请先连接串口，并等待当前操作结束。")
            return
        if mode != TEST_MODE_PROBE and not self._component_ready():
            return
        if mode == TEST_MODE_PROBE:
            self.component_supported = False
        self.component_busy = True
        self._send_frame(make_frame(mode), label, timeout_s=3.0)

    def _component_ready(self) -> bool:
        if not self.component_supported:
            messagebox.showwarning("尚未检测测试固件", "先点击“检测测试固件”。新指令需要 v156 或支持相同协议的固件。")
            return False
        return True

    def _component_motion(self, frame: bytes, label: str, timeout_s: float, actuator: str | None = None) -> None:
        if not self._component_ready() or not self._safety_check(actuator, needs_lift=actuator == "lift"):
            return
        if not messagebox.askyesno("确认单个元器件测试", f"{label}\n{hex_bytes(frame)}\n\n确认执行？"):
            return
        self.component_busy = True
        self._send_frame(frame, label, timeout_s)

    def _test_component_wheel(self, motor: int, direction: int, label: str) -> None:
        try:
            pulses = int(self.test_pulses_var.get())
            period = int(self.test_period_var.get())
            frame = make_wheel_test_frame(motor, direction, pulses, period)
        except ValueError as exc:
            messagebox.showerror("测试参数无效", str(exc))
            return
        self._component_motion(frame, f"{label} DIR {direction}，{pulses} 脉冲，半周期 {period} μs",
                               timeout_s=pulses * period * 2 / 1_000_000 * 2 + 5)

    def _test_component_lift(self) -> None:
        try:
            target = int(self.test_lift_target_var.get())
            frame = make_lift_test_frame(target, int(self.test_lift_period_var.get()))
        except ValueError as exc:
            messagebox.showerror("测试参数无效", str(exc))
            return
        # The GUI does not know the firmware's current software height. Use the
        # Full 0–360 mm travel is the timeout ceiling so a downward full-range
        # test cannot expire early.
        timeout_s = max(20.0, LIFT_TARGET_MAX_MM * 64 * int(self.test_lift_period_var.get()) * 2 / 1_000_000 * 1.5 + 20.0)
        self._component_motion(frame, f"升降单独测试 → {target} mm（目标范围 0–{LIFT_TARGET_MAX_MM} mm）", timeout_s, "lift")

    def _test_component_servo(self, servo: int) -> None:
        try:
            pulse = int(self.test_servo_vars[servo].get())
            frame = make_servo_test_frame(servo, pulse)
        except ValueError as exc:
            messagebox.showerror("测试参数无效", str(exc))
            return
        self._component_motion(frame, f"Servo{servo} 只设置脉宽 {pulse} μs（ACK 不代表机械到位）",
                               3.0, f"servo{servo}")

    def _test_chassis_motion(self, frame: bytes, label: str) -> None:
        if not self._safety_check(None, needs_lift=False):
            return
        if not messagebox.askyesno("确认底盘试跑", f"{label}\n{hex_bytes(frame)}\n\n确认底盘周围无障碍并执行？"):
            return
        self._send_frame(frame, label, timeout_s=30.0)

    def _test_chassis_linear(self) -> None:
        try:
            distance_cm = int(self.test_chassis_distance_var.get().strip())
            frame = make_chassis_linear_test_frame(distance_cm)
        except ValueError as exc:
            messagebox.showerror("底盘试跑参数无效", str(exc))
            return
        self._test_chassis_motion(frame, f"底盘直线试跑 {distance_cm} cm（mode {CHASSIS_MODE_LINEAR_DISTANCE}）")

    def _test_chassis_rotate(self) -> None:
        try:
            angle_deg = int(self.test_chassis_angle_var.get().strip())
            frame = make_chassis_rotate_test_frame(angle_deg)
        except ValueError as exc:
            messagebox.showerror("旋转试跑参数无效", str(exc))
            return
        self._test_chassis_motion(frame, f"底盘旋转试跑 {angle_deg}°（mode {CHASSIS_MODE_ROTATE}）")

    def _stop_component_test(self) -> None:
        if not self.connected or self.worker is None or self.build_busy:
            messagebox.showinfo("无法停止", "请先连接串口，等待构建结束。")
            return
        if not self._component_ready():
            return
        if self.busy and not self.component_busy:
            messagebox.showwarning("正在执行其他动作", "此停止指令仅能中断元器件测试。当前任务动作需等待完成。")
            return
        self.busy = True
        self.component_busy = True
        self.status_var.set("正在停止元器件测试")
        self.worker.request("hardware_stop", (make_frame(TEST_MODE_STOP), "停止元器件测试 / 关闭舵机信号", 3.0))

    def _build_build_tab(self, parent: ttk.Frame) -> None:
        _canvas, parent = console_ui.scroll_surface(parent)
        source_frame = ttk.LabelFrame(parent, text="01 / 参数写回", padding=14)
        source_frame.pack(fill="x", pady=(0, 10))
        ttk.Label(source_frame, text=f"目标源码：{self.source_path}", wraplength=650).pack(anchor="w")
        ttk.Label(
            source_frame,
            text="写回时只替换所列 #define 的十进制数字，并检查文件哈希；不会整体解码或转码，也不会覆盖加载后产生的外部修改。",
            style="Safe.TLabel",
            wraplength=650,
        ).pack(anchor="w", pady=(5, 10))
        ttk.Button(source_frame, text="预览修改", command=self._preview_changes).pack(side="left")
        ttk.Button(source_frame, text="保存参数到当前 .c", command=self._save_source, style="Accent.TButton").pack(side="left", padx=8)

        build_frame = ttk.LabelFrame(parent, text="02 / 编译与固件", padding=14)
        build_frame.pack(fill="x", pady=10)
        ttk.Label(build_frame, text=f"Keil 工程：{PROJECT_PATH}").grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Label(build_frame, text="新 BIN 名称").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(build_frame, textvariable=self.bin_name_var, width=28).grid(row=1, column=1, sticky="ew", padx=8, pady=(8, 0))
        ttk.Label(build_frame, text="留空时自动生成").grid(row=1, column=2, sticky="w", pady=(8, 0))
        build_frame.columnconfigure(1, weight=1)
        ttk.Button(build_frame, text="仅编译并检查日志", command=lambda: self._start_build(False, False)).grid(row=2, column=0, padx=4, pady=12, sticky="ew")
        ttk.Button(build_frame, text="编译并导出 BIN", command=lambda: self._start_build(True, False)).grid(row=2, column=1, padx=4, pady=12, sticky="ew")
        ttk.Button(build_frame, text=f"编译、导出并烧录 {SERIAL_PORT}", command=lambda: self._start_build(True, True), style="Danger.TButton").grid(row=2, column=2, padx=4, pady=12, sticky="ew")

        flash_frame = ttk.LabelFrame(parent, text="03 / 烧录前核对", padding=14)
        flash_frame.pack(fill="x", pady=10)
        ttk.Label(
            flash_frame,
            text=f"只允许 {SERIAL_PORT}；{SERIAL_PORT} 不存在立即停止。烧录前必须断开界面串口。只有日志明确出现 Verification OK，界面才会报告烧录成功。烧录后单片机会复位，请确认机械范围安全。",
            style="Warn.TLabel",
            wraplength=650,
        ).pack(anchor="w")

    def _build_log_tab(self, parent: ttk.Frame) -> None:
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill="x", pady=(0, 6))
        ttk.Label(toolbar, text="不会自动重发超时命令，避免机械动作重复执行。", style="Safe.TLabel").pack(side="left")
        ttk.Button(toolbar, text="清空日志", command=self._clear_log).pack(side="right")
        self.log = tk.Text(
            parent,
            state="disabled",
            wrap="word",
            font=("Cascadia Mono", 9),
            background="#101820",
            foreground="#e8eef2",
            padx=8,
            pady=8,
        )
        scroll = ttk.Scrollbar(parent, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.log.pack(fill="both", expand=True)
        self.log.tag_configure("tx", foreground="#69c0ff")
        self.log.tag_configure("rx", foreground="#7ee787")
        self.log.tag_configure("warn", foreground="#ffcc66")
        self.log.tag_configure("error", foreground="#ff7b72")
        self.log.tag_configure("info", foreground="#b9c4cc")
        self._append_log("调参台已启动；尚未发送任何机械动作。", "info")

    def _refresh_ports(self) -> None:
        ports = [] if list_ports is None else [item.device.upper() for item in list_ports.comports()]
        if SERIAL_PORT in ports:
            self.port_scan_var.set(f"已检测到 {SERIAL_PORT}")
        else:
            self.port_scan_var.set(f"未检测到 {SERIAL_PORT}，请重新插拔 USB")
        self._append_log("串口扫描：" + (", ".join(ports) if ports else "未发现串口"), "info")

    def _toggle_connection(self) -> None:
        if self.worker is None:
            messagebox.showerror("缺少组件", "未安装 pyserial。")
            return
        if self.connected:
            if self.component_busy and self.component_supported:
                self.worker.request("hardware_stop", (make_frame(TEST_MODE_STOP), "断开前停止测试", 3.0))
            self.worker.request("disconnect")
            return
        ports = [] if list_ports is None else [item.device.upper() for item in list_ports.comports()]
        if SERIAL_PORT not in ports:
            messagebox.showwarning(f"{SERIAL_PORT} 不存在", "请重新插拔 USB；不会改用其他串口。")
            return
        self.connection_var.set("正在连接…")
        self.connect_button.configure(state="disabled")
        self.worker.request("connect")

    def _load_source(self, show_message: bool = True) -> None:
        try:
            values, source_hash = parse_source_parameters(self.source_path)
        except Exception as exc:
            messagebox.showerror("读取源码失败", str(exc))
            return
        self.loaded_values = values
        self.loaded_hash = source_hash
        self.time_source_constants = time_catalog.source_constants(self.source_path.read_text(encoding="utf-8"))
        self._loading_parameters = True
        try:
            for define, value in values.items():
                self.value_vars[define].set(str(value))
        finally:
            self._loading_parameters = False
        self.servo2_manual_var.set(str(values["SERVO2_CLAMP_PULSE_US"]))
        self._update_degree_labels()
        self._update_dirty_state()
        self._update_task_relationship()
        try:
            version = read_firmware_version(self.source_path)
            self.bin_name_var.set(f"chassis_hardware_tuning_v{version}.bin")
            self.title(f"{APP_TITLE} · 源码 v{version} · {len(values)} 项参数 · 时间参数梳理版")
        except Exception:
            pass
        self._append_log(f"已读取源码参数，SHA-256 {source_hash[:12]}…", "info")
        if getattr(self, "timing_view_name", None) in (time_catalog.SYSTEM, time_catalog.LEGACY):
            self._show_timing_operation(self.timing_view_name)
        if show_message:
            messagebox.showinfo("读取完成", "已从当前 .c 重新载入全部调参参数。")

    def _validated_values(self) -> dict[str, int] | None:
        values: dict[str, int] = {}
        errors: list[str] = []
        for spec in PARAMETERS:
            text = self.value_vars[spec.define].get().strip()
            try:
                value = int(text, 10)
            except ValueError:
                errors.append(f"{spec.label}：必须是整数")
                continue
            if not spec.minimum <= value <= spec.maximum:
                errors.append(f"{spec.label}：允许范围 {spec.minimum}–{spec.maximum} {spec.unit}")
                continue
            values[spec.define] = value
        for higher, lower, label in (
            ("LIFT_TASK2_TRANSFER_HEIGHT_MM", "LIFT_TASK2_DISC_PLACE_HEIGHT_MM", "任务2转台旋转安全高度必须高于放入舵盘高度"),
            ("LIFT_TASK2_TRANSFER_HEIGHT_MM", "LIFT_TASK2_DISC_HEIGHT_MM", "任务2转台旋转安全高度必须高于从舵盘取杯高度"),
            ("LIFT_TASK2_RETURN_CLEARANCE_MM", "LIFT_TASK2_DISC_HEIGHT_MM", "任务2取杯后前伸/回正高度必须高于从舵盘取杯高度"),
        ):
            if higher in values and lower in values and values[higher] <= values[lower]:
                errors.append(label)
        for define in TASK2_PODIUM_HEIGHT_PARAMETER_DEFINES:
            if "LIFT_TASK2_RETURN_CLEARANCE_MM" in values and define in values and values[define] > values["LIFT_TASK2_RETURN_CLEARANCE_MM"]:
                errors.append("任务2取杯后前伸/回正高度不得低于领奖台放置目标")
        if errors:
            messagebox.showerror("参数错误", "\n".join(errors[:12]))
            return None
        return values

    def _on_parameter_changed(self) -> None:
        if self._loading_parameters:
            return
        self._update_degree_labels()
        self._update_dirty_state()
        self._update_task_relationship()

    def _update_task_relationship(self) -> None:
        values = {name: var.get() for name, var in self.value_vars.items()}
        self.task_relationship_var.set(task_relationship_text(self.task_relationship_prefix, values))
        self.command_wait_summary_var.set(time_catalog.command_summary(values))
        if self.task_relationship_prefix:
            self.task_wait_summary_var.set(time_catalog.wait_summary(self.task_relationship_prefix, values))

    def _update_degree_labels(self) -> None:
        for spec in PARAMETERS:
            if spec.define not in self.degree_vars or spec.actuator is None:
                continue
            try:
                pulse = int(self.value_vars[spec.define].get().strip())
                degrees = approximate_degrees(spec.actuator, pulse)
                self.degree_vars[spec.define].set(f"{spec.unit}  ≈ {degrees:.1f}°")
            except ValueError:
                self.degree_vars[spec.define].set("数值无效")

    def _update_dirty_state(self) -> None:
        if not self.loaded_values:
            return
        dirty = False
        for spec in PARAMETERS:
            try:
                value = int(self.value_vars[spec.define].get().strip())
            except ValueError:
                dirty = True
                break
            if value != self.loaded_values.get(spec.define):
                dirty = True
                break
        self.dirty_var.set("● 有尚未保存的参数" if dirty else "参数与源码一致")

    def _nudge(self, spec: ParameterSpec, delta: int) -> None:
        try:
            value = int(self.value_vars[spec.define].get().strip())
        except ValueError:
            value = self.loaded_values.get(spec.define, spec.minimum)
        value = max(spec.minimum, min(spec.maximum, value + delta))
        self.value_vars[spec.define].set(str(value))

    def _update_servo2_manual_degree(self) -> None:
        try:
            pulse = int(self.servo2_manual_var.get().strip())
        except ValueError:
            self.servo2_manual_degree_var.set("数值无效")
            return
        degrees = approximate_degrees("servo2", pulse)
        self.servo2_manual_degree_var.set(f"us  ≈ {degrees:.1f}°")

    def _nudge_servo2_manual(self, delta: int) -> None:
        try:
            value = int(self.servo2_manual_var.get().strip())
        except ValueError:
            value = self.loaded_values.get("SERVO2_CLAMP_PULSE_US", SERVO_TUNING_MIN_PULSE_US)
        value = max(SERVO_TUNING_MIN_PULSE_US, min(SERVO_TUNING_MAX_PULSE_US, value + delta))
        self.servo2_manual_var.set(str(value))

    def _test_servo2_manual(self) -> None:
        if not self._safety_check("servo2", needs_lift=False):
            return
        try:
            value = int(self.servo2_manual_var.get().strip())
        except ValueError:
            messagebox.showerror("参数错误", "夹爪手动脉宽必须是整数。")
            return
        if not SERVO_TUNING_MIN_PULSE_US <= value <= SERVO_TUNING_MAX_PULSE_US:
            messagebox.showerror(
                "参数错误",
                f"夹爪手动试动范围：{SERVO_TUNING_MIN_PULSE_US}–{SERVO_TUNING_MAX_PULSE_US} us",
            )
            return
        frame = make_frame(TUNING_MODE_SERVO2, value)
        detail = (
            f"Servo2 夹爪自由试动 → {value} us"
            f"（约 {approximate_degrees('servo2', value):.1f}°）\n"
            f"mode {TUNING_MODE_SERVO2}\n{hex_bytes(frame)}"
        )
        if not messagebox.askyesno("确认夹爪试动", detail + "\n\n此数值不会写入正式任务参数。确认执行？"):
            return
        self._send_frame(frame, f"Servo2 自由试动={value}us", timeout_s=8.0)

    def _changes(self, values: dict[str, int]) -> list[tuple[ParameterSpec, int, int]]:
        changes = []
        for spec in PARAMETERS:
            old = self.loaded_values.get(spec.define)
            new = values[spec.define]
            if old is not None and old != new:
                changes.append((spec, old, new))
        return changes

    def _preview_changes(self) -> None:
        values = self._validated_values()
        if values is None:
            return
        changes = self._changes(values)
        if not changes:
            messagebox.showinfo("无修改", "当前参数与已读取的源码一致。")
            return
        window = tk.Toplevel(self)
        window.title("参数修改预览")
        window.geometry("760x520")
        text = tk.Text(window, wrap="word", font=("Cascadia Mono", 10), padx=10, pady=10)
        text.pack(fill="both", expand=True)
        for spec, old, new in changes:
            text.insert("end", f"{spec.label}\n  {spec.define}: {old} -> {new} {spec.unit}\n\n")
        text.configure(state="disabled")

    def _save_source(self) -> bool:
        values = self._validated_values()
        if values is None:
            return False
        changes = self._changes(values)
        if not changes:
            messagebox.showinfo("无需保存", "参数已经与源码一致。")
            return True
        summary = "\n".join(f"{spec.label}: {old} → {new} {spec.unit}" for spec, old, new in changes[:12])
        if len(changes) > 12:
            summary += f"\n……另有 {len(changes) - 12} 项"
        if not messagebox.askyesno(
            "确认写入当前 .c",
            f"将只修改以下参数的数字：\n\n{summary}\n\n是否继续？",
        ):
            return False
        try:
            self.loaded_hash = replace_source_parameters(
                self.source_path, self.loaded_hash, values
            )
        except Exception as exc:
            messagebox.showerror("保存失败", str(exc))
            return False
        self.loaded_values = dict(values)
        self._update_dirty_state()
        self._append_log(f"已保存 {len(changes)} 项参数到 {self.source_path}", "info")
        messagebox.showinfo("保存完成", "参数已精确写回当前 .c；尚未编译或烧录。")
        return True

    def _export_profile(self) -> None:
        values = self._validated_values()
        if values is None:
            return
        path = filedialog.asksaveasfilename(
            title="导出调参方案",
            defaultextension=".json",
            filetypes=(("JSON 参数方案", "*.json"),),
            initialfile=f"hardware_tuning_{datetime.now():%Y%m%d_%H%M%S}.json",
        )
        if not path:
            return
        payload = {
            "format": 1,
            "source": str(self.source_path),
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "parameters": values,
        }
        Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        self._append_log(f"已导出参数方案：{path}", "info")

    def _import_profile(self) -> None:
        path = filedialog.askopenfilename(
            title="导入调参方案", filetypes=(("JSON 参数方案", "*.json"),)
        )
        if not path:
            return
        try:
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
            values = payload["parameters"]
            # Older profiles predate the separate disc reach; retain the
            # current value rather than rejecting the existing calibration.
            if "SERVO1_DISC_ALIGN_PULSE_US" not in values:
                values["SERVO1_DISC_ALIGN_PULSE_US"] = int(self.value_vars["SERVO1_DISC_ALIGN_PULSE_US"].get())
            for legacy, current in LEGACY_TASK2_PODIUM_PROFILE_FIELDS.items():
                if current not in values and legacy in values:
                    values[current] = values[legacy]
            for define in (CUBE_SERVO2_PARAMETER_DEFINES + CUBE_STACK_HEIGHT_PARAMETER_DEFINES +
                           TASK2_NEW_HEIGHT_PARAMETER_DEFINES + TASK2_PODIUM_HEIGHT_PARAMETER_DEFINES +
                           CAMERA_TARGET_HEIGHT_PARAMETER_DEFINES + time_catalog.NEW_TIME_DEFINES +
                           time_catalog.PROFILE_FINE_DEFINES):
                if define not in values:
                    values[define] = int(self.value_vars[define].get())
            missing = [spec.define for spec in PARAMETERS if spec.define not in values]
            if missing:
                raise ValueError("方案缺少参数：" + ", ".join(missing[:5]))
            for spec in PARAMETERS:
                self.value_vars[spec.define].set(str(int(values[spec.define])))
            if self._validated_values() is None:
                raise ValueError("方案中存在超出安全范围的数值")
        except Exception as exc:
            messagebox.showerror("导入失败", str(exc))
            return
        ignored = [key for key in time_catalog.LEGACY_TIME_DEFINES if key in values]
        if ignored:
            self._append_log("已忽略不参与正式任务的历史等待字段：" + ", ".join(ignored), "info")
        self._append_log(f"已载入参数方案：{path}；尚未写入源码或发送动作。", "info")

    def _safety_check(self, actuator: str | None, needs_lift: bool) -> bool:
        if not self.connected:
            messagebox.showwarning("尚未连接", f"请先连接 {SERIAL_PORT}。")
            return False
        if self.busy or self.build_busy:
            messagebox.showinfo("操作进行中", "请等待上一项操作结束。")
            return False
        if not self.action_armed_var.get():
            messagebox.showwarning("机械动作未授权", "请先勾选“已清空运动范围”。")
            return False
        if needs_lift or actuator == "lift":
            if not self.lift_synced_var.get():
                messagebox.showwarning(
                    "升降高度未同步",
                    "请先人工把升降机构放到最低点，确认软件 0 mm 与实际位置同步后再勾选。",
                )
                return False
        if actuator == "servo3" and not self.turntable_safe_var.get():
            messagebox.showwarning(
                "转台区域未确认", "请先确认升降/机械臂处于安全高度且转台范围无干涉。"
            )
            return False
        return True

    def _test_parameter(self, spec: ParameterSpec) -> None:
        if not self._safety_check(spec.actuator, needs_lift=False):
            return
        try:
            value = int(self.value_vars[spec.define].get().strip())
        except ValueError:
            messagebox.showerror("参数错误", f"{spec.label} 必须是整数。")
            return
        if not spec.minimum <= value <= spec.maximum:
            messagebox.showerror("参数错误", f"允许范围：{spec.minimum}–{spec.maximum} {spec.unit}")
            return
        modes = {
            "servo1": TUNING_MODE_SERVO1,
            "servo2": TUNING_MODE_SERVO2,
            "servo3": TUNING_MODE_SERVO3,
            "lift": TUNING_MODE_LIFT,
        }
        mode = modes[spec.actuator]
        frame = make_frame(mode, value)
        detail = f"{spec.label} → {value} {spec.unit}\nmode {mode}\n{hex_bytes(frame)}"
        if not messagebox.askyesno("确认单项试动", detail + "\n\n确认机械范围安全并执行？"):
            return
        self._send_frame(frame, f"{spec.label}={value}{spec.unit}", timeout_s=8.0)

    def _send_mode_action(self, mode: int, label: str, needs_lift: bool) -> None:
        actuator = "servo3" if mode == 86 else None
        if not self._safety_check(actuator, needs_lift=needs_lift):
            return
        frame = make_frame(mode)
        if not messagebox.askyesno(
            "确认执行机械动作",
            f"{label}\nmode {mode}\n{hex_bytes(frame)}\n\n确认执行？",
        ):
            return
        self._send_frame(
            frame,
            f"{label} (mode {mode})",
            timeout_s=60.0,
        )

    def _send_frame(self, frame: bytes, label: str, timeout_s: float) -> None:
        if self.worker is None:
            return
        self.busy = True
        self.status_var.set(f"正在执行：{label}")
        self.worker.request("send", (frame, label, timeout_s))

    def _start_build(self, export_bin: bool, flash: bool) -> None:
        if self.build_busy or self.busy:
            messagebox.showinfo("操作进行中", "请等待上一项操作结束。")
            return
        values = self._validated_values()
        if values is None:
            return
        if self._changes(values):
            if not messagebox.askyesno("存在未保存参数", "先把当前界面参数保存到源码再继续构建？"):
                return
            if not self._save_source():
                return
        if flash:
            if self.connected:
                messagebox.showwarning("请先断开串口", f"烧录前请点击“断开”，释放 {SERIAL_PORT}。")
                return
            ports = [] if list_ports is None else [item.device.upper() for item in list_ports.comports()]
            if SERIAL_PORT not in ports:
                messagebox.showwarning(f"{SERIAL_PORT} 不存在", "请重新插拔 USB；不会改用其他串口。")
                return
            if not messagebox.askyesno(
                f"确认烧录 {SERIAL_PORT}",
                f"将编译当前工程、导出新 BIN，并擦除/写入 {SERIAL_PORT}。\n烧录后单片机会复位，请确认机械范围安全。\n\n确认继续？",
            ):
                return
        self.build_busy = True
        self.status_var.set("正在构建…")
        requested_bin_name = self.bin_name_var.get().strip()
        threading.Thread(
            target=self._build_worker,
            args=(export_bin, flash, requested_bin_name),
            daemon=True,
        ).start()

    def _run_process(self, command: list[str], label: str) -> tuple[int, str]:
        self.events.put({"event": "build_log", "message": f"{label}: {' '.join(command)}"})
        process = subprocess.run(
            command,
            cwd=DEFAULT_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="gbk",
            errors="replace",
            check=False,
        )
        output = process.stdout or ""
        if output.strip():
            self.events.put({"event": "build_log", "message": output.rstrip()})
        return process.returncode, output

    def _build_worker(
        self, export_bin: bool, flash: bool, requested_bin_name: str
    ) -> None:
        try:
            OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
            LISTING_DIR.mkdir(parents=True, exist_ok=True)
            if KEIL_PATH is None or not KEIL_PATH.exists():
                raise FileNotFoundError(
                    "找不到 Keil UV4。请安装 MDK-ARM，或设置 KEIL_ROOT/KEIL_UV4 环境变量。"
                )
            if not PROJECT_PATH.exists():
                raise FileNotFoundError(f"找不到 Keil 工程：{PROJECT_PATH}")
            code, _ = self._run_process(
                [str(KEIL_PATH), "-b", str(PROJECT_PATH), "-j0"], "Keil 编译"
            )
            if code != 0:
                raise RuntimeError(f"Keil 进程返回 {code}")
            log_text = BUILD_LOG_PATH.read_bytes().decode("gbk", errors="replace")
            if "0 Error(s), 0 Warning(s)" not in log_text:
                match = re.search(r"\d+ Error\(s\), \d+ Warning\(s\)", log_text)
                raise RuntimeError("构建日志未通过：" + (match.group(0) if match else "未找到结果行"))
            self.events.put({"event": "build_log", "message": "构建日志确认：0 Error(s), 0 Warning(s)"})

            bin_path: Path | None = None
            if export_bin:
                if FROMELF_PATH is None or not FROMELF_PATH.exists():
                    raise FileNotFoundError(
                        "找不到 fromelf。请设置 KEIL_ROOT 或 KEIL_FROMELF 环境变量。"
                    )
                if requested_bin_name:
                    safe_name = Path(requested_bin_name).name
                    if not safe_name.lower().endswith(".bin"):
                        safe_name += ".bin"
                else:
                    version = read_firmware_version(self.source_path)
                    safe_name = f"chassis_hardware_tuning_v{version}_{datetime.now():%Y%m%d_%H%M%S}.bin"
                bin_path = OUTPUT_DIR / safe_name
                if bin_path.exists():
                    bin_path = bin_path.with_name(
                        f"{bin_path.stem}_{datetime.now():%Y%m%d_%H%M%S}{bin_path.suffix}"
                    )
                code, _ = self._run_process(
                    [str(FROMELF_PATH), "--bin", "--output", str(bin_path), str(AXF_PATH)],
                    "导出 BIN",
                )
                if code != 0 or not bin_path.exists():
                    raise RuntimeError("BIN 导出失败")
                self.events.put({"event": "build_log", "message": f"已导出：{bin_path}"})

            if flash:
                if bin_path is None:
                    raise RuntimeError("内部错误：烧录前没有 BIN")
                if not PYTHON_PATH.exists():
                    raise FileNotFoundError(f"找不到当前 Python：{PYTHON_PATH}")
                ports = [] if list_ports is None else [item.device.upper() for item in list_ports.comports()]
                self.events.put({"event": "build_log", "message": "烧录前串口扫描：" + (", ".join(ports) if ports else "无")})
                if SERIAL_PORT not in ports:
                    raise RuntimeError(f"{SERIAL_PORT} 不存在，请重新插拔 USB；未尝试其他串口")
                loader_code = (
                    "import sys; "
                    "from stm32loader.main import main; main(*sys.argv[1:])"
                )
                code, output = self._run_process(
                    [
                        str(PYTHON_PATH),
                        "-c",
                        loader_code,
                        "-p",
                        SERIAL_PORT,
                        "-b",
                        str(SERIAL_BAUD),
                        "-f",
                        "F1",
                        "-e",
                        "-w",
                        "-v",
                        "-g",
                        "0x08000000",
                        "-R",
                        "-B",
                        "-V",
                        str(bin_path),
                    ],
                    f"烧录 {SERIAL_PORT}",
                )
                if code != 0 or "Verification OK" not in output:
                    raise RuntimeError("未看到 Verification OK，不能判定烧录成功")
                self.events.put({"event": "flash_verified", "bin_path": str(bin_path)})
            else:
                self.events.put({"event": "build_success", "bin_path": str(bin_path) if bin_path else ""})
        except Exception as exc:
            self.events.put({"event": "build_error", "message": str(exc)})
        finally:
            self.events.put({"event": "build_done"})

    def _reset_controller_session(self) -> None:
        self._controller_responded = False
        self.component_supported = False
        self.action_armed_var.set(False)
        self.lift_synced_var.set(False)
        self.turntable_safe_var.set(False)

    def _poll_events(self) -> None:
        try:
            while True:
                event = self.events.get_nowait()
                kind = event["event"]
                if kind == "connected":
                    self.connected = True
                    self._reset_controller_session()
                    self.component_result_var.set("已连接串口，请点击“检测测试固件”。")
                    self.connect_button.configure(text="断开", state="normal")
                    self.connection_var.set(f"已连接 {SERIAL_PORT}")
                    self.status_var.set("串口已连接；尚未发送动作")
                    self._append_log(f"已连接 {SERIAL_PORT} @ {SERIAL_BAUD}。", "info")
                elif kind == "disconnected":
                    self.connected = False
                    self.busy = False
                    self.component_busy = False
                    self._reset_controller_session()
                    self.component_result_var.set("串口已断开；重新连接后请检测测试固件。")
                    self.connect_button.configure(text="连接", state="normal")
                    self.connection_var.set("未连接")
                    self.status_var.set("串口已断开")
                    self._append_log(f"{SERIAL_PORT} 已断开。", "warn")
                elif kind == "tx":
                    self._append_log(f"TX [{event['label']}]: {hex_bytes(event['frame'])}", "tx")
                    if self.component_busy:
                        self.component_result_var.set(f"已发送：{event['label']}")
                elif kind == "rx":
                    status = int(event["status"])
                    elapsed = event["elapsed"]
                    suffix = "" if elapsed is None else f"，{float(elapsed) * 1000:.1f} ms"
                    name = STATUS_NAMES.get(status, f"状态 0x{status:02X}")
                    if status == 2:
                        # READY repeats until the first command. Only its return
                        # after a command reply establishes a controller restart.
                        if self._controller_responded:
                            self._reset_controller_session()
                            self.component_result_var.set("控制器已重新启动；请重新检测测试固件并确认运动范围。")
                    else:
                        self._controller_responded = True
                        if status == 7:
                            self.component_supported = True
                    if status != 2 and (self.component_busy or status in (4, 5, 6, 7)):
                        detail = name
                        if status == 0:
                            detail += "：检查测试固件、参数边界和比赛试动锁定；升降每次最多 10 mm。"
                        elif status == 1:
                            detail += "：控制信号已发送，请观察器件实际响应。"
                        self.component_result_var.set(detail + suffix)
                    self._append_log(f"RX: {hex_bytes(event['frame'])} ({name}{suffix})", "rx" if status in (1, 2, 3) else "warn")
                    self.status_var.set(name + suffix)
                elif kind == "noise":
                    data = event["data"]
                    if data:
                        self._append_log(f"RX 其他数据: {hex_bytes(data)}", "warn")
                elif kind == "timeout":
                    self._append_log(f"超时 [{event['label']}]：{float(event['elapsed']):.1f}s 未收到 ACK；没有自动重发。", "error")
                    self.status_var.set("等待 ACK 超时")
                    self.component_result_var.set("等待反馈超时：请检查通信和固件。测试停止按钮仍可使用。")
                elif kind == "operation_done":
                    self.busy = False
                    self.component_busy = False
                elif kind == "serial_error":
                    self.busy = False
                    self.component_busy = False
                    self.connect_button.configure(state="normal")
                    self.status_var.set("串口错误")
                    self._append_log(str(event["message"]), "error")
                elif kind == "build_log":
                    self._append_log(str(event["message"]), "info")
                elif kind == "build_success":
                    path = str(event["bin_path"])
                    self.status_var.set("构建与导出完成" if path else "编译检查通过")
                    messagebox.showinfo("构建完成", "0 Error(s), 0 Warning(s)" + (f"\nBIN：{path}" if path else ""))
                elif kind == "flash_verified":
                    self._reset_controller_session()
                    self.status_var.set("烧录验证成功")
                    messagebox.showinfo("烧录成功", f"Verification OK\nBIN：{event['bin_path']}")
                elif kind == "build_error":
                    self.status_var.set("构建/烧录失败")
                    self._append_log(str(event["message"]), "error")
                    messagebox.showerror("构建/烧录失败", str(event["message"]))
                elif kind == "build_done":
                    self.build_busy = False
        except queue.Empty:
            pass
        self.after(50, self._poll_events)

    def _append_log(self, text: str, tag: str) -> None:
        if not hasattr(self, "log"):
            return
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log.configure(state="normal")
        for line in str(text).splitlines() or [""]:
            self.log.insert("end", f"[{timestamp}] {line}\n", tag)
        self.log.see("end")
        self.log.configure(state="disabled")
        if hasattr(self, "component_log") and tag in ("tx", "rx", "warn", "error"):
            self.component_log.configure(state="normal")
            self.component_log.insert("end", f"[{timestamp}] {text}\n")
            self.component_log.see("end")
            self.component_log.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _on_close(self) -> None:
        if self.busy or self.build_busy:
            if not messagebox.askyesno("操作仍在进行", "当前操作尚未结束，仍要关闭界面吗？"):
                return
        if self.worker is not None:
            if self.component_busy and self.component_supported:
                self.worker.request("hardware_stop", (make_frame(TEST_MODE_STOP), "关闭界面前停止测试", 3.0))
            self.worker.request("stop")
        self.destroy()


def self_test(source_path: Path) -> int:
    values, source_hash = parse_source_parameters(source_path)
    assert len(values) == len(PARAMETERS)
    assert read_firmware_version(source_path) >= 130
    assert make_frame(TUNING_MODE_SERVO1, 946) == bytes(
        [0xFF, 80, 0x03, 0xB2, 0, 0, 0, 0, 0, 0xFE]
    )
    assert make_frame(TUNING_MODE_SERVO2, SERVO_TUNING_MAX_PULSE_US) == bytes(
        [0xFF, 81, 0x09, 0xC4, 0, 0, 0, 0, 0, 0xFE]
    )
    assert make_frame(72) == bytes([0xFF, 72, 0, 0, 0, 0, 0, 0, 0, 0xFE])
    assert make_chassis_linear_test_frame(5) == bytes.fromhex("FF 01 00 05 00 00 00 00 00 FE")
    assert make_chassis_linear_test_frame(-5) == bytes.fromhex("FF 01 FF FB 00 00 00 00 00 FE")
    assert make_chassis_rotate_test_frame(-10) == bytes.fromhex("FF 03 00 00 F6 00 00 00 00 FE")
    for spec in PARAMETERS:
        assert spec.minimum <= values[spec.define] <= spec.maximum, spec.define
    print(f"SELF-TEST OK: {len(values)} parameters, source SHA-256 {source_hash}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=APP_TITLE)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    if args.self_test:
        return self_test(args.source.resolve())
    app = HardwareTuningApp(args.source)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
