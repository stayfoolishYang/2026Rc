"""Timing meanings audited against the formal chassis-controller entry.

This module describes parameters; it never changes firmware values or sends I/O.
"""
from __future__ import annotations

from dataclasses import replace
import re

LEGACY_TIME_DEFINES = (
    "SERVO3_HOME_BEFORE_CLAW_OPEN_MS", "ARM_ACTION_HOLD_MS", "GROUND_RELEASE_STAGE_INTERVAL_MS",
)
NEW_TIME_DEFINES = ("SERVO_POWER_ON_SETTLE_MS", "CHASSIS_DIRECTION_SETTLE_MS", "LIFT_DIRECTION_SETTLE_MS")
PROFILE_FINE_DEFINES = ("CHASSIS_TASK2_FINE_STEP_HALF_PERIOD_US",)
TIME_GROUPS = ("任务1直线速度", "任务2直线速度", "旋转与微调", "加减速与距离判定",
               "升降速度与末段", "公共舵机等待", "电机换向等待")
OVERVIEW = "公共等待与独立命令"
SYSTEM = "系统通信与超时（只读）"
LEGACY = "历史未使用参数（只读）"

# label, group, actual consumers and applicability. Microseconds here are
# STEP half-periods, not the position PWM used by servo parameters.
GLOBAL_INFO = {
    "CHASSIS_STEP_HALF_PERIOD_US": ("长距离巡航半周期", "任务1直线速度", "mode1/2；任务1配置（mode88），单次距离达到长距阈值时使用；越小越快。"),
    "CHASSIS_SHORT_STEP_HALF_PERIOD_US": ("短距离巡航半周期", "任务1直线速度", "mode1/2；任务1配置下低于长距阈值的厘米移动；不使用加减速。"),
    "CHASSIS_RAMP_START_HALF_PERIOD_US": ("长距起步／停车半周期", "任务1直线速度", "任务1长距起停；大于巡航半周期才启用曲线，否则按巡航值固定运行。"),
    "CHASSIS_TASK2_LONG_STEP_HALF_PERIOD_US": ("长距离巡航半周期", "任务2直线速度", "mode87启用任务2配置后，mode1/2达到长距阈值时使用。"),
    "CHASSIS_SLOW_SHORT_STEP_HALF_PERIOD_US": ("短距离巡航半周期", "任务2直线速度", "任务2配置下，低于长距阈值的厘米移动；起停用公共旋转／任务2起停值。"),
    "CHASSIS_ROTATE_SLOW_STEP_HALF_PERIOD_US": ("旋转／纠偏巡航半周期", "旋转与微调", "mode3旋转、mode20航向纠偏；任务1/2均使用此值。"),
    "CHASSIS_FINE_STEP_HALF_PERIOD_US": ("任务1快速微调半周期（mode88）", "旋转与微调", "mode88及上电默认：mode5前后／mode6横移微调使用；固定速度，不使用起停曲线，不受长距阈值影响。"),
    "CHASSIS_TASK2_FINE_STEP_HALF_PERIOD_US": ("任务2慢速微调半周期（mode87）", "旋转与微调", "mode87下的mode5前后／mode6横移微调使用；切回mode88恢复另一项快速微调；不影响mode1/2的厘米移动。"),
    "CHASSIS_ROTATE_180_PULSES": ("旋转180°的脉冲标定", "旋转与微调", "角度→脉冲的换算；不是速度或等待。mode3和mode20共用。"),
    "CHASSIS_TASK2_STOP_HALF_PERIOD_US": ("任务2／旋转起步停车半周期", "加减速与距离判定", "任务2长短距、所有旋转／纠偏共用。起停值不大于巡航值时使用固定速度。"),
    "CHASSIS_TASK2_RAMP_PULSES": ("任务2／旋转每端曲线脉冲数", "加减速与距离判定", "任务2长短距与旋转共用；0或1不启用曲线，短动作自动缩短到总脉冲的一半。"),
    "CHASSIS_RAMP_PULSES": ("任务1长距每端曲线脉冲数", "加减速与距离判定", "只用于任务1长距；0或1不启用曲线，短动作自动缩短。"),
    "CHASSIS_RAMP_MIN_DISTANCE_MM": ("厘米移动长距离判定阈值", "加减速与距离判定", "只用于mode1/2厘米移动，达到该距离时走长距分支；mode5/6始终使用当前配置的微调半周期。"),
    "LIFT_STEP_HALF_PERIOD_US": ("普通升降半周期", "升降速度与末段", "普通升降、任务升降和上升过程使用；元器件mode92的输入半周期独立。"),
    "LIFT_GROUND_SLOW_STEP_HALF_PERIOD_US": ("下降末段半周期", "升降速度与末段", "放入舵盘、任务1从盘下降夹取、赛场地面／堆叠和领奖台放置共用；任务2从盘取杯仍用普通速度。"),
    "LIFT_GROUND_SLOW_APPROACH_DISTANCE_MM": ("下降末段长度（距目标）", "升降速度与末段", "缓降动作距目标剩此高度差时切到末段半周期；0无独立末段。不是升降目标高度。"),
    "SERVO1_SETTLE_MS": ("伸缩单项／公共对位等待", "公共舵机等待", "mode80单项试动和任务中的公共伸缩对位；地面抓取任务用两次。mode52使用下面的独立命令等待。"),
    "SERVO2_SETTLE_MS": ("夹爪单项／最终松爪基础等待", "公共舵机等待", "mode71、81、初始化的张爪；任务1最终全开时也用此值，再累加该任务的额外等待。其他任务夹爪步骤用专用值。"),
    "SERVO3_SETTLE_MS": ("转台单项／初始化回正等待", "公共舵机等待", "mode82、86和mode72开始回正。完整任务的去盘／回正使用各任务专用值。"),
    "SERVO_POWER_ON_SETTLE_MS": ("独立伸缩／松爪命令等待", "公共舵机等待", "mode36前伸、52收回、70全张开及mode72初始化收尾。不是正式上电启动的延时。"),
    "CHASSIS_DIRECTION_SETTLE_MS": ("底盘换向后开始STEP的等待", "电机换向等待", "底盘移动／旋转／纠偏：设好DIR后等待，再开始发STEP；每次底盘动作都会使用。"),
    "LIFT_DIRECTION_SETTLE_MS": ("普通升降换向后等待", "电机换向等待", "普通升降设好DIR后等待，再发STEP；无位移时不等待。元器件mode92不使用此项。"),
}

# These descriptions follow the executed functions, including waits after lift
# motion and consecutive waits at the same final release point.
GRUB_ROLES = {
    "OPEN_WAIT": ("01 夹爪地面全张开后等待", "写全开PWM后，等到可以收回伸缩并下降。"),
    "CLAMP_WAIT": ("02 地面夹紧后等待", "写夹紧PWM后等待，再开始升到转盘安全高度。"),
    "CLAMP_HOLD": ("03 升至安全高度后停留", "升降已经结束，夹爪保持夹紧；等完后再伸缩对位、转台去盘。"),
    "TURNTABLE_WAIT": ("04 转台去目标盘后等待", "转台写入目标盘PWM后等待，再下降到舵盘放块高度。"),
    "RELEASE_WAIT": ("05 在舵盘松爪后等待", "在舵盘高度写松爪PWM后的基础等待。"),
    "RELEASE_HOLD": ("06 舵盘松爪后额外停留", "紧接上一项再等这段时间，然后升回安全高度；两项相加。"),
    "HOME_WAIT": ("07 转台回正后等待", "安全高度写回正PWM后，等完才全张开夹爪；不使用历史全局500ms字段。"),
    "OPEN_AFTER_HOME_WAIT": ("08 回正后夹爪全开等待", "夹爪全张开后等待，再降到待机高度。"),
}
PUT_ROLES = {
    "TURNTABLE_WAIT": ("01 转台去目标盘后等待", "安全高度写转盘PWM后等待，再张开夹爪。"),
    "PICK_OPEN_WAIT": ("02 从盘抓取前张爪等待", "夹爪张开后等待，再下降到舵盘夹取高度。"),
    "CLAMP_WAIT": ("03 舵盘夹紧后等待", "在舵盘高度夹紧物块后的基础等待。"),
    "CLAMP_HOLD": ("04 夹紧后额外停留", "紧接上一项再等这段时间，之后才升回安全高度；两项相加。"),
    "HOME_WAIT": ("05 转台回正后等待", "安全高度写回正PWM后等待，再下降到赛场释放高度。"),
    "STAGE_INTERVAL": ("06 最终松爪额外停留A", "最终全开已经等待公共Servo2时间；此项紧接着再等。当前没有分段张爪动作。"),
    "FINAL_OPEN_HOLD": ("07 最终松爪额外停留B", "紧接额外停留A再等，然后回升。最终等待=公共Servo2+A+B。"),
}
PLACE_ROLES = {
    "RETRACT_WAIT": ("01 伸缩收回后等待", "任务开始写收回PWM后等待，再升到任务2转盘高度；只影响当前领奖台任务。"),
    "TURNTABLE_WAIT": ("02 转台去取杯盘后等待", "去盘PWM写入后等待，再张开夹爪。"),
    "RELEASE_WAIT": ("03 从盘取杯前张爪等待", "这里是张开准备取杯，不是在领奖台放杯；等完再下降取杯。"),
    "CLAMP_WAIT": ("04 在舵盘夹杯后等待", "取杯高度写夹紧PWM后的基础等待。"),
    "CLAMP_HOLD": ("05 夹杯后额外停留", "紧接基础夹紧等待，再等此项，随后升到前伸／回正高度。"),
    "EXTEND_WAIT": ("06 伸缩前伸后等待", "写任务2前伸PWM后等待，再转台回正；不使用公共Servo1单项等待。"),
    "HOME_WAIT": ("07 转台回正后等待", "写回正PWM后等待，再下降到领奖台释放高度。"),
    "OPEN_WAIT": ("08 领奖台松爪后等待", "写地面全开PWM后的基础等待，只影响当前领奖台任务。"),
    "OPEN_HOLD": ("09 领奖台松爪后额外停留", "紧接上一项再等，随后回升；两项相加。"),
}


def task_family(prefix):
    if prefix.startswith("TASK2_PLACE"):
        return "place", PLACE_ROLES
    if prefix.startswith("PUT"):
        return "put", PUT_ROLES
    return "grub", GRUB_ROLES


def role_for_define(define):
    match = re.fullmatch(r"(TASK2_PLACE[123]|TASK2_GRUB[135]|GRUB[1-5]|PUT[1-5])_(.+)_MS", define)
    if match:
        return task_family(match[1])[1].get(match[2])
    return None


def organize_specs(specs):
    result = []
    for spec in specs:
        if spec.define in LEGACY_TIME_DEFINES:
            continue
        if spec.define in GLOBAL_INFO:
            label, group, detail = GLOBAL_INFO[spec.define]
            changes = {"label": label, "group": group, "description": detail}
            if spec.define in ("CHASSIS_RAMP_PULSES", "CHASSIS_TASK2_RAMP_PULSES"):
                changes["minimum"] = 0
            spec = replace(spec, **changes)
        else:
            role = role_for_define(spec.define)
            if role:
                spec = replace(spec, label=role[0], description=role[1])
        result.append(spec)
    return tuple(result)


def shared_waits(prefix):
    family, _ = task_family(prefix)
    if family == "grub":
        return [("SERVO1_SETTLE_MS", 2, "地面抓取前收回一次；升至转盘高度后对位一次。")]
    if family == "put":
        return [("SERVO1_SETTLE_MS", 1, "去目标盘前伸缩对位。"),
                ("SERVO2_SETTLE_MS", 1, "最终释放时夹爪全开后的基础等待。")]
    return [("SERVO1_SETTLE_MS", 1, "升至任务2转盘高度后对位。起始收回与后段前伸分别用任务专用值。")]


def as_int(values, name):
    return int(str(values[name]).strip())


def wait_summary(prefix, values):
    _, roles = task_family(prefix)
    try:
        private = sum(as_int(values, f"{prefix}_{suffix}_MS") for suffix in roles)
        shared = sum(as_int(values, name)*count for name, count, _ in shared_waits(prefix))
        text = f"当前舵机固定等待合计：{private + shared} ms（任务专用 {private} + 公共 {shared}）。不含升降、换向、底盘运动和通信开销。"
        if prefix.startswith("PUT"):
            base = as_int(values, "SERVO2_SETTLE_MS")
            a = as_int(values, prefix + "_STAGE_INTERVAL_MS")
            b = as_int(values, prefix + "_FINAL_OPEN_HOLD_MS")
            text += f"\n最终松爪等待：{base} + {a} + {b} = {base+a+b} ms。"
        if private + shared >= 60000:
            text += "\n固定等待已达到界面普通任务60秒超时；超时不表示机械动作已停止。"
        return text
    except (KeyError, ValueError):
        return "有等待值尚未填写为整数；填写后会显示累计时间。"


def command_summary(values):
    try:
        s1, s2, s3, independent = [as_int(values, key) for key in
            ("SERVO1_SETTLE_MS", "SERVO2_SETTLE_MS", "SERVO3_SETTLE_MS", "SERVO_POWER_ON_SETTLE_MS")]
        return (f"mode80 伸缩单项：{s1} ms；mode71/81 夹爪单项：{s2} ms；mode82/86 转台单项：{s3} ms。\n"
                f"mode36 前伸、mode52 收回、mode70 全开：均等待 {independent} ms。\n"
                f"mode72 初始化固定等待：{s3} + {s2} + {independent} = {s3+s2+independent} ms，另加升降运动和换向时间。\n"
                "元器件mode94只设置PWM，不等待舵机到位；以上均是固定等待，不是传感器反馈。")
    except (KeyError, ValueError):
        return "公共等待含有无效输入，请先填写整数。"


SYSTEM_INFO = {
    "CHASSIS_READY_REPEAT_MS": ("空闲就绪反馈名义间隔", "ms", "收到第一条上位机命令前重复；轮询开销会使实际间隔更长。"),
    "UART_RX_POLL_INTERVAL_US": ("主控串口空闲轮询间隔", "us", "只在主循环空闲时使用；不是阻塞任务的急停响应时间。"),
    "START_BUTTON_DEBOUNCE_MS": ("PE6去抖等待", "ms", "确认按键高／低状态前的固定等待。"),
    "ALIGN_GYRO_TIMEOUT_MS": ("等陀螺仪有效角度帧的窗口", "ms", "记录基准、纠偏及每个波特率诊断；到期没有有效帧则失败。"),
    "ALIGN_SETTLE_MS": ("每次纠偏后静置等待", "ms", "纠偏后先等此项，再读取下一帧角度。"),
    "GYRO_RX_POLL_INTERVAL_US": ("陀螺仪接收轮询间隔", "us", "与超时窗口一起换算接收循环次数。"),
    "GYRO_RESET_SETTLE_MS": ("陀螺仪初始化／归零稳定等待", "ms", "选择接收串口后和发归零命令后使用；一次mode19通常用两次。"),
    "GYRO_TX_TIMEOUT_POLLS": ("陀螺仪发送等待上限", "次轮询", "循环次数，不是毫秒，不能按固定时间直接换算。"),
    "GYRO_RAW_PROBE_DURATION_MS": ("每路RX原始电平探测时长", "ms", "mode126对四路接收脚逐个探测。"),
    "GYRO_RAW_PROBE_SAMPLE_US": ("原始电平采样间隔", "us", "mode126用来统计电平跳变。"),
    "GYRO_RUNTIME_BAUD_RATE": ("陀螺仪运行波特率", "baud", "通信速率不是等待；当前常规命令固定用UART4。"),
}
GUI_TIME_INFO = (
    ("参数单项试动等待反馈上限", "8 s", "mode80/81/82/83等单项试动；超时不会自动重发。"),
    ("普通任务等待反馈上限", "60 s", "完整抓放、独立动作；超时不等于固件停止。"),
    ("底盘直线／旋转试跑反馈上限", "30 s", "界面试跑mode1/3。"),
    ("功能探测／PE6读取／停止反馈上限", "3 s", "只有实际返回相应状态才算完成。"),
    ("元器件步进测试反馈上限", "按输入计算", "车轮约2×名义运动时长+5s；升降按全行程估计且至少20s。"),
    ("界面事件刷新／串口工作线程轮询", "50 ms / 10 ms", "软件刷新周期，不控制舵机或电机速度。"),
    ("参数搜索合并刷新", "150 ms", "仅界面防抖，不影响固件。"),
    ("上位机MoveControl完成后间隔", "50 ms", "位于Python上位机，不能用STM32等待参数修改。"),
    ("上位机MoveControl动作反馈窗口", "7 s", "上位机超时后会重发原命令；与调参台60秒、不自动重发的行为不同。这里仅说明现状，不修改上位机。"),
    ("上位机等待指定状态的串口读取窗口", "100 ms", "总等待期限由调用者传入；未传入时可以一直等待。不是动作持续时间。"),
)


def source_constants(source):
    return {key: int(value) for key, value in re.findall(r"^\s*#define\s+(\w+)\s+(\d+)u\b", source, re.MULTILINE)}


def audit_inventory(source, specs):
    constants = source_constants(source)
    clean = re.sub(r"/\*.*?\*/|//[^\n]*", "", source, flags=re.DOTALL)
    editable = {spec.define: spec for spec in specs}
    rows = []
    for name, value in constants.items():
        if not (name.endswith(("_MS", "_US")) or "TIMEOUT" in name or "POLL" in name or name in GLOBAL_INFO):
            continue
        uses = len(re.findall(r"\b" + re.escape(name) + r"\b", clean))-1
        spec = editable.get(name)
        kind = "固定等待" if name.endswith("_MS") else "速度半周期" if "HALF_PERIOD" in name else "系统或换算"
        if "PULSE_US" in name and name.startswith("SERVO"):
            kind = "PWM位置协议边界" if "TUNING" in name else "位置PWM"
        if name == "TEST_MIN_HALF_PERIOD_US":
            kind = "元器件速度协议边界"
        rows.append({"define": name, "value": value, "active_reference_count": uses,
                     "source_line": next(i for i, line in enumerate(source.splitlines(), 1) if re.match(r"\s*#define\s+" + name + r"\b", line)),
                     "editable": spec is not None, "group": spec.group if spec else "历史未使用" if name in LEGACY_TIME_DEFINES else "系统／协议",
                     "meaning": spec.description if spec else SYSTEM_INFO.get(name, ("", "", ""))[2],
                     "kind": kind})
    return rows


DERIVED_INFO = (
    ("UART_READY_REPEAT_POLLS", "CHASSIS_READY_REPEAT_MS×1000/UART_RX_POLL_INTERVAL_US", "主控就绪帧名义轮询次数"),
    ("GYRO_RX_TIMEOUT_POLLS", "ALIGN_GYRO_TIMEOUT_MS×1000/GYRO_RX_POLL_INTERVAL_US", "有效角度帧接收上限轮询次数"),
    ("GYRO_RAW_PROBE_SAMPLES", "GYRO_RAW_PROBE_DURATION_MS×1000/GYRO_RAW_PROBE_SAMPLE_US", "每路RX原始电平采样次数"),
)
