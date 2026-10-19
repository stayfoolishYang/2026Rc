from enum import Enum


class Mode(Enum):
    x_distance = 1  # 前进/后退，厘米距离
    y_distance = 2  # 横移，厘米距离
    rotate = 3
    #inform_color = 4
    x_dis_mm = 5  # 前进/后退，毫米微调
    y_dim_mm = 6  # 横移，毫米微调
    grub_1 = 7
    grub_2 = 8
    grub_3 = 9
    grub_4 = 10
    grub_5 = 12
    put_1 = 14
    put_2 = 15
    put_3 = 16
    put_4 = 17
    put_5 = 18

    reset_angle = 19
    motor_align = 20

    # 冠亚季军颁奖台放置
    put_1to1 = 33
    put_2to2 = 34
    put_3to3 = 35  # 爪子伸出去放35
    grub_front = 36  # 爪子伸出
    put_3to3_2 = 37  # 爪子正常直着放37

    grub_cup_1 = 42
    grub_cup_2 = 43
    grub_cup_3 = 44

    raiseToUp = 48    # 摄像头升到最高（导航移动）
    raiseToDown = 49  # 摄像头降到最低（扫码、颜色识别）
    grubTurnAround = 50   # 爪子转到前方
    grubTurnAround_2 = 51 # 爪子转到后方

    # ⚠️ Mode值冲突：put_1_1st = 52 与 grub_return = 52 使用了相同的值！
    # 以下九宫格放置模式（put_X_to_Y）与 grub_return(52)、raiseToMid(53)、reception_color(54) 值冲突
    # 2026年 m.py 使用 grub_1to1/2to2/3to3 颁奖台方案，未调用九宫格系列
    put_1_1st = 52
    grub_return = 52  # Servo1 单独收回到舵盘对位（当前2455 μs）；与 put_1_1st 共用旧值
    put_1_2nd = 53  # 与 raiseToMid 共用旧协议值，保留兼容接口
    raiseToMid = 53   # 摄像头升到中间（圆形纠偏）
    raiseTo178 = 58   # 升降到17.8cm（178mm）
    raiseTo186 = 59   # 升降到18.6cm（186mm）
    put_1_3rd = 54  # 与 reception_color 共用旧协议值，保留兼容接口
    reception_color = 54  # 接收STM32颜色回传（2026软件层不再调用）
    put_2_1st = 55
    put_2_2nd = 56
    put_2_3rd = 57
    put_3_1st = 64
    put_3_2nd = 65
    put_3_3rd = 66
    grub_to_1 = 67
    grub_to_2 = 68
    grub_to_3 = 69
    grub_open = 70
    grub_close = 71
    arm_init = 72

    # 摄像头打靶高度；任务2领奖台高度仅在mode33/34/35内部使用。
    raiseToCameraHigh = 84  # 36cm / 360mm
    raiseToCameraLow = 85   # 18cm / 180mm
    # 旧调用名保留协议兼容；其实际含义已改为摄像头打靶高度。
    raiseToChampion = 84
    raiseToRunnerUp = 85
    turntableHome = 86    # 底部旋转舵机回到24度
    chassisTask2Profile = 87     # 启用任务2独立底盘速度和对称加减速
    chassisSlowSpeed = 87        # 旧名称兼容：等同 chassisTask2Profile
    chassisTask1Profile = 88     # 恢复任务1底盘速度
    chassisSpeedRestore = 88     # 旧名称兼容：等同 chassisTask1Profile
    armStartButton = 89          # 初始化完成后启用PE6启动按键

    inform_color = 4       # 通知STM32识别颜色（2026软件层不再调用）

# ===== 2026修改：Circle_mode 枚举未被使用，已废弃 =====
# class Circle_mode(Enum):
#     get_1 = 1
#     put_1 = 2


# ===== 2026修改：颜色值统一为小写，与 detect_color() 返回值和 putToCar_based_color 保持一致 =====
# 原始值首字母大写（如 "Black", "White"），putToGround_based_qr_info 同步改为小写比较
qrcode_color = {
    1: {'A': "black", 'B': "white", 'C': "red", 'D': "green", 'E': "blue"},
    2: {'A': "white", 'B': "black", 'C': "red", 'D': "green", 'E': "blue"},
    3: {'A': "white", 'B': "black", 'C': "green", 'D': "red", 'E': "blue"},
    4: {'A': "blue", 'B': "white", 'C': "black", 'D': "red", 'E': "green"},
    5: {'A': "white", 'B': "red", 'C': "blue", 'D': "black", 'E': "green"},
    6: {'A': "black", 'B': "red", 'C': "blue", 'D': "white", 'E': "green"},
    7: {'A': "blue", 'B': "green", 'C': "black", 'D': "white", 'E': "red"},
    8: {'A': "green", 'B': "white", 'C': "blue", 'D': "black", 'E': "red"},
    9: {'A': "white", 'B': "green", 'C': "black", 'D': "blue", 'E': "red"},
    10: {'A': "black", 'B': "red", 'C': "blue", 'D': "green", 'E': "white"},
    11: {'A': "red", 'B': "blue", 'C': "green", 'D': "black", 'E': "white"},
    12: {'A': "green", 'B': "red", 'C': "black", 'D': "blue", 'E': "white"},
    13: {'A': "white", 'B': "red", 'C': "blue", 'D': "green", 'E': "black"},
    14: {'A': "red", 'B': "green", 'C': "white", 'D': "blue", 'E': "black"},
    15: {'A': "blue", 'B': "white", 'C': "green", 'D': "red", 'E': "black"},
    16: {'A': "green", 'B': "blue", 'C': "red", 'D': "white", 'E': "black"}
}

qrcode_letter = {
    1: { 1:"A",  2:"B",  3:"C"},
    2: { 1:"A",  2:"C",  3:"B"},
    3: { 1:"B",  2:"A",  3:"C"},
    4: { 1:"B",  2:"C",  3:"A"},
    5: { 1:"C",  2:"A",  3:"B"},
    6: { 1:"C",  2:"B",  3:"A"}
}


dis_between_every_circle = 0.15

pi = 3.14159

if __name__ == '__main__':
    a = [i for i in iter(Mode)]
    print(a[0])
