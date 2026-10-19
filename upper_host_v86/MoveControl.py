import serial
from .config import Mode
import time
from loguru import logger


# 输入一个[-128,127]范围内的整数，取它补码的十进制
def get_int8(num):
    assert -128 <= num <= 127, print("int8类型范围为[-128, 127]")
    if num >= 0:
        return int(num)
    else:
        return int(255 + num + 1)


# 运动控制类
class MoveControl(object):
    def __init__(self, port, baudrate) -> None:
        self.port = port
        self.baudrate = baudrate
        self.serial = serial.Serial(port, baudrate)
        print("串口初始化成功")
        self.serial.flush()
        
        # 初始化数据列表[头帧，模式，x方向，y方向，角度，二维码1，二维码2，尾帧]
        self.send_buffer = [0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0xFE]
        self.last_buffer = [0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0xFE]

    # 生成串口数据，发送数据并等待回传结果

    def __send_serial_msg(self, mode, x_dis=None, y_dis=None, angel=None):

        # x，y移动与选择
        if mode == Mode.x_distance:
            assert x_dis is not None, print("未指定x方向移动距离")
            self.send_buffer[1] = mode.value
            x_dis_int = round(x_dis)  # 米换算为厘米时避免浮点截断少 1 cm
            self.send_buffer[2] = (x_dis_int >> 8) & 0xFF
            self.send_buffer[3] = x_dis_int & 0xff

        elif mode == Mode.y_distance:
            assert y_dis is not None, print("未指定y方向移动距离")
            self.send_buffer[1] = mode.value
            y_dis_int = round(y_dis)
            self.send_buffer[2] = (y_dis_int >> 8) & 0xFF
            self.send_buffer[3] = y_dis_int & 0xff

        elif mode == Mode.rotate:
            assert angel is not None, print("未设置角度")
            self.send_buffer[1] = mode.value
            self.send_buffer[4] = get_int8(angel)


        #把123放到车上
        elif mode == Mode.grub_cup_1:
            self.send_buffer[1] = mode.value
        elif mode == Mode.grub_cup_2:
            self.send_buffer[1] = mode.value
        elif mode == Mode.grub_cup_3:
            self.send_buffer[1] = mode.value

        # 从放置点抓取物块并放到5个运载点
        elif mode == Mode.grub_1:
            self.send_buffer[1] = mode.value
        elif mode == Mode.grub_2:
            self.send_buffer[1] = mode.value
        elif mode == Mode.grub_3:
            self.send_buffer[1] = mode.value
        elif mode == Mode.grub_4:
            self.send_buffer[1] = mode.value
        elif mode == Mode.grub_5:
            self.send_buffer[1] = mode.value
    
        elif mode == Mode.grub_cup_1:
            self.send_buffer[1] = mode.value
        elif mode == Mode.grub_cup_2:
            self.send_buffer[1] = mode.value
        elif mode == Mode.grub_cup_3:
            self.send_buffer[1] = mode.value

        # 从5个运载点抓取物块并放置
        elif mode == Mode.put_1:
            self.send_buffer[1] = mode.value
        elif mode == Mode.put_2:
            self.send_buffer[1] = mode.value
        elif mode == Mode.put_3:
            self.send_buffer[1] = mode.value
        elif mode == Mode.put_4:
            self.send_buffer[1] = mode.value
        elif mode == Mode.put_5:
            self.send_buffer[1] = mode.value

            
            
        # 从三个位置放到冠亚季军台子
        elif mode == Mode.put_1to1:
            self.send_buffer[1] = mode.value
        elif mode == Mode.put_2to2:
            self.send_buffer[1] = mode.value
        elif mode == Mode.put_3to3:
            self.send_buffer[1] = mode.value
        elif mode == Mode.put_3to3_2:
            self.send_buffer[1] = mode.value
        
        #爪子前伸
        elif mode == Mode.grub_front:
            self.send_buffer[1] = mode.value
        elif mode == Mode.grub_return:
            self.send_buffer[1] = mode.value
            


        # x，y移动微调
        elif mode == Mode.x_dis_mm:
            assert x_dis, print("没有传入移动距离")
            self.send_buffer[1] = mode.value
            self.send_buffer[2] = get_int8(x_dis)
        elif mode == Mode.y_dim_mm:
            assert y_dis, print("没有传入移动距离")
            self.send_buffer[1] = mode.value
            self.send_buffer[3] = get_int8(y_dis)
            
        elif 47 < mode.value <= 72:
            self.send_buffer[1] = mode.value

        elif mode in (Mode.raiseToCameraHigh,
                       Mode.raiseToCameraLow,
                       Mode.turntableHome,
                       Mode.chassisTask2Profile,
                       Mode.chassisTask1Profile,
                       Mode.armStartButton):
            self.send_buffer[1] = mode.value

        elif mode == Mode.motor_align:
            self.send_buffer[1] = mode.value
        
        elif mode == Mode.reset_angle:
            self.send_buffer[1] = mode.value

        # 摄像头升降
        elif mode == Mode.raiseToUp:
            self.send_buffer[1] = mode.value
        elif mode == Mode.raiseToMid:
            self.send_buffer[1] = mode.value
            
        elif mode == Mode.raiseToDown:
            self.send_buffer[1] = mode.value
            
        #颜色识别
        elif mode == Mode.inform_color:
            self.send_buffer[1] = mode.value
        elif mode == Mode.reception_color:
            self.send_buffer[1] = mode.value


        # 爪子旋转
        elif mode == Mode.grubTurnAround:
            self.send_buffer[1] = mode.value
       
        elif mode == Mode.grubTurnAround_2:
            self.send_buffer[1] = mode.value

        else:
            raise ValueError("没有这个模式")
            
            
        # ===== 2026 REVIEW ：last_buffer 显式拷贝 =====
        # self.last_buffer = self.send_buffer  # ORIGINAL ：引用赋值
        self.last_buffer = list(self.send_buffer)
        self.serial.write(bytes(self.send_buffer))
        
        while not self.__wait_for_movement_done():
            
            self.serial.write(bytes(self.send_buffer))

            #self.__wait_for_movement_done()
        #print("下位机收到指令")
        time.sleep(0.05)
    
    # 距离控制，支持 x 和 y 同时输入；先横移 y，再前后移动 x
    def set_distance(self, x: float = 0, y: float = 0):
        """按米移动：x 为前后，y 为横移。"""
            
        x = 1e-9 if x == 0 else x
        y = 1e-9 if y == 0 else y
        max_one_time_dis = 1.25
        if abs(x) > max_one_time_dis or abs(y) > max_one_time_dis:
            Warning("单次设置最远距离为{0},需多次调用".format(max_one_time_dis))

        times_x, times_y = int(abs(x) / max_one_time_dis), int(
            abs(y) // max_one_time_dis
        )
        
        rest_x, rest_y = (
            abs(x) - times_x * max_one_time_dis,
            abs(y) - times_y * max_one_time_dis,
        )
        factor_x, factor_y = abs(x) // x, abs(y) // y

        for _ in range(times_y):
            dis = 125 * factor_y
            self.__send_serial_msg(mode=Mode.y_distance, y_dis=dis)
        if rest_y > 0.001:
            dis = factor_y * rest_y / max_one_time_dis * 125
            self.__send_serial_msg(mode=Mode.y_distance, y_dis=dis)
        else:
            pass

        for _ in range(times_x):
            dis = 125 * factor_x
            self.__send_serial_msg(mode=Mode.x_distance, x_dis=dis)
        if rest_x > 0.001:
            dis = factor_x * rest_x / max_one_time_dis * 125
            self.__send_serial_msg(mode=Mode.x_distance, x_dis=dis)
        else:
            pass
    
    def __wait_for_status(self, expected_status, label, timeout=None):
        original_timeout = self.serial.timeout
        self.serial.timeout = 0.1
        deadline = None if timeout is None else time.monotonic() + timeout
        rx_buffer = bytearray()
        try:
            while deadline is None or time.monotonic() < deadline:
                chunk = self.serial.read(max(1, self.serial.in_waiting))
                if not chunk:
                    continue
                rx_buffer.extend(chunk)
                while len(rx_buffer) >= 3:
                    try:
                        header_index = rx_buffer.index(0xFF)
                    except ValueError:
                        rx_buffer.clear()
                        break
                    if header_index:
                        del rx_buffer[:header_index]
                    if len(rx_buffer) < 3:
                        break
                    if rx_buffer[2] != 0xFE:
                        del rx_buffer[0]
                        continue
                    status = rx_buffer[1]
                    del rx_buffer[:3]
                    if status == expected_status:
                        logger.success(label)
                        return True
            raise TimeoutError(label + "超时")
        finally:
            self.serial.timeout = original_timeout

    # 程序启动后，等待下位机就绪帧 [0xFF, 0x02, 0xFE]
    def wait_for_start_cmd(self, timeout=None):
        logger.info("等待下位机启动指令")
        return self.__wait_for_status(0x02, "接收到下位机启动指令", timeout)
 
    # x,y距离微调函数 x,y单位为mm
    def move_in_mm(self, x=0, y=0):
        """按毫米微调：x 为前后，y 为横移。"""
        x = 128 if x > 128 else int(x)
        x = -127 if x < -127 else int(x)
        y = 128 if y > 128 else int(y)
        y = -127 if y < -127 else int(y)
        if x != 0:
            self.__send_serial_msg(mode=Mode.x_dis_mm, x_dis=x)
        if y != 0:
            self.__send_serial_msg(mode=Mode.y_dim_mm, y_dis=y)

    # 旋转函数
    def rotate(self, angle):
        assert -127 <= angle <= 128, print("角度应在[-127, 128]之间")
        self.__send_serial_msg(mode=Mode.rotate, angel=angle)

    #重置陀螺仪
    def reset_angle(self):
        self.__send_serial_msg(mode=Mode.reset_angle)
    
    #刷新角度
    def motor_align(self):
        self.__send_serial_msg(mode=Mode.motor_align)

    # 发送颜色识别指令
    def inform_color(self):
        self.__send_serial_msg(mode=Mode.inform_color)
        
    #把123从地上抓到车上
    def grub_cup_1(self):
        self.__send_serial_msg(mode = Mode.grub_cup_1)

    def grub_cup_2(self):
        self.__send_serial_msg(mode = Mode.grub_cup_2)

    def grub_cup_3(self):
        self.__send_serial_msg(mode = Mode.grub_cup_3)

    # 爪子前伸
    def grub_front(self):
        self.__send_serial_msg(mode=Mode.grub_front)
    # 爪子回复
    def grub_return(self):
        """mode 52：只将 Servo1 收回到舵盘对位标定（当前2455 μs）。"""
        self.__send_serial_msg(mode=Mode.grub_return)

    # 接收颜色识别回传
    def reception_color(self):
        self.send_buffer[1] = 54
        self.serial.write(bytes(self.send_buffer))
        time.sleep(0.05)
        #self.__send_serial_msg(mode=Mode.reception_color)
        """
        接收0XFF1-0XFF5范围的指令，返回对应的颜色字符串
        0XFF1 -> 'black'
        0XFF2 -> 'white'
        0XFF3 -> 'red'
        0XFF4 -> 'green'
        0XFF5 -> 'blue'
        """
        # 定义指令与颜色的映射关系
        command_map = {
            1: 'red',
            2: 'green',
            3: 'blue',
            4: 'white',
            5: 'black'
        }
        #self.serial.reset_input_buffer()#清空缓冲区
        
        while True:
            # 读取串口数据
            cmd_byte = self.serial.read()
            #print(cmd_byte)
            if cmd_byte:
                cmd = cmd_byte[0]
                print(cmd)
                # 检查是否为已知指令
                if cmd in command_map:
                    
                    # 返回对应的颜色字符串
                    self.serial.reset_input_buffer()#清空缓冲区
                    return command_map[cmd]
                else:
                    # 忽略未知指令，继续等待
                    pass
            # 未读取到数据时继续循环等待


    # 抓取函数
    def grubFromGround1(self):
        self.__send_serial_msg(mode=Mode.grub_1)
    def grubFromGround2(self):
        self.__send_serial_msg(mode=Mode.grub_2)
    def grubFromGround3(self):
        self.__send_serial_msg(mode=Mode.grub_3)
    def grubFromGround4(self):
        self.__send_serial_msg(mode=Mode.grub_4)
    def grubFromGround5(self):
        self.__send_serial_msg(mode=Mode.grub_5)

        
           
        
    #放到冠亚季军上
    def grub_1to1(self):
        self.__send_serial_msg(mode=Mode.put_1to1)
    def grub_2to2(self):
        self.__send_serial_msg(mode=Mode.put_2to2)
    def grub_3to3(self):
        self.__send_serial_msg(mode=Mode.put_3to3)
    def grub_3to3_2(self):
        self.__send_serial_msg(mode=Mode.put_3to3_2)


    # 摄像头升降函数
    def capToUp(self):
        self.__send_serial_msg(mode=Mode.raiseToUp)
    def capToMid(self):
        self.__send_serial_msg(mode=Mode.raiseToMid)
    def capTo178(self):
        self.__send_serial_msg(mode=Mode.raiseTo178)
    def capTo186(self):
        self.__send_serial_msg(mode=Mode.raiseTo186)
    def capToDown(self):
        self.__send_serial_msg(mode=Mode.raiseToDown)

    # 摄像头打靶高度；mode33/34/35的领奖台高度在STM32任务内处理。
    def capToCameraHigh(self):
        self.__send_serial_msg(mode=Mode.raiseToCameraHigh)
    def capToCameraLow(self):
        self.__send_serial_msg(mode=Mode.raiseToCameraLow)
    def capToChampion(self):
        """Compatibility alias for the mode84 camera high target."""
        return self.capToCameraHigh()
    def capToRunnerUp(self):
        """Compatibility alias for the mode85 camera low target."""
        return self.capToCameraLow()
    def turntable_home(self):
        self.__send_serial_msg(mode=Mode.turntableHome)

    def enable_task2_chassis_profile(self):
        """Enable task 2 speeds and symmetric acceleration/deceleration."""
        self.__send_serial_msg(mode=Mode.chassisTask2Profile)

    def enable_chassis_slow_speed(self):
        """Compatibility alias for enable_task2_chassis_profile()."""
        return self.enable_task2_chassis_profile()

    def restore_task1_chassis_profile(self):
        """Restore the task 1 chassis speeds and movement behavior."""
        self.__send_serial_msg(mode=Mode.chassisTask1Profile)

    def restore_chassis_speeds(self):
        """Compatibility alias for restore_task1_chassis_profile()."""
        return self.restore_task1_chassis_profile()

    def arm_start_button(self):
        """Enable the next valid PE6 press as the task start event."""
        self.__send_serial_msg(mode=Mode.armStartButton)

    def wait_for_start_button(self, timeout=None):
        """Wait until STM32 reports a debounced PE6 press."""
        logger.info("等待按下 PE6 启动按键")
        return self.__wait_for_status(0x03, "PE6 已按下，开始执行任务", timeout)

    def initialize_and_wait_for_start(self, timeout=None):
        """Prepare the robot at power-on, then wait for the PE6 start button."""
        self.wait_for_start_cmd(timeout=timeout)
        self.arm_init()
        self.reset_angle()
        self.arm_start_button()
        return self.wait_for_start_button(timeout=timeout)

    # 爪子旋转函数
    def grubTurnAround_front(self):
        self.__send_serial_msg(mode=Mode.grubTurnAround)
    
    def grubTurnAround_behind(self):
        self.__send_serial_msg(mode=Mode.grubTurnAround_2)


    # 放置函数
    
    def grubToGround1(self):
        self.__send_serial_msg(mode=Mode.put_1)
    def grubToGround2(self):
        self.__send_serial_msg(mode=Mode.put_2)
    def grubToGround3(self):
        self.__send_serial_msg(mode=Mode.put_3)
    def grubToGround4(self):
        self.__send_serial_msg(mode=Mode.put_4)
    def grubToGround5(self):
        self.__send_serial_msg(mode=Mode.put_5)

    def put_1_to_1(self):
        self.__send_serial_msg(mode=Mode.put_1_1st)
        
    def put_1_to_2(self):
        self.__send_serial_msg(mode=Mode.put_1_2nd)
        
    def put_1_to_3(self):
        self.__send_serial_msg(mode=Mode.put_1_3rd)

    def put_2_to_1(self):
        self.__send_serial_msg(mode=Mode.put_2_1st)

    def put_2_to_2(self):
        self.__send_serial_msg(mode=Mode.put_2_2nd)
        
    def put_2_to_3(self):
        self.__send_serial_msg(mode=Mode.put_2_3rd)
        
    def put_3_to_1(self):
        self.__send_serial_msg(mode=Mode.put_3_1st)
        
    def put_3_to_2(self):
        self.__send_serial_msg(mode=Mode.put_3_2nd)

    def put_3_to_3(self):
        self.__send_serial_msg(mode=Mode.put_3_3rd)
        
    def grub_to_1(self):
        self.__send_serial_msg(mode=Mode.grub_to_1)
        
    def grub_to_2(self):
        self.__send_serial_msg(mode=Mode.grub_to_2)
        
    def grub_to_3(self):
        self.__send_serial_msg(mode=Mode.grub_to_3)
        
    def grub_open(self):
        self.__send_serial_msg(mode=Mode.grub_open)

    def grub_close(self):
        self.__send_serial_msg(mode=Mode.grub_close)

    # 机械臂回到初始位置：底部舵机24度，升降回15cm，夹爪打开，伸缩收回
    def arm_init(self):
        self.__send_serial_msg(mode=Mode.arm_init)
        
  
        
    def __wait_for_movement_done(self, timeout=7):
        """等待下位机返回完成指令，超时抛出异常。"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.serial.in_waiting > 0:
                header_raw = self.serial.read(1)
                if not header_raw:
                    continue
                header = header_raw[0]
                if header == 0xFF:
                    status_raw = self.serial.read(1)
                    if status_raw and status_raw[0] == 0x01:
                        self.serial.read(1)  # 舍弃尾帧
                        logger.success("收到下位机回传指令")
                        return True
        logger.warning("下位机返回超时")
        return False
        # raise TimeoutError("等待下位机响应超时")


    def __del__(self):
        serial_port = getattr(self, "serial", None)
        if serial_port is not None:
            serial_port.close()
        print("程序结束，释放串口")
