#include "servo.h"

//TIM5_CH2    PA1    servo1  

/********************************
函数功能 : 舵机1的初始化
输入参数 : 无
输出参数 ；无
*********************************/
#define SERVO_TIM5_CH2_INIT_PULSE_US 2500u
#define SERVO_TIM5_CH3_INIT_PULSE_US  850u
#define SERVO_TIM5_CH4_INIT_PULSE_US    0u

static void Servo_TIM5_Init(void)
{
	static uint8_t servo_tim5_initialized = 0u;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	if (servo_tim5_initialized != 0u)
	{
		return;
	}

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	TIM_Cmd(TIM5, DISABLE);

	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 20000u - 1u;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72u - 1u;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0u;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OCInitStructure.TIM_Pulse = SERVO_TIM5_CH2_INIT_PULSE_US;
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = SERVO_TIM5_CH3_INIT_PULSE_US;
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = SERVO_TIM5_CH4_INIT_PULSE_US;
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIM5, ENABLE);
	TIM_OC3PreloadConfig(TIM5, ENABLE);
	TIM_OC4PreloadConfig(TIM5, ENABLE);
	TIM_ARRPreloadConfig(TIM5, ENABLE);

	/* Program valid pulses before enabling the counter. */
	TIM_SetCompare2(TIM5, SERVO_TIM5_CH2_INIT_PULSE_US);
	TIM_SetCompare3(TIM5, SERVO_TIM5_CH3_INIT_PULSE_US);
	TIM_SetCompare4(TIM5, SERVO_TIM5_CH4_INIT_PULSE_US);
	TIM_SetCounter(TIM5, 0u);
	TIM_GenerateEvent(TIM5, TIM_EventSource_Update);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_Cmd(TIM5, ENABLE);
	servo_tim5_initialized = 1u;
}

void Servo1_Init(void)
{
	Servo_TIM5_Init();
}
//TIM5_CH3    PA2    servo2
/********************************
函数功能 : 舵机2的初始化
输入参数 : 无
输出参数 ；无
*********************************/
void Servo2_Init(void)
{
	Servo_TIM5_Init();
}
//TIM5_CH4    PA3    servo3
/********************************
函数功能 : 舵机3的初始化
输入参数 : 无
输出参数 ；无
*********************************/
void Servo3_Init(void)
{
	Servo_TIM5_Init();
}
//TIM1_CH4    PA11    servo4

/********************************
函数功能 : 舵机1的初始化
输入参数 : 无
输出参数 ；无
*********************************/
void Servo4_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    
    // 1. 开启时钟 (关键修正：开启AFIO时钟)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); // 同时开启GPIOA和AFIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    // 2. 初始化GPIO PA11 (TIM1_CH4)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 3. 初始化定时器时基 (配置正确，保持不变)
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;     // ARR: 20ms周期
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;     // PSC: 1MHz计数频率
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;  // 高级定时器特有，必须设置
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    
    // 4. 初始化输出比较通道 (配置正确，保持不变)
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // 初始占空比
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    
    // 5. 使能预装载寄存器 (配置正确，保持不变)
    TIM_OC4PreloadConfig(TIM1, ENABLE);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    
    // 6. 【最关键的一步】使能高级定时器的主输出 --> 没有这个，PWM出不来！
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    
    // 7. 使能定时器
    TIM_Cmd(TIM1, ENABLE);
}
/*************************
270度舵机
高电平 0.5~2.5ms 对应 270度 
PWM脉宽 500~2500us 对应 0~270度 也就是0.5ms ~ 2.5ms
默认周期是20ms

72 000 000/72             

占空比
0.5/20   对应0度    500/20000

2.5/20   对应270度  2500/20000

那么 270度对应2000个步进
那么一个步进 可以对应0.135度
理论精度可以做到0.135度的分辨率

那么 1度对应7.4个步进

计算公式为CCR = 7.4*angle + 500


这里需要限制幅度，防止损坏机械结构





*************************/
/********************************
函数功能 : 舵机1的控制函数，机械臂上部舵机
输入参数 : 角度
输出参数 ；无
*********************************/
void SERVO1_CONTRAL(uint8_t angle)
{
	float temp ;
	temp = 7.407 *angle + 500; 
	TIM_SetCompare2(TIM5,(uint16_t)temp);
}
/********************************
函数功能 : 舵机2的控制函数
输入参数 : 角度
输出参数 ；无
*********************************/
void SERVO2_CONTRAL(uint8_t angle)
{
	float temp ;
	temp = 7.407 *angle + 500; 
	TIM_SetCompare3(TIM5,(uint16_t)temp);
}
/********************************
函数功能 : 舵机3的控制函数
输入参数 : 角度
输出参数 ；无
*********************************/
void SERVO3_CONTRAL(uint8_t angle)
{
	float temp ;
	temp = 7.407 *angle + 500; 
	TIM_SetCompare4(TIM5,(uint16_t)temp);
}

/********************************
函数功能 : 舵机4的控制函数，机械臂下部舵机
输入参数 : 角度
输出参数 ；无
*********************************/
void SERVO4_CONTRAL(uint8_t angle)
{
	float temp ;
	temp = 7.407 *angle + 500; 
	TIM_SetCompare4(TIM1,(uint16_t)temp);
}

