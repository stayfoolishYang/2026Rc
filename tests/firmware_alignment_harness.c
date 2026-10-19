#include <assert.h>
#include <stdint.h>
#include <stdio.h>

typedef struct { uint16_t bits; } GPIO_TypeDef;
static GPIO_TypeDef port_a, port_c, port_g;
#define GPIOA (&port_a)
#define GPIOC (&port_c)
#define GPIOG (&port_g)
#define GPIO_Pin_1 (1u << 1)
#define GPIO_Pin_6 (1u << 6)
#define GPIO_Pin_7 (1u << 7)
#define GPIO_Pin_8 (1u << 8)
#define GPIO_Pin_9 (1u << 9)
static int16_t base_angle;
static uint8_t chassis_task2_profile_enabled;
static uint8_t gyro_valid;
static float measured_angle;
static unsigned pulse_calls;
static uint32_t last_pulses;

static void GPIO_SetBits(GPIO_TypeDef *port, uint16_t bits) { port->bits |= bits; }
static void GPIO_ResetBits(GPIO_TypeDef *port, uint16_t bits) { port->bits &= (uint16_t)~bits; }
static void delay_ms(unsigned value) { (void)value; }
static void Gyro_PrepareRuntimeReceiver(void) { }
static uint8_t Gyro_ReadRelativeHeading(float *value) { *value = measured_angle; return gyro_valid; }
static void Chassis_Pulse(uint32_t pulses);
static void Chassis_PulseAtHalfPeriod(uint32_t pulses, uint32_t period) { (void)period; Chassis_Pulse(pulses); }
static void Chassis_PulseWithRamp(uint32_t pulses, uint32_t cruise, uint32_t start, uint32_t ramp)
{ (void)cruise; (void)start; (void)ramp; Chassis_Pulse(pulses); }

#include "alignment_under_test.inc"

static float abs_angle(float value) { return value < 0.0f ? -value : value; }
static void Chassis_Pulse(uint32_t pulses)
{
    unsigned reverse = (port_g.bits & GPIO_Pin_7) != 0u;
    assert(((port_g.bits & GPIO_Pin_8) != 0u) == reverse);
    assert(((port_a.bits & GPIO_Pin_8) != 0u) == reverse);
    assert(((port_c.bits & GPIO_Pin_1) != 0u) == reverse);
    /* Measured v163 response: DIR=0000, 866 pulses -> relative yaw +20.51.
     * This model tests feedback polarity, not actual motor or IMU accuracy. */
    measured_angle += (reverse ? -1.0f : 1.0f) * pulses * (20.51f / 866.0f);
    while (measured_angle > 180.0f) { measured_angle -= 360.0f; }
    while (measured_angle < -180.0f) { measured_angle += 360.0f; }
    pulse_calls++;
    last_pulses = pulses;
}

static void reset(float actual, int16_t target)
{
    port_a.bits = port_c.bits = port_g.bits = 0;
    measured_angle = actual; base_angle = target;
    gyro_valid = 1; pulse_calls = 0; last_pulses = 0;
}

int main(void)
{
    const float starts[] = {10.0f, -10.0f, 175.0f, -175.0f};
    const int16_t targets[] = {0, 0, -175, 175};
    unsigned i;
    float error_before;

    /* Check the real direction function for both translation axes and signs.
     * Motor order: 3=RR, 4=FR, 7=FL, 8=RL. */
    for (i = 0; i < 2; ++i) {
        Chassis_SetDirection(CHASSIS_VECTOR_FORWARD_BACK, (uint8_t)i);
        assert(((port_g.bits & GPIO_Pin_7) != 0u) == i);
        assert(((port_g.bits & GPIO_Pin_8) != 0u) == i);
        assert(((port_a.bits & GPIO_Pin_8) != 0u) == !i);
        assert(((port_c.bits & GPIO_Pin_1) != 0u) == !i);
        Chassis_SetDirection(CHASSIS_VECTOR_LEFT_RIGHT, (uint8_t)i);
        assert(((port_g.bits & GPIO_Pin_7) != 0u) == i);
        assert(((port_g.bits & GPIO_Pin_8) != 0u) == !i);
        assert(((port_a.bits & GPIO_Pin_8) != 0u) == !i);
        assert(((port_c.bits & GPIO_Pin_1) != 0u) == i);
    }

    /* Normal signed rotations must keep the sign established by the board. */
    reset(0.0f, 0);
    Chassis_Move(CHASSIS_VECTOR_ROTATE, 20, 1, 0);
    assert(last_pulses == 866 && abs_angle(measured_angle - 20.51f) < 0.001f);
    reset(0.0f, 0);
    Chassis_Move(CHASSIS_VECTOR_ROTATE, -20, 1, 0);
    assert(last_pulses == 866 && abs_angle(measured_angle + 20.51f) < 0.001f);

    for (i = 0; i < 4; ++i) {
        reset(starts[i], targets[i]);
        error_before = abs_angle(Chassis_HeadingError((float)base_angle, measured_angle));
        assert(Chassis_MotorAlign() == 1u);
        assert(pulse_calls >= 1 && pulse_calls <= ALIGN_MAX_CORRECTIONS);
        assert(abs_angle(Chassis_HeadingError((float)base_angle, measured_angle)) < error_before);
        assert(abs_angle(Chassis_HeadingError((float)base_angle, measured_angle)) <= ALIGN_DONE_ERROR_DEG);
        assert(!(port_c.bits & CHASSIS_STEP_PINS));
    }
    reset(0.5f, 0);
    assert(Chassis_MotorAlign() == 1u && pulse_calls == 0);
    reset(21.0f, 0);
    assert(Chassis_MotorAlign() == 0u && pulse_calls == 0);
    reset(10.0f, 0); gyro_valid = 0;
    assert(Chassis_MotorAlign() == 0u && pulse_calls == 0);
    puts("Alignment simulation OK: both signs and wrap converge; tolerance, max-error and gyro-failure guards preserved");
    return 0;
}
