/* Mock GPIO/UART test harness; the tested functions are extracted unchanged
 * from chassis_controller_main.c by test_firmware_components.py. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct { uint16_t bits; unsigned edges[16]; } GPIO_TypeDef;
typedef enum { Bit_RESET = 0, Bit_SET = 1 } BitAction;
static GPIO_TypeDef port_a, port_b, port_c, port_g;
#define GPIOA (&port_a)
#define GPIOB (&port_b)
#define GPIOC (&port_c)
#define GPIOG (&port_g)
#define TIM5 5
#define GPIO_Pin_1 (1u << 1)
#define GPIO_Pin_4 (1u << 4)
#define GPIO_Pin_6 (1u << 6)
#define GPIO_Pin_7 (1u << 7)
#define GPIO_Pin_8 (1u << 8)
#define GPIO_Pin_9 (1u << 9)
#define GPIO_Pin_10 (1u << 10)
static uint16_t lift_height_mm, lift_test_remainder_pulses;
static uint16_t compares[5];
static unsigned rx_polls, stop_at, longest_delay;

static void GPIO_SetBits(GPIO_TypeDef *port, uint16_t bits)
{
    unsigned i;
    for (i = 0; i < 16; ++i) {
        if ((bits & (1u << i)) && !(port->bits & (1u << i))) { port->edges[i]++; }
    }
    port->bits |= bits;
}
static void GPIO_ResetBits(GPIO_TypeDef *port, uint16_t bits) { port->bits &= ~bits; }
static void GPIO_WriteBit(GPIO_TypeDef *port, uint16_t pin, BitAction value)
{
    if (value) { GPIO_SetBits(port, pin); } else { GPIO_ResetBits(port, pin); }
}
static void TIM_SetCompare2(int timer, uint16_t value) { (void)timer; compares[2] = value; }
static void TIM_SetCompare3(int timer, uint16_t value) { (void)timer; compares[3] = value; }
static void TIM_SetCompare4(int timer, uint16_t value) { (void)timer; compares[4] = value; }
static void delay_us(uint16_t value) { if (value > longest_delay) { longest_delay = value; } }
static uint16_t Chassis_ReadUInt16(uint8_t high, uint8_t low) { return ((uint16_t)high << 8) | low; }
static void Chassis_SendStatus(uint8_t status) { (void)status; }
static uint8_t Chassis_TryReadFrame(uint8_t *payload)
{
    rx_polls++;
    if (stop_at && rx_polls == stop_at) {
        memset(payload, 0, 8);
        payload[0] = 91;
        return 1;
    }
    return 0;
}

#include "component_under_test.inc"

static void reset(void)
{
    memset(&port_a, 0, sizeof(port_a)); memset(&port_b, 0, sizeof(port_b));
    memset(&port_c, 0, sizeof(port_c)); memset(&port_g, 0, sizeof(port_g));
    compares[2] = compares[3] = compares[4] = 1500;
    lift_height_mm = lift_test_remainder_pulses = 0;
    rx_polls = stop_at = longest_delay = 0;
}
int main(void)
{
    uint8_t wheel[8] = {90, 3, 0, 0, 200, 3, 232, 0};
    uint8_t lift[8] = {92, 0, 5, 3, 232, 0, 0, 0};
    const uint8_t ids[4] = {3, 4, 7, 8};
    unsigned i, j;
    for (i = 0; i < 4; ++i) {
        reset(); wheel[1] = ids[i]; wheel[2] = 1;
        assert(HardwareTest_Wheel(wheel) == COMMAND_RESULT_DONE);
        for (j = 6; j <= 9; ++j) { assert(port_c.edges[j] == (j == i + 6 ? 200u : 0u)); }
        assert(port_b.edges[10] == 0);
        assert(!(port_c.bits & CHASSIS_STEP_PINS));
        assert(compares[2] == 1500 && compares[3] == 1500 && compares[4] == 1500);
        assert(longest_delay <= 100);
    }
    reset(); wheel[1] = 3; wheel[3] = wheel[4] = 0;
    assert(HardwareTest_Wheel(wheel) == COMMAND_RESULT_INVALID);
    wheel[4] = 200; wheel[5] = 0; wheel[6] = 0;
    assert(HardwareTest_Wheel(wheel) == COMMAND_RESULT_INVALID); /* 0 us */
    assert(port_c.edges[6] == 0);
    reset(); wheel[1] = 3; wheel[2] = 0; wheel[3] = 0; wheel[4] = 1;
    wheel[5] = 255; wheel[6] = 255;
    assert(HardwareTest_Wheel(wheel) == COMMAND_RESULT_DONE); /* 65535 us */
    assert(port_c.edges[6] == 1);
    reset(); wheel[4] = 200; wheel[5] = 3; wheel[6] = 232; stop_at = 221;
    assert(HardwareTest_Wheel(wheel) == COMMAND_RESULT_TEST_STOPPED);
    assert(port_c.edges[6] == 2);
    assert(!(port_c.bits & CHASSIS_STEP_PINS));
    assert(compares[2] == 0 && compares[3] == 0 && compares[4] == 0);

    reset(); lift[2] = 11;
    assert(HardwareTest_Lift(lift) == COMMAND_RESULT_DONE);
    assert(lift_height_mm == 11 && lift_test_remainder_pulses == 0);
    assert(port_b.edges[10] == 704);
    lift[2] = 0;
    assert(HardwareTest_Lift(lift) == COMMAND_RESULT_DONE);
    assert(lift_height_mm == 0 && port_b.edges[10] == 1408);
    lift[1] = 1; lift[2] = 24;
    assert(HardwareTest_Lift(lift) == COMMAND_RESULT_DONE);
    assert(lift_height_mm == 280 && port_b.edges[10] == 17920);
    lift[2] = 25;
    assert(HardwareTest_Lift(lift) == COMMAND_RESULT_INVALID);
    assert(lift_height_mm == 280);
    reset(); lift[1] = 0; lift[2] = 5;
    lift[2] = 5;
    assert(HardwareTest_Lift(lift) == COMMAND_RESULT_DONE);
    assert(lift_height_mm == 5 && lift_test_remainder_pulses == 0);
    assert(port_b.edges[10] == 320 && (port_a.bits & GPIO_Pin_4));
    assert(compares[2] == 1500 && compares[3] == 1500 && compares[4] == 1500);
    for (j = 6; j <= 9; ++j) { assert(port_c.edges[j] == 0); }
    lift[2] = 0;
    assert(HardwareTest_Lift(lift) == COMMAND_RESULT_DONE);
    assert(port_b.edges[10] == 640 && !(port_a.bits & GPIO_Pin_4));
    assert(lift_height_mm == 0);

    reset(); lift[2] = 5; stop_at = 221;
    assert(HardwareTest_Lift(lift) == COMMAND_RESULT_TEST_STOPPED);
    assert(port_b.edges[10] == 2 && !(port_b.bits & GPIO_Pin_10));
    assert(lift_height_mm == 0 && lift_test_remainder_pulses == 2);
    stop_at = 0;
    assert(HardwareTest_Lift(lift) == COMMAND_RESULT_DONE);
    assert(port_b.edges[10] == 320 && lift_height_mm == 5 && lift_test_remainder_pulses == 0);
    assert(HardwareTest_IsLiftCommand(RPI_CMD_GRUB_DISC1));
    assert(HardwareTest_IsLiftCommand(RPI_CMD_SOFTWARE_INIT));
    assert(!HardwareTest_IsLiftCommand(RPI_CMD_TEST_LIFT));
    puts("Firmware component simulation OK: isolated wheel/lift pulses, bounds, STOP, partial lift recovery");
    return 0;
}
