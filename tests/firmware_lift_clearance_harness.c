/* Execute actual firmware functions with measured-by-STEP mock position.
 * Proves lift isolation and fixed default reach; no physical servo feedback. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct { uint16_t bits; } GPIO_TypeDef;
static GPIO_TypeDef port_a, port_b, port_c;
#define GPIOA (&port_a)
#define GPIOB (&port_b)
#define GPIOC (&port_c)
#define TIM5 5
#define GPIO_Pin_4 (1u << 4)
#define GPIO_Pin_5 (1u << 5)
#define GPIO_Pin_10 (1u << 10)
static uint16_t lift_height_mm;
static uint32_t physical_steps, step_count, settle_remaining;
static uint16_t compares[5];
static unsigned disc_events, ground_events, disc_turn_events;
static unsigned servo1_writes;
static unsigned disc_speed_samples, disc_slow_samples, final_slow_samples;
static uint32_t last_servo1_height;
static uint16_t last_servo1_value;
typedef struct {
    uint16_t pulse, height, turntable, servo1;
    uint32_t at_ms;
} ClawEvent;
static ClawEvent claw_events[32], servo1_events[32], turn_events[32];
static unsigned claw_count, servo1_count, turn_count;
static uint32_t elapsed_ms;
static void GPIO_SetBits(GPIO_TypeDef *port, uint16_t bits);
static void GPIO_ResetBits(GPIO_TypeDef *port, uint16_t bits);
static void TIM_SetCompare2(int timer, uint16_t value);
static void TIM_SetCompare3(int timer, uint16_t value);
static void TIM_SetCompare4(int timer, uint16_t value);
static void delay_ms(uint32_t value)
{
    elapsed_ms += value;
    settle_remaining = (value >= settle_remaining) ? 0 : settle_remaining - value;
}
static void delay_us(uint32_t value);
static void Servo1_Init(void) { }
static void Servo2_Init(void) { }
static void Servo3_Init(void) { }

#include "lift_clearance_under_test.inc"

static void delay_us(uint32_t value)
{
    if (compares[4] != 0 && compares[4] != SERVO3_HOME_PULSE_US) {
        /* Check every half-period while the real task is aligned with a disc. */
        assert(value == LIFT_STEP_HALF_PERIOD_US || value == LIFT_GROUND_SLOW_STEP_HALF_PERIOD_US);
        disc_speed_samples++;
        if (value == LIFT_GROUND_SLOW_STEP_HALF_PERIOD_US) { disc_slow_samples++; }
    } else if (value == LIFT_GROUND_SLOW_STEP_HALF_PERIOD_US) {
        final_slow_samples++;
    }
}

static void GPIO_SetBits(GPIO_TypeDef *port, uint16_t bits)
{
    if (port == GPIOB && (bits & GPIO_Pin_10) && !(port->bits & GPIO_Pin_10)) {
        if (port_a.bits & GPIO_Pin_4) {
            if (claw_count == 3 && compares[2] != SERVO1_TASK2_EXTEND_PULSE_US &&
                physical_steps == claw_events[claw_count - 1].height * LIFT_PULSES_PER_MM &&
                claw_events[claw_count - 1].pulse == SERVO2_BOTTOM_OPEN_PULSE_US &&
                claw_events[claw_count - 1].turntable == SERVO3_HOME_PULSE_US) {
                /* Do not start lifting away while the ground release is still settling. */
                assert(elapsed_ms - claw_events[claw_count - 1].at_ms >= SERVO2_SETTLE_MS);
            }
            physical_steps++;
        }
        else {
            assert(physical_steps > 0);
            physical_steps--;
        }
        step_count++;
    }
    port->bits |= bits;
}
static void GPIO_ResetBits(GPIO_TypeDef *port, uint16_t bits)
{ port->bits &= (uint16_t)~bits; }

static void TIM_SetCompare2(int timer, uint16_t value)
{
    (void)timer;
    compares[2] = value;
    servo1_writes++;
    last_servo1_height = physical_steps;
    last_servo1_value = value;
    assert(servo1_count < sizeof(servo1_events) / sizeof(servo1_events[0]));
    servo1_events[servo1_count++] = (ClawEvent){value, physical_steps / LIFT_PULSES_PER_MM,
                                               compares[4], value, elapsed_ms};
    if (value != 0) { settle_remaining = SERVO1_SETTLE_MS; }
}
static void TIM_SetCompare3(int timer, uint16_t value)
{
    (void)timer;
    compares[3] = value;
    assert(value == 0 || value == SERVO2_CLAMP_PULSE_US ||
           value == SERVO2_DISC_PICK_OPEN_PULSE_US || value == SERVO2_DISC_RELEASE_PULSE_US ||
           value == SERVO2_BOTTOM_OPEN_PULSE_US || value == SERVO2_CUBE_GROUND_CLAMP_PULSE_US ||
           value == SERVO2_CUBE_DISC_OPEN_PULSE_US || value == SERVO2_CUBE_DISC_CLAMP_PULSE_US);
    assert(claw_count < sizeof(claw_events) / sizeof(claw_events[0]));
    claw_events[claw_count++] = (ClawEvent){value, physical_steps / LIFT_PULSES_PER_MM,
                                         compares[4], compares[2], elapsed_ms};
    if (physical_steps == LIFT_PLACE_HEIGHT_MM * LIFT_PULSES_PER_MM ||
        physical_steps == LIFT_TASK2_DISC_PLACE_HEIGHT_MM * LIFT_PULSES_PER_MM ||
        physical_steps == LIFT_TASK2_DISC_HEIGHT_MM * LIFT_PULSES_PER_MM) {
        assert(compares[2] == SERVO1_DISC_ALIGN_PULSE_US);
        disc_events++;
    }
    if (physical_steps == LIFT_GROUND_PICK_HEIGHT_MM * LIFT_PULSES_PER_MM ||
        physical_steps == LIFT_GROUND_PLACE_HEIGHT_MM * LIFT_PULSES_PER_MM) {
        assert(compares[2] == SERVO1_DISC_ALIGN_PULSE_US);
        ground_events++;
    }
}
static void TIM_SetCompare4(int timer, uint16_t value)
{
    (void)timer;
    if (value == SERVO3_DISC1_PULSE_US || value == SERVO3_DISC2_PULSE_US ||
        value == SERVO3_DISC3_PULSE_US || value == SERVO3_PUT_DISC4_PULSE_US ||
        value == SERVO3_DISC5_PULSE_US) {
        assert(compares[2] == SERVO1_DISC_ALIGN_PULSE_US);
        assert(settle_remaining == 0);
        disc_turn_events++;
    }
    assert(turn_count < sizeof(turn_events) / sizeof(turn_events[0]));
    turn_events[turn_count++] = (ClawEvent){value, physical_steps / LIFT_PULSES_PER_MM,
                                           value, compares[2], elapsed_ms};
    compares[4] = value;
}

static void reset(uint16_t height, uint16_t servo1)
{
    memset(&port_a, 0, sizeof(port_a));
    memset(&port_b, 0, sizeof(port_b));
    memset(&port_c, 0, sizeof(port_c));
    lift_height_mm = height;
    physical_steps = height * LIFT_PULSES_PER_MM;
    step_count = settle_remaining = 0;
    compares[2] = servo1; compares[3] = compares[4] = 0;
    servo1_writes = disc_events = ground_events = disc_turn_events = 0;
    last_servo1_height = last_servo1_value = 0;
    claw_count = servo1_count = turn_count = 0;
    disc_speed_samples = disc_slow_samples = final_slow_samples = 0;
    elapsed_ms = 0;
}

static void assert_disc_pick_sequence(uint16_t disc, uint16_t pickup_height, uint16_t release_height,
                                      uint16_t transfer_height, uint16_t return_height,
                                      uint16_t expected_open, uint16_t expected_clamp,
                                      uint16_t release_servo1)
{
    unsigned i;
    assert(claw_count == 3);
    assert(claw_events[0].pulse == expected_open);
    assert(claw_events[0].height == transfer_height && claw_events[0].turntable == disc);
    assert(claw_events[1].pulse == expected_clamp);
    assert(claw_events[1].height == pickup_height && claw_events[1].turntable == disc);
    assert(claw_events[2].pulse == SERVO2_BOTTOM_OPEN_PULSE_US);
    assert(claw_events[2].height == release_height && claw_events[2].turntable == SERVO3_HOME_PULSE_US);
    assert(claw_events[2].servo1 == release_servo1);
    assert(turn_count == 2);
    assert(turn_events[0].pulse == disc && turn_events[0].height == transfer_height);
    assert(turn_events[0].servo1 == SERVO1_DISC_ALIGN_PULSE_US);
    assert(turn_events[1].pulse == SERVO3_HOME_PULSE_US && turn_events[1].height == return_height);
    if (release_servo1 == SERVO1_DISC_ALIGN_PULSE_US) {
        for (i = 0; i < servo1_count; ++i) {
            assert(servo1_events[i].pulse == SERVO1_DISC_ALIGN_PULSE_US);
        }
        assert(compares[2] == SERVO1_DISC_ALIGN_PULSE_US);
    } else {
        unsigned extension_seen = 0;
        assert(release_servo1 == SERVO1_TASK2_EXTEND_PULSE_US);
        assert(turn_events[1].servo1 == SERVO1_TASK2_EXTEND_PULSE_US);
        for (i = 0; i < servo1_count; ++i) {
            if (servo1_events[i].pulse == SERVO1_TASK2_EXTEND_PULSE_US) {
                assert(servo1_events[i].height == return_height);
                extension_seen = 1;
            }
            if (extension_seen) {
                assert(servo1_events[i].pulse == SERVO1_TASK2_EXTEND_PULSE_US);
            }
        }
        assert(extension_seen);
        assert(compares[2] == SERVO1_TASK2_EXTEND_PULSE_US);
    }
    assert(compares[3] == SERVO2_BOTTOM_OPEN_PULSE_US);
}

static void assert_ground_to_disc_sequence(uint16_t disc, uint16_t expected_clamp,
                                          uint16_t place_height, uint16_t transfer_height)
{
    assert(claw_count == 4);
    assert(claw_events[0].pulse == SERVO2_BOTTOM_OPEN_PULSE_US);
    assert(claw_events[1].pulse == expected_clamp &&
           claw_events[1].height == LIFT_GROUND_PICK_HEIGHT_MM);
    assert(claw_events[2].pulse == SERVO2_DISC_RELEASE_PULSE_US &&
           claw_events[2].height == place_height && claw_events[2].turntable == disc);
    assert(claw_events[2].servo1 == SERVO1_DISC_ALIGN_PULSE_US);
    assert(claw_events[3].pulse == SERVO2_BOTTOM_OPEN_PULSE_US &&
           claw_events[3].height == transfer_height &&
           claw_events[3].turntable == SERVO3_HOME_PULSE_US);
    assert(turn_count == 2);
    assert(turn_events[0].pulse == disc && turn_events[0].height == transfer_height);
    assert(turn_events[1].pulse == SERVO3_HOME_PULSE_US && turn_events[1].height == transfer_height);
    assert(compares[2] == SERVO1_DISC_ALIGN_PULSE_US && lift_height_mm == LIFT_PICK_HEIGHT_MM);
}

int main(void)
{
    const ArmGrubTiming grub = {200, 200, 200, 200, 200, 200, 200, 200};
    /* Even zero extra holds must leave time for the full ground opening. */
    const ArmPutTiming put = {200, 200, 200, 200, 200, 0, 0};
    const uint16_t podiums[] = {LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM,
                               LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM,
                               LIFT_TASK2_THIRD_PLACE_HEIGHT_MM};
    const uint16_t discs[] = {SERVO3_DISC1_PULSE_US, SERVO3_DISC2_PULSE_US,
                             SERVO3_DISC3_PULSE_US, SERVO3_PUT_DISC4_PULSE_US,
                             SERVO3_DISC5_PULSE_US};
    const uint8_t put_modes[] = {RPI_CMD_PUT_DISC1, RPI_CMD_PUT_DISC2, RPI_CMD_PUT_DISC3,
                                 RPI_CMD_PUT_DISC4, RPI_CMD_PUT_DISC5};
    const uint8_t grub_modes[] = {RPI_CMD_GRUB_DISC1, RPI_CMD_GRUB_DISC2, RPI_CMD_GRUB_DISC3,
                                  RPI_CMD_GRUB_DISC4, RPI_CMD_GRUB_DISC5};
    const uint16_t field_heights[] = {LIFT_GROUND_PLACE_HEIGHT_MM, LIFT_DISC2_STACK_PLACE_HEIGHT_MM,
                                      LIFT_GROUND_PLACE_HEIGHT_MM, LIFT_DISC4_STACK_PLACE_HEIGHT_MM,
                                      LIFT_GROUND_PLACE_HEIGHT_MM};
    const uint8_t task2_modes[] = {RPI_CMD_TASK2_PUT_1TO1, RPI_CMD_TASK2_PUT_2TO2,
                                   RPI_CMD_TASK2_PUT_3TO3};
    const uint8_t task2_grub_modes[] = {RPI_CMD_TASK2_GRUB_CUP1, RPI_CMD_TASK2_GRUB_CUP2,
                                        RPI_CMD_TASK2_GRUB_CUP3};
    uint16_t disc_pulse, podium_height;
    unsigned i;

#ifdef EXPECT_INVALID_TASK2_HEIGHTS
    assert(!Arm_Task2HeightsValid());
    for (i = 0; i < 3; ++i) {
        reset(170, SERVO1_DISC_ALIGN_PULSE_US);
        assert(!test_handle_arm_mode(task2_modes[i]));
        assert(!test_handle_arm_mode(task2_grub_modes[i]));
        assert(step_count == 0 && servo1_writes == 0 && claw_count == 0 && turn_count == 0);
    }
    puts("Invalid task2 heights rejected before mock movement: modes 33/34/35/42/43/44");
    return 0;
#endif
    assert(Arm_Task2HeightsValid());

    assert(Arm_IsCubeDiscMode(RPI_CMD_GRUB_DISC2));
    assert(Arm_IsCubeDiscMode(RPI_CMD_GRUB_DISC4));
    assert(Arm_IsCubeDiscMode(RPI_CMD_PUT_DISC2));
    assert(Arm_IsCubeDiscMode(RPI_CMD_PUT_DISC4));
    assert(!Arm_IsCubeDiscMode(RPI_CMD_GRUB_DISC1));
    assert(!Arm_IsCubeDiscMode(RPI_CMD_PUT_DISC5));

    /* Crossing the old boundary, even with PWM disabled or task2 extended,
     * must issue no Servo1 writes. */
    {
        const uint16_t starts[] = {170, 31, 30, 16, 30, 16, 0, 0, 10};
        const uint16_t targets[] = {16, 30, 29, 170, 40, 16, 280, 360, 0};
        const uint16_t positions[] = {SERVO1_DISC_ALIGN_PULSE_US, SERVO1_TASK2_EXTEND_PULSE_US, 0};
        unsigned j;
        for (i = 0; i < 3; ++i) {
            for (j = 0; j < 9; ++j) {
                uint32_t distance = starts[j] > targets[j] ? starts[j]-targets[j] : targets[j]-starts[j];
                reset(starts[j], positions[i]);
                Lift_MoveToHeight(targets[j]);
                assert(lift_height_mm == targets[j]);
                assert(step_count == distance * LIFT_PULSES_PER_MM);
                assert(servo1_writes == 0 && compares[2] == positions[i]);
                assert(!(port_b.bits & GPIO_Pin_10));
            }
        }
    }
    reset(170, SERVO1_DISC_ALIGN_PULSE_US);
    Lift_MoveToPlaceHeightSlow(15);
    assert(servo1_writes == 0 && compares[2] == SERVO1_DISC_ALIGN_PULSE_US);
    assert(step_count == 155 * LIFT_PULSES_PER_MM && lift_height_mm == 15);
    reset(0, SERVO1_DISC_ALIGN_PULSE_US);
    Lift_MoveToHeight(361);
    assert(step_count == 0 && servo1_writes == 0);

    reset(0, 0);
    Arm_PWM_Init();
    assert(step_count == 0 && compares[2] == SERVO1_DISC_ALIGN_PULSE_US);
    assert(compares[3] == 0 && compares[4] == 0);
    Arm_RunSoftwareInit();
    assert(lift_height_mm == LIFT_PICK_HEIGHT_MM);
    assert(compares[2] == SERVO1_DISC_ALIGN_PULSE_US);

    reset(170, SERVO1_DISC_ALIGN_PULSE_US);
    Arm_RunGrubToDisc(SERVO3_DISC1_PULSE_US, SERVO2_CLAMP_PULSE_US,
                     LIFT_PLACE_HEIGHT_MM, LIFT_TRANSFER_HEIGHT_MM, &grub);
    assert(disc_events > 0 && ground_events > 0 && disc_turn_events == 1);
    assert(lift_height_mm == LIFT_PICK_HEIGHT_MM && compares[2] == SERVO1_DISC_ALIGN_PULSE_US);
    reset(170, SERVO1_TASK2_EXTEND_PULSE_US);
    Arm_RunGrubToDisc(SERVO3_DISC1_PULSE_US, SERVO2_CLAMP_PULSE_US,
                     LIFT_PLACE_HEIGHT_MM, LIFT_TRANSFER_HEIGHT_MM, &grub);
    assert(disc_events > 0 && ground_events > 0 && disc_turn_events == 1);
    assert(lift_height_mm == LIFT_PICK_HEIGHT_MM && compares[2] == SERVO1_DISC_ALIGN_PULSE_US);
    reset(170, SERVO1_TASK2_EXTEND_PULSE_US);
    Arm_RunPutFromDisc(SERVO3_DISC1_PULSE_US, SERVO2_DISC_PICK_OPEN_PULSE_US,
                       SERVO2_CLAMP_PULSE_US, LIFT_GROUND_PLACE_HEIGHT_MM, &put);
    assert(disc_events > 0 && ground_events > 0 && disc_turn_events == 1);
    assert(lift_height_mm == LIFT_PICK_HEIGHT_MM && compares[2] == SERVO1_DISC_ALIGN_PULSE_US);

    for (i = 0; i < 3; ++i) {
        reset(170, SERVO1_DISC_ALIGN_PULSE_US);
        assert(Arm_Task2ParamsFromMode(task2_modes[i], &disc_pulse, &podium_height));
        assert(disc_pulse == discs[i * 2] && podium_height == podiums[i]);
        assert(test_handle_arm_mode(task2_modes[i]));
        assert_disc_pick_sequence(disc_pulse, LIFT_TASK2_DISC_HEIGHT_MM, podiums[i],
                                  LIFT_TASK2_TRANSFER_HEIGHT_MM, LIFT_TASK2_RETURN_CLEARANCE_MM,
                                  SERVO2_DISC_PICK_OPEN_PULSE_US, SERVO2_CLAMP_PULSE_US,
                                  SERVO1_TASK2_EXTEND_PULSE_US);
        assert(disc_speed_samples > 0 && disc_slow_samples == 0 && final_slow_samples > 0);
        assert(disc_events > 0 && disc_turn_events == 1 && lift_height_mm == LIFT_PICK_HEIGHT_MM);
        assert(compares[2] == SERVO1_TASK2_EXTEND_PULSE_US);
        reset(170, SERVO1_TASK2_EXTEND_PULSE_US);
        assert(Arm_Task2GrubPulseFromMode(task2_grub_modes[i], &disc_pulse));
        assert(disc_pulse == discs[i * 2]);
        assert(test_handle_arm_mode(task2_grub_modes[i]));
        assert_ground_to_disc_sequence(disc_pulse, SERVO2_CLAMP_PULSE_US,
                                      LIFT_TASK2_DISC_PLACE_HEIGHT_MM, LIFT_TASK2_TRANSFER_HEIGHT_MM);
        assert(disc_slow_samples > 0 && final_slow_samples == 0);
        assert(disc_events > 0 && ground_events > 0 && disc_turn_events == 1);
    }
    for (i = 0; i < 5; ++i) {
        const uint16_t ground_clamp = Arm_IsCubeDiscMode(grub_modes[i]) ?
                                      SERVO2_CUBE_GROUND_CLAMP_PULSE_US : SERVO2_CLAMP_PULSE_US;
        const uint16_t disc_open = Arm_IsCubeDiscMode(put_modes[i]) ?
                                   SERVO2_CUBE_DISC_OPEN_PULSE_US : SERVO2_DISC_PICK_OPEN_PULSE_US;
        const uint16_t disc_clamp = Arm_IsCubeDiscMode(put_modes[i]) ?
                                    SERVO2_CUBE_DISC_CLAMP_PULSE_US : SERVO2_CLAMP_PULSE_US;
        reset(170, SERVO1_DISC_ALIGN_PULSE_US);
        assert(test_handle_arm_mode(grub_modes[i]));
        assert_ground_to_disc_sequence(discs[i], ground_clamp, LIFT_PLACE_HEIGHT_MM, LIFT_TRANSFER_HEIGHT_MM);
        assert(disc_slow_samples > 0 && final_slow_samples == 0);
        assert(disc_turn_events == 1 && disc_events > 0);
        reset(170, SERVO1_DISC_ALIGN_PULSE_US);
        assert(Arm_DiscPulseFromMode(put_modes[i], &disc_pulse) && disc_pulse == discs[i]);
        assert(Arm_FieldPlaceHeightFromMode(put_modes[i]) == field_heights[i]);
        assert(test_handle_arm_mode(put_modes[i]));
        assert_disc_pick_sequence(disc_pulse, LIFT_PLACE_HEIGHT_MM, field_heights[i],
                                  LIFT_TRANSFER_HEIGHT_MM, LIFT_TRANSFER_HEIGHT_MM,
                                  disc_open, disc_clamp, SERVO1_DISC_ALIGN_PULSE_US);
        assert(disc_slow_samples > 0 && final_slow_samples > 0);
        assert(disc_turn_events == 1 && disc_events > 0);
    }
    reset(170, SERVO1_DISC_ALIGN_PULSE_US);
    Arm_RunGrubToDisc(SERVO3_DISC2_PULSE_US, SERVO2_CUBE_GROUND_CLAMP_PULSE_US,
                     LIFT_PLACE_HEIGHT_MM, LIFT_TRANSFER_HEIGHT_MM, &grub);
    assert_ground_to_disc_sequence(SERVO3_DISC2_PULSE_US, SERVO2_CUBE_GROUND_CLAMP_PULSE_US,
                                  LIFT_PLACE_HEIGHT_MM, LIFT_TRANSFER_HEIGHT_MM);
    reset(170, SERVO1_DISC_ALIGN_PULSE_US);
    Arm_RunPutFromDisc(SERVO3_DISC2_PULSE_US, SERVO2_CUBE_DISC_OPEN_PULSE_US,
                       SERVO2_CUBE_DISC_CLAMP_PULSE_US, Arm_FieldPlaceHeightFromMode(RPI_CMD_PUT_DISC2), &put);
    assert_disc_pick_sequence(SERVO3_DISC2_PULSE_US, LIFT_PLACE_HEIGHT_MM,
                              LIFT_DISC2_STACK_PLACE_HEIGHT_MM, LIFT_TRANSFER_HEIGHT_MM, LIFT_TRANSFER_HEIGHT_MM,
                              SERVO2_CUBE_DISC_OPEN_PULSE_US, SERVO2_CUBE_DISC_CLAMP_PULSE_US,
                              SERVO1_DISC_ALIGN_PULSE_US);
    reset(170, SERVO1_DISC_ALIGN_PULSE_US);
    Arm_RunPutFromDisc(SERVO3_PUT_DISC4_PULSE_US, SERVO2_CUBE_DISC_OPEN_PULSE_US,
                       SERVO2_CUBE_DISC_CLAMP_PULSE_US, Arm_FieldPlaceHeightFromMode(RPI_CMD_PUT_DISC4), &put);
    assert_disc_pick_sequence(SERVO3_PUT_DISC4_PULSE_US, LIFT_PLACE_HEIGHT_MM,
                              LIFT_DISC4_STACK_PLACE_HEIGHT_MM, LIFT_TRANSFER_HEIGHT_MM, LIFT_TRANSFER_HEIGHT_MM,
                              SERVO2_CUBE_DISC_OPEN_PULSE_US, SERVO2_CUBE_DISC_CLAMP_PULSE_US,
                              SERVO1_DISC_ALIGN_PULSE_US);
    puts("Fixed retract simulation OK: lift isolation, task dispatch, task2 extension, claw states and startup");
    return 0;
}
