#include "system_init.h"

/*
 * Raspberry Pi -> STM32 chassis controller.
 *
 * UART1: PA9 TX, PA10 RX, 115200 8N1
 * Frame: FF + 8 payload bytes + FE
 * payload[0] = mode
 *
 * Modes used by MoveControl.py:
 *   1: signed X distance in cm, payload[1:2] (big endian int16)
 *   2: signed Y distance in cm, payload[1:2] (big endian int16)
 *   3: signed rotation in degrees, payload[3] (int8)
 *   5: signed X fine movement in mm, payload[1] (int8)
 *   6: signed Y fine movement in mm, payload[2] (int8)
 *   7/8/9/10/12: pick from front and place to disc 1/2/3/4/5
 *   14/15/16/17/18: reverse disc 1/2/3/4/5 back to front
 *   33/34/35: task 2, disc 1/3/5 to champion/runner-up/third-place
 *   36: Servo1 task 2 extension position
 *   52: Servo1 retract to 270 degrees
 *   58: lift to 178 mm
 *   59: lift to 186 mm
 *   70: open the Servo2 claw
 *   71: close the Servo2 claw at SERVO2_CLAMP_PULSE_US
 *   80/81/82: hardware-tuning pulse for Servo1/2/3, payload[1:2]
 *   83: hardware-tuning lift height in mm, payload[1:2]
 *   84: lift to the configured high camera-target height
 *   85: lift to the configured low camera-target height
 *   86: Servo3 turntable home (24 degrees)
 *   87: enable the task 2 chassis profile and acceleration/deceleration ramps
 *   88: restore the task 1 chassis profile
 *   89: arm the PE6 start button after startup preparation
 *   90: one wheel, payload[1]=MOTOR3/4/7/8, [2]=DIR, [3:4]=pulses,
 *       [5:6]=nominal half-period us; polled STOP accepted during test
 *   91: stop component tests and remove the three servo PWM signals
 *   92: lift test, [1:2]=target mm, [3:4]=nominal half-period us, target 0-360 mm
 *   93: component-test capability probe (no movement)
 *   94: one servo, [1]=1/2/3, [2:3]=pulse us (ACK means signal applied)
 *   95: read PE6 level without arming the competition start button
 *
 * STM32 -> Raspberry Pi:
 *   FF 02 FE: controller ready
 *   FF 01 FE: command completed
 *   FF 00 FE: unsupported/invalid command
 *   FF 03 FE: PE6 start button pressed
 *   FF 04 FE: component test stopped, servo PWM signals removed
 *   FF 05/06 FE: PE6 low/high snapshot
 *   FF 07 FE: component-test protocol supported
 */

#define CHASSIS_STEP_HALF_PERIOD_US        75u
#define CHASSIS_SHORT_STEP_HALF_PERIOD_US 100u
#define CHASSIS_TASK2_LONG_STEP_HALF_PERIOD_US 150u
#define CHASSIS_SLOW_SHORT_STEP_HALF_PERIOD_US 200u
#define CHASSIS_ROTATE_SLOW_STEP_HALF_PERIOD_US 100u
#define CHASSIS_TASK2_STOP_HALF_PERIOD_US 350u
#define CHASSIS_TASK2_RAMP_PULSES         700u
#define CHASSIS_FINE_STEP_HALF_PERIOD_US  200u
#define CHASSIS_TASK2_FINE_STEP_HALF_PERIOD_US 600u
#define CHASSIS_RAMP_START_HALF_PERIOD_US 200u
#define CHASSIS_RAMP_PULSES               700u
#define CHASSIS_RAMP_MIN_DISTANCE_MM      500u
#define CHASSIS_DIRECTION_SETTLE_MS        20u
#define CHASSIS_READY_REPEAT_MS           1000u
#define UART_RX_POLL_INTERVAL_US             10u
#define UART_READY_REPEAT_POLLS \
    ((CHASSIS_READY_REPEAT_MS * 1000u) / UART_RX_POLL_INTERVAL_US)

/* Software-init arm pose: turntable home, lift to 15 cm, claw baseline, retract. */
#define SERVO1_DISC_ALIGN_PULSE_US         2455u  /* Default retracted position, also used for disc alignment. */
#define SERVO1_TASK2_EXTEND_PULSE_US         946u  /* 60 degrees. */
/* Four task claw states, shared by every disc and both tasks. */
#define SERVO2_BOTTOM_OPEN_PULSE_US         765u  /* Ground pickup/release and podium release. */
#define SERVO2_DISC_RELEASE_PULSE_US       1200u  /* Release after descending onto a disc. */
#define SERVO2_DISC_PICK_OPEN_PULSE_US     1200u  /* Open at transfer height before disc pickup. */
#define SERVO2_CLAMP_PULSE_US              1350u  /* Hold the object for lifting/transferring. */
/* 2/4-disc cuboids: ground opening stays the common bottom-open value. */
#define SERVO2_CUBE_GROUND_CLAMP_PULSE_US  1360u  /* Cuboid clamp after ground pickup. */
#define SERVO2_CUBE_DISC_OPEN_PULSE_US     1200u  /* Cuboid disc pickup opening. */
#define SERVO2_CUBE_DISC_CLAMP_PULSE_US    1360u  /* Cuboid clamp after disc pickup. */
#define SERVO3_HOME_PULSE_US                663u  /* 24 degrees. */
#define SERVO3_DISC1_PULSE_US              1575u  /* 158 degrees. */
#define SERVO3_DISC2_PULSE_US              1790u  /* 183 degrees. */
#define SERVO3_DISC3_PULSE_US              2004u  /* Calibrated disc 3 position. */
#define SERVO3_PUT_DISC4_PULSE_US          2243u  /* 226 degrees for task1 grub/put. */
#define SERVO3_DISC5_PULSE_US              2452u  /* 250 degrees for task1/task2. */
#define SERVO_POWER_ON_SETTLE_MS            500u
#define SERVO1_SETTLE_MS                    200u
#define SERVO2_SETTLE_MS                    380u
#define SERVO3_SETTLE_MS                    850u
#define SERVO3_HOME_BEFORE_CLAW_OPEN_MS     500u
#define ARM_ACTION_HOLD_MS                  200u
#define GROUND_RELEASE_STAGE_INTERVAL_MS    100u

/* Per-disc/per-operation timing for the grab/place task routines
 * (all in ms). Defaults keep the previous shared settle/hold behaviour;
 * tune each value to shorten the race run. Task1 grabs/puts are
 * independent per disc; task2 grab/place use their own shared sets. */
#define GRUB1_OPEN_WAIT_MS                      250u
#define GRUB1_CLAMP_WAIT_MS                     350u
#define GRUB1_CLAMP_HOLD_MS                     200u
#define GRUB1_TURNTABLE_WAIT_MS                 400u
#define GRUB1_RELEASE_WAIT_MS                   380u
#define GRUB1_RELEASE_HOLD_MS                   200u
#define GRUB1_HOME_WAIT_MS                      300u
#define GRUB1_OPEN_AFTER_HOME_WAIT_MS           200u
#define GRUB2_OPEN_WAIT_MS                      300u
#define GRUB2_CLAMP_WAIT_MS                     400u
#define GRUB2_CLAMP_HOLD_MS                     400u
#define GRUB2_TURNTABLE_WAIT_MS                 500u
#define GRUB2_RELEASE_WAIT_MS                   200u
#define GRUB2_RELEASE_HOLD_MS                   200u
#define GRUB2_HOME_WAIT_MS                      200u
#define GRUB2_OPEN_AFTER_HOME_WAIT_MS           150u
#define GRUB3_OPEN_WAIT_MS                      300u
#define GRUB3_CLAMP_WAIT_MS                     300u
#define GRUB3_CLAMP_HOLD_MS                     300u
#define GRUB3_TURNTABLE_WAIT_MS                 600u
#define GRUB3_RELEASE_WAIT_MS                   200u
#define GRUB3_RELEASE_HOLD_MS                   200u
#define GRUB3_HOME_WAIT_MS                      450u
#define GRUB3_OPEN_AFTER_HOME_WAIT_MS           200u
#define GRUB4_OPEN_WAIT_MS                      300u
#define GRUB4_CLAMP_WAIT_MS                     300u
#define GRUB4_CLAMP_HOLD_MS                     350u
#define GRUB4_TURNTABLE_WAIT_MS                 700u
#define GRUB4_RELEASE_WAIT_MS                   200u
#define GRUB4_RELEASE_HOLD_MS                   200u
#define GRUB4_HOME_WAIT_MS                      500u
#define GRUB4_OPEN_AFTER_HOME_WAIT_MS           200u
#define GRUB5_OPEN_WAIT_MS                      380u
#define GRUB5_CLAMP_WAIT_MS                     380u
#define GRUB5_CLAMP_HOLD_MS                     200u
#define GRUB5_TURNTABLE_WAIT_MS                 850u
#define GRUB5_RELEASE_WAIT_MS                   380u
#define GRUB5_RELEASE_HOLD_MS                   200u
#define GRUB5_HOME_WAIT_MS                      500u
#define GRUB5_OPEN_AFTER_HOME_WAIT_MS           380u
#define TASK2_GRUB1_OPEN_WAIT_MS                      300u
#define TASK2_GRUB1_CLAMP_WAIT_MS                     300u
#define TASK2_GRUB1_CLAMP_HOLD_MS                     200u
#define TASK2_GRUB1_TURNTABLE_WAIT_MS                 400u
#define TASK2_GRUB1_RELEASE_WAIT_MS                   300u
#define TASK2_GRUB1_RELEASE_HOLD_MS                   200u
#define TASK2_GRUB1_HOME_WAIT_MS                      400u
#define TASK2_GRUB1_OPEN_AFTER_HOME_WAIT_MS           200u
#define TASK2_GRUB3_OPEN_WAIT_MS                      200u
#define TASK2_GRUB3_CLAMP_WAIT_MS                     380u
#define TASK2_GRUB3_CLAMP_HOLD_MS                     380u
#define TASK2_GRUB3_TURNTABLE_WAIT_MS                 600u
#define TASK2_GRUB3_RELEASE_WAIT_MS                   380u
#define TASK2_GRUB3_RELEASE_HOLD_MS                   200u
#define TASK2_GRUB3_HOME_WAIT_MS                      600u
#define TASK2_GRUB3_OPEN_AFTER_HOME_WAIT_MS           200u
#define TASK2_GRUB5_OPEN_WAIT_MS                      380u
#define TASK2_GRUB5_CLAMP_WAIT_MS                     380u
#define TASK2_GRUB5_CLAMP_HOLD_MS                     200u
#define TASK2_GRUB5_TURNTABLE_WAIT_MS                 800u
#define TASK2_GRUB5_RELEASE_WAIT_MS                   200u
#define TASK2_GRUB5_RELEASE_HOLD_MS                   200u
#define TASK2_GRUB5_HOME_WAIT_MS                      500u
#define TASK2_GRUB5_OPEN_AFTER_HOME_WAIT_MS           100u
#define PUT1_TURNTABLE_WAIT_MS                 400u
#define PUT1_PICK_OPEN_WAIT_MS                 200u
#define PUT1_CLAMP_WAIT_MS                     150u
#define PUT1_CLAMP_HOLD_MS                     200u
#define PUT1_HOME_WAIT_MS                      400u
#define PUT1_STAGE_INTERVAL_MS                 100u
#define PUT1_FINAL_OPEN_HOLD_MS                0u
#define PUT2_TURNTABLE_WAIT_MS                 200u
#define PUT2_PICK_OPEN_WAIT_MS                 200u
#define PUT2_CLAMP_WAIT_MS                     200u
#define PUT2_CLAMP_HOLD_MS                     200u
#define PUT2_HOME_WAIT_MS                      450u
#define PUT2_STAGE_INTERVAL_MS                 100u
#define PUT2_FINAL_OPEN_HOLD_MS                0u
#define PUT3_TURNTABLE_WAIT_MS                 380u
#define PUT3_PICK_OPEN_WAIT_MS                 200u
#define PUT3_CLAMP_WAIT_MS                     200u
#define PUT3_CLAMP_HOLD_MS                     200u
#define PUT3_HOME_WAIT_MS                      500u
#define PUT3_STAGE_INTERVAL_MS                 100u
#define PUT3_FINAL_OPEN_HOLD_MS                0u
#define PUT4_TURNTABLE_WAIT_MS                 500u
#define PUT4_PICK_OPEN_WAIT_MS                 300u
#define PUT4_CLAMP_WAIT_MS                     300u
#define PUT4_CLAMP_HOLD_MS                     300u
#define PUT4_HOME_WAIT_MS                      500u
#define PUT4_STAGE_INTERVAL_MS                 100u
#define PUT4_FINAL_OPEN_HOLD_MS                0u
#define PUT5_TURNTABLE_WAIT_MS                 850u
#define PUT5_PICK_OPEN_WAIT_MS                 380u
#define PUT5_CLAMP_WAIT_MS                     380u
#define PUT5_CLAMP_HOLD_MS                     200u
#define PUT5_HOME_WAIT_MS                      850u
#define PUT5_STAGE_INTERVAL_MS                 100u
#define PUT5_FINAL_OPEN_HOLD_MS                0u
#define TASK2_PLACE1_RETRACT_WAIT_MS                   200u
#define TASK2_PLACE1_TURNTABLE_WAIT_MS                 400u
#define TASK2_PLACE1_RELEASE_WAIT_MS                   380u
#define TASK2_PLACE1_CLAMP_WAIT_MS                     380u
#define TASK2_PLACE1_CLAMP_HOLD_MS                     200u
#define TASK2_PLACE1_EXTEND_WAIT_MS                    200u
#define TASK2_PLACE1_HOME_WAIT_MS                      400u
#define TASK2_PLACE1_OPEN_WAIT_MS                      200u
#define TASK2_PLACE1_OPEN_HOLD_MS                      200u
#define TASK2_PLACE2_RETRACT_WAIT_MS                   200u
#define TASK2_PLACE2_TURNTABLE_WAIT_MS                 600u
#define TASK2_PLACE2_RELEASE_WAIT_MS                   380u
#define TASK2_PLACE2_CLAMP_WAIT_MS                     200u
#define TASK2_PLACE2_CLAMP_HOLD_MS                     300u
#define TASK2_PLACE2_EXTEND_WAIT_MS                    250u
#define TASK2_PLACE2_HOME_WAIT_MS                      600u
#define TASK2_PLACE2_OPEN_WAIT_MS                      200u
#define TASK2_PLACE2_OPEN_HOLD_MS                      200u
#define TASK2_PLACE3_RETRACT_WAIT_MS                   200u
#define TASK2_PLACE3_TURNTABLE_WAIT_MS                 750u
#define TASK2_PLACE3_RELEASE_WAIT_MS                   380u
#define TASK2_PLACE3_CLAMP_WAIT_MS                     380u
#define TASK2_PLACE3_CLAMP_HOLD_MS                     200u
#define TASK2_PLACE3_EXTEND_WAIT_MS                    250u
#define TASK2_PLACE3_HOME_WAIT_MS                      700u
#define TASK2_PLACE3_OPEN_WAIT_MS                      100u
#define TASK2_PLACE3_OPEN_HOLD_MS                      100u
typedef struct {
    uint32_t open_wait_ms;
    uint32_t clamp_wait_ms;
    uint32_t clamp_hold_ms;
    uint32_t turntable_wait_ms;
    uint32_t release_wait_ms;
    uint32_t release_hold_ms;
    uint32_t home_wait_ms;
    uint32_t open_after_home_wait_ms;
} ArmGrubTiming;

typedef struct {
    uint32_t turntable_wait_ms;
    uint32_t pick_open_wait_ms;
    uint32_t clamp_wait_ms;
    uint32_t clamp_hold_ms;
    uint32_t home_wait_ms;
    uint32_t stage_interval_ms;
    uint32_t final_open_hold_ms;
} ArmPutTiming;

typedef struct {
    uint32_t retract_wait_ms;
    uint32_t turntable_wait_ms;
    uint32_t release_wait_ms;
    uint32_t clamp_wait_ms;
    uint32_t clamp_hold_ms;
    uint32_t extend_wait_ms;
    uint32_t home_wait_ms;
    uint32_t open_wait_ms;
    uint32_t open_hold_ms;
} ArmTask2PlaceTiming;

static const ArmGrubTiming grub1_timing = {
    GRUB1_OPEN_WAIT_MS, GRUB1_CLAMP_WAIT_MS, GRUB1_CLAMP_HOLD_MS, GRUB1_TURNTABLE_WAIT_MS, GRUB1_RELEASE_WAIT_MS, GRUB1_RELEASE_HOLD_MS, GRUB1_HOME_WAIT_MS, GRUB1_OPEN_AFTER_HOME_WAIT_MS
};

static const ArmGrubTiming grub2_timing = {
    GRUB2_OPEN_WAIT_MS, GRUB2_CLAMP_WAIT_MS, GRUB2_CLAMP_HOLD_MS, GRUB2_TURNTABLE_WAIT_MS, GRUB2_RELEASE_WAIT_MS, GRUB2_RELEASE_HOLD_MS, GRUB2_HOME_WAIT_MS, GRUB2_OPEN_AFTER_HOME_WAIT_MS
};

static const ArmGrubTiming grub3_timing = {
    GRUB3_OPEN_WAIT_MS, GRUB3_CLAMP_WAIT_MS, GRUB3_CLAMP_HOLD_MS, GRUB3_TURNTABLE_WAIT_MS, GRUB3_RELEASE_WAIT_MS, GRUB3_RELEASE_HOLD_MS, GRUB3_HOME_WAIT_MS, GRUB3_OPEN_AFTER_HOME_WAIT_MS
};

static const ArmGrubTiming grub4_timing = {
    GRUB4_OPEN_WAIT_MS, GRUB4_CLAMP_WAIT_MS, GRUB4_CLAMP_HOLD_MS, GRUB4_TURNTABLE_WAIT_MS, GRUB4_RELEASE_WAIT_MS, GRUB4_RELEASE_HOLD_MS, GRUB4_HOME_WAIT_MS, GRUB4_OPEN_AFTER_HOME_WAIT_MS
};

static const ArmGrubTiming grub5_timing = {
    GRUB5_OPEN_WAIT_MS, GRUB5_CLAMP_WAIT_MS, GRUB5_CLAMP_HOLD_MS, GRUB5_TURNTABLE_WAIT_MS, GRUB5_RELEASE_WAIT_MS, GRUB5_RELEASE_HOLD_MS, GRUB5_HOME_WAIT_MS, GRUB5_OPEN_AFTER_HOME_WAIT_MS
};

static const ArmGrubTiming task2_grub1_timing = {
    TASK2_GRUB1_OPEN_WAIT_MS, TASK2_GRUB1_CLAMP_WAIT_MS, TASK2_GRUB1_CLAMP_HOLD_MS, TASK2_GRUB1_TURNTABLE_WAIT_MS, TASK2_GRUB1_RELEASE_WAIT_MS, TASK2_GRUB1_RELEASE_HOLD_MS, TASK2_GRUB1_HOME_WAIT_MS, TASK2_GRUB1_OPEN_AFTER_HOME_WAIT_MS
};
static const ArmGrubTiming task2_grub3_timing = {
    TASK2_GRUB3_OPEN_WAIT_MS, TASK2_GRUB3_CLAMP_WAIT_MS, TASK2_GRUB3_CLAMP_HOLD_MS, TASK2_GRUB3_TURNTABLE_WAIT_MS, TASK2_GRUB3_RELEASE_WAIT_MS, TASK2_GRUB3_RELEASE_HOLD_MS, TASK2_GRUB3_HOME_WAIT_MS, TASK2_GRUB3_OPEN_AFTER_HOME_WAIT_MS
};
static const ArmGrubTiming task2_grub5_timing = {
    TASK2_GRUB5_OPEN_WAIT_MS, TASK2_GRUB5_CLAMP_WAIT_MS, TASK2_GRUB5_CLAMP_HOLD_MS, TASK2_GRUB5_TURNTABLE_WAIT_MS, TASK2_GRUB5_RELEASE_WAIT_MS, TASK2_GRUB5_RELEASE_HOLD_MS, TASK2_GRUB5_HOME_WAIT_MS, TASK2_GRUB5_OPEN_AFTER_HOME_WAIT_MS
};

static const ArmPutTiming put1_timing = {
    PUT1_TURNTABLE_WAIT_MS, PUT1_PICK_OPEN_WAIT_MS, PUT1_CLAMP_WAIT_MS, PUT1_CLAMP_HOLD_MS, PUT1_HOME_WAIT_MS, PUT1_STAGE_INTERVAL_MS, PUT1_FINAL_OPEN_HOLD_MS
};

static const ArmPutTiming put2_timing = {
    PUT2_TURNTABLE_WAIT_MS, PUT2_PICK_OPEN_WAIT_MS, PUT2_CLAMP_WAIT_MS, PUT2_CLAMP_HOLD_MS, PUT2_HOME_WAIT_MS, PUT2_STAGE_INTERVAL_MS, PUT2_FINAL_OPEN_HOLD_MS
};

static const ArmPutTiming put3_timing = {
    PUT3_TURNTABLE_WAIT_MS, PUT3_PICK_OPEN_WAIT_MS, PUT3_CLAMP_WAIT_MS, PUT3_CLAMP_HOLD_MS, PUT3_HOME_WAIT_MS, PUT3_STAGE_INTERVAL_MS, PUT3_FINAL_OPEN_HOLD_MS
};

static const ArmPutTiming put4_timing = {
    PUT4_TURNTABLE_WAIT_MS, PUT4_PICK_OPEN_WAIT_MS, PUT4_CLAMP_WAIT_MS, PUT4_CLAMP_HOLD_MS, PUT4_HOME_WAIT_MS, PUT4_STAGE_INTERVAL_MS, PUT4_FINAL_OPEN_HOLD_MS
};

static const ArmPutTiming put5_timing = {
    PUT5_TURNTABLE_WAIT_MS, PUT5_PICK_OPEN_WAIT_MS, PUT5_CLAMP_WAIT_MS, PUT5_CLAMP_HOLD_MS, PUT5_HOME_WAIT_MS, PUT5_STAGE_INTERVAL_MS, PUT5_FINAL_OPEN_HOLD_MS
};

static const ArmTask2PlaceTiming task2_place1_timing = {
    TASK2_PLACE1_RETRACT_WAIT_MS, TASK2_PLACE1_TURNTABLE_WAIT_MS, TASK2_PLACE1_RELEASE_WAIT_MS, TASK2_PLACE1_CLAMP_WAIT_MS, TASK2_PLACE1_CLAMP_HOLD_MS, TASK2_PLACE1_EXTEND_WAIT_MS, TASK2_PLACE1_HOME_WAIT_MS, TASK2_PLACE1_OPEN_WAIT_MS, TASK2_PLACE1_OPEN_HOLD_MS
};
static const ArmTask2PlaceTiming task2_place2_timing = {
    TASK2_PLACE2_RETRACT_WAIT_MS, TASK2_PLACE2_TURNTABLE_WAIT_MS, TASK2_PLACE2_RELEASE_WAIT_MS, TASK2_PLACE2_CLAMP_WAIT_MS, TASK2_PLACE2_CLAMP_HOLD_MS, TASK2_PLACE2_EXTEND_WAIT_MS, TASK2_PLACE2_HOME_WAIT_MS, TASK2_PLACE2_OPEN_WAIT_MS, TASK2_PLACE2_OPEN_HOLD_MS
};
static const ArmTask2PlaceTiming task2_place3_timing = {
    TASK2_PLACE3_RETRACT_WAIT_MS, TASK2_PLACE3_TURNTABLE_WAIT_MS, TASK2_PLACE3_RELEASE_WAIT_MS, TASK2_PLACE3_CLAMP_WAIT_MS, TASK2_PLACE3_CLAMP_HOLD_MS, TASK2_PLACE3_EXTEND_WAIT_MS, TASK2_PLACE3_HOME_WAIT_MS, TASK2_PLACE3_OPEN_WAIT_MS, TASK2_PLACE3_OPEN_HOLD_MS
};

#define RPI_FRAME_PAYLOAD_SIZE               8u
#define FIRMWARE_DIAGNOSTIC_VERSION         181u

#define RPI_CMD_X_DISTANCE                    1u
#define RPI_CMD_Y_DISTANCE                    2u
#define RPI_CMD_ROTATE                        3u
#define RPI_CMD_X_MM                          5u
#define RPI_CMD_Y_MM                          6u
#define RPI_CMD_GRUB_DISC1                    7u
#define RPI_CMD_GRUB_DISC2                    8u
#define RPI_CMD_GRUB_DISC3                    9u
#define RPI_CMD_GRUB_DISC4                   10u
#define RPI_CMD_GRUB_DISC5                   12u
#define RPI_CMD_PUT_DISC1                    14u
#define RPI_CMD_PUT_DISC2                    15u
#define RPI_CMD_PUT_DISC3                    16u
#define RPI_CMD_PUT_DISC4                    17u
#define RPI_CMD_PUT_DISC5                    18u
#define RPI_CMD_RESET_ANGLE                  19u
#define RPI_CMD_MOTOR_ALIGN                  20u
#define RPI_CMD_TASK2_PUT_1TO1               33u
#define RPI_CMD_TASK2_PUT_2TO2               34u
#define RPI_CMD_TASK2_PUT_3TO3               35u
#define RPI_CMD_SERVO1_EXTEND                36u
#define RPI_CMD_TASK2_GRUB_CUP1              42u
#define RPI_CMD_TASK2_GRUB_CUP2              43u
#define RPI_CMD_TASK2_GRUB_CUP3              44u
#define RPI_CMD_LIFT_UP                      48u
#define RPI_CMD_LIFT_DOWN                    49u
#define RPI_CMD_SERVO1_RETRACT               52u
#define RPI_CMD_LIFT_MID                     53u
#define RPI_CMD_LIFT_178MM                   58u
#define RPI_CMD_LIFT_186MM                   59u
#define RPI_CMD_CLAW_OPEN                    70u
#define RPI_CMD_CLAW_CLOSE                   71u
#define RPI_CMD_SOFTWARE_INIT                72u
#define RPI_CMD_TUNING_SERVO1_PULSE          80u
#define RPI_CMD_TUNING_SERVO2_PULSE          81u
#define RPI_CMD_TUNING_SERVO3_PULSE          82u
#define RPI_CMD_TUNING_LIFT_HEIGHT           83u
#define RPI_CMD_CAMERA_TARGET_HIGH           84u
#define RPI_CMD_CAMERA_TARGET_LOW            85u
#define RPI_CMD_SERVO3_HOME                  86u
#define RPI_CMD_TASK2_CHASSIS_PROFILE       87u
#define RPI_CMD_TASK1_CHASSIS_PROFILE       88u
#define RPI_CMD_ARM_START_BUTTON            89u
#define RPI_CMD_TEST_WHEEL                  90u
#define RPI_CMD_TEST_STOP                   91u
#define RPI_CMD_TEST_LIFT                   92u
#define RPI_CMD_TEST_PROBE                  93u
#define RPI_CMD_TEST_SERVO                  94u
#define RPI_CMD_TEST_BUTTON                 95u
#define RPI_CMD_GYRO_DIAGNOSTIC             126u

#define COMMAND_RESULT_INVALID                0u
#define COMMAND_RESULT_DONE                   1u
#define COMMAND_RESULT_TEST_STOPPED           4u
#define COMMAND_RESULT_BUTTON_LOW             5u
#define COMMAND_RESULT_BUTTON_HIGH            6u
#define COMMAND_RESULT_TEST_SUPPORTED         7u

#define TEST_MAX_WHEEL_PULSES               3200u
#define TEST_MIN_HALF_PERIOD_US                1u

#define RPI_STATUS_START_BUTTON               3u
#define START_BUTTON_DEBOUNCE_MS              20u

#define LIFT_MIN_HEIGHT_MM                    0u
#define LIFT_GROUND_PICK_HEIGHT_MM            14u
#define LIFT_GROUND_PLACE_HEIGHT_MM          11u
/* Final field placement from discs 2/4; absolute software heights, not offsets. */
#define LIFT_DISC2_STACK_PLACE_HEIGHT_MM     56u
#define LIFT_DISC4_STACK_PLACE_HEIGHT_MM     56u
#define LIFT_PICK_HEIGHT_MM                 170u
#define LIFT_PLACE_HEIGHT_MM                163u
#define LIFT_TRANSFER_HEIGHT_MM             250u
#define LIFT_MID_HEIGHT_MM                  170u
#define LIFT_MAX_HEIGHT_MM                  360u
#define LIFT_178_HEIGHT_MM                  178u
#define LIFT_186_HEIGHT_MM                  186u
#define LIFT_TASK2_DISC_PLACE_HEIGHT_MM     163u
#define LIFT_TASK2_TRANSFER_HEIGHT_MM       270u
#define LIFT_TASK2_DISC_HEIGHT_MM           169u
#define LIFT_TASK2_RETURN_CLEARANCE_MM      221u
#define LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM  55u
#define LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM 37u
#define LIFT_TASK2_THIRD_PLACE_HEIGHT_MM     19u
#define LIFT_CAMERA_TARGET_HIGH_HEIGHT_MM   36u
#define LIFT_CAMERA_TARGET_LOW_HEIGHT_MM    18u
#define LIFT_PULSES_PER_MM                   64u
#define LIFT_STEP_HALF_PERIOD_US             15u
/* Shared by disc pickup/placement, field/stack and podium placement. */
#define LIFT_GROUND_SLOW_APPROACH_DISTANCE_MM 40u
#define LIFT_GROUND_SLOW_STEP_HALF_PERIOD_US 120u
#define LIFT_DIRECTION_SETTLE_MS             20u

#define SERVO_TUNING_MIN_PULSE_US           500u
#define SERVO_TUNING_MAX_PULSE_US          2500u

#define ALIGN_MAX_CORRECTIONS                 3u
#define ALIGN_MAX_ERROR_DEG                  20.0f
#define ALIGN_DONE_ERROR_DEG                  0.8f
#define ALIGN_GYRO_TIMEOUT_MS                500u
#define ALIGN_SETTLE_MS                      120u
#define GYRO_RX_POLL_INTERVAL_US              10u
#define GYRO_RX_TIMEOUT_POLLS \
    ((ALIGN_GYRO_TIMEOUT_MS * 1000u) / GYRO_RX_POLL_INTERVAL_US)
#define GYRO_FRAME_SIZE                       11u
#define GYRO_FRAME_HEADER                   0x55u
#define GYRO_ANGLE_FRAME_TYPE               0x53u
#define GYRO_RUNTIME_BAUD_RATE              9600u
#define GYRO_RESET_SETTLE_MS                  250u
#define GYRO_TX_TIMEOUT_POLLS              100000u
#define GYRO_DIAGNOSTIC_BAUD_COUNT             8u
#define GYRO_DIAGNOSTIC_PORT_COUNT             4u
#define GYRO_RAW_PROBE_DURATION_MS           2000u
#define GYRO_RAW_PROBE_SAMPLE_US               10u
#define GYRO_RAW_PROBE_SAMPLES \
    ((GYRO_RAW_PROBE_DURATION_MS * 1000u) / GYRO_RAW_PROBE_SAMPLE_US)

/* Historical 1/16-microstep calibration. Do not retune this constant until
 * the current non-linear wheel response and translation drift are fixed. */
#define CHASSIS_LINEAR_PULSES_NUMERATOR   13403u
#define CHASSIS_LINEAR_PULSES_DENOM_MM     1000u
#define CHASSIS_ROTATE_180_PULSES          7790u

#define CHASSIS_STEP_PINS  (GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9)

typedef enum {
    CHASSIS_VECTOR_FORWARD_BACK = 0,
    CHASSIS_VECTOR_LEFT_RIGHT,
    CHASSIS_VECTOR_ROTATE
} ChassisVector;

typedef enum {
    RX_WAIT_HEADER = 0,
    RX_READ_PAYLOAD,
    RX_WAIT_TAIL
} RpiRxState;

typedef struct {
    uint16_t received_bytes;
    uint8_t header_bytes;
    uint8_t valid_frames;
    uint8_t angle_frames;
    uint8_t checksum_errors;
    uint8_t has_angle;
    int16_t last_angle_cdeg;
} GyroTrafficStats;

typedef enum {
    GYRO_PORT_UART4_PC10_PC11 = 0,
    GYRO_PORT_USART2_PD5_PD6,
    GYRO_PORT_USART3_PD8_PD9,
    GYRO_PORT_UART5_PC12_PD2
} GyroDiagnosticPort;

/* Compatibility globals referenced by legacy modules in the project. */
int16_t base_angle = 0;
int16_t code1 = 0;
int16_t code2 = 0;
static uint16_t lift_height_mm = LIFT_MIN_HEIGHT_MM;
/* Preserve the fractional mm if a component test is stopped mid-step. */
static uint16_t lift_test_remainder_pulses = 0u;
static uint8_t chassis_task2_profile_enabled = 0u;
static uint8_t start_button_armed = 0u;
static uint8_t start_button_released = 0u;
/* Once PE6 startup is armed, reject manual servo tuning commands. */
static uint8_t servo_tuning_locked = 0u;
static USART_TypeDef *gyro_active_uart = UART4;
static float gyro_reference_angle_deg = 0.0f;
static uint8_t gyro_reference_valid = 0u;
static const uint32_t gyro_diagnostic_baud_rates[GYRO_DIAGNOSTIC_BAUD_COUNT] = {
    2400u,
    4800u,
    9600u,
    19200u,
    38400u,
    57600u,
    115200u,
    230400u
};

static uint32_t Chassis_AbsInt32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

static int16_t Chassis_ReadInt16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static uint16_t Chassis_ReadUInt16(uint8_t high, uint8_t low)
{
    return (uint16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static int8_t Chassis_ReadInt8(uint8_t value)
{
    return (int8_t)value;
}

static uint32_t Chassis_LinearPulses(uint32_t millimetres)
{
    return (millimetres * CHASSIS_LINEAR_PULSES_NUMERATOR +
            (CHASSIS_LINEAR_PULSES_DENOM_MM / 2u)) /
           CHASSIS_LINEAR_PULSES_DENOM_MM;
}

static uint32_t Chassis_RotationPulses(uint32_t degrees)
{
    return (degrees * CHASSIS_ROTATE_180_PULSES + 90u) / 180u;
}

static int16_t Chassis_NormalizeHeading(int32_t heading)
{
    while (heading > 180) {
        heading -= 360;
    }
    while (heading < -180) {
        heading += 360;
    }
    return (int16_t)heading;
}

static float Chassis_HeadingError(float target, float actual)
{
    float error = target - actual;

    while (error > 180.0f) {
        error -= 360.0f;
    }
    while (error < -180.0f) {
        error += 360.0f;
    }
    return error;
}

static void Arm_RestoreDiscAlignment(void)
{
    TIM_SetCompare2(TIM5, SERVO1_DISC_ALIGN_PULSE_US);
    delay_ms(SERVO1_SETTLE_MS);
}

static void Arm_PWM_Init(void)
{
    Servo1_Init();
    Servo2_Init();
    Servo3_Init();

    /* Keep claw and bottom turntable uncommanded at power-on. */
    TIM_SetCompare2(TIM5, SERVO1_DISC_ALIGN_PULSE_US);
    TIM_SetCompare3(TIM5, 0u);
    TIM_SetCompare4(TIM5, 0u);
}

static void Arm_SetPowerOnTurntableHome(void)
{
    TIM_SetCompare4(TIM5, SERVO3_HOME_PULSE_US);
    delay_ms(SERVO3_SETTLE_MS);
    TIM_SetCompare3(TIM5, SERVO2_DISC_PICK_OPEN_PULSE_US);
    delay_ms(SERVO2_SETTLE_MS);
}

static void Arm_SetPowerOnReadyPose(void)
{
    TIM_SetCompare3(TIM5, SERVO2_BOTTOM_OPEN_PULSE_US);
    TIM_SetCompare2(TIM5, SERVO1_DISC_ALIGN_PULSE_US);
    delay_ms(SERVO_POWER_ON_SETTLE_MS);
}

static void Lift_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC,
                           ENABLE);

    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;

    gpio.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOC, &gpio);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    GPIO_ResetBits(GPIOC, GPIO_Pin_5);
    lift_height_mm = LIFT_MIN_HEIGHT_MM;
}

static void Lift_MoveToHeightRawAtHalfPeriod(uint16_t target_height_mm,
                                             uint32_t step_half_period_us)
{
    uint16_t distance_mm;
    uint32_t pulses;
    uint32_t pulse;

    if (target_height_mm > LIFT_MAX_HEIGHT_MM ||
        target_height_mm == lift_height_mm) {
        return;
    }

    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    GPIO_ResetBits(GPIOC, GPIO_Pin_5);

    if (target_height_mm > lift_height_mm) {
        GPIO_SetBits(GPIOA, GPIO_Pin_4);
        distance_mm = target_height_mm - lift_height_mm;
    } else {
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
        distance_mm = lift_height_mm - target_height_mm;
    }

    delay_ms(LIFT_DIRECTION_SETTLE_MS);
    pulses = (uint32_t)distance_mm * LIFT_PULSES_PER_MM;
    for (pulse = 0u; pulse < pulses; pulse++) {
        GPIO_SetBits(GPIOB, GPIO_Pin_10);
        delay_us(step_half_period_us);
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);
        delay_us(step_half_period_us);
    }

    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    lift_height_mm = target_height_mm;
}

static void Lift_MoveToHeightAtHalfPeriod(uint16_t target_height_mm,
                                          uint32_t step_half_period_us)
{
    Lift_MoveToHeightRawAtHalfPeriod(target_height_mm, step_half_period_us);
}

static void Lift_MoveToHeight(uint16_t target_height_mm)
{
    Lift_MoveToHeightAtHalfPeriod(target_height_mm,
                                  LIFT_STEP_HALF_PERIOD_US);
}

static void Lift_MoveToPlaceHeightSlow(uint16_t target_height_mm)
{
    uint16_t slow_start_height_mm;

    if (target_height_mm > LIFT_MAX_HEIGHT_MM) {
        return;
    }

    slow_start_height_mm =
        target_height_mm + LIFT_GROUND_SLOW_APPROACH_DISTANCE_MM;

    if (lift_height_mm <= target_height_mm) {
        Lift_MoveToHeight(target_height_mm);
        return;
    }

    if (slow_start_height_mm > LIFT_MAX_HEIGHT_MM) {
        slow_start_height_mm = LIFT_MAX_HEIGHT_MM;
    }
    if (lift_height_mm > slow_start_height_mm) {
        Lift_MoveToHeight(slow_start_height_mm);
    }
    if (lift_height_mm > target_height_mm) {
        Lift_MoveToHeightAtHalfPeriod(
            target_height_mm,
            LIFT_GROUND_SLOW_STEP_HALF_PERIOD_US);
    }
}

static void Arm_RunSoftwareInit(void)
{
    Arm_SetPowerOnTurntableHome();
    Lift_MoveToHeight(LIFT_PICK_HEIGHT_MM);
    Arm_SetPowerOnReadyPose();
}

static void Arm_Servo1Move(uint16_t pulse_us)
{
    TIM_SetCompare2(TIM5, pulse_us);
    delay_ms(SERVO1_SETTLE_MS);
}

static void Arm_Servo2Move(uint16_t pulse_us)
{
    TIM_SetCompare3(TIM5, pulse_us);
    delay_ms(SERVO2_SETTLE_MS);
}

static void Arm_Servo3Move(uint16_t pulse_us)
{
    TIM_SetCompare4(TIM5, pulse_us);
    delay_ms(SERVO3_SETTLE_MS);
}

static void Arm_Servo1MoveWait(uint16_t pulse_us, uint32_t wait_ms)
{
    TIM_SetCompare2(TIM5, pulse_us);
    delay_ms(wait_ms);
}

static void Arm_Servo2MoveWait(uint16_t pulse_us, uint32_t wait_ms)
{
    TIM_SetCompare3(TIM5, pulse_us);
    delay_ms(wait_ms);
}

static void Arm_Servo3MoveWait(uint16_t pulse_us, uint32_t wait_ms)
{
    TIM_SetCompare4(TIM5, pulse_us);
    delay_ms(wait_ms);
}

static const ArmGrubTiming *Arm_GrubTimingFromMode(uint8_t mode)
{
    switch (mode) {
    case RPI_CMD_GRUB_DISC1:
        return &grub1_timing;
    case RPI_CMD_GRUB_DISC2:
        return &grub2_timing;
    case RPI_CMD_GRUB_DISC3:
        return &grub3_timing;
    case RPI_CMD_GRUB_DISC4:
        return &grub4_timing;
    case RPI_CMD_GRUB_DISC5:
        return &grub5_timing;
    default:
        return &task2_grub1_timing;
    }
}

static const ArmPutTiming *Arm_PutTimingFromMode(uint8_t mode)
{
    switch (mode) {
    case RPI_CMD_PUT_DISC1:
        return &put1_timing;
    case RPI_CMD_PUT_DISC2:
        return &put2_timing;
    case RPI_CMD_PUT_DISC3:
        return &put3_timing;
    case RPI_CMD_PUT_DISC4:
        return &put4_timing;
    case RPI_CMD_PUT_DISC5:
        return &put5_timing;
    default:
        return &put1_timing;
    }
}

static const ArmGrubTiming *Arm_Task2GrubTimingFromMode(uint8_t mode)
{
    switch (mode) {
    case RPI_CMD_TASK2_GRUB_CUP1:
        return &task2_grub1_timing;
    case RPI_CMD_TASK2_GRUB_CUP2:
        return &task2_grub3_timing;
    case RPI_CMD_TASK2_GRUB_CUP3:
        return &task2_grub5_timing;
    default:
        return &task2_grub1_timing;
    }
}

static const ArmTask2PlaceTiming *Arm_Task2PlaceTimingFromMode(uint8_t mode)
{
    switch (mode) {
    case RPI_CMD_TASK2_PUT_1TO1:
        return &task2_place1_timing;
    case RPI_CMD_TASK2_PUT_2TO2:
        return &task2_place2_timing;
    case RPI_CMD_TASK2_PUT_3TO3:
        return &task2_place3_timing;
    default:
        return &task2_place1_timing;
    }
}

static uint8_t Arm_DiscPulseFromMode(uint8_t mode, uint16_t *disc_pulse_us)
{
    switch (mode) {
    case RPI_CMD_GRUB_DISC1:
    case RPI_CMD_PUT_DISC1:
        *disc_pulse_us = SERVO3_DISC1_PULSE_US;
        return 1u;

    case RPI_CMD_GRUB_DISC2:
    case RPI_CMD_PUT_DISC2:
        *disc_pulse_us = SERVO3_DISC2_PULSE_US;
        return 1u;

    case RPI_CMD_GRUB_DISC3:
    case RPI_CMD_PUT_DISC3:
        *disc_pulse_us = SERVO3_DISC3_PULSE_US;
        return 1u;

    case RPI_CMD_GRUB_DISC4:
    case RPI_CMD_PUT_DISC4:
        *disc_pulse_us = SERVO3_PUT_DISC4_PULSE_US;
        return 1u;

    case RPI_CMD_GRUB_DISC5:
    case RPI_CMD_PUT_DISC5:
        *disc_pulse_us = SERVO3_DISC5_PULSE_US;
        return 1u;

    default:
        return 0u;
    }
}

static uint8_t Arm_IsPutMode(uint8_t mode)
{
    return (mode == RPI_CMD_PUT_DISC1 ||
            mode == RPI_CMD_PUT_DISC2 ||
            mode == RPI_CMD_PUT_DISC3 ||
            mode == RPI_CMD_PUT_DISC4 ||
            mode == RPI_CMD_PUT_DISC5) ? 1u : 0u;
}

static uint8_t Arm_IsCubeDiscMode(uint8_t mode)
{
    return (mode == RPI_CMD_GRUB_DISC2 || mode == RPI_CMD_GRUB_DISC4 ||
            mode == RPI_CMD_PUT_DISC2 || mode == RPI_CMD_PUT_DISC4) ? 1u : 0u;
}

static uint16_t Arm_FieldPlaceHeightFromMode(uint8_t mode)
{
    switch (mode) {
    case RPI_CMD_PUT_DISC2:
        return LIFT_DISC2_STACK_PLACE_HEIGHT_MM;
    case RPI_CMD_PUT_DISC4:
        return LIFT_DISC4_STACK_PLACE_HEIGHT_MM;
    default:
        return LIFT_GROUND_PLACE_HEIGHT_MM;
    }
}

static uint8_t Arm_Task2ParamsFromMode(uint8_t mode,
                                       uint16_t *disc_pulse_us,
                                       uint16_t *podium_height_mm)
{
    switch (mode) {
    case RPI_CMD_TASK2_PUT_1TO1:
        *disc_pulse_us = SERVO3_DISC1_PULSE_US;
        *podium_height_mm = LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM;
        return 1u;

    case RPI_CMD_TASK2_PUT_2TO2:
        *disc_pulse_us = SERVO3_DISC3_PULSE_US;
        *podium_height_mm = LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM;
        return 1u;

    case RPI_CMD_TASK2_PUT_3TO3:
        *disc_pulse_us = SERVO3_DISC5_PULSE_US;
        *podium_height_mm = LIFT_TASK2_THIRD_PLACE_HEIGHT_MM;
        return 1u;

    default:
        return 0u;
    }
}

static uint8_t Arm_Task2GrubPulseFromMode(uint8_t mode,
                                          uint16_t *disc_pulse_us)
{
    switch (mode) {
    case RPI_CMD_TASK2_GRUB_CUP1:
        *disc_pulse_us = SERVO3_DISC1_PULSE_US;
        return 1u;

    case RPI_CMD_TASK2_GRUB_CUP2:
        *disc_pulse_us = SERVO3_DISC3_PULSE_US;
        return 1u;

    case RPI_CMD_TASK2_GRUB_CUP3:
        *disc_pulse_us = SERVO3_DISC5_PULSE_US;
        return 1u;

    default:
        return 0u;
    }
}

static uint8_t Arm_Task2HeightsValid(void)
{
    return (LIFT_TASK2_TRANSFER_HEIGHT_MM <= LIFT_MAX_HEIGHT_MM &&
            LIFT_TASK2_DISC_PLACE_HEIGHT_MM <= LIFT_MAX_HEIGHT_MM &&
            LIFT_TASK2_DISC_HEIGHT_MM <= LIFT_MAX_HEIGHT_MM &&
            LIFT_TASK2_RETURN_CLEARANCE_MM <= LIFT_MAX_HEIGHT_MM &&
            LIFT_TASK2_TRANSFER_HEIGHT_MM > LIFT_TASK2_DISC_PLACE_HEIGHT_MM &&
            LIFT_TASK2_TRANSFER_HEIGHT_MM > LIFT_TASK2_DISC_HEIGHT_MM &&
            LIFT_TASK2_RETURN_CLEARANCE_MM > LIFT_TASK2_DISC_HEIGHT_MM &&
            LIFT_TASK2_RETURN_CLEARANCE_MM >= LIFT_TASK2_CHAMPION_PLACE_HEIGHT_MM &&
            LIFT_TASK2_RETURN_CLEARANCE_MM >= LIFT_TASK2_RUNNER_UP_PLACE_HEIGHT_MM &&
            LIFT_TASK2_RETURN_CLEARANCE_MM >= LIFT_TASK2_THIRD_PLACE_HEIGHT_MM) ? 1u : 0u;
}

static void Arm_RunTask2Place(uint16_t disc_pulse_us,
                              uint16_t podium_height_mm,
                              const ArmTask2PlaceTiming *timing)
{
    Arm_Servo1MoveWait(SERVO1_DISC_ALIGN_PULSE_US, timing->retract_wait_ms);
    Lift_MoveToHeight(LIFT_TASK2_TRANSFER_HEIGHT_MM);
    Arm_RestoreDiscAlignment();
    Arm_Servo3MoveWait(disc_pulse_us, timing->turntable_wait_ms);
    Arm_Servo2MoveWait(SERVO2_DISC_PICK_OPEN_PULSE_US, timing->release_wait_ms);

    Lift_MoveToHeight(LIFT_TASK2_DISC_HEIGHT_MM);
    Arm_Servo2MoveWait(SERVO2_CLAMP_PULSE_US, timing->clamp_wait_ms);
    delay_ms(timing->clamp_hold_ms);

    Lift_MoveToHeight(LIFT_TASK2_RETURN_CLEARANCE_MM);
    Arm_Servo1MoveWait(SERVO1_TASK2_EXTEND_PULSE_US, timing->extend_wait_ms);
    Arm_Servo3MoveWait(SERVO3_HOME_PULSE_US, timing->home_wait_ms);

    Lift_MoveToPlaceHeightSlow(podium_height_mm);
    Arm_Servo2MoveWait(SERVO2_BOTTOM_OPEN_PULSE_US, timing->open_wait_ms);
    delay_ms(timing->open_hold_ms);

    Lift_MoveToHeight(LIFT_PICK_HEIGHT_MM);
}

static void Arm_RunGrubToDisc(uint16_t disc_pulse_us,
                              uint16_t ground_clamp_pulse_us,
                              uint16_t disc_place_height_mm,
                              uint16_t transfer_height_mm,
                              const ArmGrubTiming *timing)
{
    Arm_Servo2MoveWait(SERVO2_BOTTOM_OPEN_PULSE_US, timing->open_wait_ms);

    /* Restore the default reach if task2 previously left Servo1 extended. */
    Arm_Servo1Move(SERVO1_DISC_ALIGN_PULSE_US);
    Lift_MoveToHeight(LIFT_GROUND_PICK_HEIGHT_MM);
    Arm_Servo2MoveWait(ground_clamp_pulse_us, timing->clamp_wait_ms);
    Lift_MoveToHeight(transfer_height_mm);
    delay_ms(timing->clamp_hold_ms);

    Arm_RestoreDiscAlignment();
    Arm_Servo3MoveWait(disc_pulse_us, timing->turntable_wait_ms);

    Lift_MoveToPlaceHeightSlow(disc_place_height_mm);
    Arm_Servo2MoveWait(SERVO2_DISC_RELEASE_PULSE_US, timing->release_wait_ms);
    delay_ms(timing->release_hold_ms);

    Lift_MoveToHeight(transfer_height_mm);
    Arm_Servo3MoveWait(SERVO3_HOME_PULSE_US, timing->home_wait_ms);
    Arm_Servo2MoveWait(SERVO2_BOTTOM_OPEN_PULSE_US, timing->open_after_home_wait_ms);
    Lift_MoveToHeight(LIFT_PICK_HEIGHT_MM);
}

static void Arm_RunPutFromDisc(uint16_t disc_pulse_us,
                               uint16_t disc_open_pulse_us,
                               uint16_t disc_clamp_pulse_us,
                               uint16_t field_place_height_mm,
                               const ArmPutTiming *timing)
{
    Lift_MoveToHeight(LIFT_TRANSFER_HEIGHT_MM);
    Arm_RestoreDiscAlignment();
    Arm_Servo3MoveWait(disc_pulse_us, timing->turntable_wait_ms);
    Arm_Servo2MoveWait(disc_open_pulse_us, timing->pick_open_wait_ms);

    Lift_MoveToPlaceHeightSlow(LIFT_PLACE_HEIGHT_MM);
    Arm_Servo2MoveWait(disc_clamp_pulse_us, timing->clamp_wait_ms);
    delay_ms(timing->clamp_hold_ms);

    Lift_MoveToHeight(LIFT_TRANSFER_HEIGHT_MM);
    Arm_Servo3MoveWait(SERVO3_HOME_PULSE_US, timing->home_wait_ms);

    Lift_MoveToPlaceHeightSlow(field_place_height_mm);
    Arm_Servo2MoveWait(SERVO2_BOTTOM_OPEN_PULSE_US, SERVO2_SETTLE_MS);
    delay_ms(timing->stage_interval_ms);
    delay_ms(timing->final_open_hold_ms);
    Lift_MoveToHeight(LIFT_PICK_HEIGHT_MM);
}

static void Chassis_UART1_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_USART1,
                           ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_StructInit(&uart);
    uart.USART_BaudRate = 115200u;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &uart);
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    USART_Cmd(USART1, ENABLE);
}

static void Chassis_UART1_SendByte(uint8_t value)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
    }
    USART_SendData(USART1, value);
}

static void Chassis_SendStatus(uint8_t status)
{
    Chassis_UART1_SendByte(0xFFu);
    Chassis_UART1_SendByte(status);
    Chassis_UART1_SendByte(0xFEu);
}

static void StartButton_Arm(void)
{
    start_button_armed = 1u;
    start_button_released =
        (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6) == Bit_RESET) ? 1u : 0u;
    servo_tuning_locked = 1u;
}

static uint8_t StartButton_TryConsumePress(void)
{
    if (start_button_armed == 0u) {
        return 0u;
    }

    if (start_button_released == 0u) {
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6) == Bit_RESET) {
            delay_ms(START_BUTTON_DEBOUNCE_MS);
            if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6) == Bit_RESET) {
                start_button_released = 1u;
            }
        }
        return 0u;
    }

    if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6) != Bit_RESET) {
        delay_ms(START_BUTTON_DEBOUNCE_MS);
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6) != Bit_RESET) {
            start_button_armed = 0u;
            start_button_released = 0u;
            return 1u;
        }
    }

    return 0u;
}

static uint8_t Chassis_TrySendReadyByte(uint8_t *index)
{
    static const uint8_t ready_frame[3] = {0xFFu, 0x02u, 0xFEu};

    if (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
        return 0u;
    }

    USART_SendData(USART1, ready_frame[*index]);
    (*index)++;
    if (*index >= 3u) {
        *index = 0u;
        return 1u;
    }

    return 0u;
}

static uint8_t Chassis_TryReadFrame(uint8_t payload[RPI_FRAME_PAYLOAD_SIZE])
{
    static RpiRxState state = RX_WAIT_HEADER;
    static uint8_t index = 0u;
    static uint8_t staging[RPI_FRAME_PAYLOAD_SIZE];
    uint8_t value;
    uint8_t i;

    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
        value = (uint8_t)USART_ReceiveData(USART1);

        if (state == RX_WAIT_HEADER) {
            if (value == 0xFFu) {
                index = 0u;
                state = RX_READ_PAYLOAD;
            }
        } else if (state == RX_READ_PAYLOAD) {
            staging[index++] = value;
            if (index >= RPI_FRAME_PAYLOAD_SIZE) {
                state = RX_WAIT_TAIL;
            }
        } else {
            if (value == 0xFEu) {
                for (i = 0u; i < RPI_FRAME_PAYLOAD_SIZE; i++) {
                    payload[i] = staging[i];
                }
                state = RX_WAIT_HEADER;
                index = 0u;
                return 1u;
            }

            /* Recover immediately if the bad tail is the next header. */
            if (value == 0xFFu) {
                index = 0u;
                state = RX_READ_PAYLOAD;
            } else {
                index = 0u;
                state = RX_WAIT_HEADER;
            }
        }
    }

    return 0u;
}

static void Chassis_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_GPIOG,
                           ENABLE);

    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;

    /* STEP3/4/7/8: PC6, PC7, PC8, PC9. */
    gpio.GPIO_Pin = CHASSIS_STEP_PINS;
    GPIO_Init(GPIOC, &gpio);

    /* DIR3/4: PG7, PG8. */
    gpio.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_Init(GPIOG, &gpio);

    /* DIR7: PA8. */
    gpio.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA, &gpio);

    /* DIR8: PC1. */
    gpio.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOC, &gpio);

    /* Shared enables: PC5 and PD7, active low. */
    gpio.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOD, &gpio);

    GPIO_ResetBits(GPIOC, CHASSIS_STEP_PINS | GPIO_Pin_5);
    GPIO_ResetBits(GPIOD, GPIO_Pin_7);
}

/* Component tests use only the selected STEP/DIR pair. Shared EN pins stay
 * enabled so stopping a wheel does not release the lift's holding torque. */
static void HardwareTest_Stop(void)
{
    GPIO_ResetBits(GPIOC, CHASSIS_STEP_PINS);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    TIM_SetCompare2(TIM5, 0u);
    TIM_SetCompare3(TIM5, 0u);
    TIM_SetCompare4(TIM5, 0u);
}

/* Poll at 100 us intervals even at the slowest test speed. Without this,
 * USART1's polled receiver would overrun and lose a STOP frame. */
static uint8_t HardwareTest_Wait(uint16_t duration_us)
{
    uint8_t payload[RPI_FRAME_PAYLOAD_SIZE];
    uint16_t slice_us;

    while (duration_us != 0u) {
        if (Chassis_TryReadFrame(payload) != 0u) {
            if (payload[0] == RPI_CMD_TEST_STOP) {
                HardwareTest_Stop();
                return 0u;
            }
            /* No other operation may start while a component is moving. */
            Chassis_SendStatus(COMMAND_RESULT_INVALID);
        }
        slice_us = (duration_us > 100u) ? 100u : duration_us;
        delay_us(slice_us);
        duration_us -= slice_us;
    }
    return 1u;
}

static uint8_t HardwareTest_Wheel(const uint8_t payload[RPI_FRAME_PAYLOAD_SIZE])
{
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    uint16_t step_pin;
    uint16_t pulses = Chassis_ReadUInt16(payload[3], payload[4]);
    uint16_t half_period_us = Chassis_ReadUInt16(payload[5], payload[6]);
    uint16_t pulse;

    if (payload[2] > 1u || pulses == 0u || pulses > TEST_MAX_WHEEL_PULSES ||
        half_period_us < TEST_MIN_HALF_PERIOD_US || payload[7] != 0u) {
        return COMMAND_RESULT_INVALID;
    }
    switch (payload[1]) {
    case 3u: dir_port = GPIOG; dir_pin = GPIO_Pin_7; step_pin = GPIO_Pin_6; break;
    case 4u: dir_port = GPIOG; dir_pin = GPIO_Pin_8; step_pin = GPIO_Pin_7; break;
    case 7u: dir_port = GPIOA; dir_pin = GPIO_Pin_8; step_pin = GPIO_Pin_8; break;
    case 8u: dir_port = GPIOC; dir_pin = GPIO_Pin_1; step_pin = GPIO_Pin_9; break;
    default: return COMMAND_RESULT_INVALID;
    }
    GPIO_WriteBit(dir_port, dir_pin, (payload[2] != 0u) ? Bit_SET : Bit_RESET);
    if (HardwareTest_Wait(20000u) == 0u) {
        return COMMAND_RESULT_TEST_STOPPED;
    }
    for (pulse = 0u; pulse < pulses; pulse++) {
        GPIO_SetBits(GPIOC, step_pin);
        if (HardwareTest_Wait(half_period_us) == 0u) {
            return COMMAND_RESULT_TEST_STOPPED;
        }
        GPIO_ResetBits(GPIOC, step_pin);
        if (HardwareTest_Wait(half_period_us) == 0u) {
            return COMMAND_RESULT_TEST_STOPPED;
        }
    }
    return COMMAND_RESULT_DONE;
}

static uint8_t HardwareTest_Lift(const uint8_t payload[RPI_FRAME_PAYLOAD_SIZE])
{
    uint16_t target_mm = Chassis_ReadUInt16(payload[1], payload[2]);
    uint16_t half_period_us = Chassis_ReadUInt16(payload[3], payload[4]);
    uint32_t position = (uint32_t)lift_height_mm * LIFT_PULSES_PER_MM +
                        lift_test_remainder_pulses;
    uint32_t target = (uint32_t)target_mm * LIFT_PULSES_PER_MM;
    uint32_t distance = (target > position) ? target - position : position - target;
    uint8_t ascending = (target > position) ? 1u : 0u;
    uint8_t result = COMMAND_RESULT_DONE;

    if (target_mm > LIFT_MAX_HEIGHT_MM ||
        half_period_us < TEST_MIN_HALF_PERIOD_US ||
        payload[5] != 0u || payload[6] != 0u || payload[7] != 0u) {
        return COMMAND_RESULT_INVALID;
    }
    if (distance == 0u) {
        return COMMAND_RESULT_DONE;
    }
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, (ascending != 0u) ? Bit_SET : Bit_RESET);
    if (HardwareTest_Wait(20000u) == 0u) {
        return COMMAND_RESULT_TEST_STOPPED;
    }
    while (position != target) {
        GPIO_SetBits(GPIOB, GPIO_Pin_10);
        /* Count the issued rising edge, including an interrupted high phase. */
        if (ascending != 0u) { position++; } else { position--; }
        if (HardwareTest_Wait(half_period_us) == 0u) {
            result = COMMAND_RESULT_TEST_STOPPED;
            break;
        }
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);
        if (HardwareTest_Wait(half_period_us) == 0u) {
            result = COMMAND_RESULT_TEST_STOPPED;
            break;
        }
    }
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    lift_height_mm = (uint16_t)(position / LIFT_PULSES_PER_MM);
    lift_test_remainder_pulses = (uint16_t)(position % LIFT_PULSES_PER_MM);
    return result;
}

static uint8_t HardwareTest_IsLiftCommand(uint8_t mode)
{
    switch (mode) {
    case RPI_CMD_GRUB_DISC1: case RPI_CMD_GRUB_DISC2: case RPI_CMD_GRUB_DISC3:
    case RPI_CMD_GRUB_DISC4: case RPI_CMD_GRUB_DISC5:
    case RPI_CMD_PUT_DISC1: case RPI_CMD_PUT_DISC2: case RPI_CMD_PUT_DISC3:
    case RPI_CMD_PUT_DISC4: case RPI_CMD_PUT_DISC5:
    case RPI_CMD_TASK2_PUT_1TO1: case RPI_CMD_TASK2_PUT_2TO2: case RPI_CMD_TASK2_PUT_3TO3:
    case RPI_CMD_TASK2_GRUB_CUP1: case RPI_CMD_TASK2_GRUB_CUP2: case RPI_CMD_TASK2_GRUB_CUP3:
    case RPI_CMD_LIFT_UP: case RPI_CMD_LIFT_DOWN: case RPI_CMD_LIFT_MID:
    case RPI_CMD_LIFT_178MM: case RPI_CMD_LIFT_186MM: case RPI_CMD_SOFTWARE_INIT:
    case RPI_CMD_TUNING_LIFT_HEIGHT: case RPI_CMD_CAMERA_TARGET_HIGH:
    case RPI_CMD_CAMERA_TARGET_LOW: return 1u;
    default: return 0u;
    }
}

static void Chassis_SetDirection(ChassisVector vector, uint8_t reverse)
{
    if (vector == CHASSIS_VECTOR_FORWARD_BACK) {
        /* Modes 1/5: physical forward/back with the current wheel wiring. */
        if (reverse == 0u) {
            GPIO_ResetBits(GPIOG, GPIO_Pin_7); /* MOTOR3 / physical RR = 0 */
            GPIO_ResetBits(GPIOG, GPIO_Pin_8); /* MOTOR4 / physical FR = 0 */
            GPIO_SetBits(GPIOA, GPIO_Pin_8);   /* MOTOR7 / physical FL = 1 */
            GPIO_SetBits(GPIOC, GPIO_Pin_1);   /* MOTOR8 / physical RL = 1 */
        } else {
            GPIO_SetBits(GPIOG, GPIO_Pin_7);
            GPIO_SetBits(GPIOG, GPIO_Pin_8);
            GPIO_ResetBits(GPIOA, GPIO_Pin_8);
            GPIO_ResetBits(GPIOC, GPIO_Pin_1);
        }
    } else if (vector == CHASSIS_VECTOR_LEFT_RIGHT) {
        /* Modes 2/6: physical lateral motion with the current wheel wiring. */
        if (reverse == 0u) {
            GPIO_ResetBits(GPIOG, GPIO_Pin_7);
            GPIO_SetBits(GPIOG, GPIO_Pin_8);
            GPIO_SetBits(GPIOA, GPIO_Pin_8);
            GPIO_ResetBits(GPIOC, GPIO_Pin_1);
        } else {
            GPIO_SetBits(GPIOG, GPIO_Pin_7);
            GPIO_ResetBits(GPIOG, GPIO_Pin_8);
            GPIO_ResetBits(GPIOA, GPIO_Pin_8);
            GPIO_SetBits(GPIOC, GPIO_Pin_1);
        }
    } else {
        /* With MOTOR7=FL, MOTOR4=FR, MOTOR8=RL and MOTOR3=RR, equal DIR
         * levels form the two pure-rotation directions. */
        if (reverse == 0u) {
            GPIO_ResetBits(GPIOG, GPIO_Pin_7);
            GPIO_ResetBits(GPIOG, GPIO_Pin_8);
            GPIO_ResetBits(GPIOA, GPIO_Pin_8);
            GPIO_ResetBits(GPIOC, GPIO_Pin_1);
        } else {
            GPIO_SetBits(GPIOG, GPIO_Pin_7);
            GPIO_SetBits(GPIOG, GPIO_Pin_8);
            GPIO_SetBits(GPIOA, GPIO_Pin_8);
            GPIO_SetBits(GPIOC, GPIO_Pin_1);
        }
    }
}

static void Chassis_PulseAtHalfPeriod(uint32_t pulses,
                                      uint32_t half_period_us)
{
    uint32_t pulse;

    for (pulse = 0u; pulse < pulses; pulse++) {
        GPIO_SetBits(GPIOC, CHASSIS_STEP_PINS);
        delay_us(half_period_us);
        GPIO_ResetBits(GPIOC, CHASSIS_STEP_PINS);
        delay_us(half_period_us);
    }
}

static void Chassis_PulseWithRamp(uint32_t pulses,
                                  uint32_t cruise_half_period_us,
                                  uint32_t start_stop_half_period_us,
                                  uint32_t ramp_pulses);

static void Chassis_Pulse(uint32_t pulses)
{
    /* Rotation and heading correction share the conservative ramp in both
     * chassis profiles. An abrupt 70 us start can make one wheel lose steps,
     * which creates translation and invalidates the angle calibration. */
    Chassis_PulseWithRamp(
        pulses,
        CHASSIS_ROTATE_SLOW_STEP_HALF_PERIOD_US,
        CHASSIS_TASK2_STOP_HALF_PERIOD_US,
        CHASSIS_TASK2_RAMP_PULSES);
}

static void Chassis_PulseWithRamp(uint32_t pulses,
                                  uint32_t cruise_half_period_us,
                                  uint32_t start_stop_half_period_us,
                                  uint32_t ramp_pulses)
{
    uint32_t ramp_span;
    uint32_t period_delta;
    uint32_t ramp_position;
    uint32_t half_period_us;
    uint32_t pulse;

    if (pulses < 4u ||
        ramp_pulses < 2u ||
        start_stop_half_period_us <= cruise_half_period_us) {
        Chassis_PulseAtHalfPeriod(pulses, cruise_half_period_us);
        return;
    }

    if (ramp_pulses > (pulses / 2u)) {
        ramp_pulses = pulses / 2u;
    }
    ramp_span = ramp_pulses - 1u;
    period_delta = start_stop_half_period_us - cruise_half_period_us;

    for (pulse = 0u; pulse < pulses; pulse++) {
        if (pulse < ramp_pulses) {
            ramp_position = pulse;
            half_period_us = start_stop_half_period_us -
                             ((period_delta * ramp_position) / ramp_span);
        } else if (pulse >= (pulses - ramp_pulses)) {
            ramp_position = pulses - pulse - 1u;
            half_period_us = start_stop_half_period_us -
                             ((period_delta * ramp_position) / ramp_span);
        } else {
            half_period_us = cruise_half_period_us;
        }

        GPIO_SetBits(GPIOC, CHASSIS_STEP_PINS);
        delay_us(half_period_us);
        GPIO_ResetBits(GPIOC, CHASSIS_STEP_PINS);
        delay_us(half_period_us);
    }
}

static void Chassis_Move(ChassisVector vector,
                         int32_t amount,
                         uint8_t amount_is_degrees,
                         uint16_t linear_unit_mm)
{
    uint32_t magnitude;
    uint32_t pulses;
    uint32_t linear_distance_mm = 0u;

    if (amount == 0) {
        return;
    }

    magnitude = Chassis_AbsInt32(amount);
    if (amount_is_degrees != 0u) {
        pulses = Chassis_RotationPulses(magnitude);
    } else {
        linear_distance_mm = magnitude * linear_unit_mm;
        pulses = Chassis_LinearPulses(linear_distance_mm);
    }

    Chassis_SetDirection(vector, (amount < 0) ? 1u : 0u);
    delay_ms(CHASSIS_DIRECTION_SETTLE_MS);
    if (amount_is_degrees != 0u) {
        Chassis_Pulse(pulses);
    } else if (linear_unit_mm == 1u) {
        /* Modes 5/6 always use their active profile's dedicated fine speed. */
        Chassis_PulseAtHalfPeriod(pulses,
            (chassis_task2_profile_enabled != 0u) ?
                CHASSIS_TASK2_FINE_STEP_HALF_PERIOD_US :
                CHASSIS_FINE_STEP_HALF_PERIOD_US);
    } else if (linear_distance_mm >= CHASSIS_RAMP_MIN_DISTANCE_MM) {
        if (chassis_task2_profile_enabled != 0u) {
            Chassis_PulseWithRamp(
                pulses,
                CHASSIS_TASK2_LONG_STEP_HALF_PERIOD_US,
                CHASSIS_TASK2_STOP_HALF_PERIOD_US,
                CHASSIS_TASK2_RAMP_PULSES);
        } else {
            Chassis_PulseWithRamp(
                pulses,
                CHASSIS_STEP_HALF_PERIOD_US,
                CHASSIS_RAMP_START_HALF_PERIOD_US,
                CHASSIS_RAMP_PULSES);
        }
    } else if (chassis_task2_profile_enabled != 0u) {
        Chassis_PulseWithRamp(
            pulses,
            CHASSIS_SLOW_SHORT_STEP_HALF_PERIOD_US,
            CHASSIS_TASK2_STOP_HALF_PERIOD_US,
            CHASSIS_TASK2_RAMP_PULSES);
    } else {
        Chassis_PulseAtHalfPeriod(pulses,
                                  CHASSIS_SHORT_STEP_HALF_PERIOD_US);
    }
    GPIO_ResetBits(GPIOC, CHASSIS_STEP_PINS);
}

static void Gyro_SelectPort(GyroDiagnosticPort port)
{
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    if (port == GYRO_PORT_UART4_PC10_PC11) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

        gpio.GPIO_Pin = GPIO_Pin_10;
        gpio.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOC, &gpio);
        gpio.GPIO_Pin = GPIO_Pin_11;
        gpio.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIOC, &gpio);

        gyro_active_uart = UART4;
    } else if (port == GYRO_PORT_USART2_PD5_PD6) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,
                               ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

        gpio.GPIO_Pin = GPIO_Pin_5;
        gpio.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOD, &gpio);
        gpio.GPIO_Pin = GPIO_Pin_6;
        gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOD, &gpio);

        gyro_active_uart = USART2;
    } else if (port == GYRO_PORT_USART3_PD8_PD9) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,
                               ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);

        gpio.GPIO_Pin = GPIO_Pin_8;
        gpio.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOD, &gpio);
        gpio.GPIO_Pin = GPIO_Pin_9;
        gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOD, &gpio);

        gyro_active_uart = USART3;
    } else {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD,
                               ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

        gpio.GPIO_Pin = GPIO_Pin_12;
        gpio.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOC, &gpio);
        gpio.GPIO_Pin = GPIO_Pin_2;
        gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOD, &gpio);

        gyro_active_uart = UART5;
    }
}

static void Gyro_DiscardPendingBytes(void)
{
    uint32_t status;

    do {
        status = gyro_active_uart->SR;
        (void)gyro_active_uart->DR;
    } while ((status & USART_FLAG_RXNE) != 0u);
}

static uint8_t Gyro_WaitForAngle(float *angle_degrees)
{
    uint8_t frame[GYRO_FRAME_SIZE];
    uint8_t index = 0u;
    uint8_t checksum;
    uint8_t byte;
    uint8_t i;
    uint16_t yaw_raw;
    uint32_t poll;

    Gyro_DiscardPendingBytes();

    for (poll = 0u; poll < GYRO_RX_TIMEOUT_POLLS; poll++) {
        while (USART_GetFlagStatus(gyro_active_uart, USART_FLAG_RXNE) != RESET) {
            byte = (uint8_t)USART_ReceiveData(gyro_active_uart);

            if (index == 0u) {
                if (byte == GYRO_FRAME_HEADER) {
                    frame[index++] = byte;
                }
                continue;
            }

            frame[index++] = byte;
            if (index == 2u && frame[1] != GYRO_ANGLE_FRAME_TYPE) {
                if (frame[1] == GYRO_FRAME_HEADER) {
                    frame[0] = GYRO_FRAME_HEADER;
                    index = 1u;
                } else {
                    index = 0u;
                }
                continue;
            }

            if (index >= GYRO_FRAME_SIZE) {
                checksum = 0u;
                for (i = 0u; i < (GYRO_FRAME_SIZE - 1u); i++) {
                    checksum = (uint8_t)(checksum + frame[i]);
                }

                if (checksum == frame[GYRO_FRAME_SIZE - 1u]) {
                    yaw_raw = (uint16_t)frame[6] |
                              ((uint16_t)frame[7] << 8);
                    *angle_degrees = ((float)(int16_t)yaw_raw / 32768.0f) *
                                     180.0f;
                    global_angle = *angle_degrees;
                    return 1u;
                }
                index = 0u;
            }
        }
        delay_us(GYRO_RX_POLL_INTERVAL_US);
    }
    return 0u;
}

static void Gyro_SetBaudRate(uint32_t baud_rate)
{
    USART_InitTypeDef uart;

    USART_Cmd(gyro_active_uart, DISABLE);
    USART_StructInit(&uart);
    uart.USART_BaudRate = baud_rate;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(gyro_active_uart, &uart);
    USART_ITConfig(gyro_active_uart, USART_IT_RXNE, DISABLE);
    USART_Cmd(gyro_active_uart, ENABLE);
    Gyro_DiscardPendingBytes();
}

static uint8_t Gyro_SendByte(uint8_t byte)
{
    uint32_t timeout = GYRO_TX_TIMEOUT_POLLS;

    while (USART_GetFlagStatus(gyro_active_uart, USART_FLAG_TXE) == RESET) {
        if (timeout-- == 0u) {
            return 0u;
        }
    }
    USART_SendData(gyro_active_uart, byte);
    return 1u;
}

static uint8_t Gyro_ResetHeading(void)
{
    static const uint8_t reset_command[5] = {
        0xFFu, 0xAAu, 0x76u, 0x00u, 0x00u
    };
    uint32_t timeout;
    uint8_t i;

    for (i = 0u; i < sizeof(reset_command); i++) {
        if (Gyro_SendByte(reset_command[i]) == 0u) {
            return 0u;
        }
    }

    timeout = GYRO_TX_TIMEOUT_POLLS;
    while (USART_GetFlagStatus(gyro_active_uart, USART_FLAG_TC) == RESET) {
        if (timeout-- == 0u) {
            return 0u;
        }
    }
    delay_ms(GYRO_RESET_SETTLE_MS);
    Gyro_DiscardPendingBytes();
    return 1u;
}

static uint8_t Gyro_CaptureReference(void)
{
    float angle;

    if (Gyro_WaitForAngle(&angle) == 0u) {
        gyro_reference_valid = 0u;
        return 0u;
    }

    gyro_reference_angle_deg = angle;
    gyro_reference_valid = 1u;
    global_angle = 0.0f;
    base_angle = 0;
    return 1u;
}

static uint8_t Gyro_ReadRelativeHeading(float *heading_degrees)
{
    float raw_angle;

    if (gyro_reference_valid == 0u ||
        Gyro_WaitForAngle(&raw_angle) == 0u) {
        return 0u;
    }

    /* Positive chassis rotation produces decreasing yaw on this mounting. */
    *heading_degrees = Chassis_HeadingError(gyro_reference_angle_deg,
                                             raw_angle);
    global_angle = *heading_degrees;
    return 1u;
}

static uint8_t Gyro_InitializeRuntime(void)
{
    Gyro_SelectPort(GYRO_PORT_UART4_PC10_PC11);
    Gyro_SetBaudRate(GYRO_RUNTIME_BAUD_RATE);
    delay_ms(GYRO_RESET_SETTLE_MS);

    if (Gyro_ResetHeading() == 0u) {
        return 0u;
    }
    return Gyro_CaptureReference();
}

static void Gyro_PrepareRuntimeReceiver(void)
{
    Gyro_SelectPort(GYRO_PORT_UART4_PC10_PC11);
    Gyro_SetBaudRate(GYRO_RUNTIME_BAUD_RATE);
    delay_ms(GYRO_RESET_SETTLE_MS);
}

static void Gyro_CountTraffic(uint32_t baud_rate, GyroTrafficStats *stats)
{
    uint8_t frame[GYRO_FRAME_SIZE];
    uint8_t index = 0u;
    uint8_t byte;
    uint8_t checksum;
    uint8_t i;
    int16_t yaw_raw;
    uint32_t poll;

    stats->received_bytes = 0u;
    stats->header_bytes = 0u;
    stats->valid_frames = 0u;
    stats->angle_frames = 0u;
    stats->checksum_errors = 0u;
    stats->has_angle = 0u;
    stats->last_angle_cdeg = 0;
    Gyro_SetBaudRate(baud_rate);

    for (poll = 0u; poll < GYRO_RX_TIMEOUT_POLLS; poll++) {
        while (USART_GetFlagStatus(gyro_active_uart, USART_FLAG_RXNE) != RESET) {
            byte = (uint8_t)USART_ReceiveData(gyro_active_uart);
            if (stats->received_bytes < 65535u) {
                stats->received_bytes++;
            }
            if (byte == GYRO_FRAME_HEADER && stats->header_bytes < 255u) {
                stats->header_bytes++;
            }

            if (index == 0u) {
                if (byte == GYRO_FRAME_HEADER) {
                    frame[index++] = byte;
                }
                continue;
            }

            frame[index++] = byte;
            if (index >= GYRO_FRAME_SIZE) {
                checksum = 0u;
                for (i = 0u; i < (GYRO_FRAME_SIZE - 1u); i++) {
                    checksum = (uint8_t)(checksum + frame[i]);
                }
                if (checksum == frame[GYRO_FRAME_SIZE - 1u]) {
                    if (stats->valid_frames < 255u) {
                        stats->valid_frames++;
                    }
                    if (frame[1] == GYRO_ANGLE_FRAME_TYPE) {
                        if (stats->angle_frames < 255u) {
                            stats->angle_frames++;
                        }
                        yaw_raw = (int16_t)((uint16_t)frame[6] |
                                            ((uint16_t)frame[7] << 8));
                        stats->last_angle_cdeg =
                            (int16_t)(((int32_t)yaw_raw * 18000L) / 32768L);
                        stats->has_angle = 1u;
                    }
                } else if (stats->checksum_errors < 255u) {
                    stats->checksum_errors++;
                }
                index = 0u;
            }
        }
        delay_us(GYRO_RX_POLL_INTERVAL_US);
    }
}

static void Gyro_SendTrafficStats(uint8_t port_index,
                                  uint8_t baud_index,
                                  const GyroTrafficStats *stats)
{
    Chassis_UART1_SendByte(0xFFu);
    Chassis_UART1_SendByte(0x05u);
    Chassis_UART1_SendByte(FIRMWARE_DIAGNOSTIC_VERSION);
    Chassis_UART1_SendByte(port_index);
    Chassis_UART1_SendByte(baud_index);
    Chassis_UART1_SendByte((uint8_t)(stats->received_bytes >> 8));
    Chassis_UART1_SendByte((uint8_t)stats->received_bytes);
    Chassis_UART1_SendByte(stats->header_bytes);
    Chassis_UART1_SendByte(stats->valid_frames);
    Chassis_UART1_SendByte(stats->angle_frames);
    Chassis_UART1_SendByte(stats->checksum_errors);
    Chassis_UART1_SendByte(0xFEu);
}

static void Gyro_SendAngleSample(uint8_t port_index,
                                 uint8_t baud_index,
                                 int16_t angle_cdeg)
{
    uint16_t raw_angle = (uint16_t)angle_cdeg;

    Chassis_UART1_SendByte(0xFFu);
    Chassis_UART1_SendByte(0x06u);
    Chassis_UART1_SendByte(FIRMWARE_DIAGNOSTIC_VERSION);
    Chassis_UART1_SendByte(port_index);
    Chassis_UART1_SendByte(baud_index);
    Chassis_UART1_SendByte((uint8_t)(raw_angle >> 8));
    Chassis_UART1_SendByte((uint8_t)raw_angle);
    Chassis_UART1_SendByte(0xFEu);
}

static void Gyro_ProbeRxPin(GyroDiagnosticPort port,
                            uint8_t *level_flags,
                            uint16_t *transitions)
{
    GPIO_TypeDef *rx_port;
    uint16_t rx_pin;
    uint32_t sample;
    uint8_t level;
    uint8_t last_level;

    Gyro_SelectPort(port);
    if (port == GYRO_PORT_UART4_PC10_PC11) {
        rx_port = GPIOC;
        rx_pin = GPIO_Pin_11;
    } else if (port == GYRO_PORT_USART2_PD5_PD6) {
        rx_port = GPIOD;
        rx_pin = GPIO_Pin_6;
    } else if (port == GYRO_PORT_USART3_PD8_PD9) {
        rx_port = GPIOD;
        rx_pin = GPIO_Pin_9;
    } else {
        rx_port = GPIOD;
        rx_pin = GPIO_Pin_2;
    }

    *level_flags = 0u;
    *transitions = 0u;
    last_level = (GPIO_ReadInputDataBit(rx_port, rx_pin) != Bit_RESET) ?
                 1u : 0u;

    for (sample = 0u; sample < GYRO_RAW_PROBE_SAMPLES; sample++) {
        level = (GPIO_ReadInputDataBit(rx_port, rx_pin) != Bit_RESET) ?
                1u : 0u;
        if (level != 0u) {
            *level_flags |= 0x02u;
        } else {
            *level_flags |= 0x01u;
        }
        if (level != last_level) {
            if (*transitions < 65535u) {
                (*transitions)++;
            }
            last_level = level;
        }
        delay_us(GYRO_RAW_PROBE_SAMPLE_US);
    }
}

static void Gyro_SendRawProbe(uint8_t port_index,
                              uint8_t level_flags,
                              uint16_t transitions)
{
    Chassis_UART1_SendByte(0xFFu);
    Chassis_UART1_SendByte(0x07u);
    Chassis_UART1_SendByte(FIRMWARE_DIAGNOSTIC_VERSION);
    Chassis_UART1_SendByte(port_index);
    Chassis_UART1_SendByte(level_flags);
    Chassis_UART1_SendByte((uint8_t)(transitions >> 8));
    Chassis_UART1_SendByte((uint8_t)transitions);
    Chassis_UART1_SendByte(0xFEu);
}

static void Gyro_SendDiagnostic(void)
{
    GyroTrafficStats stats;
    uint32_t detected_baud_rate = 0u;
    GyroDiagnosticPort detected_port = GYRO_PORT_UART4_PC10_PC11;
    uint8_t port_index;
    uint8_t baud_index;
    uint8_t level_flags;
    uint16_t transitions;

    for (port_index = 0u;
         port_index < GYRO_DIAGNOSTIC_PORT_COUNT;
         port_index++) {
        Gyro_ProbeRxPin((GyroDiagnosticPort)port_index,
                        &level_flags,
                        &transitions);
        Gyro_SendRawProbe(port_index, level_flags, transitions);
    }

    for (port_index = 0u;
         port_index < GYRO_DIAGNOSTIC_PORT_COUNT;
         port_index++) {
        Gyro_SelectPort((GyroDiagnosticPort)port_index);
        for (baud_index = 0u;
             baud_index < GYRO_DIAGNOSTIC_BAUD_COUNT;
             baud_index++) {
            Gyro_CountTraffic(gyro_diagnostic_baud_rates[baud_index], &stats);
            if (detected_baud_rate == 0u && stats.angle_frames != 0u) {
                detected_baud_rate = gyro_diagnostic_baud_rates[baud_index];
                detected_port = (GyroDiagnosticPort)port_index;
            }
            Gyro_SendTrafficStats(port_index, baud_index, &stats);
            if (stats.has_angle != 0u) {
                Gyro_SendAngleSample(port_index,
                                     baud_index,
                                     stats.last_angle_cdeg);
            }
        }
    }

    Gyro_SelectPort(detected_port);
    if (detected_baud_rate != 0u) {
        Gyro_SetBaudRate(detected_baud_rate);
    } else {
        Gyro_SetBaudRate(9600u);
    }
}

static void Chassis_AlignRotate(float correction_degrees)
{
    float magnitude;
    uint32_t milli_degrees;
    uint32_t pulses;

    magnitude = (correction_degrees < 0.0f) ?
                -correction_degrees : correction_degrees;
    milli_degrees = (uint32_t)(magnitude * 1000.0f + 0.5f);
    pulses = (milli_degrees * CHASSIS_ROTATE_180_PULSES + 90000u) / 180000u;

    if (pulses == 0u) {
        return;
    }

    /* After the wheel-layout correction, +20 deg measured +20.51 deg.
     * Apply target-minus-actual with the same sign as normal rotation. */
    Chassis_SetDirection(CHASSIS_VECTOR_ROTATE,
                         (correction_degrees < 0.0f) ? 1u : 0u);
    delay_ms(CHASSIS_DIRECTION_SETTLE_MS);
    Chassis_Pulse(pulses);
    GPIO_ResetBits(GPIOC, CHASSIS_STEP_PINS);
}

static uint8_t Chassis_MotorAlign(void)
{
    uint8_t correction;
    float current_angle;
    float error;
    float magnitude;

    Gyro_PrepareRuntimeReceiver();

    for (correction = 0u; correction < ALIGN_MAX_CORRECTIONS; correction++) {
        if (Gyro_ReadRelativeHeading(&current_angle) == 0u) {
            return 0u;
        }

        error = Chassis_HeadingError((float)base_angle, current_angle);
        magnitude = (error < 0.0f) ? -error : error;

        if (magnitude <= ALIGN_DONE_ERROR_DEG) {
            return 1u;
        }
        if (magnitude > ALIGN_MAX_ERROR_DEG) {
            return 0u;
        }

        Chassis_AlignRotate(error);
        delay_ms(ALIGN_SETTLE_MS);
    }

    if (Gyro_ReadRelativeHeading(&current_angle) == 0u) {
        return 0u;
    }
    error = Chassis_HeadingError((float)base_angle, current_angle);
    magnitude = (error < 0.0f) ? -error : error;
    return (magnitude <= ALIGN_DONE_ERROR_DEG) ? 1u : 0u;
}

static uint8_t Chassis_HandleCommand(const uint8_t payload[RPI_FRAME_PAYLOAD_SIZE])
{
    uint16_t disc_pulse_us;
    uint16_t ground_clamp_pulse_us;
    uint16_t disc_open_pulse_us;
    uint16_t disc_clamp_pulse_us;
    uint16_t field_place_height_mm;
    uint16_t podium_height_mm;
    uint16_t tuning_value;

    if (servo_tuning_locked != 0u &&
        (payload[0] == RPI_CMD_TUNING_SERVO1_PULSE ||
         payload[0] == RPI_CMD_TUNING_SERVO2_PULSE ||
         payload[0] == RPI_CMD_TUNING_SERVO3_PULSE ||
         payload[0] == RPI_CMD_TEST_SERVO ||
         payload[0] == RPI_CMD_TEST_WHEEL ||
         payload[0] == RPI_CMD_TEST_LIFT)) {
        return COMMAND_RESULT_INVALID;
    }
    /* An interrupted lift test must first finish at a whole-mm test target;
     * do not silently discard its partial position in competition actions. */
    if (lift_test_remainder_pulses != 0u && HardwareTest_IsLiftCommand(payload[0]) != 0u) {
        return COMMAND_RESULT_INVALID;
    }

    switch (payload[0]) {
    case RPI_CMD_TEST_WHEEL:
        return HardwareTest_Wheel(payload);
    case RPI_CMD_TEST_STOP:
        HardwareTest_Stop();
        return COMMAND_RESULT_TEST_STOPPED;
    case RPI_CMD_TEST_LIFT:
        return HardwareTest_Lift(payload);
    case RPI_CMD_TEST_PROBE:
        return COMMAND_RESULT_TEST_SUPPORTED;
    case RPI_CMD_TEST_BUTTON:
        return (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6) == Bit_RESET) ?
            COMMAND_RESULT_BUTTON_LOW : COMMAND_RESULT_BUTTON_HIGH;
    case RPI_CMD_TEST_SERVO:
        tuning_value = Chassis_ReadUInt16(payload[2], payload[3]);
        if (tuning_value < SERVO_TUNING_MIN_PULSE_US ||
            tuning_value > SERVO_TUNING_MAX_PULSE_US ||
            payload[4] != 0u || payload[5] != 0u ||
            payload[6] != 0u || payload[7] != 0u) {
            return COMMAND_RESULT_INVALID;
        }
        switch (payload[1]) {
        case 1u: TIM_SetCompare2(TIM5, tuning_value); break;
        case 2u: TIM_SetCompare3(TIM5, tuning_value); break;
        case 3u: TIM_SetCompare4(TIM5, tuning_value); break;
        default: return COMMAND_RESULT_INVALID;
        }
        /* ACK means PWM applied; servo position has no feedback sensor. */
        return COMMAND_RESULT_DONE;
    case RPI_CMD_X_DISTANCE:
        Chassis_Move(CHASSIS_VECTOR_FORWARD_BACK,
                     Chassis_ReadInt16(payload[1], payload[2]),
                     0u,
                     10u);
        return 1u;

    case RPI_CMD_Y_DISTANCE:
        Chassis_Move(CHASSIS_VECTOR_LEFT_RIGHT,
                     Chassis_ReadInt16(payload[1], payload[2]),
                     0u,
                     10u);
        return 1u;

    case RPI_CMD_ROTATE:
    {
        int8_t angle = Chassis_ReadInt8(payload[3]);
        Chassis_Move(CHASSIS_VECTOR_ROTATE,
                     angle,
                     1u,
                     0u);
        base_angle = Chassis_NormalizeHeading((int32_t)base_angle + angle);
        return 1u;
    }

    case RPI_CMD_X_MM:
        Chassis_Move(CHASSIS_VECTOR_FORWARD_BACK,
                     Chassis_ReadInt8(payload[1]),
                     0u,
                     1u);
        return 1u;

    case RPI_CMD_Y_MM:
        Chassis_Move(CHASSIS_VECTOR_LEFT_RIGHT,
                     Chassis_ReadInt8(payload[2]),
                     0u,
                     1u);
        return 1u;

    case RPI_CMD_GRUB_DISC1:
    case RPI_CMD_GRUB_DISC2:
    case RPI_CMD_GRUB_DISC3:
    case RPI_CMD_GRUB_DISC4:
    case RPI_CMD_GRUB_DISC5:
    case RPI_CMD_PUT_DISC1:
    case RPI_CMD_PUT_DISC2:
    case RPI_CMD_PUT_DISC3:
    case RPI_CMD_PUT_DISC4:
    case RPI_CMD_PUT_DISC5:
        if (Arm_DiscPulseFromMode(payload[0], &disc_pulse_us) == 0u) {
            return 0u;
        }
        ground_clamp_pulse_us = SERVO2_CLAMP_PULSE_US;
        disc_open_pulse_us = SERVO2_DISC_PICK_OPEN_PULSE_US;
        disc_clamp_pulse_us = SERVO2_CLAMP_PULSE_US;
        if (Arm_IsCubeDiscMode(payload[0]) != 0u) {
            ground_clamp_pulse_us = SERVO2_CUBE_GROUND_CLAMP_PULSE_US;
            disc_open_pulse_us = SERVO2_CUBE_DISC_OPEN_PULSE_US;
            disc_clamp_pulse_us = SERVO2_CUBE_DISC_CLAMP_PULSE_US;
        }
        if (Arm_IsPutMode(payload[0]) != 0u) {
            field_place_height_mm = Arm_FieldPlaceHeightFromMode(payload[0]);
            if (field_place_height_mm > LIFT_MAX_HEIGHT_MM) {
                return 0u;
            }
            Arm_RunPutFromDisc(
                disc_pulse_us,
                disc_open_pulse_us,
                disc_clamp_pulse_us,
                field_place_height_mm,
                Arm_PutTimingFromMode(payload[0]));
        } else {
            Arm_RunGrubToDisc(disc_pulse_us,
                              ground_clamp_pulse_us,
                              LIFT_PLACE_HEIGHT_MM,
                              LIFT_TRANSFER_HEIGHT_MM,
                              Arm_GrubTimingFromMode(payload[0]));
        }
        return 1u;

    case RPI_CMD_TASK2_PUT_1TO1:
    case RPI_CMD_TASK2_PUT_2TO2:
    case RPI_CMD_TASK2_PUT_3TO3:
        if (Arm_Task2ParamsFromMode(payload[0],
                                    &disc_pulse_us,
                                    &podium_height_mm) == 0u ||
            podium_height_mm > LIFT_MAX_HEIGHT_MM || Arm_Task2HeightsValid() == 0u) {
            return 0u;
        }
        Arm_RunTask2Place(disc_pulse_us, podium_height_mm,
                          Arm_Task2PlaceTimingFromMode(payload[0]));
        return 1u;

    case RPI_CMD_TASK2_GRUB_CUP1:
    case RPI_CMD_TASK2_GRUB_CUP2:
    case RPI_CMD_TASK2_GRUB_CUP3:
        if (Arm_Task2GrubPulseFromMode(payload[0], &disc_pulse_us) == 0u ||
            Arm_Task2HeightsValid() == 0u) {
            return 0u;
        }
        Arm_RunGrubToDisc(disc_pulse_us,
                          SERVO2_CLAMP_PULSE_US,
                          LIFT_TASK2_DISC_PLACE_HEIGHT_MM,
                          LIFT_TASK2_TRANSFER_HEIGHT_MM,
                          Arm_Task2GrubTimingFromMode(payload[0]));
        return 1u;

    case RPI_CMD_RESET_ANGLE:
        Gyro_PrepareRuntimeReceiver();
        if (Gyro_ResetHeading() == 0u) {
            return 0u;
        }
        return Gyro_CaptureReference();

    case RPI_CMD_MOTOR_ALIGN:
        return Chassis_MotorAlign();

    case RPI_CMD_SERVO1_EXTEND:
        TIM_SetCompare2(TIM5, SERVO1_TASK2_EXTEND_PULSE_US);
        delay_ms(SERVO_POWER_ON_SETTLE_MS);
        return 1u;

    case RPI_CMD_LIFT_UP:
        Lift_MoveToHeight(LIFT_MAX_HEIGHT_MM);
        return 1u;

    case RPI_CMD_LIFT_DOWN:
        Lift_MoveToHeight(LIFT_MIN_HEIGHT_MM);
        return 1u;

    case RPI_CMD_SERVO1_RETRACT:
        TIM_SetCompare2(TIM5, SERVO1_DISC_ALIGN_PULSE_US);
        delay_ms(SERVO_POWER_ON_SETTLE_MS);
        return 1u;

    case RPI_CMD_LIFT_MID:
        Lift_MoveToHeight(LIFT_MID_HEIGHT_MM);
        return 1u;

    case RPI_CMD_LIFT_178MM:
        Lift_MoveToHeight(LIFT_178_HEIGHT_MM);
        return 1u;

    case RPI_CMD_LIFT_186MM:
        Lift_MoveToHeight(LIFT_186_HEIGHT_MM);
        return 1u;

    case RPI_CMD_CLAW_OPEN:
        TIM_SetCompare3(TIM5, SERVO2_BOTTOM_OPEN_PULSE_US);
        delay_ms(SERVO_POWER_ON_SETTLE_MS);
        return 1u;

    case RPI_CMD_CLAW_CLOSE:
        Arm_Servo2Move(SERVO2_CLAMP_PULSE_US);
        return COMMAND_RESULT_DONE;

    case RPI_CMD_SOFTWARE_INIT:
        Arm_RunSoftwareInit();
        return COMMAND_RESULT_DONE;

    case RPI_CMD_TUNING_SERVO1_PULSE:
        tuning_value = Chassis_ReadUInt16(payload[1], payload[2]);
        if (tuning_value < SERVO_TUNING_MIN_PULSE_US ||
            tuning_value > SERVO_TUNING_MAX_PULSE_US) {
            return COMMAND_RESULT_INVALID;
        }
        Arm_Servo1Move(tuning_value);
        return COMMAND_RESULT_DONE;

    case RPI_CMD_TUNING_SERVO2_PULSE:
        tuning_value = Chassis_ReadUInt16(payload[1], payload[2]);
        if (tuning_value < SERVO_TUNING_MIN_PULSE_US ||
            tuning_value > SERVO_TUNING_MAX_PULSE_US) {
            return COMMAND_RESULT_INVALID;
        }
        Arm_Servo2Move(tuning_value);
        return COMMAND_RESULT_DONE;

    case RPI_CMD_TUNING_SERVO3_PULSE:
        tuning_value = Chassis_ReadUInt16(payload[1], payload[2]);
        if (tuning_value < SERVO_TUNING_MIN_PULSE_US ||
            tuning_value > SERVO_TUNING_MAX_PULSE_US) {
            return COMMAND_RESULT_INVALID;
        }
        Arm_Servo3Move(tuning_value);
        return COMMAND_RESULT_DONE;

    case RPI_CMD_TUNING_LIFT_HEIGHT:
        tuning_value = Chassis_ReadUInt16(payload[1], payload[2]);
        if (tuning_value > LIFT_MAX_HEIGHT_MM) {
            return COMMAND_RESULT_INVALID;
        }
        Lift_MoveToHeight(tuning_value);
        return COMMAND_RESULT_DONE;

    case RPI_CMD_CAMERA_TARGET_HIGH:
        Lift_MoveToHeight(LIFT_CAMERA_TARGET_HIGH_HEIGHT_MM);
        return COMMAND_RESULT_DONE;

    case RPI_CMD_CAMERA_TARGET_LOW:
        Lift_MoveToHeight(LIFT_CAMERA_TARGET_LOW_HEIGHT_MM);
        return COMMAND_RESULT_DONE;

    case RPI_CMD_SERVO3_HOME:
        Arm_Servo3Move(SERVO3_HOME_PULSE_US);
        return COMMAND_RESULT_DONE;

    case RPI_CMD_TASK2_CHASSIS_PROFILE:
        chassis_task2_profile_enabled = 1u;
        return COMMAND_RESULT_DONE;

    case RPI_CMD_TASK1_CHASSIS_PROFILE:
        chassis_task2_profile_enabled = 0u;
        return COMMAND_RESULT_DONE;

    case RPI_CMD_ARM_START_BUTTON:
        StartButton_Arm();
        return COMMAND_RESULT_DONE;

    case RPI_CMD_GYRO_DIAGNOSTIC:
        Gyro_SendDiagnostic();
        return 1u;

    default:
        return 0u;
    }
}

int main(void)
{
    uint8_t payload[RPI_FRAME_PAYLOAD_SIZE];
    uint32_t ready_idle_polls = 0u;
    uint8_t host_connected = 0u;
    uint8_t ready_pending = 1u;
    uint8_t ready_tx_index = 0u;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    /* Drive all servo signals safely before the gyro startup delays. */
    Arm_PWM_Init();
    (void)Gyro_InitializeRuntime();
    Lift_GPIO_Init();
    Chassis_UART1_Init();
    Chassis_GPIO_Init();
    KEY_Init();

    for (;;) {
        if (Chassis_TryReadFrame(payload) != 0u) {
            uint8_t command_result;

            host_connected = 1u;
            ready_pending = 0u;
            ready_tx_index = 0u;
            command_result = Chassis_HandleCommand(payload);
            if (command_result <= COMMAND_RESULT_TEST_SUPPORTED) {
                Chassis_SendStatus(command_result);
            }
        } else {
            delay_us(UART_RX_POLL_INTERVAL_US);
            if (StartButton_TryConsumePress() != 0u) {
                Chassis_SendStatus(RPI_STATUS_START_BUTTON);
            } else if (host_connected == 0u) {
                if (ready_pending != 0u) {
                    if (Chassis_TrySendReadyByte(&ready_tx_index) != 0u) {
                        ready_pending = 0u;
                        ready_idle_polls = 0u;
                    }
                } else {
                    ready_idle_polls++;
                    if (ready_idle_polls >= UART_READY_REPEAT_POLLS) {
                        ready_pending = 1u;
                    }
                }
            }
        }
    }
}
