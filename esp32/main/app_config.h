/* Board wiring, timing and tuning constants for the drive node.
 *
 * Everything that would change when moving to different hardware lives here,
 * so the task and control code below stays board-independent.
 *
 * Reference wiring: ESP32-WROOM-32 + SN65HVD230 CAN transceiver +
 * TB6612FNG dual H-bridge + 2x quadrature encoders + 3x digital IR sensors.
 */
#pragma once

/* ------------------------------------------------------------------ CAN */
#define CAN_TX_GPIO              21
#define CAN_RX_GPIO              22
#define CAN_BITRATE_KBPS         500
#define CANOPEN_NODE_ID          0x22

/* --------------------------------------------------------------- Motors */
/* TB6612FNG: one PWM + two direction pins per channel, one shared standby. */
#define MOTOR_L_PWM_GPIO         25
#define MOTOR_L_IN1_GPIO         26
#define MOTOR_L_IN2_GPIO         27
#define MOTOR_R_PWM_GPIO         32
#define MOTOR_R_IN1_GPIO         33
#define MOTOR_R_IN2_GPIO         14
#define MOTOR_STBY_GPIO          13

/* 20 kHz keeps the switching whine above hearing and is still slow enough
 * that the TB6612's ~1 us transition time is a negligible duty error. */
#define MOTOR_PWM_FREQ_HZ        20000
#define MOTOR_PWM_RES_BITS       10          /* 0..1023 */
#define MOTOR_PWM_MAX            ((1 << MOTOR_PWM_RES_BITS) - 1)
/* Below this duty the gearmotors do not turn but do draw current and heat. */
#define MOTOR_MIN_DUTY           40

/* ------------------------------------------------------------- Encoders */
#define ENC_L_A_GPIO             34
#define ENC_L_B_GPIO             35
#define ENC_R_A_GPIO             36
#define ENC_R_B_GPIO             39
/* Counts per output-shaft revolution, after gearbox and 4x quadrature. */
#define ENCODER_CPR              1440.0f

/* ------------------------------------------------------- IR safety layer */
/* Digital obstacle detectors. Active LOW = obstacle present (open-collector
 * outputs with pull-ups), so a disconnected sensor reads "blocked" and fails
 * safe rather than silently disabling the safety layer. */
#define IR_LEFT_GPIO             16
#define IR_CENTER_GPIO           17
#define IR_RIGHT_GPIO            4
#define IR_ACTIVE_LEVEL          0
#define IR_DEBOUNCE_US           2000
#define IR_MASK_LEFT             0x01
#define IR_MASK_CENTER           0x02
#define IR_MASK_RIGHT            0x04

/* --------------------------------------------------------- Drive geometry */
#define WHEEL_RADIUS_M           0.0325f
#define WHEEL_BASE_M             0.180f
#define V_MAX_MM_S               1500.0f
#define W_MAX_MRAD_S             4000.0f

/* ------------------------------------------------------------- Timing */
#define CONTROL_HZ               1000
#define CONTROL_PERIOD_US        (1000000 / CONTROL_HZ)
#define STATUS_TPDO_HZ           50
#define DIAG_TPDO_HZ             5
#define HEARTBEAT_HZ             5

/* --------------------------------------------------------- FreeRTOS map */
/* The whole point of the split: control owns core 1 at a priority nothing
 * else on that core can reach; every byte of comms work happens on core 0. */
#define CORE_CONTROL             1
#define CORE_COMMS               0
#define PRIO_CONTROL             23
#define PRIO_CAN_RX              12
#define PRIO_CAN_TX              11
#define PRIO_LOADGEN              6
#define PRIO_STATS                5

#define STACK_CONTROL            4096
#define STACK_CAN_RX             4096
#define STACK_CAN_TX             3072
#define STACK_LOADGEN            2560
#define STACK_STATS              3072

/* ------------------------------------------------------------- Failsafe */
/* If no RPDO1 arrives for this long the node stops trusting the Pi.  The Pi
 * sends at 10 Hz (100 ms), so this is 1.5 missed cycles - tight enough to be
 * meaningful, loose enough not to trip on a single late frame. */
#define WATCHDOG_TIMEOUT_MS_DEFAULT   150
#define WATCHDOG_TIMEOUT_MS_MIN       20
#define WATCHDOG_TIMEOUT_MS_MAX       1000
/* Controlled deceleration used by the failsafe ramp (mm/s per second). */
#define FAILSAFE_DECEL_MM_S2     1200.0f
#define FAILSAFE_DECEL_MIN       100.0f
/* Speed below which the ramp is considered finished. */
#define ZERO_SPEED_EPS_MM_S      5.0f

/* ------------------------------------------------------------------ PID */
#define PID_KP                   0.85f
#define PID_KI                   6.50f
#define PID_KD                   0.010f
#define PID_I_LIMIT              600.0f
#define PID_D_LPF_HZ             60.0f

/* ------------------------------------------------- Diagnostics / loadgen */
#define JITTER_BIN_US            2
#define JITTER_BINS              128
/* Set to 1 to run the comms-load generator that produces the numbers in
 * docs/jitter-report.md.  It exists only to prove the isolation claim. */
#ifndef ENABLE_LOAD_GENERATOR
#define ENABLE_LOAD_GENERATOR    0
#endif
#define LOADGEN_BURST_FRAMES     24
#define LOADGEN_PERIOD_MS        5
