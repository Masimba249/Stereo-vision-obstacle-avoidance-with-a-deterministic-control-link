/* CANopen PDO layout - the byte-for-byte mirror of pi/stereolink/pdo.py.
 *
 * If you change anything here, change it there too; tests/test_pdo.py pins
 * the Python side and tools/check_protocol_sync.py diffs the two.
 */
#pragma once

#include <stdint.h>

#define COB_NMT             0x000u
#define COB_SYNC            0x080u
#define COB_EMCY_BASE       0x080u
#define COB_TPDO1_BASE      0x180u   /* status      node -> master */
#define COB_RPDO1_BASE      0x200u   /* setpoint    master -> node */
#define COB_TPDO2_BASE      0x280u   /* diagnostics node -> master */
#define COB_RPDO2_BASE      0x300u   /* limits      master -> node */
#define COB_TPDO3_BASE      0x380u   /* latency echo node -> master */
#define COB_HEARTBEAT_BASE  0x700u

#define COB(base, node)     ((base) + (node))

/* NMT commands (byte 0 of a 0x000 frame; byte 1 is the target node). */
#define NMT_CMD_START        0x01
#define NMT_CMD_STOP         0x02
#define NMT_CMD_ENTER_PREOP  0x80
#define NMT_CMD_RESET_NODE   0x81
#define NMT_CMD_RESET_COMM   0x82

/* NMT states, as reported in the heartbeat byte. */
#define NMT_STATE_BOOTUP      0x00
#define NMT_STATE_STOPPED     0x04
#define NMT_STATE_OPERATIONAL 0x05
#define NMT_STATE_PREOP       0x7F

/* Firmware state machine (TPDO1 byte 4). */
typedef enum {
    DRIVE_INIT      = 0,
    DRIVE_IDLE      = 1,
    DRIVE_RUN       = 2,
    DRIVE_DEGRADED  = 3,  /* link lost, actively ramping to zero */
    DRIVE_SAFE_STOP = 4,  /* ramp finished, outputs off, latched */
    DRIVE_FAULT     = 5,
} drive_state_t;

/* Fault bits (TPDO1 byte 5). */
#define FAULT_NONE             0x00
#define FAULT_LINK_LOSS        0x01
#define FAULT_IR_BLOCKED       0x02
#define FAULT_OVERCURRENT      0x04
#define FAULT_ENCODER_STALL    0x08
#define FAULT_ESTOP            0x10
#define FAULT_CAN_BUS_OFF      0x20
#define FAULT_SETPOINT_RANGE   0x40
#define FAULT_CONTROL_OVERRUN  0x80

/* Setpoint flags (RPDO1 byte 7). */
#define SP_FLAG_VISION_VALID   0x01
#define SP_FLAG_ESTOP_REQUEST  0x02
#define SP_FLAG_OBSTACLE_NEAR  0x04
#define SP_FLAG_CLEAR_FAULT    0x08

/* EMCY error codes. */
#define EMCY_LINK_LOSS         0x8130u
#define EMCY_BUS_OFF           0x8140u
#define EMCY_IR_BLOCKED        0xFF01u
#define EMCY_CONTROL_OVERRUN   0xFF02u

/* All PDOs are exactly 8 bytes, little-endian, no padding.  The ESP32 is
 * little-endian so these structs map straight onto the wire. */
#pragma pack(push, 1)

typedef struct {            /* RPDO1: master -> node, 10 Hz */
    int16_t  v_mm_s;
    int16_t  w_mrad_s;
    uint16_t obstacle_cm;
    uint8_t  seq;
    uint8_t  flags;
} pdo_setpoint_t;

typedef struct {            /* RPDO2: master -> node, on change */
    uint16_t v_max_mm_s;
    uint16_t decel_mm_s2;
    uint16_t watchdog_ms;
    uint8_t  ir_stop_cm;
    uint8_t  reserved;
} pdo_limits_t;

typedef struct {            /* TPDO1: node -> master, 50 Hz */
    int16_t v_meas_mm_s;
    int16_t w_meas_mrad_s;
    uint8_t state;
    uint8_t faults;
    uint8_t seq_echo;
    uint8_t ir_mask;
} pdo_status_t;

typedef struct {            /* TPDO2: node -> master, 5 Hz */
    uint16_t jitter_p99_us;
    uint16_t period_max_us;
    uint16_t rpdo_age_ms;
    uint8_t  ctrl_cpu_pct;
    uint8_t  comms_cpu_pct;
} pdo_diag_t;

typedef struct {            /* TPDO3: node -> master, per consumed RPDO1 */
    uint32_t rx_to_apply_us;
    uint8_t  seq_echo;
    uint8_t  watchdog_misses;
    uint8_t  rpdo_rate_hz;
    uint8_t  reserved;
} pdo_latency_echo_t;

typedef struct {            /* EMCY: node -> master, on fault transition */
    uint16_t error_code;
    uint8_t  error_register;
    uint8_t  state;
    uint8_t  faults;
    uint8_t  reserved[3];
} pdo_emcy_t;

#pragma pack(pop)

_Static_assert(sizeof(pdo_setpoint_t)     == 8, "RPDO1 must be 8 bytes");
_Static_assert(sizeof(pdo_limits_t)       == 8, "RPDO2 must be 8 bytes");
_Static_assert(sizeof(pdo_status_t)       == 8, "TPDO1 must be 8 bytes");
_Static_assert(sizeof(pdo_diag_t)         == 8, "TPDO2 must be 8 bytes");
_Static_assert(sizeof(pdo_latency_echo_t) == 8, "TPDO3 must be 8 bytes");
_Static_assert(sizeof(pdo_emcy_t)         == 8, "EMCY must be 8 bytes");
