/* CANopen slave: TWAI driver, RPDO consumption, TPDO/heartbeat production. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

void canopen_start(void);
uint8_t canopen_nmt_state(void);
uint32_t canopen_rx_count(void);
uint32_t canopen_tx_count(void);
uint32_t canopen_rx_dropped(void);
