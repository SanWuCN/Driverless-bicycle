#ifndef UART7_BRIDGE_H
#define UART7_BRIDGE_H

#include <stdint.h>

extern volatile uint8_t uart7_last_rx;
extern volatile uint32_t uart7_rx_count;
extern volatile uint32_t uart7_rx_overflow_count;

void uart7_tuning_service(void);

#endif
