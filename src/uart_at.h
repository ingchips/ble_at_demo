#ifndef _CMDS_H
#define _CMDS_H
#include <stdint.h>

void at_rx_data(const char *d, const uint8_t len);
uint8_t *at_clear_tx_data(uint16_t *len);
void uart_at_start(void);
void at_tx_ok(void);
#endif
