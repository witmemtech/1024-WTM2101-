#ifndef _PROTOCOL_HCI_H_
#define _PROTOCOL_HCI_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "string.h"
#include "uart.h"
#include "spi.h"

typedef int (*spi_send_hook)(uint8_t *buffer, uint32_t len);
typedef void (*spi_irq_hook)(void);
typedef struct{
    spi_send_hook   send_func;
    spi_irq_hook    irq_func;
}wtmproto_spi_config; 

extern uint32_t protocol_hci_init(uint8_t *buff, int buffer_len);

extern uint32_t protocol_hci_send_data(uint8_t *p_data, size_t size);

extern uint32_t protocol_hci_send_data_spi(uint8_t *p_data, size_t size);

uint8_t wtm2101ac_if_data_send(uint8_t *p_rsp_buf,uint8_t slip_len);

uint8_t protocol_hci_receive_data_slip_handle(uint8_t input_char);

void protocol_reset_buffer_state();

void protocol_spi_set_callback(wtmproto_spi_config config);

#endif
