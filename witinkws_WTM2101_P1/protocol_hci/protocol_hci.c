/******************************************************************************
Copyright   : 深圳小澈科技有限公司
File name   : protocol_hci.c
Author      : dengjq
Version     : 1.0
Date        : 2023.02.09
Description : 双机通讯协议
History     :
******************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include "slip.h"
#include "protocol_hci.h"
#include "wtm2101ac_protocol.h"
#include "config_ram_iismic_realtime_hsi24mx4.h"

#ifdef UART0_TX_PIN
#define HCI_TARGET_UART UART0
#else
#define HCI_TARGET_UART UART1
#endif

uint8_t wtm2101ac_if_data_send(uint8_t *p_rsp_buf,uint8_t slip_len){
  for(uint8_t i = 0;i<slip_len;i++)
  {
    while(UART_Flag_GetStatBit(HCI_TARGET_UART,UART_FLAG_TEMT) == RESET);
    UART_SendData(HCI_TARGET_UART, p_rsp_buf[i]);   
  }
  return 1;
}

/**************************************************************************
 *                            数据类型定义                                 *
 **************************************************************************/
static  slip_t m_slip;
wtmproto_spi_config spi_config;

/**************************************************************************
 *                               函数实现                                     *
 **************************************************************************/
uint8_t p_rsp_buf[300];
uint32_t protocol_hci_send_data(uint8_t *p_data, size_t size)
{
    uint32_t ret = ERR_SUCCESS;
    uint32_t slip_len;
    //uint8_t *p_rsp_buf = (uint8_t *)PROTOCOL_MALLOC(WTM_RECEIVE_DATA_SIZE_MAX + 8);
    if(p_rsp_buf == NULL){
        return ERR_NO_MEM;
    }
    (void) slip_encode(p_rsp_buf, (uint8_t *)p_data, size, &slip_len);
    ret = wtm2101ac_if_data_send(p_rsp_buf, slip_len);
    return ret;
}

uint32_t protocol_hci_send_data_spi(uint8_t *p_data, size_t size)
{
    uint32_t ret = ERR_SUCCESS;
    uint32_t slip_len;
    if(p_rsp_buf == NULL){
        return ERR_NO_MEM;
    }
    (void) slip_encode(p_rsp_buf, (uint8_t *)p_data, size, &slip_len);
    
    spi_config.send_func(p_rsp_buf, slip_len);
    spi_config.irq_func();
    return ret;
}

uint8_t protocol_hci_receive_data_slip_handle(uint8_t input_char){
    uint32_t ret_code = slip_decode_add_byte(&m_slip, input_char);
    if(ret_code == ERR_SUCCESS){
        m_slip.current_index = 0;
        m_slip.state = SLIP_STATE_WAITING;
        return 1;
    }
    return 0;
}


uint32_t protocol_hci_init(uint8_t *buff, int buffer_len)
{
    m_slip.p_buffer = buff;
    m_slip.current_index = 0;
    m_slip.buffer_len = buffer_len;
    m_slip.state = SLIP_STATE_DECODING;
    return ERR_SUCCESS;
}

void protocol_reset_buffer_state(){
    m_slip.state = SLIP_STATE_DECODING;
    m_slip.current_index = 0;
}

void protocol_spi_set_callback(wtmproto_spi_config config){
    spi_config.irq_func = config.irq_func;
    spi_config.send_func = config.send_func;
}