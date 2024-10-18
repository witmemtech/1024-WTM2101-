#ifndef _WTM2101AC_PROTOCOL_H_
#define _WTM2101AC_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef enum {
    WTM2101_CMD_INFO           =    0x01,                  
    WTM2101_CMD_SET_MIC_GAIN   =    0x02,             
    WTM2101_CMD_START_KWS      =    0x03,  
    WTM2101_CMD_STOP_KWS       =    0x04,  
    WTM2101_CMD_START_ENC      =    0x05,
    WTM2101_CMD_STOP_ENC       =    0x06,  
    WTM2101_CMD_SLEEP          =    0x07,
    WTM2101_CMD_SET_ENC_PARAMS =    0x08,
    WTM2101_CMD_SET_MUTE       =    0x09,
    WTM2101_CMD_WAKEUP         =    0xFC,
    WTM2101_CMD_LOGGER         =    0xFD,
    WTM2101_CMD_KWS_RESULT     =    0xFE,
} wtm2101_protocol_cmd_e;

#pragma pack(1)

// 0x01
struct wtmproto_info {
    uint8_t fix_head;  
    uint8_t cmd;
    uint32_t crc;       
};

struct wtmproto_info_reply {
    uint8_t fix_head;       
    uint8_t cmd;
    uint64_t chip_id;
    uint8_t fw_major_ver;
    uint8_t fw_minor_ver;
    uint8_t fw_fixed_ver;
    uint8_t nw_major_ver;
    uint8_t nw_minor_ver;  
    uint8_t nw_fixed_ver;    
    uint8_t gain_level;
    uint32_t crc;            
};

// 0x02
struct wtmproto_set_gain {
    uint8_t fix_head;   
    uint8_t cmd;         
    uint8_t gain_level;  
    uint32_t crc;        
};

struct wtmproto_set_gain_reply {
    uint8_t fix_head;  
    uint8_t cmd;        
    uint8_t is_ok;      
    uint32_t crc;
};

// 0x03
struct wtmproto_start_kws {
    uint8_t fix_head;  
    uint8_t cmd;
    uint8_t mode;   
    uint32_t crc;      
};

struct wtmproto_start_kws_reply {
    uint8_t fix_head;  
    uint8_t cmd;        
    uint8_t is_ok;      
    uint32_t crc;      
};

// 0x04
struct wtmproto_stop_kws {
    uint8_t fix_head;  
    uint8_t cmd;   
    uint32_t crc;
};

struct wtmproto_stop_kws_reply {
    uint8_t fix_head;  
    uint8_t cmd;        
    uint8_t is_ok;      
    uint32_t crc;      
};

// 0x05
struct wtmproto_start_enc {
    uint8_t fix_head;  
    uint8_t cmd;
    uint8_t mode;   
    uint32_t crc;      
};

struct wtmproto_start_enc_reply {
    uint8_t fix_head;  
    uint8_t cmd;        
    uint8_t is_ok;      
    uint32_t crc;      
};

// 0x06
struct wtmproto_stop_enc {
    uint8_t fix_head;  
    uint8_t cmd;
    uint32_t crc;      
};

struct wtmproto_stop_enc_reply {
    uint8_t fix_head;  
    uint8_t cmd;        
    uint8_t is_ok;      
    uint32_t crc;      
};

// 0x07
struct wtmproto_start_sleep {
    uint8_t fix_head;
    uint8_t cmd;       
    uint32_t crc;       
};

struct wtmproto_start_sleep_reply {
    uint8_t fix_head;  
    uint8_t cmd;        
    uint8_t is_ok;      
    uint32_t crc;      
};

// 0x08
struct wtmproto_set_enc {
    uint8_t fix_head;   
    uint8_t cmd;         
    uint8_t aec_gain_level;
    uint8_t enc_level;  
    uint32_t crc;        
};

struct wtmproto_set_enc_reply {
    uint8_t fix_head;  
    uint8_t cmd;        
    uint8_t is_ok;      
    uint32_t crc;      
};

// 0x09 is same as 0x03

// 0xFC
struct wtmproto_wakeup {
    uint8_t fix_head; 
    uint8_t cmd; 
    uint32_t crc;
};

// 0xFD
struct wtmproto_logger{
    uint8_t fix_head;  
    uint8_t cmd;
    uint8_t msg_len;
    uint8_t log_level;
};

// 0xFE
struct wtmproto_kws_result {
    uint8_t fix_head;  
    uint8_t cmd;
    uint8_t result_id;
    uint32_t crc;
};

#pragma pack()

///////////////////////////////////////////////////

struct wtm2101ac_protoc_status {
    uint8_t use_kws;
    uint8_t use_enc;
    uint8_t use_aec;
    uint8_t wanna_sleep;
    uint8_t start_sleep;
    uint8_t wakeup;
    uint8_t start_call;
    uint8_t stop_call;
    uint8_t start_bypass;
    uint8_t start_kws;
    uint8_t start_enhanced_kws;
    uint8_t stop_kws;
    uint8_t run_mode;         // 0: kws    1: enc     2: enhanced_kws
    uint8_t use_kws_i2s;
    uint8_t buffer_finished;
    uint8_t nw_major_version;
    uint8_t nw_minor_version;
    uint8_t nw_fixed_version;
    uint8_t gain_level;
    uint8_t aec_gain_level;
    uint8_t enc_level;
    uint8_t mute_state;       
    uint64_t chipid;
};

int wtm2101ac_init(uint8_t *proto_buff, struct wtm2101ac_protoc_status *proto_status);
int wtm2101ac_protocol_data_receive_cb();
int wtm2101ac_send_kws_to_master(uint8_t result_idx);
int wtm2101ac_send_wakeup();
int wtm2101ac_send_info(char* msg, uint8_t len, uint8_t log_level);

uint32_t reply_common(uint8_t cmd);

#endif
