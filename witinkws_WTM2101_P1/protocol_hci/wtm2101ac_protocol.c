#include "wtm2101ac_protocol.h"
#include "config_common.h"
#include "protocol_hci.h"
#include "slip.h"
#include "string.h"
#include "stdio.h"

#define WTM2101_FIX_HEAD 0x68

#define PROTOCOL_DEBUG_LOG   (0)

struct wtm2101ac_protoc_status* _proto_status = NULL;
uint8_t* _proto_buff = NULL;

int wtm2101ac_init(uint8_t* proto_buff, struct wtm2101ac_protoc_status* proto_status) {
    _proto_buff = proto_buff;
    _proto_status = proto_status;
    return 0;
}

uint32_t crc_calc(uint8_t* data, uint16_t len) { 
    uint32_t _sum = 0;
    for(int i=0;i<len;i++){
        _sum += data[i];
    }
    return _sum; 
}

uint32_t reply_0x01() {
    struct wtmproto_info_reply dst;
    dst.fix_head = WTM2101_FIX_HEAD;
    dst.cmd = 0x01;
    dst.chip_id = _proto_status->chipid;
    dst.fw_major_ver = FW_MAJOR_VERSION;
    dst.fw_minor_ver = FW_MINOR_VERSION;
    dst.fw_fixed_ver = FW_FIXED_VERSION;
    dst.nw_major_ver = _proto_status->nw_major_version;
    dst.nw_minor_ver = _proto_status->nw_minor_version;
    dst.nw_fixed_ver = _proto_status->nw_fixed_version;
    dst.gain_level = _proto_status->gain_level;
    dst.crc = crc_calc((uint8_t*)&dst, sizeof(dst)-sizeof(uint32_t));
    protocol_hci_send_data_spi((uint8_t*)&dst, sizeof(dst));
    return ERR_SUCCESS;
}

uint32_t reply_common(uint8_t cmd){
    struct wtmproto_set_gain_reply dst;
    dst.fix_head = WTM2101_FIX_HEAD;
    dst.cmd = cmd;
    dst.is_ok = 1;
    dst.crc = crc_calc((uint8_t*)&dst, sizeof(dst)-sizeof(uint32_t));
    protocol_hci_send_data_spi((uint8_t*)&dst, sizeof(dst));
    return 0;
}

const uint8_t msg_crc_calc_len_table[] = {
    0,  
    2,  //WTM2101_CMD_INFO =      0x01 ,         
    3,  //WTM2101_CMD_SET_MIC_GAIN = 0x02,         
    3,  //WTM2101_CMD_START_KWS = 0x03,   
    2,  //WTM2101_CMD_STOP_KWS = 0x04,  
    3,  //WTM2101_CMD_START_ENC = 0x05,          
    2,  //WTM2101_CMD_STOP_ENC = 0x06,            
    2,  //WTM2101_CMD_SLEEP = 0x07,
    4,  //WTM2101_CMD_SET_ENC_PARAMS = 0x08,
    3,  //WTM2101_CMD_SET_MUTE = 0x09,
};

int check_crc(uint8_t msg_id){
    uint32_t get_crc;
    int msg_len = msg_crc_calc_len_table[msg_id];
    memcpy(&get_crc, &_proto_buff[msg_len],4);
    uint32_t calc_crc =  crc_calc(_proto_buff, msg_len);
    if(get_crc != calc_crc){
        printf("%d crc err\r\n",msg_id);
        return -1;
    }
    return 0;
}

inline uint8_t get_mode(){
    struct wtmproto_start_kws* data = (struct wtmproto_start_kws*)_proto_buff;
    return data->mode;
}

int wtm2101ac_protocol_data_receive_cb() {
    if(_proto_status->wakeup){
        return 0;
    }
    struct wtmproto_info* tmp_data = (struct wtmproto_info*)_proto_buff;
    uint8_t cmd = tmp_data->cmd; 
    uint8_t fix_head = tmp_data->fix_head; 
    #if PROTOCOL_DEBUG_LOG
    printf("-cmd:%d-",(int)cmd);
    //char _cmd[5]={0};
    //sprintf(_cmd,"cmd%d",(int)cmd);
    //wtm2101ac_send_info(_cmd,5,0);
    #endif
    if (fix_head != WTM2101_FIX_HEAD) {
        //printf("fix_head ERR=0x%0x \r\n", fix_head);
        for(int i=0;i<12;i++){
            printf("fix_head ERR %d=0x%0x \r\n", i,_proto_buff[i]);
        }
        return -1;
    }
    if(check_crc(cmd)) {
        return -2;
    }
    uint8_t mode = get_mode();
    switch (cmd) {
        case WTM2101_CMD_INFO: 
            reply_0x01();
            break;
        case WTM2101_CMD_SET_MIC_GAIN: 
            if(mode>16 || mode<1){
                mode = 4;
            }
            _proto_status->gain_level = mode;
            reply_common(WTM2101_CMD_SET_MIC_GAIN);
            break;
        case WTM2101_CMD_START_KWS:
            if(mode==0){
                _proto_status->start_enhanced_kws = 1;
            }else if(mode==1){
                _proto_status->start_enhanced_kws = 1;
                _proto_status->use_kws_i2s = 1;
            }else if(mode==2){
                _proto_status->start_kws = 1;
            }else if(mode==3){
                _proto_status->start_kws = 1;
                _proto_status->use_kws_i2s = 1;
            }
            break;
        case WTM2101_CMD_STOP_KWS:
            _proto_status->stop_kws = 1;
            break;
        case WTM2101_CMD_START_ENC:
            if(mode==0){
                _proto_status->start_call = 1;
            }else if(mode==1){
                _proto_status->start_bypass = 1;
            }
            break;
        case WTM2101_CMD_STOP_ENC:
            _proto_status->stop_call = 1;
            break;
        case WTM2101_CMD_SLEEP:
            _proto_status->wanna_sleep = 1;
            break;
        case WTM2101_CMD_SET_ENC_PARAMS:
            {
                struct wtmproto_set_enc* data = (struct wtmproto_set_enc*)_proto_buff;
                uint8_t aec_gain_level = data->aec_gain_level;
                uint8_t enc_level = data->enc_level;
                if(aec_gain_level>10){
                    aec_gain_level = 10;
                }
                if(enc_level>20 || enc_level<1){
                    enc_level = 10;
                }
                _proto_status->aec_gain_level = aec_gain_level;
                _proto_status->enc_level = enc_level;
                reply_common(WTM2101_CMD_SET_ENC_PARAMS);
            }
            break;
          case WTM2101_CMD_SET_MUTE:
              _proto_status-> mute_state= mode;
              reply_common(WTM2101_CMD_SET_MUTE);
            break;
        default:
            return -3;
            break;
    }
    return cmd;
}

int wtm2101ac_send_kws_to_master(uint8_t result_idx){
    struct wtmproto_kws_result dst;
    dst.fix_head = WTM2101_FIX_HEAD;
    dst.cmd = WTM2101_CMD_KWS_RESULT;
    dst.result_id = result_idx;
    dst.crc = crc_calc((uint8_t*)&dst, sizeof(dst)-sizeof(uint32_t));
    protocol_hci_send_data_spi((uint8_t*)&dst, sizeof(dst));
    return 0;
}

int wtm2101ac_send_wakeup(){
    struct wtmproto_wakeup dst;
    dst.fix_head = WTM2101_FIX_HEAD;
    dst.cmd = WTM2101_CMD_WAKEUP;
    dst.crc = crc_calc((uint8_t*)&dst, sizeof(dst)-sizeof(uint32_t));
    protocol_hci_send_data_spi((uint8_t*)&dst, sizeof(dst));
    return 0;
}

int wtm2101ac_send_info(char* msg, uint8_t len, uint8_t log_level){
    
    struct wtmproto_logger dst;
    dst.fix_head = WTM2101_FIX_HEAD;
    dst.cmd = WTM2101_CMD_LOGGER;
    dst.msg_len = len;
    dst.log_level = log_level;
    char tmp_buffer[128];
    memcpy(tmp_buffer, (uint8_t*)&dst, sizeof(dst));
    uint32_t idx = sizeof(dst);
    memcpy(&tmp_buffer[idx], msg, len);
    uint32_t crc = crc_calc(tmp_buffer, len+idx);
    memcpy(&tmp_buffer[idx+len], &crc,4);
    protocol_hci_send_data_spi(tmp_buffer, len+idx+4);
    return 0;
}