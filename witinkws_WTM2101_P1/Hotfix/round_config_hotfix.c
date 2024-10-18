#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cnn_framework.h"
#include "net_parse.h"

#define  ENC_IDX   1
#define  KWS_IDX   0

typedef struct {
    uint8_t model_id;// ignored
    uint8_t net_id;
    uint8_t flow_id;
    uint8_t event_id;
    uint16_t offset; // bytes
    uint16_t length; // bytes
    uint32_t old_config[2];
    uint32_t new_config[2];
}model_hotfix_round_config_t;

#define MODEL_PATCHES_COUNT 4
model_hotfix_round_config_t model_patches[MODEL_PATCHES_COUNT] = {
    {
        // round5, mul write 576 -> write 1730
        .model_id   = 0,
        .net_id     = ENC_IDX,
        .flow_id    = 0,
        .event_id   = 0,
        .offset     = sizeof(uint32_t)*56,
        .length     = sizeof(uint32_t)*2,
        .old_config = {0x21600f00, 0x48004004},
        .new_config = {0x21600f00, 0xd8404004},
    },
    {
        // round5, reactv read 576 -> read 1730
        .model_id   = 0,
        .net_id     = ENC_IDX,
        .flow_id    = 0,
        .event_id   = 0,
        .offset     = sizeof(uint32_t)*68,
        .length     = sizeof(uint32_t)*2,
        .old_config = {0x31200900, 0x08020000},
        .new_config = {0x33610900, 0x08020000},
    },

    {
        // round12, mul write 2754 -> write 3330
        .model_id   = 0,
        .net_id     = ENC_IDX,
        .flow_id    = 0,
        .event_id   = 0,
        .offset     = sizeof(uint32_t)*142,
        .length     = sizeof(uint32_t)*2,
        .old_config = {0x21601b05, 0x58404004},
        .new_config = {0x21601b05, 0xa0404004},
    },
    {
        // round12, reactv read 2754 -> read 3330
        .model_id   = 0,
        .net_id     = ENC_IDX,
        .flow_id    = 0,
        .event_id   = 0,
        .offset     = sizeof(uint32_t)*154,
        .length     = sizeof(uint32_t)*2,
        .old_config = {0x35612b08, 0x08020000},
        .new_config = {0x36812b08, 0x08020000},
    },

};

extern WITIN_MODEL_T *_witin_model;

void hotfix_apply()
{
    Compute_flow_T* flowInfo;
    Event_T*        event;
    SysEvent_T*     sysEvent;

    for (int patch_id = 0;patch_id < MODEL_PATCHES_COUNT;patch_id++) {
        const model_hotfix_round_config_t* patch = &model_patches[patch_id];

        if (_witin_model == NULL) {
            //printf("no model\r\n");
            break;
        }

        if (_witin_model->netInfo == NULL) {
            //printf("no net\r\n");
            break;
        }

        if (_witin_model->netInfo[patch->net_id].flowInfo == NULL) {
            //printf("no flow\r\n");
            continue;
        }

        flowInfo = &_witin_model->netInfo[patch->net_id].flowInfo[patch->flow_id];
        if (flowInfo->event == NULL) {
            //printf("no event\r\n");
            continue;
        }

        event = &flowInfo->event[patch->event_id];
        if (event->payload == NULL) {
            //printf("no payload\r\n");
            continue;
        }

        sysEvent = event->payload;
        if (memcmp(((char*)sysEvent->data) + patch->offset, patch->old_config, patch->length) != 0) {
            //printf("no match\r\n");
            continue;
        }
        
        printf("patch%d apply\r\n", patch_id);
        memcpy(((char*)sysEvent->data) + patch->offset, patch->new_config, patch->length);
    }
}

////////////////////////////////////////////////////////////////////////////////
//  Example
////////////////////////////////////////////////////////////////////////////////
//      int cnn_init() {
//          memset(&config, 0, sizeof(WITIN_CONFIG_T));
//          memset(&hook, 0, sizeof(WITIN_HOOK_T));
//          
//          config.isUseDiff = 1;
//          config.isUseWfi = 1;
//          config.isUseDmaTransport = 0;
//          config.model_data_addr = (unsigned char *)register_data;
//          config.allocFunc = pvPortMalloc;
//          config.freeFunc = vPortFree;
//          hook.logFunc = printf;
//          hook.layerCallBack = witin_layer_handler;
//          hook.log_level = LOG_ERROR;
//      
//          wengine_set_hook(&hook);
//      
//          int res = wengine_init(&config);
//          if(res < 0) {
//              printf("wengine err\r\n");
//               return -1;
//          }else{
//              printf("wengine init ok\r\n");
//          }
//      
//      
//          hotfix_apply(); // <----------------------------
//      
//          
//          #if USE_ENC
//          wengine_get_input_info(ENC_IDX, &inputInfo[ENC_IDX], &intputCount);
//          wengine_get_output_info(ENC_IDX, &outputInfo[ENC_IDX], &outputCount);
//          #endif
//          #if USE_KWS
//          wengine_get_input_info(KWS_IDX, &inputInfo[KWS_IDX], &intputCount);
//          wengine_get_output_info(KWS_IDX, &outputInfo[KWS_IDX], &outputCount);
//          #endif
//          return 0;
//      }

