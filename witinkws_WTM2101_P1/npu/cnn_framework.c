#include "cnn_framework.h"
#include "string.h"
#include "stdio.h"
#include "heap.h"
#include "wtm2101_hal.h"
#include "wnpu_config.h"
#include "basic_config.h"
#include "WTM2101.h"
#include "rcc.h"
#include "config_common.h"
#include "round_config_hotfix.h"
#include "riscv_abs8.h"

#define	 ENC_IDX   1
#define	 KWS_IDX   0
#define	 NET_IDX_0   0

static uint8_t g_ctx_switch = 0;
extern int wnpu_set_npu_colck_gate(int);
DataIO_T inputInfo[3] =	{0};
DataIO_T outputInfo[2] = {0};

int intputCount	= 0;
int outputCount	= 0;
extern char __attribute__ ((section(".algo_buff"))) register_data[];
WITIN_CONFIG_T config;
WITIN_HOOK_T hook;

#if defined(CONFIG_RAM_UART_PESQ) || defined(CONFIG_RAM_JTAG_PESQ) || defined(CONFIG_RAM_UART_VERIFY_HSI24Mx6)

static int8_t g_regfile_bkup0[8192] = {0};

void npu_regfile_switch(uint8_t	is_soc2npu, int8_t *buffer){
    wnpu_set_npu_colck_gate(1);
    if(is_soc2npu){
	wnpu_data_soc2npu((char	*)buffer, 0, 8192);
    }else{
	wnpu_data_npu2soc(0, buffer, 8192);
    }
    wnpu_set_npu_colck_gate(0);
}

#endif


extern void coroutine_enable(void);
extern void coroutine_disable(void);
extern int coroutine_is_enable(void);

extern void wnpu_wfi_prepare(void) ;
extern void wnpu_wfi_wait(void);
extern void wnpu_wfi_cleanup(void);

extern void time_debug(int status, char	*str);
extern void yield();
void witin_layer_handler(WitinCallBackInfo_T info, WITIN_CALL_TYPE_t type) {
   
#if 0
    if(info.flow_index == 0 && type == LAYER_START) {
	time_debug(0, "in");
    }

    if(info.flow_index == 27 &&	type ==	LAYER_END) {
	char test[40] =	{0};
	sprintf(test,"flow_%d_round_%d",info.flow_index, info.round_index);
	time_debug(1, test);
    }
#else
    #if	0
    if(type == LAYER_RUN) {
	if(info.net_index == 1 && coroutine_is_enable()	&& (info.flow_index == 1 || 
	      info.flow_index == 3 ||
	      info.flow_index == 15 ||
	      info.flow_index == 19 || 
	      info.flow_index == 20)) {
		  yield();
	} else {
	    wnpu_wfi_prepare();
	    wnpu_wfi_wait();
	    wnpu_wfi_cleanup();
	}
    }
    #endif
#endif
}

int cnn_init() {
    memset(&config, 0, sizeof(WITIN_CONFIG_T));
    memset(&hook, 0, sizeof(WITIN_HOOK_T));
    
    config.isUseDiff = 0;
    config.isUseWfi = 1;
    config.isUseDmaTransport = 0;
    config.model_data_addr = (unsigned char *)register_data;
    config.allocFunc = pvPortMalloc;
    config.freeFunc = vPortFree;
    hook.logFunc = printf;
    hook.layerCallBack = witin_layer_handler;
    hook.log_level = LOG_ERROR;

    wengine_set_hook(&hook);

    int	res = wengine_init(&config);
    if(res < 0)	{
	printf("wengine	err\n");
	 return	-1;
    }else{
	printf("wengine	init ok\n");
    }

    #if	defined(CONFIG_RAM_UART_PESQ) || defined(CONFIG_RAM_JTAG_PESQ) || defined(CONFIG_RAM_UART_VERIFY_HSI24Mx6)
    witin_reset_net(1, 2);
    //npu_regfile_switch(0, g_regfile_bkup0);
    #endif

    wengine_get_input_info(NET_IDX_0, &inputInfo[NET_IDX_0], &intputCount);
    wengine_get_output_info(NET_IDX_0, &outputInfo[NET_IDX_0], &outputCount);

    return 0;
}

#if USE_ENC

unsigned char net_input_mic[512];
unsigned char net_input_far[512];
void enc_nn_process(unsigned char *input_mic, unsigned char *input_ref,	signed char *output) {

    self_fast_copy(net_input_mic,&net_input_mic[256], 256);
    self_fast_copy(&net_input_mic[256],input_mic, 256);
    self_fast_copy(net_input_far,&net_input_far[256], 256);
    self_fast_copy(&net_input_far[256],input_ref, 256);

    wengine_set_input_data(&inputInfo[ENC_IDX],	net_input_far);
    wengine_set_input_data(&inputInfo[ENC_IDX+1], net_input_mic);


    wengine_process_net(ENC_IDX,g_ctx_switch,g_ctx_switch,2);
    wengine_get_output_data(&outputInfo[ENC_IDX], output);

}

void npu_infer (unsigned char *input, signed char *output) {

    wengine_set_input_data(&inputInfo[NET_IDX_0], input);
    wengine_process_net(NET_IDX_0, 0, 0, 2);
    wengine_get_output_data(&outputInfo[NET_IDX_0], output);

}
#endif //USE_ENC

#if USE_KWS
unsigned char kws_net_in[280] =	{0};
void kws_nn_process(int	frame_mod, unsigned char *input, signed	char *output) {

    //for(int i=0;i<inputInfo[KWS_IDX].len;i++){
    //	  kws_net_in[i]	= input[i];
    //}
    memcpy(kws_net_in, input, inputInfo[KWS_IDX].len);

    wengine_set_input_data(&inputInfo[KWS_IDX],	kws_net_in);
    wengine_process_net(KWS_IDX,g_ctx_switch,g_ctx_switch,2);
    wengine_get_output_data(&outputInfo[KWS_IDX],output);
}

void nn_switch(uint8_t switch_flag){
    g_ctx_switch = switch_flag;
}

void get_kws_input_output(int *inputDim, int *outputDim) {
    *inputDim =	inputInfo[KWS_IDX].len;
    *outputDim = outputInfo[KWS_IDX].len;
}
#endif // USE_KWS

#if defined(CONFIG_RAM_UART_PESQ) || defined(CONFIG_RAM_JTAG_PESQ) || defined(CONFIG_RAM_UART_VERIFY_HSI24Mx6)
void wtm2101_reset_npu(){
    wengine_init(NULL);
    npu_regfile_switch(1, g_regfile_bkup0);
    witin_reset_net(1, 2);
}
#endif