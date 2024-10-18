#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

#include <basic_config.h>
#include <wtm2101_config.h>
#include <WTM2101.h>
#include <wtm2101_hal.h>
#include <bb04p1_hal.h>

#include "audio.h"
#include "dma.h"
#include "fbank.h"
#include "gpio.h"
#include "i2c.h"
#include "i2s.h"
#include "npu.h"
#include "pmu.h"
#include "rcc.h"
#include "rtc.h"
#include "uart.h"
#include "gpio.h"
#include "spi.h"

#include "main.h"

#include "wtm2101_mmap.h"
#include "retarget.h"

#include "gpio_config.h"
#include "audio_config.h"
#include "fbank_config.h"
#include "rtc_config.h"
#include "uart_config.h"
#include "uart_tx_dma.h"
#include "uart_rx_dma.h"
#include "spi_config.h"

#include "witin_npu_engine.h"
#include "wnpu_config.h"
//#include "denoise_process.h"
#include "cnn_framework.h"
//#include "kws_process.h"

#include "riscv_intrinsic.h"
#include "ucontext.h"
#include "ram_ld.h"

#include "crc.h"
#include "datalink.h"
#include "applink_rv32.h"
#include "jtaglink_rv32.h"

#include "rsm_driver.h"
#include "heap.h"
#include "LibNPU.h"
#include "witin_npu_interface.h"

#include "protocol_hci.h"
#include "wtm2101ac_protocol.h"

#include "wtm2101_chipid.h"

#include "hal_i2s.h"
#include "hal_audio.h"

// aec
//#include "aec_core.h"
//agc
//#include "witin_agc.h"
// linear aec
//#include "fdaf_aec.h"

//#include "kws_switch.h"
//#include "kws_with_mcra.h"
#include "noise_suppression_mcra.h"
//#include "net_dnn.h"
//#include "kws_check_pause_music_again.h"
#include "rnnoise.h"

#define	MIN(X, Y)  ((X)	< (Y) ?	(X) : (Y))
#define	MAX(X, Y)  ((X)	< (Y) ?	(Y) : (X))
#define	HOP_LEN	      (480)
int iis_init(void);
void stop_i2s(void);
int i2s0_start_or_stop(uint8_t state);
int i2s1_start_or_stop(uint8_t state);
static void system_clock_init_internal_24M(uint8_t mul_ratio);

//denoise_handle_t* denoise[1] = {0};  //enc
//denoise_handle_t_ref*	denoise_ref[1] = {0}; //enc ref
//AecObject_t* aecInstace=0;
//AecCore aec_inst;		       //aec
//LegacyAgc agc_handle = {0};	     //agc

uint8_t	g_i2s0_state = DISABLE;//ENABLE
uint8_t	g_i2s1_state = DISABLE;
uint8_t	g_audio_state =	DISABLE;
uint8_t	g_linein_state = DISABLE;

uint32_t g_audio_not_ok_cnt = 0;
uint32_t g_audio_analog_register[4] = {0};
uint8_t	g_gpio13_pu = 0;

int open_audio_stereo(void);
int open_audio_mono(void);
void stop_audio(void);
void close_audio(void);

#if !defined(CONFIG_RAM_UART_VERIFY_HSI24Mx6) && !defined(CONFIG_RAM_UART_PESQ)	&& !defined(CONFIG_RAM_JTAG_PESQ)
static int hal_start_audio(uint8_t start_linein);
static int hal_stop_audio(uint8_t stop_linein);
#endif

/////////************************************************************/////////////////////////
typedef	enum {
    RUN_MODE_KWS      =	0,
    RUN_MODE_AEC_ENC  =	1,
    RUN_MODE_ENC_KWS  =	2,
    RUN_MODE_EMPTY  = 255,
}run_mode_t;

struct wtm2101ac_protoc_status g_proto_status =	{
    .run_mode =	RUN_MODE_EMPTY,
    .use_aec = 0,
    .use_enc = 0,
    .use_kws = 0,
    .wanna_sleep = 0,
    .start_sleep = 0,
    .wakeup = 0,
    .start_call	= 0,		  // auto enc
    .start_bypass = 0,
    .stop_call = 0,
    .start_kws = 0,
    .start_enhanced_kws	= 0,	  // auto kws
    .stop_kws =	0,
    .use_kws_i2s = 0,
    .buffer_finished = 0,
    .nw_major_version =	253,
    .nw_minor_version =	254,
    .nw_fixed_version =	255,
    .gain_level	= 7,
    .aec_gain_level = 10,
    .enc_level = 10,
    .mute_state	= 0,
    .chipid = 0,
};

#define	WTM_RECEIVE_DATA_SIZE_MAX  128
static uint8_t proto_rx_buf[WTM_RECEIVE_DATA_SIZE_MAX];
//static int8_t	g_regfile_bkup0[8192] =	{0};
//static int8_t	g_regfile_bkup1[8192] =	{0};

////////////////////////////////////////////////////////////////////////////////
//
//  FIXED placement memory, which alloc	explicit in ram.ld
//
//#ifndef	HOP_LEN
//#error "missing	HOP_LEN"
//#endif

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#ifdef AUDIO_ENABLE
// 32bits access, DMA read audio::ram, write audio_noisy_buffer
int16_t	audio_noisy_buffer[AUDIO_FRAME_SIZE];
int16_t	audio_clean_buffer[AUDIO_FRAME_SIZE];
int16_t	audio_zero_buffer[AUDIO_FRAME_SIZE];

volatile uint32_t audio_ram_buffer_write_flag =	0;  // write: audio_handle.c, read: this file
volatile uint32_t audio_ram_buffer_read_flag = 0;   // read-write: here
volatile uint16_t audio_dma_flag = 0;		    // write: audio_handle.c, read: this file

volatile uint16_t audio_ram_buffer[500]	= {0};
volatile uint16_t audio_ram_buffer_out[500] = {0};
#endif

#if USE_AGC
void init_agc();
#endif

//#if USE_ENC
//void init_enc();
//#endif

void Dump_System_Config(void);
void low_power_handle(uint64_t tick);
void vad_handle(void);

#ifdef CONFIG_RAM_KWS_SPI
#undef USE_SPI
#endif

////////////////////////////////////////////////////////////////////////
//#define STACK_SIZE_NPU   ( 1024/sizeof(uint32_t))
//#define STACK_SIZE_ALGO  (7000/sizeof(uint32_t))
//static ucontext_t	  ctx_algo;
//static ucontext_t	  ctx_npu;
//static ucontext_t*	  ctx_running =	NULL;
//static volatile int	  ctx_switch_enable = 1;

#if !defined(CONFIG_RAM_UART_VERIFY_HSI24Mx6) && !defined(CONFIG_RAM_UART_PESQ)	&& !defined(CONFIG_RAM_JTAG_PESQ)
static void interrupt_handle_example(void)
{
    /*The hal audio instance1*/
    Audio_InitTypeDef* hal_audio_instance1 = hal_audio_instance_get(HAL_AUDIO_INSTANCE1);
    /*The hal audio instance2*/
    Audio_InitTypeDef* hal_audio_instance2 = hal_audio_instance_get(HAL_AUDIO_INSTANCE2);
    //Audio_InitTypeDef* hal_audio_instance3 = hal_audio_instance_get(HAL_AUDIO_INSTANCE3);

    uint8_t audio_flag[3];
    audio_flag[0] = AUDIO_Get_FIFO_Interrupt_Status(hal_audio_instance1->instance, hal_audio_instance1->channel.ChannelNumber) & HAL_AUDIO_BUFFER_FIFO_HALF_FULL_INTERRUPT;
    audio_flag[1] = AUDIO_Get_FIFO_Interrupt_Status(hal_audio_instance2->instance, hal_audio_instance2->channel.ChannelNumber) & HAL_AUDIO_BUFFER_FIFO_HALF_FULL_INTERRUPT;
    //audio_flag[2] = AUDIO_Get_FIFO_Interrupt_Status(hal_audio_instance3->instance, hal_audio_instance3->channel.ChannelNumber) & HAL_AUDIO_BUFFER_FIFO_HALF_FULL_INTERRUPT;

    do {
	if (audio_flag[0])
	{
	    /*the data is push to ring buffer with hal audio instance1*/
	    hal_audio_instance1->Data_handle_info.audio_receive_handler(hal_audio_instance1);
	}

	if (audio_flag[1])
	{
	    /*the data is push to ring buffer with hal audio instance2*/
	    hal_audio_instance2->Data_handle_info.audio_receive_handler(hal_audio_instance2);
	}
	//if (audio_flag[2])
	//{
	//    /* the data is push to ring buffer with hal audio	instance */
	//    hal_audio_instance3->Data_handle_info.audio_receive_handler(hal_audio_instance3);
	//}

	audio_flag[0] =	AUDIO_Get_FIFO_Interrupt_Status(hal_audio_instance1->instance, hal_audio_instance1->channel.ChannelNumber) & HAL_AUDIO_BUFFER_FIFO_HALF_FULL_INTERRUPT;
	audio_flag[1] =	AUDIO_Get_FIFO_Interrupt_Status(hal_audio_instance2->instance, hal_audio_instance2->channel.ChannelNumber) & HAL_AUDIO_BUFFER_FIFO_HALF_FULL_INTERRUPT;
	//audio_flag[2]	= AUDIO_Get_FIFO_Interrupt_Status(hal_audio_instance3->instance, hal_audio_instance3->channel.ChannelNumber) & HAL_AUDIO_BUFFER_FIFO_HALF_FULL_INTERRUPT;
    }while(audio_flag[0] || audio_flag[1]);

}

static volatile	int audio_frame_ready =	0;

void AUDIO_IRQHandler(void)
{
    #if	USE_DEBUG_PIN
    GPIO_OutputHi(GPIOA, GPIO_PIN_4);
    #endif

    Audio_InitTypeDef* hal_audio_instance = hal_audio_instance_get(HAL_AUDIO_INSTANCE1);
    if(AUDIO_Get_Ram_Interrupt_Status(hal_audio_instance->instance,hal_audio_instance->channel.ChannelNumber) &	HAL_AUDIO_BUFFER_RAM_VLD_INTERRUPT) {
	AUDIO_Clear_Ram_Interrupt(hal_audio_instance->instance,hal_audio_instance->channel.ChannelNumber,HAL_AUDIO_BUFFER_RAM_VLD_INTERRUPT);
	//hal_audio_instance->Data_handle_info.audio_receive_handler(hal_audio_instance);
	audio_frame_ready = 1;
    } else {
	interrupt_handle_example();
    }

    #if	USE_DEBUG_PIN
    GPIO_OutputLo(GPIOA, GPIO_PIN_4);
    #endif
}

#endif

#if USE_PROTOCOL
#ifdef UART1_TX_PIN
void UART1_IRQHandler(void)
{
    ECLIC_ClearPendingIRQ(UART1_IRQn);
    uint8_t data_receive = UART_ReceiveData(UART1);
    int	ret = protocol_hci_receive_data_slip_handle(data_receive);
    if(ret){
	g_proto_status.buffer_finished = 1;
    }
}
#else
void UART0_IRQHandler(void)
{
    //GPIO_Toggle(GPIOA,GPIO_PIN_5);
    ECLIC_ClearPendingIRQ(UART0_IRQn);
    uint8_t data_receive = UART_ReceiveData(UART0);
    int	ret = protocol_hci_receive_data_slip_handle(data_receive);
    if(ret){
	g_proto_status.buffer_finished = 1;
    }
}
#endif

#endif

#define	NUM_VAD_STAT_SAVED 200
//#define GATE_AFTER_MCRA_DENOISE 50
int8_t g_is_get_kw = 0;
int8_t his_mcra_res[NUM_VAD_STAT_SAVED]	= {0};
int his_mcra_res_idx = 0;
int _id;
float _score;
char* sil = "silence";
char* noisy = "noisy";

void kws_get_res_callback(int id, float	score, int isSuccess){
    g_is_get_kw	= 1;
    _id	= id;
    _score = score;
    /*#if UART_ENABLE
    printf("%s-%d\r\n",get_kws_tips_cmds_byid(id),(int)(score*1000));
    #endif

    #if	USE_PROTOCOL
    wtm2101ac_send_kws_to_master(id);
    #endif*/
}

//void kws_check_again(int8_t mcra_vad_stat,int	voice_lenth)
//{
//    //printf("----------- his_mcra_res -----------\r\n");

//    int8_t is_check_OK = 0;

//    int8_t backgroud_stat = get_cur_backgroud_state();
//    char* str_backgroud_stat = NULL;
//    if(backgroud_stat) str_backgroud_stat=noisy;
//    else str_backgroud_stat=sil;

//    /*
//    for(int i=0;i<NUM_VAD_STAT_SAVED;i++)
//    {
//	printf("%d,",his_mcra_res[i]);
//    }
//    printf("\n");
//    */
//    // ==================================== mcra check ==================================
//    if ((MCRA_VAD_SIL	== mcra_vad_stat)	|| (MCRA_VAD_VOICE2SIL==mcra_vad_stat) || (MCRA_VAD_VOICE_GAP==mcra_vad_stat)) {

//	//if((VOICE_LEN_MIN<=voice_lenth) && (voice_lenth<=VOICE_LEN_MAX))
//	if (voice_lenth<=VOICE_LEN_MAX)
//	{
//	    is_check_OK	= 1;
//	}
//	else
//	{
//	    is_check_OK	= 0;
//	    //printf("0712D:ERR	voice len:%s-%d,vad status=%d,voice len	%d\r\n",get_kws_tips_cmds_byid(_id),(int)(_score*1000),mcra_vad_stat,voice_lenth);
//	}
//    }
//    else
//    {
//	 is_check_OK = 0;
//	 //printf("0712D:ERR vad stat:%s-%d,vad	status=%d,voice	len %d\r\n",get_kws_tips_cmds_byid(_id),(int)(_score*1000),mcra_vad_stat,voice_lenth);
//    }

//    //==================================== starting VAD	stat check ==================================
//    if (is_check_OK  &&	(0==backgroud_stat) )
//    {
//	 int num_zeros = 0;
//	 int8_t	is_start = 0;
//	 for (int i = NUM_VAD_STAT_SAVED - 1; i	> 0; i--)
//	 {
//	     int8_t stat = his_mcra_res[i];
//	     if(MCRA_VAD_SIL2VOICE == stat) is_start = 1;
//	     if(is_start && (MCRA_VAD_SIL==stat)) num_zeros++;

//	     if((num_zeros>0) && (MCRA_VAD_SIL!=stat)) break;
//	 }
//	 //printf("num_zeros=%d\n",num_zeros);
//	 if(num_zeros<=30)
//	 {
//	     is_check_OK = 0;
//	     //printf("0712D:ERR starting VAD check fail: num_zeros=%d\n",num_zeros);
//	 }
//    }

//    //==================================== ending VAD	stat check ==================================

//    if (is_check_OK)
//    {
//	//int num_vad_gap = 0;
//	int gap_start =	0;
//	int gap_end = NUM_VAD_STAT_SAVED-30;
//	for(int	i=NUM_VAD_STAT_SAVED-1;i>NUM_VAD_STAT_SAVED-30;i--)
//	{
//	    if((MCRA_VAD_VOICE_GAP==his_mcra_res[i]) &&(0 == gap_start))
//	    {
//		gap_start = i;
//	    }
//	    if(	(gap_start>0) && (MCRA_VAD_VOICE_GAP!=his_mcra_res[i]) )
//	    {
//		gap_end	= i;
//		break;
//	    }
//	}
//	//if( (0!=gap_start) &&	(0==gap_end))
//	int last_gap_num = gap_start - gap_end;
//	//printf("last gap num:%d,%d,%d,\n",last_gap_num,gap_end,gap_start);
//	// 1, the passed stats may be all zeros,  such as: 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,	so 0!=gap_start,
//	// 2, we only check the	recently states,so gap_end>=15
//	// 3, num gap should have enough,  last_gap_num<15 will	be err report.
//	if((0!=gap_start) && (gap_end>=NUM_VAD_STAT_SAVED-15) && (last_gap_num<15))
//	{
//	    is_check_OK	= 0;
//	    //printf("0712D:ERR	num vad	gap:%d,%s,gap_start,gap_end,last_gap_num:%d,%d,%d\n",last_gap_num,get_kws_tips_cmds_byid(_id),gap_start,gap_end,last_gap_num);
//	}
//    }

//    //==================================== MCRA_VAD_VOICE state	lenth of segment check ==================================
//    int8_t is_start =	0;
//    int	num_voice = 0;
//    for	(int i = NUM_VAD_STAT_SAVED - 1; i > 0;	i--)
//    {
//	int8_t a = his_mcra_res[i];
//	if (MCRA_VAD_VOICE == a)
//	{
//		num_voice++;
//	}
//	if ((num_voice > 0) && (MCRA_VAD_SIL2VOICE == a)) break;
//    }
//    // do this check when back ground	is noisy
//    if (is_check_OK && (0==backgroud_stat)  && (num_voice<5) )
//    {
//	is_check_OK = 0;
//	//printf("0712D:ERR MCRA_VAD_VOICE len check fail,num_voice=%d,%s\n ",num_voice,get_kws_tips_cmds_byid(_id));
//    }
//    // ==================================== kcpma check	==================================
//    if (is_check_OK && (2<=_id)	&& (_id<=5) ) // when get next/prev song
//    {
//	int kcpma_ret =	kcpma_forward();
//	if (kcpma_ret) {
//	    //printf("one-hot check OK\n ");
//	    is_check_OK	= 1;
//	}
//	else{
//	    printf("0909B:ERR one-hot  check fail\n ");
//	    is_check_OK	= 0;
//	}
//	// if back groud is silence, we	will reduce score gate
//	 //if(	(0==backgroud_stat) && (_score>=1.0) ) {
//	 if(_score>=1.0) {
//	   printf("0909B:ERR get %s,but	score too big %d\n ",get_kws_tips_cmds_byid(_id),(int)(_score*1000));
//	   is_check_OK = 0;
//	 }
//    }

//    if(is_check_OK)
//    {
//	  #if UART_ENABLE
//	  //printf("0712D:OK %s-%d,voice_lenth:%d, back	groud %s\r\n",get_kws_tips_cmds_byid(_id),(int)(_score*1000),voice_lenth,str_backgroud_stat);
//	  printf("%s-%d,VER:%d.%d.%d,%s\r\n",get_kws_tips_cmds_byid(_id),(int)(_score*1000),FW_MAJOR_VERSION,FW_MINOR_VERSION,FW_FIXED_VERSION,str_backgroud_stat);
//	  #endif
//	  #if USE_PROTOCOL
//	  wtm2101ac_send_kws_to_master(_id);
//	  #endif
//    }



//}

static float agc_current_gain=1;
static float scale_ = 0;
static uint32_t	gain_now = 0;
static float pownum = 0;
//static float agc_level = 0.6;
static float agc_th = 16;
static const float reverse_gain_table[]	= {
    1.57, // agc.pgaval= 0, gain=3.93 dB
    1.49, // agc.pgaval= 1, gain=3.47 dB
    1.42, // agc.pgaval= 2, gain=3.04 dB
    1.35, // agc.pgaval= 3, gain=2.63 dB
    1.30, // agc.pgaval= 4, gain=2.25 dB
    1.24, // agc.pgaval= 5, gain=1.89 dB
    1.20, // agc.pgaval= 6, gain=1.57 dB
    1.16, // agc.pgaval= 7, gain=1.27 dB
    1.12, // agc.pgaval= 8, gain=1.00 dB
    1.09, // agc.pgaval= 9, gain=0.75 dB
    1.06, // agc.pgaval=10, gain=0.54 dB
    1.04, // agc.pgaval=11, gain=0.36 dB
    1.03, // agc.pgaval=12, gain=0.22 dB
    1.01, // agc.pgaval=13, gain=0.10 dB
    1.00, // agc.pgaval=14, gain=0.03 dB
    1.00, // agc.pgaval=15, gain=0.00 dB
};

//static float get_audio_agc_gain() {
//    // audio gain
//    Audio_InitTypeDef* hal_audio_instance1 = hal_audio_instance_get(HAL_AUDIO_INSTANCE1);
//    if(hal_audio_instance1->Analog.AGC_Enable	== 1) {
//	gain_now = hal_audio_instance1->instance->ANA18CFG & 0x3F;
//	if(gain_now < agc_th &&	 denoise_get_current_mask_average(denoise[0]) <	0.3) {
//	    //pownum = (((float)gain_now - (float)agc_th) * 0.75)/20.0;
//	    //scale_ = 1.0 / pow(10, pownum);
//	    //scale_ = (scale_-1.0) * agc_level	+ 1.0;
//	    scale_ = reverse_gain_table[gain_now];
//	} else {
//	    scale_ = 1.0;
//	}
//    }	else {
//	scale_ = 1.0;
//    }
//    agc_current_gain = agc_current_gain	* 0.75 + scale_	* 0.25;
//    if(agc_current_gain	< 1) agc_current_gain =	1;
//    if(agc_current_gain	> 4) agc_current_gain =4;
//    printf("current gain %f\n",agc_current_gain);
//    return agc_current_gain;
//}

#if 0
void coroutine_enable(void)
{
    ctx_switch_enable =	1;
}

void coroutine_disable(void)
{
    ctx_switch_enable =	0;
}

int coroutine_is_enable(void)
{
    return ctx_switch_enable;
}

void yield(void)
{
    if (!ctx_switch_enable) {
	return;
    }

    //usage:
    //	swapcontext(&save_current, &switch_new);
    //
    if (ctx_running == &ctx_algo) {
	ctx_running = &ctx_npu;
	swapcontext(&ctx_algo, &ctx_npu);
    } else {
	ctx_running = &ctx_algo;
	swapcontext(&ctx_npu, &ctx_algo);
    }
}
#endif

void time_debug(int status, char *str) {
    #if	defined(UART1_TX_PIN)
    return;
    #else
    static uint64_t t0;
    static uint64_t t1;
    if(status == 0) {
       t0 = __get_rv_cycle();
    } else {
	t1 = __get_rv_cycle();
	uint64_t ahbclock = RCC_Get_SYSClk() / (RCC_AHB_Get_ClkDiv() + 1);
	int res	= (int)((t1-t0)	* 1000 * 1000/ahbclock);
	printf("%2s: %d	us\r\n", str, res);
    }
    #endif
}

void time_debug2(int status, char *str)	{
    #if	1
    return;
    #else
    static uint64_t t0;
    static uint64_t t1;
    if(status == 0) {
       t0 = __get_rv_cycle();
    } else {
	t1 = __get_rv_cycle();
	uint64_t ahbclock = RCC_Get_SYSClk() / (RCC_AHB_Get_ClkDiv() + 1);
	int res	= (int)((t1-t0)	* 1000 * 1000/ahbclock);
	printf("%2s: %d	us\r\n", str, res);
    }
    #endif
}

int get_delta_ms_gpio13(int status){
    static uint64_t t0;
    static uint64_t t1;
    if(status == 0) {
       t0 = __get_rv_cycle();
       return 0;
    }
    t1 = __get_rv_cycle();
    uint64_t ahbclock =	RCC_Get_SYSClk() / (RCC_AHB_Get_ClkDiv() + 1);
    int	delta_ms = (int)((t1-t0) * 1000	/ahbclock);
    if(delta_ms<0){
	t0 = 0;
	t1 = 0;
    }
    return delta_ms;
}

void debug_init() {
      RCC_CLK_EN_Ctl(RCC_GPIO_HCLKEN,ENABLE);
      GPIO_InitTypeDef GPIO_InitStructure;
      GPIO_InitStructure.Pin  =	GPIO_PIN_4|GPIO_PIN_5;
      GPIO_InitStructure.Mode =	GPIO_MODE_OUT;
      GPIO_InitStructure.Alternate = GPIO_AF4_GPIO;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      GPIO_InitStructure.Pin  =	GPIO_PIN_6|GPIO_PIN_7;
      GPIO_InitStructure.Mode =	GPIO_MODE_OUT;
      GPIO_InitStructure.Alternate = GPIO_AF6_GPIO;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      GPIO_OutputLo(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
}

// AUDIO_IRQn wake this
void wfi_wait(void) {
    PMU_Standby_Mode_Cmd(PMU);
    __WFI();
}

void led_init(void) {
    RCC_CLK_EN_Ctl(RCC_GPIO_HCLKEN,ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin	    = LED_PIN;
    GPIO_InitStructure.Mode	    = GPIO_MODE_OUT;
    GPIO_InitStructure.Alternate    = LED_AF;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    led_hi();
}

#if USE_SPI
uint8_t	spi_send_buffer[128] = {0};
uint8_t	spi_send_len = 0;
void init_gpio14_nss(){
    GPIO_DeInit(GPIOA, GPIO_PIN_14);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_14;
    GPIO_InitStructure.Alternate = GPIO_AF14_GPIO;
    GPIO_InitStructure.Mode = GPIO_MODE_INPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ECLIC_ClearPendingIRQ(GPIO_IRQn);
    ECLIC_SetPriorityIRQ(GPIO_IRQn, 1);
    ECLIC_SetTrigIRQ(GPIO_IRQn,	ECLIC_LEVEL_TRIGGER);
    ECLIC_EnableIRQ(GPIO_IRQn);

    GPIO_IT_InitTypeDef	GPIO_IT_InitStructure;
    GPIO_IT_InitStructure.Pin =	GPIO_PIN_14;
    GPIO_IT_InitStructure.Trig = GPIO_IT_TRIG_FALLING;
    GPIO_IT_InitStructure.State	= ENABLE;
    GPIO_IT_Cfg(GPIOA, &GPIO_IT_InitStructure);

    GPIO_IT_MskCfg(DISABLE);
    GPIO_IT_PinMskCfg(GPIOA,GPIO_PIN_14,DISABLE);
    PMU_Set_Ie_Msk(PMU,PMU_GPIO_IMSK | PMU_GPIO_EMSK,ENABLE);
}

void init_gpio14_sleep(){
    GPIO_DeInit(GPIOA, GPIO_PIN_14);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_14;
    GPIO_InitStructure.Mode = GPIO_MODE_INPU;
    GPIO_InitStructure.Alternate = GPIO_AF14_GPIO;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_IT_InitTypeDef	IT_InitTypeDef;
    IT_InitTypeDef.Pin = GPIO_PIN_14;
    IT_InitTypeDef.Trig	= GPIO_IT_TRIG_RISING;
    IT_InitTypeDef.State = ENABLE;
    GPIO_IT_Cfg(GPIOA,&IT_InitTypeDef);
}

void close_gpio14_nss(){
    GPIO_IT_PinMskCfg(GPIOA,GPIO_PIN_14,DISABLE);
    PMU_Set_Ie_Msk(PMU,PMU_GPIO_IMSK | PMU_GPIO_EMSK,DISABLE);
    ECLIC_DisableIRQ(DISABLE);
    GPIO_IT_InitTypeDef	GPIO_IT_InitStructure;
    GPIO_IT_InitStructure.Pin =	GPIO_PIN_14;
    GPIO_IT_InitStructure.Trig = GPIO_IT_TRIG_FALLING;
    GPIO_IT_InitStructure.State	= DISABLE;
    GPIO_IT_Cfg(GPIOA, &GPIO_IT_InitStructure);
    GPIO_DeInit(GPIOA, GPIO_PIN_14);
}

void init_spi14(){
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_14;
    GPIO_InitStructure.Alternate = GPIO_AF14_SPIS;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void empty_spi_rcv_fifo(){
     while(SPI_RECEIVE_FIFO_NOT_EMPTY&SPI_Get_Status_Cmd(SPIS)){
	volatile uint8_t _data = *((uint8_t *)&SPIS->DR);
    }
}

void SPIS_IRQHandler(void)
{
    uint32_t status = SPI_Get_Interrupt_Status(SPIS);
    if (status & SPI_RECEIVE_FIFO_FULL_INT) {
	do{
	  uint8_t _data	= *((uint8_t *)&SPIS->DR);
	  int ret = protocol_hci_receive_data_slip_handle(_data);
	  if(ret){
	      g_proto_status.buffer_finished = 1;
	  }
	}while(SPI_RECEIVE_FIFO_NOT_EMPTY&SPI_Get_Status_Cmd(SPIS));
    }
}

static uint8_t init_spi14_flag = 0;

void GPIO_IRQHandler(void) {

    uint8_t data_idx = 0;
    while(data_idx<spi_send_len){
	while(SPI_TRANSMIT_FIFO_NOT_FULL&SPI_Get_Status_Cmd(SPIS)){
	    SPI_Send(SPIS, 1, &spi_send_buffer[data_idx++], 1);
	    if(data_idx>=spi_send_len){
		break;
	    }
	}
    }
    init_spi14_flag = 1;
    GPIO_IT_ClrPending(GPIOA, GPIO_PIN_14);
    ECLIC_ClearPendingIRQ(GPIO_IRQn);
}

void GPIO_WAKEUP_IRQHandler(void) {
    GPIO_IT_ClrWakeUpPending(GPIOA, GPIO_PIN_14);
    GPIO_IT_ClrPending(GPIOA, GPIO_PIN_14);
}

void empty_spi_data(){
    init_spi14_flag = 0;
    while(!GPIO_ReadInputDataBit(GPIOA,	GPIO_PIN_14)){};
    empty_spi_rcv_fifo();
    close_gpio14_nss();
    init_spi14();
    SPI_Set_Rx_Fifo_Full_Interrupt(SPIS, ENABLE);
    SPI_Cmd(SPIS, DISABLE);
    SPI_Cmd(SPIS, ENABLE);
}

int init_spi_slave(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef spi_init_type;
    RCC_CLK_EN_Ctl(RCC_SPIS_PCLKEN,ENABLE);

    SPI_Uninit(SPIS);

    GPIO_InitStructure.Pin = GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_16 | GPIO_PIN_17;    /* spi slave pin	config */
    GPIO_InitStructure.Alternate = GPIO_AF14_SPIS | GPIO_AF15_SPIS | GPIO_AF16_SPIS | GPIO_AF17_SPIS;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    spi_init_type.cpol = SPI_CPOL_0;		 /* cpol 0 */
    spi_init_type.cpha = SPI_CPHA_1;		 /* cpha 0 */
    spi_init_type.datasize = SPI_DATASIZE_8B;	 /* spi	data width 8 bit */
    SPI_Init(SPIS, &spi_init_type);

    SPIS->IMR =	0;				    /* disable all interrupt */
    SPI_Set_Rx_Fifo_Threshold(SPIS, 0);
    SPI_Set_Rx_Fifo_Full_Interrupt(SPIS, ENABLE);

    /* init spi	interrupt */
    ECLIC_ClearPendingIRQ(SPIS_IRQn);
    ECLIC_SetPriorityIRQ(SPIS_IRQn, 2);
    ECLIC_SetTrigIRQ(SPIS_IRQn,	ECLIC_POSTIVE_EDGE_TRIGGER);
    ECLIC_EnableIRQ(SPIS_IRQn);

    SPI_Cmd(SPIS, ENABLE);

    return 0;
}

int spi_send_data(uint8_t *buffer, uint32_t len){
    memcpy(spi_send_buffer, buffer, len);
    spi_send_len = len;
    return 0;
}

void spi_send_irq(){
    init_gpio14_nss();
    SPI_Set_Rx_Fifo_Full_Interrupt(SPIS, DISABLE);
    GPIO_OutputHi(GPIOA, GPIO_PIN_13);
    g_gpio13_pu	= 1;
    get_delta_ms_gpio13(0);
    return;
}

#endif

///////////////////////////////////////////////////////////////////////

//void kwsProcessHooker(KWS_STAGE_T stage) {
//    //if(coroutine_is_enable())	{
//    //	  yield();
//    //}
//}

#ifdef AUDIO_ENABLE
void dmacpy_audio_to_mcu(void)
{
#ifndef	I2S_ENABLE
    RCC_CLK_EN_Ctl(RCC_DMA_CLKEN, ENABLE);
#endif

    DMA_Set_Addr(DMA,
	    AUDIO_DMA_CHANNEL,
	    mmap_to_sys((uint32_t)&(AUD->RAM0DATA)),
	    mmap_to_sys((uint32_t)audio_noisy_buffer),
	    AUDIO_DMA_COUNT,
	    0/*LLP=NULL, normal	mode*/);
    DMA_Set_Channel_Enable_Cmd(DMA, AUDIO_DMA_CHANNEL, ENABLE);
    while (!audio_dma_flag) {
    }
    audio_dma_flag = 0;

#ifndef	I2S_ENABLE
    RCC_CLK_EN_Ctl(RCC_DMA_CLKEN, DISABLE);
#endif
}
#endif


#if defined(CONFIG_RAM_UART_VERIFY_HSI24Mx6) ||	\
    defined(CONFIG_RAM_UART_PESQ) || \
    defined(CONFIG_QSPI_UART_VERIFY_HSI24Mx6) || \
    defined(CONFIG_RAM_UART_VERIFY_NPUTRIM) || \
    defined(CONFIG_QSPI_UART_VERIFY_NPUTRIM) ||	\
    defined(CONFIG_RAM_UART_VERIFY_MCLK)
#if (APPLINK_FRAME_SIZE	!= 160)	&& (HOP_LEN > 160)
#error "expect 160*int16_t frame size"
#endif

//#define POOR_DATA_LINK

int uart_recv_msg(applink_handle_t* self)
{
    uint8_t ch;
    int	nr_bytes;
    #if	defined(UART0_RX_PIN)
    nr_bytes = UART_RecvFIFO_Read(UART0, &ch, 1);
    #else
    nr_bytes = UART_RecvFIFO_Read(UART1, &ch, 1);
    #endif
    if (nr_bytes <= 0) {
	return 0;
    }
    if (!datalink_parse_char(ch, &self->rx_msg,	&self->parser))	{
	return 0;
    }

    self->msg =	&self->rx_msg;
    //if (self->msg->header.u.msgid == DATALINK_MSG_ID_HEARTBEAT) {
    //	  printf("Info:	recv heartbeat %6d\r\n", self->msg->header.u.seq);
    //}	else if	(self->msg->header.u.msgid == DATALINK_MSG_ID_FILE_ACK)	{
    //	  printf("Info:	recv ack       %6d\r\n", self->msg->header.u.seq);
    //}	else if	(self->msg->header.u.msgid == DATALINK_MSG_ID_FILE_INFO) {
    //	  printf("Info:	recv info      %6d\r\n", self->msg->header.u.seq);
    //}	else if	(self->msg->header.u.msgid == DATALINK_MSG_ID_FILE_DATA) {
    //	  printf("Info:	recv data      %6d\r\n", self->msg->header.u.seq);
    //}	else if	(self->msg->header.u.msgid == DATALINK_MSG_ID_FILE_CRC)	{
    //	  printf("Info:	recv crc       %6d\r\n", self->msg->header.u.seq);
    //}
    return self->msg->header.u.dlen + SIZEOF_DATALINK_MESSAGE_HEADER;
}

#ifdef POOR_DATA_LINK
int uart_send_msg(const	datalink_message_t* msg)
{
    static datalink_message_t buf;
    static uint32_t nr_packet_sent = 0;
    int	nr_bytes = SIZEOF_DATALINK_MESSAGE_HEADER + msg->header.u.dlen;

    memcpy(&buf, msg, nr_bytes);
    nr_packet_sent++;
    if ((nr_packet_sent	% 1000)	== 17) {
	printf("Simulation: force data lost\r\n");
	return nr_bytes;
    } else if ((nr_packet_sent % 1000) == 51) {
	buf.header.bytes[7] = ~buf.header.bytes[7];
    } else if ((nr_packet_sent % 1000) == 99) {
	buf.payload[2] = buf.payload[2]	+ 1;
    }

    UART_SendBlock_Async(UART0,	&buf, nr_bytes,	NULL);
    return nr_bytes;
}
#else
int uart_send_msg(const	datalink_message_t* msg)
{
    int	nr_bytes = SIZEOF_DATALINK_MESSAGE_HEADER + msg->header.u.dlen;
    #if	defined(UART0_TX_PIN)
    UART_SendBlock_Async(UART0,	msg, nr_bytes, NULL);
    #else
    UART_SendBlock_Async(UART1,	msg, nr_bytes, NULL);
    #endif
    return nr_bytes;
}
#endif

void uartxfer_callback_reset(void)
{

    init_enc();
    #if	USE_AGC
    init_agc();
    #endif

    wtm2101_reset_npu();

}

void uartxfer_callback_process(int16_t*	in, int16_t* out, int nr_bytes)
{

 static	int16_t	inL_mic[160];
    static int16_t inR_ref[160];

    static int aec_ref[160];
    static int aec_mic[160];
    static int aec_out[160];

    // to channel use aec
    if(nr_bytes	== 640)	{
	for (int i = 0;i < 160;i++) {
	    inL_mic[i] = in[2*i	+ 0];
	    inR_ref[i] = in[2*i	+ 1];
	    aec_ref[i] = (int)inR_ref[i];
	    aec_mic[i] = (int)inL_mic[i];
	}

	#if 0
	linear_aec_process(aecInstace, aec_ref,	aec_mic, aec_out);
	for (int i = 0;i < 160;i++) {
	    inL_mic[i] = aec_out[i];
	}
	#endif

    // use enc only
    } else if(nr_bytes == 320) {
	for (int i = 0;i < 160;i++) {
	    inL_mic[i] = in[i];
	}
	memset(inR_ref,	0, sizeof(int16_t) * 160);
    }

    int16_t tmp_out[160];
    denoise_process_16bits_ref(denoise_ref[0], inR_ref);
    denoise_process_16bits(denoise[0], denoise_ref[0], &agc_handle, inL_mic, tmp_out);

    if(nr_bytes	== 640)	{
	for (int i = 0;i < 160;i++) {
	    out[2*i + 0] = inL_mic[i];
	    out[2*i + 1] = tmp_out[i];
	}
    } else if(nr_bytes == 320) {
	for (int i = 0;i < 160;i++) {
	    out[i] = tmp_out[i];
	}
    }

}



// CONFIG_RAM_UART_VERIFY_MCLK:	    49MHz
// CONFIG_RAM_UART_VERIFY_HSI24Mx6: 147MHz
// CONFIG_QSPI_UART_VERIFY_HSI24Mx6: 147MHz
static void _task_algo_uartxfer_highspeed(void)
{

    static applink_handle_t inst;

    applink_init(&inst,	sizeof(int16_t)*HOP_LEN*2);

    //enc
    init_enc();

#if defined(CONFIG_RAM_UART_VERIFY_HSI24Mx6)
    denoise_switch(1);
#elif defined(CONFIG_RAM_UART_PESQ)
    denoise_switch(0);
#endif

    //aecInstace = linear_aec_init();

    //agc
    #if	USE_AGC
    init_agc();
    #endif

    cnn_init();

    // -tip-
    // UART_Handshake_Bugfix() must exist
    //
    UART_Handshake_Bugfix();
    #if	defined(UART0_TX_PIN)
    UART_RecvFIFO_Start(UART0);
    #else
    UART_RecvFIFO_Start(UART1);
    #endif
    printf("PESQ READY\r\n");
    printf_output_redirect_set(PRINTF_RETARGET_NONE);

    while (1) {
	#if 1
	inst.ts	= HAL_GetTick();
	uart_recv_msg(&inst);
	applink_send_heartbeat(&inst);
	applink_recv_heartbeat(&inst);
	applink_recv_file_ack(&inst);
	applink_recv_file_info(&inst);
	applink_recv_file_data(&inst);
	inst.msg = NULL;
	#else
	char ch;
	char msg[4] = {0};
	msg[0] = 'r';
	msg[1] = 'c';
	int nr_bytes = UART_RecvFIFO_Read(UART0, &ch, 1);
	if(nr_bytes){
	    msg[2] = ch;
	    UART_SendBlock_Async(UART0,	msg,4, NULL);
	}
	#endif

    }
}
#endif

#if !defined(CONFIG_RAM_UART_VERIFY_HSI24Mx6) && !defined(CONFIG_RAM_UART_PESQ)	&& !defined(CONFIG_RAM_JTAG_PESQ)
void stop_i2s(){
    #if	USE_I2S1
    if(g_i2s1_state) i2s1_start_or_stop(DISABLE);
    #endif
    if(g_i2s0_state) i2s0_start_or_stop(DISABLE);
}

void stop_audio(){
    hal_stop_audio(g_linein_state);
    RCC_Peri_Rst(RCC_AUD_RSTN);
}

void close_audio(){
    Audio_InitTypeDef* micin_instance;
    Audio_InitTypeDef* linein_instance;
    micin_instance  = hal_audio_instance_get(HAL_AUDIO_INSTANCE1);
    linein_instance = hal_audio_instance_get(HAL_AUDIO_INSTANCE2);
    if (micin_instance && micin_instance->enable) {
	//printf("close	micin\r\n");
	hal_audio_close(micin_instance);
    }
    if (linein_instance	&& linein_instance->enable) {
	//printf("close	linein\r\n");
	hal_audio_close(linein_instance);
    }
}

void getNpuInfo(){
    uint16_t data[256] = {0};
    int	ret = NPU_READ_TRIM(data);
    wtm2101_chipinfo chipinfo;
    get_wtm2101_chipinfo(data, &chipinfo);
    printf("chipinfo.tag_len:%d\r\n", (int)chipinfo.tag_len);
    printf("chipinfo.tag:%s\r\n", chipinfo.tag);
    printf("chipinfo.name:%s\r\n", chipinfo.name);
    //printf("chipinfo.bootloader_name:%s\r\n",	chipinfo.bootloader_name);
    //printf("chipinfo.chipid:%llu\r\n", chipinfo.chipid);
    printf("chipinfo.chipid:%d\r\n", (int)chipinfo.chipid);

    g_proto_status.chipid = chipinfo.chipid;
    g_proto_status.nw_major_version = (chipinfo.name[3]-48)*10 + (chipinfo.name[4]-48);
    g_proto_status.nw_minor_version = (chipinfo.name[5]-48)*10 + (chipinfo.name[6]-48);
    g_proto_status.nw_fixed_version = 0;

}

void checkModelVersion(){

    uint8_t mismatch_flag = 0;
    if(g_proto_status.nw_major_version!=MATCH_NW_MAJOR ||
       g_proto_status.nw_minor_version!=MATCH_NW_MINOR ||
       g_proto_status.nw_fixed_version!=MATCH_NW_FIXED){
	mismatch_flag =	1;
    }

    if(mismatch_flag){
	char _tmp[64] =	{0};
	sprintf(_tmp,"modelVerERR:%d.%d.%d",g_proto_status.nw_major_version,
		g_proto_status.nw_minor_version, g_proto_status.nw_fixed_version);
	int char_len = strlen(_tmp);
	while(1){
	    printf("modelVerERR:%d.%d.%d\r\n",g_proto_status.nw_major_version,
		g_proto_status.nw_minor_version, g_proto_status.nw_fixed_version);
	    wtm2101ac_send_info(_tmp,char_len,2);
	    system_delay_s(2);
	}
    }
}

#endif

// AUDIO_IRQn wake this
void standby_wait(void)
{
    //PMU_Set_Rsm_Mode_Parameter(PMU, PMU_RSM_MODE_ALL,	ENABLE);
    // *(uint8_t*)0x40002020 = 0x82;// 0.9V
    PMU_Standby_Mode_Cmd(PMU);
    __WFI();

}

void _RSM2_Wait(uint64_t tick) {
    PMU_Set_Rsm_Mode_Parameter(PMU, PMU_RSM_MODE_ALL, DISABLE);
    PMU_Set_Rsm_Mode_Parameter(PMU, PMU_RSM_MODE_KEEP_OSC2M | PMU_RSM_MODE_KEEP_VDD | PMU_RSM_MODE_KEEP_AUDIO, ENABLE);

    AUD->CLKSWI_CFG = 0x10;
    PMU_Rsm_Mode_Cmd(PMU);
    RCC_CLK_EN_Ctl(RCC_RTC_PCLKEN, ENABLE);
    RTC_Set_Compare_Time_Value(RTC, tick);
    RTC_Set_Initial_Time_Value(RTC, 0);
    system_delay_us(100);

    __WFI();

    AUD->CLKSWI_CFG = 0x00;

    RTC_Set_Compare_Time_Value(RTC, 0);
    RTC_Set_Initial_Time_Value(RTC, 0);
    system_delay_us(100);
    RCC_CLK_EN_Ctl(RCC_RTC_PCLKEN, DISABLE);
}


void spi_irq_init(void)
{
    /* The GPIO	init */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_13;
    GPIO_InitStructure.Mode = GPIO_MODE_OUT;
    GPIO_InitStructure.Alternate = GPIO_AF13_GPIO;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //GPIO_OutputHi(GPIOA, GPIO_PIN_13);
    GPIO_OutputLo(GPIOA, GPIO_PIN_13);
}

typedef	struct heap_tag	{
    struct heap_tag *next;
    unsigned size;
} heap_t;

void Dump_Heap_List(void)
{
    extern uint32_t __heap_start__;
    extern uint32_t __heap_end__;

    const heap_t* head = (heap_t*)&__heap_start__;
    const heap_t* p;
    uint32_t last_end_addr = 0;
    uint32_t curr_start_addr;

    p =	head;

    printf("  %p: ------ start ------\r\n", &__heap_start__);
    while (p !=	NULL) {
	if (p != head) {
	    curr_start_addr = (uint32_t)p;
	    if (last_end_addr <	curr_start_addr) {
		printf("  %p: %5d bytes\r\n", last_end_addr, curr_start_addr - last_end_addr);
	    }
	}
	printf("  %p: %5d bytes	(free)\r\n", p,	p->size);
	last_end_addr =	(uint32_t)p + p->size;
	p = p->next;
    }

    curr_start_addr = (uint32_t)&__heap_end__;
    if (last_end_addr <	curr_start_addr) {
	printf("  %p: (used) %5d bytes\r\n", last_end_addr, curr_start_addr - last_end_addr);
    }
    printf("  %p: ------- end -------\r\n", &__heap_end__);
}

#if !defined(CONFIG_RAM_UART_VERIFY_HSI24Mx6) && \
    !defined(CONFIG_RAM_UART_PESQ) &&  \
    !defined(CONFIG_RAM_JTAG_PESQ) && \
    !defined(CONFIG_RAM_KWS_SPI)

void _RSM2_Wait_GPIO() {

    ECLIC_ClearPendingIRQ(GPIO_WAKEUP_IRQn);
    ECLIC_SetPriorityIRQ(GPIO_WAKEUP_IRQn, 1);
    ECLIC_SetTrigIRQ(GPIO_WAKEUP_IRQn, ECLIC_LEVEL_TRIGGER);
    ECLIC_EnableIRQ(GPIO_WAKEUP_IRQn);

    /* the mask	bit is ebabled */
    PMU_Set_Ie_Msk(PMU,	PMU_GPIO_IMSK, ENABLE);
    PMU_Set_Ie_Msk(PMU,	PMU_GPIO_EMSK, ENABLE);

    PMU_Set_Rsm_Mode_Parameter(PMU, PMU_RSM_MODE_ALL,DISABLE);
    PMU_Set_Rsm_Mode_Parameter(PMU, PMU_RSM_MODE_KEEP_VDD, ENABLE);
    uint8_t temp = PMU->PMU_RESV_ADC_REG1;
    PMU->PMU_RESV_ADC_REG1 = 0x15;

    PMU_Rsm_Mode_Cmd(PMU);
    __WFI();

    PMU->PMU_RESV_ADC_REG1 = temp;
    PMU_Set_Ie_Msk(PMU,	PMU_GPIO_IMSK, DISABLE);
    PMU_Set_Ie_Msk(PMU,	PMU_GPIO_EMSK, DISABLE);

}

void GPIO_Sleep_Switch(uint8_t start_sleep){

    GPIO_InitTypeDef GPIO_InitStructure;
    if(start_sleep==1){

	GPIO_InitStructure.Pin = GPIO_PIN_0;
	GPIO_InitStructure.Mode	= GPIO_MODE_OUT;
	GPIO_InitStructure.Alternate = GPIO_AF0_GPIO;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_OutputLo(GPIOA, GPIO_PIN_0);

	GPIO_InitStructure.Pin = GPIO_PIN_1;
	GPIO_InitStructure.Mode	= GPIO_MODE_OUT;
	GPIO_InitStructure.Alternate = GPIO_AF1_GPIO;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_OutputLo(GPIOA, GPIO_PIN_1);

	GPIO_InitStructure.Pin = GPIO_PIN_2;
	GPIO_InitStructure.Mode	= GPIO_MODE_OUT;
	GPIO_InitStructure.Alternate = GPIO_AF2_GPIO;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_OutputLo(GPIOA, GPIO_PIN_2);

	GPIO_InitStructure.Pin = GPIO_PIN_3;
	GPIO_InitStructure.Mode	= GPIO_MODE_OUT;
	GPIO_InitStructure.Alternate = GPIO_AF3_GPIO;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_OutputLo(GPIOA, GPIO_PIN_3);

	GPIO_InitStructure.Pin = GPIO_PIN_4;
	GPIO_InitStructure.Mode	= GPIO_MODE_OUT;
	GPIO_InitStructure.Alternate = GPIO_AF4_GPIO;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_OutputLo(GPIOA, GPIO_PIN_4);

	GPIO_InitStructure.Pin = GPIO_PIN_5;
	GPIO_InitStructure.Mode	= GPIO_MODE_OUT;
	GPIO_InitStructure.Alternate = GPIO_AF5_GPIO;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_OutputLo(GPIOA, GPIO_PIN_5);

	GPIO_InitStructure.Pin = GPIO_PIN_6;
	GPIO_InitStructure.Mode	= GPIO_MODE_INPD;
	GPIO_InitStructure.Alternate = GPIO_AF6_GPIO;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_7;
	GPIO_InitStructure.Mode	= GPIO_MODE_INPD;
	GPIO_InitStructure.Alternate = GPIO_AF7_GPIO;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_10;
	GPIO_InitStructure.Mode	= GPIO_MODE_OUT;
	GPIO_InitStructure.Alternate = GPIO_AF10_GPIO;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_OutputLo(GPIOA, GPIO_PIN_10);

	GPIO_InitStructure.Pin = GPIO_PIN_11;
	GPIO_InitStructure.Mode	= GPIO_MODE_OUT;
	GPIO_InitStructure.Alternate = GPIO_AF11_GPIO;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_OutputLo(GPIOA, GPIO_PIN_11);

	GPIO_InitStructure.Pin = GPIO_PIN_12;
	GPIO_InitStructure.Mode	= GPIO_MODE_OUT;
	GPIO_InitStructure.Alternate = GPIO_AF12_GPIO;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_OutputLo(GPIOA, GPIO_PIN_12);

	GPIO_InitStructure.Pin = GPIO_PIN_14;
	GPIO_InitStructure.Mode	= GPIO_MODE_OUT;
	GPIO_InitStructure.Alternate = GPIO_AF14_GPIO;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_OutputHi(GPIOA, GPIO_PIN_14);  // dmic

    }else{
	GPIO_InitStructure.Pin = GPIO_PIN_14;
	GPIO_InitStructure.Mode	= GPIO_MODE_OUT;
	GPIO_InitStructure.Alternate = GPIO_AF14_GPIO;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_OutputLo(GPIOA, GPIO_PIN_14);  // dmic

	GPIO_DeInit(GPIOA, GPIO_PIN_6);
	GPIO_DeInit(GPIOA, GPIO_PIN_7);

    }
}

inline void GPIO_TogglePin(GPIO_TypeDef* GPIOx,	uint32_t GPIO_Pin)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
    assert_param(IS_GPIO_PIN(GPIO_Pin));
    GPIOx->ODR ^= GPIO_Pin;
}

extern int wnpu_set_npu_colck_gate(int);
void switch_to_start_call(){
    stop_i2s();
    stop_audio();
    close_audio();

    // switch system clock
    system_delay_us(200);
    wnpu_set_npu_colck_gate(0);
    system_clock_init_internal_24M(PLL_ENC);
    wnpu_set_npu_colck_gate(1);
    system_delay_us(100);

    #if	UART_ENABLE
    uart_open();
    #endif
    protocol_reset_buffer_state();

    // start i2s
    #if	USE_I2S1
    RCC_CLK_EN_Ctl(RCC_I2S1_CLKEN,ENABLE);
    #endif
    RCC_CLK_EN_Ctl(RCC_I2S0_CLKEN,ENABLE);
    RCC_CLK_EN_Ctl(RCC_DMA_CLKEN,ENABLE);

    Hal_I2s_InitTypeDef* hal_i2s_instance0 = hal_i2s_instance_get(HAL_I2S_INSTANCE0);
    hal_i2s_ctl(hal_i2s_instance0,HAL_I2S_CHANNEL_UPDATA_CLOCK_COMMAND);
    #if	USE_I2S1
    g_i2s1_state = ENABLE;
    Hal_I2s_InitTypeDef* hal_i2s_instance1 = hal_i2s_instance_get(HAL_I2S_INSTANCE1);
    hal_i2s_ctl(hal_i2s_instance1,HAL_I2S_CHANNEL_UPDATA_CLOCK_COMMAND);
    #endif

    // init NPU
    NPU_INFER_CONFIG_6N_MHz(4*PLL_ENC);
    system_delay_us(20);
    pd_stage();
    system_delay_us(1);
    wnpu_set_npu_colck_gate(0);

    g_audio_state = ENABLE;
    g_linein_state = ENABLE;
    g_i2s0_state = ENABLE;

    // start audio
    system_delay_us(20);
    open_audio_stereo();
    hal_start_audio(g_linein_state);
    system_delay_us(20);
}

void switch_to_start_kws(){
    stop_i2s();
    stop_audio();
    close_audio();

    wnpu_set_npu_colck_gate(0);
    uint8_t pll_ratio =	g_proto_status.start_enhanced_kws? PLL_ENHANCED_KWS:PLL_KWS;
    system_clock_init_internal_24M(pll_ratio);
    wnpu_set_npu_colck_gate(1);
    system_delay_us(50);

    #if	UART_ENABLE
    uart_open();
    #endif
    protocol_reset_buffer_state();

    if(g_proto_status.use_kws_i2s){
	RCC_CLK_EN_Ctl(RCC_DMA_CLKEN,ENABLE);
	RCC_CLK_EN_Ctl(RCC_I2S0_CLKEN,ENABLE);
	Hal_I2s_InitTypeDef* hal_i2s_instance0 = hal_i2s_instance_get(HAL_I2S_INSTANCE0);
	hal_i2s_ctl(hal_i2s_instance0,HAL_I2S_CHANNEL_UPDATA_CLOCK_COMMAND);
	#if USE_I2S1
	RCC_CLK_EN_Ctl(RCC_I2S1_CLKEN,ENABLE);
	g_i2s1_state = ENABLE;
	Hal_I2s_InitTypeDef* hal_i2s_instance1 = hal_i2s_instance_get(HAL_I2S_INSTANCE1);
	hal_i2s_ctl(hal_i2s_instance1,HAL_I2S_CHANNEL_UPDATA_CLOCK_COMMAND);
	#endif
    }else{
	RCC_CLK_EN_Ctl(RCC_I2S0_CLKEN,DISABLE);
	RCC_CLK_EN_Ctl(RCC_I2S1_CLKEN,DISABLE);
	RCC_CLK_EN_Ctl(RCC_DMA_CLKEN,DISABLE);
    }

    NPU_INFER_CONFIG_6N_MHz(4*pll_ratio);
    system_delay_us(20);
    pd_stage();
    system_delay_us(1);
    wnpu_set_npu_colck_gate(0);
    g_audio_state = ENABLE;
    g_linein_state = DISABLE;
    g_i2s0_state = DISABLE;
    open_audio_mono();
    hal_start_audio(g_linein_state);
    system_delay_us(20);
}

void switch_to_stop_algo(){
    stop_i2s();
    stop_audio();
    close_audio();
    wnpu_set_npu_colck_gate(0);
    system_clock_init_internal_24M(1);
    wnpu_set_npu_colck_gate(1);
    system_delay_us(50);
    #if	UART_ENABLE
    uart_open();
    #endif
    protocol_reset_buffer_state();
    NPU_INFER_CONFIG_6N_MHz(4);
    system_delay_us(20);
    pd_stage();
    system_delay_us(1);
    wnpu_set_npu_colck_gate(0);
    g_audio_state = DISABLE;
    g_linein_state = DISABLE;
    g_i2s0_state = DISABLE;
    g_i2s1_state = DISABLE;
    #if	USE_I2S1
    Hal_I2s_InitTypeDef* hal_i2s_instance1 = hal_i2s_instance_get(HAL_I2S_INSTANCE1);
    hal_i2s_ctl(hal_i2s_instance1,HAL_I2S_CHANNEL_UPDATA_CLOCK_COMMAND);
    i2s1_start_or_stop(DISABLE);
    #endif
    open_audio_stereo();
}

void switch_to_sleep(){
    stop_i2s();
    stop_audio();
    close_audio();
    SPI_Cmd(SPIS, DISABLE);
    //TODO GPIO	state
    //GPIO_Sleep_Switch(1);
    init_gpio14_sleep();
    _RSM2_Wait_GPIO();
    close_gpio14_nss();
    init_spi14();
    SPI_Cmd(SPIS, ENABLE);
    //GPIO_Sleep_Switch(0);
    system_delay_us(100);
    #if	UART_ENABLE
    uart_open();
    #endif
    system_delay_us(20);
    protocol_reset_buffer_state();
    g_audio_state = DISABLE;
    g_linein_state = DISABLE;
    g_i2s0_state = DISABLE;
    g_i2s1_state = DISABLE;
    open_audio_stereo();
}
#endif	// #if !defined	CONFIG_RAM_UART_VERIFY_HSI24Mx6, CONFIG_RAM_UART_PESQ, CONFIG_RAM_KWS_SPI

void delay_4uS(void)
{
    for(int i=0;i<4;i++){
      __NOP(); __NOP();	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
      __NOP(); __NOP();	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
      __NOP(); __NOP();	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
      __NOP(); __NOP();	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    }
}

//#if USE_KWS
//void init_kws(){

//    kws_config_t kws_config =	{
//	.free_func = vPortFree,
//	.malloc_func = pvPortMalloc,
//	.kwsNNProcess =	kws_nn_process,
//	.kwsProceHooker	= NULL,
//	.log_func = printf,
//	.wakeupCallBack	= kws_get_res_callback
//    };

//    #if	USE_NPU
//    get_kws_input_output(&(kws_config.input_dim), &(kws_config.output_dim));
//    printf("kws	input_out_len:%d-%d\n",	kws_config.input_dim, kws_config.output_dim);
//    #else
//    #error "kws	must use npu"
//    #endif
//    kwsAlg_init(&kws_config);
//    printf("init kws ok\r\n");
//};
//#endif



#if USE_AEC
void init_aec(){
    #if	USE_LINEIN
    AecCreate(&aec_inst);
    AecInit(&aec_inst, 16000);
    AecFillNearend(&aec_inst, 132);
    #else
    #error "aec	must use linein"
    #endif
}
#endif

#if USE_AGC
void init_agc(){
    int	minLevel = 0;
    int	maxLevel = 255;
    int16_t agcMode = kAgcModeFixedDigital;
    uint32_t sampleRate	= 16000;
    WebRtcAgc_Init(&agc_handle,	minLevel, maxLevel, agcMode, sampleRate);

    WebRtcAgcConfig agc_config;
    agc_config.compressionGaindB = 10;	 // default 9 dB
    agc_config.limiterEnable = 1;	 // default kAgcTrue (on) 1
    agc_config.targetLevelDbfs = 1;	 // default 3 (-3 dBOv)
    WebRtcAgc_set_config(&agc_handle, agc_config);
    printf("init agc ok\r\n");
}
#endif

#if defined(CONFIG_RAM_KWS_SPI)
void _task_kws_spi(){
    cnn_init();
    init_kws();
    outside_data_process();
}
#endif

#if  defined(CONFIG_RAM_IISMIC_REALTIME_HSI24Mx2) || \
     defined(CONFIG_RAM_IISMIC_REALTIME_HSI24Mx4) || \
      defined(CONFIG_QSPI_IISMIC_REALTIME_HSI24Mx2) || \
      defined(CONFIG_QSPI_IISMIC_REALTIME_HSI24Mx4) || \
      defined(CONFIG_RAM_K1_IISMIC_REALTIME_HSI24Mx2) || \
      defined(CONFIG_QSPI_K1_IISMIC_REALTIME_HSI24Mx2) || \
      defined(CONFIG_RAM_Y1_IISMIC_REALTIME_HSI24Mx2) || \
      defined(CONFIG_QSPI_Y1_IISMIC_REALTIME_HSI24Mx2)

int16_t	buffer_0[160] =	{0};
int16_t	dmic_src[160] =	{0};
int16_t	linein_src[160]	= {0};
int16_t	audio_buffer1[160] = {0};
int16_t	audio_buffer2[160] = {0};
const float g_gain_level_map[17] = {1.0, 0.5012, 0.6310, 0.7943, 1.0, 1.2589, 1.5849, 1.9953, 2.5119, 3.1623, 3.9811, 5.0119, 6.3096,
				    7.9433, 10,	12.5393, 15.8489};
const float g_aec_level_map[21]	= {1.0,	0.3548,	0.3981,	0.4467,	0.5012,	0.5623,	0.6309,	0.7079,	0.7943,	0.8913,	1,
				    1.122, 1.2589, 1.4125, 1.5849, 1.7783, 1.9953, 2.2387, 2.5119, 2.8184, 3.1623};

static uint64_t	algo_t0,algo_t1,algo_t2,algo_t3;
int get_debug_str1(uint64_t delta1, uint64_t delta2){
    uint64_t ahbclock =	RCC_Get_SYSClk() / (RCC_AHB_Get_ClkDiv() + 1);
    int	res1 = (int)((delta1) *	1000 * 1000/ahbclock);
    int	res2 = (int)((delta2) *	1000 * 1000/ahbclock);
    char _tmp[32] = {0};
    sprintf(_tmp,"-%d,%d-\r\n",res1,res2);
    int	char_len = strlen(_tmp);
    wtm2101ac_send_info(_tmp,char_len,0);
    return char_len;
}

void enter_core_exception(){
   int qq = pow(8,0);
   printf("ready to exception\r\n");
   system_delay_ms(500);
   printf("%d",6/(qq-1));
}

void rcc_clk_switch(){

    RCC_CLK_EN_Ctl(RCC_I2S_MCLKEN,DISABLE);

    #if	!(USE_I2S1)
    RCC_CLK_EN_Ctl(RCC_I2S1_CLKEN,DISABLE);
    #endif
    //RCC_CLK_EN_Ctl(RCC_I2S0_CLKEN,DISABLE);

    #if	!(UART_ENABLE)
    RCC_CLK_EN_Ctl(RCC_UART0_CLKEN,DISABLE);
    RCC_CLK_EN_Ctl(RCC_UART1_CLKEN,DISABLE);
    #endif

    //RCC_CLK_EN_Ctl(RCC_DMA_CLKEN,DISABLE);
    RCC_CLK_EN_Ctl(RCC_SPIM_PCLKEN,DISABLE);
    //RCC_CLK_EN_Ctl(RCC_SPIS_PCLKEN,DISABLE);
    RCC_CLK_EN_Ctl(RCC_RTC_PCLKEN,DISABLE);
    RCC_CLK_EN_Ctl(RCC_WDT_PCLKEN,DISABLE);
    RCC_CLK_EN_Ctl(RCC_TIME_CLKEN,DISABLE);
    RCC_CLK_EN_Ctl(RCC_I2C_PCLKEN,DISABLE);
    RCC_CLK_EN_Ctl(RCC_FFT_CLKEN,DISABLE);
    RCC_CLK_EN_Ctl(RCC_TRIM_CLKEN,DISABLE);
}

void get_stack_info(){
    extern uint32_t __stack_start__;
    extern uint32_t __stack_end__;
    uint32_t sp_bottom = (uint32_t)&__stack_end__;
    uint32_t sp_limits = (uint32_t)&__stack_start__;

    if (__get_rv_sp() <	sp_limits) {
	    printf("Error: stack overflow !\r\n");
    }else{
	printf("stackfree  %d\r\n", __get_rv_sp() - sp_limits);
    }
}

void npu_regfile_switch(uint8_t	is_soc2npu, int8_t *buffer){
    wnpu_set_npu_colck_gate(NPU_CLK_ON);
    if(is_soc2npu){
	wnpu_data_soc2npu((char	*)buffer, 0, 8192);
    }else{
	wnpu_data_npu2soc(0, buffer, 8192);
    }
    wnpu_set_npu_colck_gate(NPU_CLK_OFF);
}

void decode_reset();
void _task_algo_iismic_iisout(void)
{
/////////////////////////////////////

    int	ret;
    int	tmp_int;

#if USE_PROTOCOL
    protocol_hci_init(proto_rx_buf, WTM_RECEIVE_DATA_SIZE_MAX);
    wtm2101ac_init(proto_rx_buf, &g_proto_status);
#endif

#if USE_SPI
    {
	wtmproto_spi_config _config;
	_config.irq_func = spi_send_irq;
	_config.send_func = spi_send_data;
	protocol_spi_set_callback(_config);
    }
    init_spi_slave();
    spi_irq_init();
    printf("spi	init ok\r\n");
#endif

#if USE_DEBUG_PIN
    debug_init();
#endif

    open_audio_stereo();
    //open_audio_mono();
    printf("audio init ok\r\n");

    iis_init();
    printf("iis	init ok\r\n");

    static int data_flag1 = 0;
    static int i2s0_flag = 0;
    Audio_InitTypeDef* hal_audio_instance1 = hal_audio_instance_get(HAL_AUDIO_INSTANCE1);
    Audio_InitTypeDef* hal_audio_instance2 = hal_audio_instance_get(HAL_AUDIO_INSTANCE2);
    Hal_I2s_InitTypeDef* hal_i2s_instance0 = hal_i2s_instance_get(HAL_I2S_INSTANCE0);
    Hal_I2s_InitTypeDef* hal_i2s_instance1 = hal_i2s_instance_get(HAL_I2S_INSTANCE1);

#if USE_NPU
    getNpuInfo();
    checkModelVersion();
    ret	= cnn_init();
    //npu_regfile_switch(0, g_regfile_bkup0);
    if(ret){
	while(1){
	    wtm2101ac_send_info("npuERR", 6, 2);
	    system_delay_s(2);
	}
    }
#endif

#if USE_KWS
    init_kws();
#endif

#if USE_ENC
    init_enc();
#endif

#if USE_AEC
    init_aec();
#endif

#if USE_AGC
    init_agc();
#endif

    RCC_Peri_Rst(RCC_AUD_RSTN);
    hal_start_audio(g_linein_state);

    int16_t* vOut = NULL;
    vOut = dmic_src;
    static uint32_t loop_running = 0;

#if UART_ENABLE
    uart_open();
#endif
    protocol_reset_buffer_state();

    printf("READY ---- \r\n");
    printf("--sys clk:%d\r\n",RCC_Get_SYSClk());
    printf("VERSION:%d.%d.%d\n",FW_MAJOR_VERSION,FW_MINOR_VERSION,FW_FIXED_VERSION);

    rcc_clk_switch();
    empty_spi_rcv_fifo();

    while (1) {

	if(g_proto_status.run_mode!=RUN_MODE_EMPTY){
	    PMU_Standby_Mode_Cmd(PMU);
	    __WFI();
	}

	if(init_spi14_flag){
	    empty_spi_data();
	    if(g_proto_status.wakeup){
		g_proto_status.start_sleep = 1;
	    }
	}

	if(g_gpio13_pu){
	    if(get_delta_ms_gpio13(1)>10){
		g_gpio13_pu = 0;
		GPIO_OutputLo(GPIOA, GPIO_PIN_13);
	    }
	}

	if(g_proto_status.buffer_finished){
	    g_proto_status.buffer_finished = 0;
	    protocol_reset_buffer_state();
	    ret	= wtm2101ac_protocol_data_receive_cb();
	    if(ret<0){
		wtm2101ac_send_info(proto_rx_buf,12,2);
	    }
	}

	#if 1
	if(g_audio_state){

	    if(g_proto_status.run_mode==RUN_MODE_KWS ||	g_proto_status.run_mode==RUN_MODE_ENC_KWS){
		if (!audio_frame_ready)	{
		    continue;
		}
		audio_frame_ready = 0;

		for (int i = 0;i < 160/2;i++) {
		    ((uint32_t*)dmic_src)[i] = AUD->RAM0DATA;
		}

	    } else {// AEC_ENC
		if(hal_audio_read(hal_audio_instance1,(uint8_t*)dmic_src) > 0){
		    data_flag1 = 1;
		}
		if(data_flag1==0){
		    g_audio_not_ok_cnt++;
		    if(g_audio_not_ok_cnt%800000==0){
			printf("AuIRQ\r\n");
			g_audio_not_ok_cnt = 0;
			wtm2101ac_send_info("AuIRQ",5,2);
		    }
		    continue;
		}else{
		    g_audio_not_ok_cnt = 0;
		}
		data_flag1 = 0;
	    }

	    vOut = dmic_src;

	}
	#endif

	#if 1
	if(loop_running!=0 && loop_running%2000==0){
	    if(g_proto_status.run_mode!=255){
		printf("mode%d\r\n",(int)g_proto_status.run_mode);
	    }
	    //char _tmp[5];
	    //sprintf(_tmp,"mode%d",(int)g_proto_status.run_mode);
	    //wtm2101ac_send_info(_tmp,5,0);
	}
	#endif

	if(g_proto_status.run_mode==RUN_MODE_KWS) { // kws
	    #if	USE_KWS
	    if(g_proto_status.use_kws){
		kws_process(dmic_src, 160);
		vOut = dmic_src;
	    }
	    #endif
	}else if (g_proto_status.run_mode==RUN_MODE_AEC_ENC) { // aec +	enc

	    if(g_i2s0_state && loop_running >= 2)  {

		#if USE_LINEIN
		while(1){
		    if(hal_audio_read(hal_audio_instance2,(uint8_t*)linein_src)	> 0){
			break;
		    }
		}
		#endif

		float aec_min_gain_level = (-2.5 * (float)g_proto_status.aec_gain_level) + -35 ;
		denoise_set_all_min_gain(denoise[0], aec_min_gain_level);
		//denoise_set_nn_level(denoise[0], nn_level);

		//float	aec_scale = g_aec_level_map[g_proto_status.aec_level];
		for(int	i=0;i<160;i++){
		    //tmp_int =	linein_src[i] *	aec_scale;
		    tmp_int = linein_src[i] * 1;
		    tmp_int = MIN(MAX(-32768, tmp_int),	32767);
		    audio_buffer2[i] = tmp_int;
		}

		float enc_scale	= g_aec_level_map[g_proto_status.enc_level];
		for(int	i=0;i<160;i++){
		    tmp_int = dmic_src[i] * enc_scale;
		    tmp_int = MIN(MAX(-32768, tmp_int),	32767);
		    audio_buffer1[i] = tmp_int;
		}

		#if USE_AEC
		if(g_proto_status.use_aec) {
		    for(int i=0;i<160;i++){
			tmp_int	= dmic_src[i] *	aec_scale;
			tmp_int	= MIN(MAX(-32768, tmp_int), 32767);
			audio_buffer1[i] = tmp_int;
		    }
		    AecWriteNearendBlock(&aec_inst, audio_buffer1, AEC_DATA_BLOCK_SAMPLES);
		    AecWriteFarendBlock(&aec_inst, linein_src, AEC_DATA_BLOCK_SAMPLES);
		    AecProcess(&aec_inst);
		    AecReadOutputBlock(&aec_inst, audio_buffer2, AEC_DATA_BLOCK_SAMPLES);
		    vOut = audio_buffer2;
		}
		#endif

		#if USE_ENC
		if(g_proto_status.use_enc) {
		    denoise_process_16bits_ref(denoise_ref[0], audio_buffer2);
		    denoise_hp_process_16bits(denoise[0],audio_buffer1,	audio_buffer1, AUDIO_FRAME_SIZE);
		    denoise_process_16bits(denoise[0], denoise_ref[0], &agc_handle, audio_buffer1, audio_buffer2);
		    vOut = audio_buffer2;
		}
		#endif

	    }

	}else if(g_proto_status.run_mode==RUN_MODE_ENC_KWS){ //	nn_enc + kws

	    int16_t num_max = run_mcra(denoise[0],dmic_src,NULL);
	    mcra_vad_loop(num_max);
	    int	lenth =	get_last_voice_lenth();
	    int8_t mcra_vad_stat = get_mcra_vad_status();
	    int8_t backgroud_stat = get_cur_backgroud_state();

	    memmove(his_mcra_res,his_mcra_res+1,NUM_VAD_STAT_SAVED-1);
	    his_mcra_res[NUM_VAD_STAT_SAVED-1] = mcra_vad_stat;

	    if (MCRA_VAD_SIL ==	mcra_vad_stat) { ; }
	    else if (MCRA_VAD_SIL2VOICE	== mcra_vad_stat) {if(0==backgroud_stat) decode_reset();} //printf("---->> voice start!\n");}
	    else if (MCRA_VAD_VOICE == mcra_vad_stat) {	; }
	    else if (MCRA_VAD_VOICE2SIL	== mcra_vad_stat) {;}//printf("	   voice end! last_voice_lenth=%d\n", lenth);


	    #if	USE_KWS
	    kws_process(dmic_src, 160);
	    #endif

	    vOut = dmic_src;
	    if(g_is_get_kw) {
		// 以下代码不再使用，后面项目稳定量产后，可以删��
		/*
		his_mcra_res[his_mcra_res_idx] = num_max;
		his_mcra_res_idx++;

		if(his_mcra_res_idx>=NUM_FRM_CHECK_AFTER_WKUP) {
		    kws_check_again();
		    his_mcra_res_idx = 0;
		    g_is_get_kw	= 0;
		}*/
		his_mcra_res_idx++;
		if(his_mcra_res_idx>=15) // delay sometime and check
		{
		    kws_check_again(mcra_vad_stat,lenth);
		    his_mcra_res_idx = 0;
		    g_is_get_kw	= 0;
		}
	    }
	}//end run mode

	loop_running++;
	float mic_gain_scale = g_gain_level_map[g_proto_status.gain_level];

	if(g_i2s0_state){
	    if(g_proto_status.run_mode==RUN_MODE_KWS ||	g_proto_status.run_mode==RUN_MODE_ENC_KWS){
		for(int	i=0;i<160;i++) {
		    tmp_int = dmic_src[i] * mic_gain_scale;
		    tmp_int = MIN(MAX(-32768, tmp_int),	32767);
		    dmic_src[i]	= tmp_int;
		}
		hal_i2s_write(hal_i2s_instance0, dmic_src, NULL, hal_i2s_instance0->lr_channel_need_sizes_by_width / 2);
	    }else if(g_proto_status.run_mode==RUN_MODE_AEC_ENC)	{
		if(g_proto_status.use_enc){
		    float hardAgcRevertGain = get_audio_agc_gain();
		    for(int i=0;i<160;i++) {
			tmp_int	= (vOut[i]) * mic_gain_scale * hardAgcRevertGain;
			tmp_int	= MIN(MAX(-32768, tmp_int), 32767);
			audio_buffer1[i] = tmp_int;
		    }
		    if(g_proto_status.mute_state){
		       hal_i2s_write(hal_i2s_instance0,	buffer_0, buffer_0,hal_i2s_instance0->lr_channel_need_sizes_by_width / 2);
		    }
		    else{
		       hal_i2s_write(hal_i2s_instance0,	audio_buffer1, audio_buffer1, hal_i2s_instance0->lr_channel_need_sizes_by_width	/ 2);
		    }

		}else{
		    if(g_proto_status.mute_state){
		       hal_i2s_write(hal_i2s_instance0,	buffer_0, buffer_0,hal_i2s_instance0->lr_channel_need_sizes_by_width / 2);
		    }
		    else{
		       hal_i2s_write(hal_i2s_instance0,	dmic_src, linein_src, hal_i2s_instance0->lr_channel_need_sizes_by_width	/ 2);
		    }
		}
	    }

	    if(loop_running == 2) {
	       i2s0_start_or_stop(ENABLE);
	    }
	}

	#if USE_I2S1
	if(g_i2s1_state){
	    hal_i2s_write(hal_i2s_instance1,vOut, NULL,hal_i2s_instance1->lr_channel_need_sizes_by_width / 2);
	    //hal_i2s_write(hal_i2s_instance1,dmic_double, NULL,hal_i2s_instance1->lr_channel_need_sizes_by_width / 2);
	    //hal_i2s_write(hal_i2s_instance1,buff_music_inL_16, NULL,hal_i2s_instance1->lr_channel_need_sizes_by_width	/ 2);
	    if(loop_running == 2) {
	       i2s1_start_or_stop(ENABLE);
	    }
	}
	#endif

	if(g_proto_status.wanna_sleep){
	    g_proto_status.wanna_sleep = 0;
	    g_proto_status.wakeup = 1;
	    reply_common(WTM2101_CMD_SLEEP);
	}
	if(g_proto_status.start_sleep){
	    if(GPIO_ReadOutputDataBit(GPIOA,GPIO_PIN_13)){
	    }else{
		g_proto_status.start_sleep = 0;
		printf("sleep\r\n");
		switch_to_sleep();
		printf("wkup\r\n");
		loop_running = 0;
		data_flag1 = 0;
		g_proto_status.use_aec = 0;
		g_proto_status.use_enc = 0;
		g_proto_status.use_kws = 0;
		g_proto_status.run_mode	= RUN_MODE_EMPTY;
		wtm2101ac_send_wakeup();
		g_proto_status.wakeup =	0;
	    }
	}
	if(g_proto_status.start_call ||	g_proto_status.start_bypass){
	    switch_to_start_call();
	    printf("\r\nstartcall\r\n");
	    g_proto_status.run_mode = RUN_MODE_AEC_ENC;
	    set_mcra_dynamic_params(denoise[0],6,NS_MCRA_MAG_LEN);
	    if(g_proto_status.start_call){
		g_proto_status.start_call = 0;
		g_proto_status.use_enc = 1;
		g_proto_status.use_aec = 1;
		g_proto_status.mute_state = 0;
	    }
	    if(g_proto_status.start_bypass){
		g_proto_status.start_bypass = 0;
		g_proto_status.use_enc = 0;
		g_proto_status.use_aec = 0;
		g_proto_status.mute_state = 0;
	    }
	    //nn_switch(0);
	    denoise_switch(1);
	    reply_common(WTM2101_CMD_START_ENC);
	    loop_running = 0;
	    data_flag1 = 0;
	} else if(g_proto_status.start_enhanced_kws || g_proto_status.start_kws){
	    switch_to_start_kws();
	    g_proto_status.use_aec = 0;
	    g_proto_status.use_kws = 1;
	    if(g_proto_status.use_kws_i2s){
		g_proto_status.use_kws_i2s = 0;
		g_i2s0_state = ENABLE;
	    }
	    if(g_proto_status.start_enhanced_kws){
		printf("\r\nKWS+\r\n");
		g_proto_status.start_enhanced_kws = 0;
		g_proto_status.use_enc = 1;
		g_proto_status.run_mode	= RUN_MODE_ENC_KWS;
		//nn_switch(0);
		//denoise_switch(1);
		set_mcra_dynamic_params(denoise[0],KWS_MCRA_LEVEL,KWS_MCRA_BAND_LEN);
	    }
	    if(g_proto_status.start_kws){
		printf("\r\nKWSN\r\n");
		g_proto_status.start_kws = 0;
		g_proto_status.use_enc = 0;
		g_proto_status.run_mode	= RUN_MODE_KWS;
		//nn_switch(0);
	    }
	    reply_common(WTM2101_CMD_START_KWS);
	    loop_running = 0;
	    data_flag1 = 0;
	} else if(g_proto_status.stop_kws || g_proto_status.stop_call){
	    switch_to_stop_algo();
	    printf("\r\nstop_algo\r\n");
	    if(g_proto_status.stop_kws){
		g_proto_status.stop_kws	= 0;
		reply_common(WTM2101_CMD_STOP_KWS);
	    }
	    if(g_proto_status.stop_call){
		g_proto_status.stop_call = 0;
		reply_common(WTM2101_CMD_STOP_ENC);
	    }
	    g_proto_status.use_aec = 0;
	    g_proto_status.use_kws = 0;
	    g_proto_status.use_enc = 0;
	    g_proto_status.run_mode = RUN_MODE_EMPTY;
	    loop_running = 0;
	    data_flag1 = 0;
	}

    } // end while (1)
}
#endif

#if defined(CONFIG_RAM_JTAG_PESQ)
int jtagxfer_callback_debug(char* buf, int max_len)
{
    return sprintf(buf,	"%s\n",	"hello world");
}

int jtagxfer_callback_spectrum_mask(char label[8], float* buf, int max_count)
{
    return 0;
}

void jtagxfer_callback_reset(void)
{
    //init_enc();
    #if	USE_AGC
    init_agc();
    #endif
    wtm2101_reset_npu();
}

void jtagxfer_callback_process_mono(DenoiseState *st, int16_t* in, int16_t* out, int nr_bytes)
{
    float tmp_in[HOP_LEN] = {0};
    float tmp_out[HOP_LEN] = {0};

    for	(int i = 0; i <	HOP_LEN; i++)
    {
	tmp_in[i] = in[i];
    }

      rnnoise_process_frame(st,	tmp_out, tmp_in);

      for (int i = 0; i	< HOP_LEN; i++)
      {
	out[i] = tmp_out[i];

      }
}

void jtagxfer_callback_process_stereo(int16_t* in, int16_t* out, int nr_bytes)
{
}

void _task_jtagxfer()
{
    static jtaglink_handle_t inst;
    const datalink_message_t* msg;
    int	nr_bytes;
    uint32_t ticks = 0;

    jtaglink_init(&inst, sizeof(int16_t) * HOP_LEN * 2);

    cnn_init();

    DenoiseState *st;
    st = rnnoise_create(NULL);

    while (1) {

	jtaglink_reset(&inst);
	if (jtaglink_wait_handshake(&inst) <= 0) {
	    continue;
	}

	while (1) {
	    ////////////////////////////////////////////////
	    //
	    //	receiving
	    //
	    inst.ts = HAL_GetTick();
	    if (jtaglink_wait_msg(&inst, &inst.rx_msg) <= 0) {
		printf("Error: datalink	timeout\r\n");
		break;
	    }

	    ////////////////////////////////////////////////
	    //
	    //	dispatch
	    //
	    inst.msg = &inst.rx_msg;
	    jtaglink_recv_file_info(&inst);
	    jtaglink_recv_file_data(st,	&inst);

	    if (jtaglink_recv_file_eof(&inst)) {
		rnnoise_destroy(st);
		printf("Info: file done\r\n");
		jtaglink_flush(&inst);
		break;
	    }
	    inst.msg = NULL;
	}
    }
}

#endif



void task_algorithm(void)
{

    _task_jtagxfer();
}

void task_npu(void)
{
    while (1) {

    #if	 defined(CONFIG_RAM_IISMIC_REALTIME_HSI24Mx2) || \
      defined(CONFIG_QSPI_IISMIC_REALTIME_HSI24Mx2) || \
      defined(CONFIG_RAM_K1_IISMIC_REALTIME_HSI24Mx2) || \
      defined(CONFIG_QSPI_K1_IISMIC_REALTIME_HSI24Mx2) || \
      defined(CONFIG_RAM_Y1_IISMIC_REALTIME_HSI24Mx2) || \
      defined(CONFIG_QSPI_Y1_IISMIC_REALTIME_HSI24Mx2)
	//////////////////////////////////////////////////////////////////////////
      //kws_process(buff_in_16bit, AUDIO_FRAME_SIZE);
      //howling_suppress(buff_in1_16bit, buff_out1_16bit);
    #endif
    }
}

#if !defined(CONFIG_RAM_UART_VERIFY_HSI24Mx6) && !defined(CONFIG_RAM_UART_PESQ)
int iis_init(void)
{
    int	ret = 0;

    #if	USE_I2S1
    /*The hal i2s instance0*/
    Hal_I2s_InitTypeDef* hal_i2s_instance1 = hal_i2s_instance_get(HAL_I2S_INSTANCE1);
    if(!hal_i2s_instance1)
    {
	printf("iis_init null");
	return -1;
    }

    /*Init hal i2s instance0,using I2S1,only sending,master,32bits width word,320 words	data of	left and right channel is handled application*/
    ret	= hal_i2s_init(hal_i2s_instance1,I2S1,HAL_I2S_ONLY_SEND,HAL_I2S_MASTER,HAL_I2S_16BITS_WIDTH_WORD,AUDIO_FRAME_SIZE*2);
    if(ret <= 0)
    {
	printf("hal_i2s_init err:%d",ret);
	return -2;
    }
    hal_i2s_instance1->dma.dma_channel = DMA_CHANNEL3;

    /*Open the hal i2s hardware	with the hal i2s instance0*/
    ret	= hal_i2s_open(hal_i2s_instance1);
    if(ret <= 0)
    {
	printf("hal_i2s_open err:%d",ret);
	return -3;
    }
    #endif

    /*The hal i2s instance1*/
    Hal_I2s_InitTypeDef* hal_i2s_instance0 = hal_i2s_instance_get(HAL_I2S_INSTANCE0);
    if(!hal_i2s_instance0)
    {
	printf("iis_init null");
	return -4;
    }

    /*Init hal i2s instance1,using I2S0,only receiving,master,32bits width word,320 words data of left and right channel is handled application*/
    ret	= hal_i2s_init(hal_i2s_instance0,I2S0,HAL_I2S_ONLY_SEND,HAL_I2S_MASTER,HAL_I2S_16BITS_WIDTH_WORD,AUDIO_FRAME_SIZE*2);
    if(ret <= 0)
    {
	printf("hal_i2s_init err:%d",ret);
	return -5;
    }

    /*Open the hal i2s hardware	with the hal i2s instance1*/
    ret	= hal_i2s_open(hal_i2s_instance0);
    if(ret <= 0)
    {
	printf("hal err:%d",ret);
	return -6;
    }

    return 1;
}


int open_audio_stereo(void)
{
    int	ret;
    Audio_InitTypeDef* hal_audio_instance = hal_audio_instance_get(HAL_AUDIO_INSTANCE1);
    if(!hal_audio_instance) {
	printf("audio_ins_err\r\n");
	return -1;
    }
    ret	= hal_audio_init(hal_audio_instance,HAL_AUDIO_MIC_INPUT_AMIC);
    if(ret <= 0) {
	printf("audio_initerr:%d\r\n",ret);
	return -2;
    }

    /*Using the	fifo buffer replace ram	buffer with the	hal audio instance1*/
    hal_audio_instance->channel.BufferMode = HAL_AUDIO_BUFFER_FIFO_MODE;
    /*Using the	fifo half full interrupt replace ram buffer interrupt with the hal audio instance1*/
    hal_audio_instance->interrupt.type = HAL_AUDIO_BUFFER_FIFO_HALF_FULL_INTERRUPT;
    /*Disable the dma with the hal audio instance1*/
    hal_audio_instance->audio_rst = ENABLE;

    hal_audio_instance->dma_enable = DISABLE;
    /*Each 80 words of data are	acquired, then the buffer is handled by	hal audio instance1*/

    ///* the frame shift of audio interrupt is 80 words	*/
    //hal_audio_instance->channel.Buffer_Ram_Frame_Move	= 80;
    ///*Each 80	words of data are acquired, then an audio interrupt is triggered */
    hal_audio_instance->channel.Buffer_Ram_Length = 80;

    hal_audio_instance->Analog.MicBoostGain = HAL_AUDIO_ANALOG_MIC_BOOST_0DB;
    hal_audio_instance->Analog.LinBoostGain = HAL_AUDIO_ANALOG_LIN_BOOST_M12DB;
    hal_audio_instance->channel.Filter.FuncHighPass = DISABLE;
    // bugfixed: 2024.08.08 mic	recording `pop`	noise begin
    // bugfixed: 2024.08.27 mic	recording `pop`	noise begin
    // bugfixed: 2024.08.30 {
    //	  patch1 "remove hardware typec	switch"
    //	  or patch2 "increase vbias voltage"
    //	    - val |= AUDIO_ANA2CFG_BIAS_SET_O_Msk & (2 << AUDIO_ANA2CFG_BIAS_SET_O_Pos);
    //	    + val |= AUDIO_ANA2CFG_BIAS_SET_O_Msk & (1 << AUDIO_ANA2CFG_BIAS_SET_O_Pos);
    // }
    // begin
    hal_audio_instance->Analog.AGCTargetThreshold  = (int)(32768 * 0.1);
    hal_audio_instance->Analog.AGCMaximalThreshold = (int)(32768 * 0.5);
    hal_audio_instance->Analog.PGAGainDefault	   =  68; // skip it
    hal_audio_instance->Analog.PGAGainMaximal	   =  24; // maximal  +6.0dB
    hal_audio_instance->Analog.PGAGainMinmal	   =   2; // minimal -10.5dB
    hal_audio_instance->Analog.AGCOverSampling	   =  80;
    hal_audio_instance->Analog.AGCShortTermAttack  =-104;
    hal_audio_instance->Analog.AGCLongTermAttack   = -35;
    hal_audio_instance->Analog.AGCLongTermRelease  =   5;
    // end
    hal_audio_instance->channel.ChannelGain = 10;
    hal_audio_instance->Analog.AGC_Enable = ENABLE;

    /* Open the	audio hardware with the	hal audio instance */
    ret	= hal_audio_open(hal_audio_instance);
    if (ret <= 0) {
	printf("hal_audio_open is error:%d\r\n",ret);
	return -3;
    }

    #if	USE_LINEIN
    /*------------------- Line in -----------------*/
    /* The hal audio instance */
    Audio_InitTypeDef* hal_audio_instance2 = hal_audio_instance_get(HAL_AUDIO_INSTANCE2);
    if (!hal_audio_instance2) {
	printf("hal_audio_instance2 null",ret);
	return ret;
    }

    /* Init hal	audio instance,using the input linein */
    ret	= hal_audio_init(hal_audio_instance2,HAL_AUDIO_MIC_INPUT_LINEIN);
    if (ret <= 0) {
	printf("hal_audio_init2	err:%d\r\n",ret);
	return ret;
    }
    /* Each 80 words of	data are acquired, then	the buffer is handled by hal audio instance */
    hal_audio_instance2->channel.Buffer_Ram_Length = 80;
    hal_audio_instance2->audio_rst = DISABLE;
    hal_audio_instance2->divider = 2;
    hal_audio_instance2->prescale = 6;
    hal_audio_instance2->Analog.MicBoostGain = HAL_AUDIO_ANALOG_MIC_BOOST_12DB;
    hal_audio_instance2->Analog.LinBoostGain = HAL_AUDIO_ANALOG_LIN_BOOST_M12DB;
    hal_audio_instance2->channel.ChannelGain = 10;
    hal_audio_instance2->Analog.AGC_Enable = ENABLE;
    /* Open the	audio hardware with the	hal audio instance */
    ret	= hal_audio_open(hal_audio_instance2);
    if (ret <= 0) {
	printf("hal_audio_open2	err:%d\r\n",ret);
	return ret;
    }
    #endif

    return 0;
}


//
// yL: KWS only, no linein
//
int open_audio_mono(void)
{
    int	ret;
    Audio_InitTypeDef* hal_audio_instance = hal_audio_instance_get(HAL_AUDIO_INSTANCE1);
    if(!hal_audio_instance) {
	printf("audio_ins_err\r\n");
	return -1;
    }
    ret	= hal_audio_init(hal_audio_instance,HAL_AUDIO_MIC_INPUT_AMIC);
    if(ret <= 0) {
	printf("audio_initerr:%d\r\n",ret);
	return -2;
    }

    hal_audio_instance->channel.BufferMode	      =	HAL_AUDIO_BUFFER_RAM_MODE;
    hal_audio_instance->interrupt.enable	      =	DISABLE;
    hal_audio_instance->interrupt.type		      =	AUDIO_RAM_FRAME_VLD_INTERRUPT;
    hal_audio_instance->audio_rst		      =	ENABLE;
    hal_audio_instance->dma_enable		      =	DISABLE;
    hal_audio_instance->channel.Buffer_Ram_Depth      =	480/2;
    hal_audio_instance->channel.Buffer_Ram_Frame_Move =	160/2;
    hal_audio_instance->channel.Buffer_Ram_Length     =	160/2;
    hal_audio_instance->Analog.MicBoostGain	      =	HAL_AUDIO_ANALOG_MIC_BOOST_12DB;
    hal_audio_instance->Analog.LinBoostGain	      =	HAL_AUDIO_ANALOG_LIN_BOOST_M12DB;
    hal_audio_instance->channel.ChannelGain	      =	10;
    //// bugfixed: 2024.08.08 mic recording `pop` noise	begin
    //if (hal_audio_instance->Analog.AGC_Enable) {
    //	  hal_audio_instance->Analog.PGAGainDefault	= hal_audio_instance->Analog.PGAGainMaximal;
    //}
    //// end
    // bugfixed: 2024.08.27 mic	recording `pop`	noise begin
    if (hal_audio_instance->Analog.AGC_Enable) {
	hal_audio_instance->Analog.PGAGainDefault = 68;
    }
    // end

    /* Open the	audio hardware with the	hal audio instance */
    ret	= hal_audio_open(hal_audio_instance);
    if (ret <= 0) {
	printf("hal_audio_open is error:%d\r\n",ret);
	return -3;
    }

    return 0;
}


int i2s0_start_or_stop(uint8_t state)
{
    int	ret = 0;
    Hal_I2s_InitTypeDef* hal_i2s_instance0 = hal_i2s_instance_get(HAL_I2S_INSTANCE0);

    /*Start the	i2s0 with the hal i2s instance1*/
    ret	= hal_i2s_ctl(hal_i2s_instance0,HAL_I2S_CHANNEL_ENABLE_COMMAND,state);
    if(ret <= 0)
    {
	printf("hal_i2s_ctl is error:%d\r\n",ret);
	return -1;
    }

    return 1;
}

#if USE_I2S1
int i2s1_start_or_stop(uint8_t state)
{
    int	ret = 0;
    Hal_I2s_InitTypeDef* hal_i2s_instance1 = hal_i2s_instance_get(HAL_I2S_INSTANCE1);
    //if(state){
    //	  int16_t tmp_[160] = {0};
    //	  for(int i=0;i<I2S1_BUFFER_NUM-1;i++){
    //	      hal_i2s_write(hal_i2s_instance1,tmp_, buff_music_inL_16,hal_i2s_instance1->lr_channel_need_sizes_by_width	/ 2);
    //	  }
    //}

    /*Start the	i2s1 with the hal i2s instance1*/
    ret	= hal_i2s_ctl(hal_i2s_instance1,HAL_I2S_CHANNEL_ENABLE_COMMAND,state);
    if(ret <= 0)
    {
	printf("hal_i2s_ctl is error:%d\r\n",ret);
	return -1;
    }

    return 1;
}
#endif


static int hal_start_audio(uint8_t start_linein){

    int	ret = 0;
    Audio_InitTypeDef* hal_audio_instance1 = hal_audio_instance_get(HAL_AUDIO_INSTANCE1);
    Audio_InitTypeDef* hal_audio_instance2 = hal_audio_instance_get(HAL_AUDIO_INSTANCE2);

     /*Enable the audio	interrupt with the hal audio instance1*/
    if (hal_audio_instance1->enable) {
	ret = hal_audio_ctl(hal_audio_instance1,HAL_AUDIO_INTERRUPT_ENABLE_COMMAND,ENABLE);
	if(ret <= 0) {
	    printf("hal_audio_ctl err:%d\r\n",ret);
	    return -1;
	}
    }

    #if	USE_LINEIN
    if(start_linein && hal_audio_instance2->enable){
	ret = hal_audio_ctl(hal_audio_instance2,HAL_AUDIO_INTERRUPT_ENABLE_COMMAND,ENABLE);
	if(ret <= 0) {
	    printf("hal_audio_ctl err:%d\r\n",ret);
	    return -3;
	}
    }
    #endif

    if (hal_audio_instance1->enable) {
	if (hal_audio_instance1->channel.BufferMode == HAL_AUDIO_BUFFER_FIFO_MODE) {
	    AUDIO_Clear_Fifo_Interrupt(hal_audio_instance1->instance, hal_audio_instance1->channel.ChannelNumber, AUDIO_FIFO_OVERFLOW_INTERRUPT	| AUDIO_FIFO_UNDERFLOW_INTERRUPT);
	} else {
	    AUDIO_Clear_Ram_Interrupt(hal_audio_instance1->instance, hal_audio_instance1->channel.ChannelNumber, AUDIO_RAM_FRAME_VLD_INTERRUPT);
	}
    }

    #if	USE_LINEIN
    if(start_linein && hal_audio_instance2->enable){
	AUDIO_Clear_Fifo_Interrupt(hal_audio_instance2->instance, hal_audio_instance2->channel.ChannelNumber, AUDIO_FIFO_OVERFLOW_INTERRUPT | AUDIO_FIFO_UNDERFLOW_INTERRUPT);
    }
    #endif

    /*Start the	audio with the hal audio instance1*/
    if (hal_audio_instance1->enable) {
	ret = hal_audio_ctl(hal_audio_instance1,HAL_AUDIO_CHANNEL_ENABLE_COMMAND,ENABLE);
	if(ret <= 0){
	    printf("hal_audio_ctl err:%d\r\n",ret);
	    return -4;
	}
    }

    #if	USE_LINEIN
    if(start_linein && hal_audio_instance2->enable){
	ret = hal_audio_ctl(hal_audio_instance2,HAL_AUDIO_CHANNEL_ENABLE_COMMAND,ENABLE);
	if(ret <= 0)
	{
	    printf("hal_audio_ctl err:%d\r\n",ret);
	    return -6;
	}
    }
    #endif

    ECLIC_SetTrigIRQ(AUDIO_IRQn, ECLIC_LEVEL_TRIGGER);
    //ECLIC_SetTrigIRQ(AUDIO_IRQn, ECLIC_POSTIVE_EDGE_TRIGGER);
    ECLIC_EnableIRQ(AUDIO_IRQn);

}

static int hal_stop_audio(uint8_t stop_linein){

    //stop_linein = 1;

    ECLIC_DisableIRQ(AUDIO_IRQn);
    ECLIC_ClearPendingIRQ(AUDIO_IRQn);

    int	ret = 0;
    Audio_InitTypeDef* hal_audio_instance1 = hal_audio_instance_get(HAL_AUDIO_INSTANCE1);
    Audio_InitTypeDef* hal_audio_instance2 = hal_audio_instance_get(HAL_AUDIO_INSTANCE2);

    /*Enable the audio interrupt with the hal audio instance1*/
    if (hal_audio_instance1->enable) {
	ret = hal_audio_ctl(hal_audio_instance1,HAL_AUDIO_INTERRUPT_ENABLE_COMMAND,DISABLE);
	if(ret <= 0) {
	    printf("hal_audio_ctl err:%d\r\n",ret);
	    return -1;
	}
    }

    #if	USE_LINEIN
    if(stop_linein && hal_audio_instance2->enable){
	ret = hal_audio_ctl(hal_audio_instance2,HAL_AUDIO_INTERRUPT_ENABLE_COMMAND,DISABLE);
	if(ret <= 0) {
	    printf("hal_audio_ctl err:%d\r\n",ret);
	    return -3;
	}
    }
    #endif

    /*Start the	audio with the hal audio instance1*/
    if (hal_audio_instance1->enable) {
	ret = hal_audio_ctl(hal_audio_instance1,HAL_AUDIO_CHANNEL_ENABLE_COMMAND,DISABLE);
	if(ret <= 0)
	{
	    printf("hal_audio_ctl err:%d\r\n",ret);
	    return -4;
	}
    }

    #if	USE_LINEIN
    if(stop_linein && hal_audio_instance2->enable){
	ret = hal_audio_ctl(hal_audio_instance2,HAL_AUDIO_CHANNEL_ENABLE_COMMAND,DISABLE);
	if(ret <= 0)
	{
	    printf("hal_audio_ctl err:%d\r\n",ret);
	    return -6;
	}
    }
    #endif

    if (hal_audio_instance1->enable) {
	if (hal_audio_instance1->channel.BufferMode == HAL_AUDIO_BUFFER_FIFO_MODE) {
	    Ring_Cache_Clean(&(hal_audio_instance1->audio_cache.cache.ring));
	}
    }

    #if	USE_LINEIN
    if(stop_linein && hal_audio_instance2->enable){
	if (hal_audio_instance2->channel.BufferMode == HAL_AUDIO_BUFFER_FIFO_MODE) {
	    Ring_Cache_Clean(&(hal_audio_instance2->audio_cache.cache.ring));
	}
    }
    #endif

}

#endif //CONFIG_RAM_UART_VERIFY_HSI24Mx6


static void system_clock_init_internal_24M(uint8_t mul_ratio)
{
    int	ret = 0;
    /*The clock	instance*/
    Hal_Clock_InitTypeDef* hal_clock_instance =	hal_clock_instance_get();
    Hal_Clock_24mosc_Configuration hal_clock_24mosc_Configuration;

    /*Construct	24mosc related configuration parameters*/
    hal_clock_24mosc_Configuration.use_24mosc_way = HAL_CLOCK_24MOSC_INTERNAL;
    hal_clock_24mosc_Configuration.clock_hz = 24576000;
    hal_clock_24mosc_Configuration.clock_divider = 1;
    hal_clock_24mosc_Configuration.internal_24mosc_calibration_flag = ENABLE;
    hal_clock_24mosc_Configuration.calibration_way = HAL_CLOCK_24MOSC_USE_NPU_CALIBRATION_PARAMETER;

    /*Initialize the clock instance.the	system clock is	from internal 24mosc.the 24mosc	uses calibration parameters that are saved in the NPU */
    if(mul_ratio==1) {
	ret = hal_clock_init(hal_clock_instance,HAL_CLOCK_24MOSC,HAL_CLOCK_24MOSC,&hal_clock_24mosc_Configuration,0,1,1);
    }else {
	ret = hal_clock_init(hal_clock_instance,HAL_CLOCK_24MOSC | HAL_CLOCK_PLL,HAL_CLOCK_PLL,&hal_clock_24mosc_Configuration,mul_ratio,1,1);
    }
    if(ret <= 0) {
	printf("hal_clock_init error:%d\r\n", ret);
    }

    /*According	to the clock instance, initialize the hardware*/
    ret	= hal_clock_open(hal_clock_instance);
    if(ret <= 0) {
	printf("hal_clock_init error:%d\r\n", ret);
    }
}

void main(void)
{
    printf_output_redirect_set(PRINTF_RETARGER_SEGGER);

    volatile uint64_t _init = 0, _end =	0;
    volatile double time = 0;
    volatile uint16_t *temp = NULL;

    RCC_Peri_Rst(0xFFFFFFFF & (~(RCC_PMU_PRSTN | RCC_GPIO_RSTN)));

    /* Enable N307 Interrupt */
    __enable_irq();
    __enable_mcycle_counter();

    system_clock_init_internal_24M(PLL_N);

    // close first
    RCC_CLK_EN_Ctl(RCC_UART0_CLKEN,DISABLE);
    RCC_CLK_EN_Ctl(RCC_UART1_CLKEN,DISABLE);
#if UART_ENABLE
    uart_open();
    printf_output_redirect_set(USE_PRINTF);
#endif

    task_algorithm();
}
