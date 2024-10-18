#ifndef	 __CNN_FRAMEWORK_H__
#define	__CNN_FRAMEWORK_H__
#include "witin_npu_interface.h"
#include "witin_npu_engine.h"
#include "witin_type.h"


int cnn_init();

void enc_nn_process(unsigned char *input_mic, unsigned char *input_ref,	signed char *output);

void kws_nn_process(int	frame_mod, unsigned char *input, signed	char *output);

void get_kws_input_output(int *inputDim, int *outputDim);

void nn_switch(uint8_t switch_flag);

void wtm2101_reset_npu();
void npu_infer (unsigned char *input, signed char *output);

#endif