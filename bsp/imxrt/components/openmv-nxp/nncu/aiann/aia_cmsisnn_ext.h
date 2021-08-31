#ifndef _AIA_CMSISNN_EXT_H_
#define _AIA_CMSISNN_EXT_H_
#include "arm_math.h"
#include "arm_nnfunctions.h"
#if __ARMCOMPILER_VERSION < 6000000
#pragma anon_unions
#endif
typedef struct _CI_TnsDim_t {
	union {
		uint32_t tnsAddr;
		const int8_t* pcq7;
		const int16_t* pcq15;
	};
	int8_t fracBits;
	int8_t rsvd[3];
	int32_t dims[4]; // supports max to 4 dims
}CI_TnsDim_t;

void aia_softmax_multi_q7(const q7_t* vec_in, const int dim_vec, int vecCnt, q7_t* p_out);
void aia_softmax_multi_q15(const q15_t* vec_in, const uint16_t dim_vec, int vecCnt, q15_t* p_out);
arm_status aia_concat(int quantBits, int axis, int tnsCnt, int axisCnt, const CI_TnsDim_t* pcTnss, 
	void* pvOut, int outFracBits);
void aia_relu6_q7(q7_t * data, uint32_t size, uint32_t shfIn);
void aia_relu8_q7(q7_t* data, uint32_t size, uint32_t shfIn);
void aia_relu4_q7(q7_t* data, uint32_t size, uint32_t shfIn);
void aia_HWC_q7_add2(q7_t *outBuf, uint32_t cnt, uint32_t so, q7_t *in1, uint32_t si1, q7_t *in2, uint32_t si2);
void aia_relu6_q15(q15_t * data, uint32_t size, uint32_t shfIn);
void aia_relu8_q15(q15_t* data, uint32_t size, uint32_t shfIn);
void aia_relu4_q15(q15_t* data, uint32_t size, uint32_t shfIn);
void aia_HWC_q15_add2(q15_t *outBuf, uint32_t cnt, uint32_t so, q15_t *in1, uint32_t si1, q15_t *in2, uint32_t si2);
void aia_sigmoid_q7(q7_t* data, uint32_t size, uint32_t int_width);
void aia_tanh_q7(q7_t* data, uint32_t size, uint32_t int_width);
void aia_sigmoid_q15(q15_t* data, uint32_t size, uint32_t int_width);
void aia_tanh_q15(q15_t* data, uint32_t size, uint32_t int_width);

arm_status aia_convolve_1x1_HWC_q15_fast_nonsquare(const q15_t* Im_in,
	const int32_t dim_im_in_x,
	const int32_t dim_im_in_y,
	const int32_t ch_im_in,
	const q15_t* wt,
	const int32_t ch_im_out,
	const int32_t dim_kernel_x,
	const int32_t dim_kernel_y,
	const int32_t padding_x,
	const int32_t padding_y,
	const int32_t stride_x,
	const int32_t stride_y,
	const q15_t* bias,
	const int32_t bias_shift,
	const int32_t out_shift,
	q15_t* Im_out,
	const int32_t dim_im_out_x,
	const int32_t dim_im_out_y,
	q15_t* bufferA,
	q15_t* bufferB);

arm_status aia_depthwise_separable_conv_HWC_q15(const q15_t* Im_in,
	const int32_t dim_im_in,
	const int32_t ch_im_in,
	const q15_t* wt,
	const int32_t ch_im_out,
	const int32_t dim_kernel,
	const int32_t padding,
	const int32_t stride,
	const q15_t* bias,
	const int32_t bias_shift,
	const int32_t out_shift,
	q15_t* Im_out,
	const int32_t dim_im_out,
	q15_t* bufferA,
	q15_t* bufferB);

arm_status
aia_convolve_HWC_q15_RGB(const q15_t* Im_in,
	const int32_t dim_im_in,
	const int32_t ch_im_in,
	const q15_t* wt,
	const int32_t ch_im_out,
	const int32_t dim_kernel,
	const int32_t padding,
	const int32_t stride,
	const q15_t* bias,
	const int32_t bias_shift,
	const int32_t out_shift,
	q15_t* Im_out, const int32_t dim_im_out, q15_t* bufferA, q15_t* bufferB);

arm_status
aia_convolve_HWC_q15_fast_nonsquare(const q15_t * Im_in,
    const uint16_t dim_im_in_x,
    const uint16_t dim_im_in_y,
    const uint16_t ch_im_in,
    const q15_t * wt,
    const uint16_t ch_im_out,
    const uint16_t dim_kernel_x,
    const uint16_t dim_kernel_y,
    const uint16_t padding_x,
    const uint16_t padding_y,
    const uint16_t stride_x,
    const uint16_t stride_y,
    const q15_t * bias,
    const uint16_t bias_shift,
    const uint16_t out_shift,
    q15_t * Im_out,
    const uint16_t dim_im_out_x,
    const uint16_t dim_im_out_y, 
    q15_t * bufferA, 
    q7_t * bufferB);
    
arm_status aia_convolve_HWC_q15_basic_nonsquare(
    const q15_t * Im_in,
    const int32_t dim_im_in_x,
    const int32_t dim_im_in_y,
    const int32_t ch_im_in,
    const q15_t * wt,
    const int32_t ch_im_out,
    const int32_t dim_kernel_x,
    const int32_t dim_kernel_y,
    const int32_t padding_x,
    const int32_t padding_y,
    const int32_t stride_x,
    const int32_t stride_y,
    const q15_t * bias,
    const int32_t bias_shift,
    const int32_t out_shift,
    q15_t * Im_out,
    const int32_t dim_im_out_x,
    const int32_t dim_im_out_y, 
    q15_t * bufferA, 
    q7_t * bufferB
);
    
arm_status aia_depthwise_separable_conv_HWC_q15_nonsquare(const q15_t* Im_in,
	const int32_t dim_im_in_x,
	const int32_t dim_im_in_y,
	const int32_t ch_im_in,
	const q15_t* wt,
	const int32_t ch_im_out,
	const int32_t dim_kernel_x,
	const int32_t dim_kernel_y,
	const int32_t padding_x,
	const int32_t padding_y,
	const int32_t stride_x,
	const int32_t stride_y,
	const q15_t* bias,
	const int32_t bias_shift,
	const int32_t out_shift,
	q15_t* Im_out,
	const int32_t dim_im_out_x,
	const int32_t dim_im_out_y,
	q15_t* bufferA,
	q15_t* bufferB);

arm_status
aia_convolve_HWC_q15_fast(const q15_t * Im_in,
                          const int32_t dim_im_in,
                          const int32_t ch_im_in,
                          const q15_t * wt,
                          const int32_t ch_im_out,
                          const int32_t dim_kernel,
                          const int32_t padding,
                          const int32_t stride,
                          const q15_t * bias,
                          const int32_t bias_shift,
                          const int32_t out_shift,
                          q15_t * Im_out, 
                          const int32_t dim_im_out, 
                          q15_t * bufferA, 
                          q7_t * bufferB)
;

arm_status
aia_convolve_HWC_q7_fast(const q7_t* Im_in,
	const int32_t dim_im_in,
	const int32_t ch_im_in,
	const q7_t* wt,
	const int32_t ch_im_out,
	const int32_t dim_kernel,
	const int32_t padding,
	const int32_t stride,
	const q7_t* bias,
	const int32_t bias_shift,
	const int32_t out_shift,
	q7_t* Im_out,
	const int32_t dim_im_out,
	q15_t* bufferA,
	q7_t* bufferB)
;


void aia_maxpool_q15_HWC(q15_t* Im_in,
	const int32_t dim_im_in,
	const int32_t ch_im_in,
	const int32_t dim_kernel,
	const int32_t padding,
	const int32_t stride, const int32_t dim_im_out, q15_t* bufferA, q15_t* Im_out);

void aia_avepool_q15_HWC(q15_t* Im_in,
	const int32_t dim_im_in,
	const int32_t ch_im_in,
	const int32_t dim_kernel,
	const int32_t padding,
	const int32_t stride, const int32_t dim_im_out, q15_t* bufferA, q15_t* Im_out);

void aia_avepool_q7_HWC_nonsquare(
	const q7_t* Im_in,           // input image
	const int32_t dim_im_in_x,   // input image dimension
	const int32_t dim_im_in_y,   // input image dimension
	const int32_t ch_im_in,      // number of input image channels
	const int32_t dim_kernel_x,  // window kernel size
	const int32_t dim_kernel_y,  // window kernel size
	const int32_t padding_x,     // padding sizes
	const int32_t padding_y,     // padding sizes
	const int32_t stride_x,      // stride
	const int32_t stride_y,      // stride
	const int32_t dim_im_out_x,  // output image dimension
	const int32_t dim_im_out_y,  // output image dimension
	q7_t* bufferA,               // a buffer for local storage
	q7_t* Im_out,                // output feature
	const int32_t out_lshift);    // output left shift (scaling)

void
aia_avepool_q15_HWC(q15_t* Im_in,
	const int32_t dim_im_in,
	const int32_t ch_im_in,
	const int32_t dim_kernel,
	const int32_t padding,
	const int32_t stride, const int32_t dim_im_out, q15_t* bufferA, q15_t* Im_out);

void aia_avepool_q15_HWC_nonsquare(
	const q15_t* Im_in,           // input image
	const int32_t dim_im_in_x,   // input image dimension
	const int32_t dim_im_in_y,   // input image dimension
	const int32_t ch_im_in,      // number of input image channels
	const int32_t dim_kernel_x,  // window kernel size
	const int32_t dim_kernel_y,  // window kernel size
	const int32_t padding_x,     // padding sizes
	const int32_t padding_y,     // padding sizes
	const int32_t stride_x,      // stride
	const int32_t stride_y,      // stride
	const int32_t dim_im_out_x,  // output image dimension
	const int32_t dim_im_out_y,  // output image dimension
	q15_t* bufferA,               // a buffer for local storage
	q15_t* Im_out,                // output feature
	const int32_t out_lshift);    // output left shift (scaling)

void aia_maxpool_q7_HWC_nonsquare(
	const q7_t* Im_in,           // input image
	const int32_t dim_im_in_x,   // input image dimension
	const int32_t dim_im_in_y,   // input image dimension
	const int32_t ch_im_in,      // number of input image channels
	const int32_t dim_kernel_x,  // window kernel size
	const int32_t dim_kernel_y,  // window kernel size
	const int32_t padding_x,     // padding sizes
	const int32_t padding_y,     // padding sizes
	const int32_t stride_x,      // stride
	const int32_t stride_y,      // stride
	const int32_t dim_im_out_x,  // output image dimension
	const int32_t dim_im_out_y,  // output image dimension
	q7_t* bufferA,               // a buffer for local storage
	q7_t* Im_out,                // output feature
	const int32_t out_lshift);   // output left shift (scaling)

void aia_maxpool_q15_HWC_nonsquare(
	const q15_t* Im_in,           // input image
	const int32_t dim_im_in_x,   // input image dimension
	const int32_t dim_im_in_y,   // input image dimension
	const int32_t ch_im_in,      // number of input image channels
	const int32_t dim_kernel_x,  // window kernel size
	const int32_t dim_kernel_y,  // window kernel size
	const int32_t padding_x,     // padding sizes
	const int32_t padding_y,     // padding sizes
	const int32_t stride_x,      // stride
	const int32_t stride_y,      // stride
	const int32_t dim_im_out_x,  // output image dimension
	const int32_t dim_im_out_y,  // output image dimension
	q15_t* bufferA,               // a buffer for local storage
	q15_t* Im_out,                // output feature
	const int32_t out_lshift);    // output left shift (scaling)
  
    
void aia_nn_activations_direct_q15(q15_t* data, int size, int int_width, arm_nn_activation_type type, int quantBits);
#endif
