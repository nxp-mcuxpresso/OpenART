#pragma once
#if defined(__cplusplus)
extern "C" {
#endif

#include "aiann/aia_cmsisnn_ext.h"

#ifdef __CC_ARM
#pragma anon_unions
#endif
// may consider to align to cache line, but seems has hardly effect.
// however, better to align to 4 or 8 byte. If you are sure your heap allocator always return aligned 
// addresses, then set this to 0.
#define CI_CFG_TENSOR_ALIGN		8
#define CI_CFG_TENSOR_FORMAT	0	// 0=[H][W][C], 1=[C][H][W]
// below "CFG" items should not be changed as they must match model data file's header structure length 
#define CI_MAX_ALLOC_BUF		16
#define CI_MAX_BUF_PER_CMD		8
/*
NNCIE is a very simple "inference engine" that interprets the model.
Currently only supports models burned in NVM, and model itself provides how to allocate activation buffers
*/

#define CI_ASSERT(x) do { \
	if (!(x)) while(1){} \
	}while(0)

#define CI_COMPLETE		32767L
#define CI_OK			0
#define CI_ERR_GNRC		-1L
#define READ_REG32(adr) (((volatile uint32_t*)(adr))[0])
typedef struct _CI_UserCtx {
	uint64_t dat64[(32 + 192 + CI_MAX_ALLOC_BUF * 16 + 7) / 8];
}CI_UserContext;
typedef struct _CI_UserModelHeader {
	uint64_t dat64[160 / 8];
}CI_UserModelHeader;

typedef struct _CI_OutMetricInfo {
	int h, w, c, fracBits;
}CI_OutMetricInfo, CI_MetricInfo;  


typedef struct _ModelInfo_t
{
    int quantBits;
    const char* pszString;
}ModelInfo_t;

typedef struct _ModelInfo_V2_t
{
    int quantBits;
    const char* pszString;
    CI_MetricInfo inMetric;
    CI_MetricInfo outMetric;
}ModelInfo_V2_t;

typedef int (*pfnReadData_t)(uint32_t modelHandle,
                             const CI_UserModelHeader* p,
                             uint32_t ofs,
                             size_t bytes,
                             void* pvBuf);

/*
 ***************************************************************************************
Internal used data structures and macro defines, put here as reference
 ***************************************************************************************
*/
#define CI_MODELFLAG_LGST                                                      \
  (1 << 0) // logistic regression based binary classification
#define CI_MODELFLAG_REGR                                                      \
  (1 << 1) // regression, may also behaves like classification, such as age
           // estimation
#define CI_MODELFLAG_ANGL (1 << 2)
#define CI_MODELFLAG_CLSF (1 << 3)
#define CI_MODELFLAG_RECON (1 << 4) // Reconstruction error
#define CI_MODELFLAG_IN_I16 (1 << 14)
#define CI_MODELFLAG_IN_F32 (1 << 15)
  typedef struct _CI_ModelHeader
  {
    char sMagic[4]; // 'NCIE'
    uint32_t version;

    uint8_t isXIP;     // mem mapped model usually get burned in NOR flash,
                       // otherwise usually in FS or NAND
    uint8_t quantBits; // 8=int8, 9-16=int16
    uint16_t cmdCnt;
    uint32_t inBufSize;
    uint32_t outBufSize;
    // ------------------

    union
    {
      uint8_t du8;
      uint16_t du16;
      int8_t ds8;
      int16_t ds16;
      int32_t ds32;
    } inMean;
    uint32_t peakColBufSize;
    uint32_t peakActBufCnt;
    uint32_t peakActBufTotalSize;
    uint32_t peakActBufSizes[8];
    union
    {
      uint32_t miscFlags;
      struct
      {
        uint32_t flag_lgst : 1;
        uint32_t flag_regr : 1;
        uint32_t flag_angl : 1;
        uint32_t flag_clsf : 1;
        uint32_t flag_recon : 1;
        uint32_t flag_res9 : 9;
        uint32_t flag_i16in : 1;
        uint32_t flag_f32in : 1;
        uint32_t flag_res1631 : 16;
      };
    };

    union
    {
      uint32_t rsvd[2];
      float f32Params[2];
      uint32_t u32Params[2];
      int16_t i16Params[4];
    };
  } CI_ModelHeader;

  typedef enum _ci_quanttype_e
  {
    ci_quant_i8 = 8,
    ci_quant_i16 = 16,
  } ci_quanttype_e;

  typedef struct _CI_OpcodeMap_t
  {
    uint32_t opcode;
    void* pfn;
  } CI_OpcodeMap_t;

  typedef enum _AuxAct_e
  {
    auxact_linear = 0,
    auxact_relu,
    auxact_sigmoid,
    auxact_tanh,
    auxact_relu4 = 4,
    auxact_softmax,
    auxact_relu6 = 6,
    auxact_rsvd1 = 7,
    auxact_relu8 = 8,
    auxact_na = 128,
  } AuxAct_e;

  typedef struct _CI_CmdParamNN
  {
    // for ob, 255 means use output buffer
    uint8_t ib, ob, cb;
    union
    {
      uint8_t
        aa; // buffer indice. cb: col buf,  aa: auxilary activation parameter
      uint8_t intBits; // int bits of sigmoid/tanh input
    };
    uint8_t ai, so, sb, sw; // quantization settings: shift of input (input frac
                            // bits), shift (right) to get output, bias, weight
    uint8_t kx, ky, sx, sy, px, py, dx, dy; // conv settings
    // ofs = 16
    uint16_t ix, iy, ic;
    // ofs = 22
    uint16_t ox, oy, oc; // feature map settings
    union
    {
      int pad;
      struct
      {
        uint16_t smaxVecDim;
        uint16_t smaxVecCnt;
      };
    };
    uint16_t outFracBits;
    uint8_t zzdbg_4_SB_AMP;
  } CI_CmdParamNN;

  typedef struct _CI_ConcatDims
  {
    uint32_t constDims[3];
    uint32_t varDims[7];
  } CI_ConcatDims;

  typedef struct _CI_CmdConcat
  {
    int8_t ibs[7]; // support up to 7 input tensors to concatenate
    int8_t ob;
    int8_t axisCnt;
    int8_t ofb; // output frac bits (ampShf)
    // ofs = 10
    int8_t ifbs[7]; // input frac bits
    int8_t cbWordLen, axis, tnsCnt;
    // ofs = 20
    uint16_t u16Pad, ox, oy, oz;

  } CI_CmdParamConcat;

  typedef struct _CI_CmdParamMix
  {
    uint8_t ob, so, ibCnt, rsvd;
    uint32_t cbSize;
    uint8_t ibs[6];
    uint8_t sis[6];
    // ofs = 20
    uint16_t u16Pad, ox, oy, oz;
  } CI_CmdParamMix;

  typedef union _CI_CmdParamMax
  {
    // ALL headers, if supports output shape, MUST put OX, OY, OZ at offset 22,
    // 24, 26 !!!
    struct
    {
      uint8_t outShpPads[22];
      uint16_t ox, oy, oc;
    };
    int32_t d32s[10]; // max size is 40 bytes
    CI_CmdParamNN nn;
    CI_CmdParamMix add;
    CI_CmdParamConcat concat;
  } CI_CmdParam;

#define CIBUF_NEW_BEFORE (1 << 0)
#define CIBUF_DEL_AFTER (1 << 1)

  typedef struct _CI_BufDesc
  {
    uint8_t usage; // 0=in, 1=out, 2=colbuf, 3=aux
    uint8_t num;   // buffer num is specified by generator tool
    uint8_t
      alloc; // 0 = do nothing, 1 = alloc before run, 2 = del after run, 3=both
    uint8_t rsvd;
    size_t cbSize;
  } CI_BufDesc;

  typedef struct _CI_Buf
  {
    CI_BufDesc desc;
    union
    {
      void* pvRaw; // 0 means not allocated
      uint8_t* pu8Raw;
      unsigned int addrRaw;
    };
    union
    {
      void* pv;
      uint8_t* pu8;
      unsigned int addr;
    };
  } CI_Buf;

  typedef union _CICmd_t
  {
    struct
    {
      uint8_t isChw : 1;
      uint8_t shape : 2; // XX=0, XY=1, 1D=2
      uint8_t isMax : 1;
      uint8_t dType : 2; // i8,i16,f16,f32
      uint8_t op2 : 2;   // -,RGB,Fast,-
      uint8_t op1;
    };
    uint32_t d32;
  } CICmd_u;

  // Command header must be 160 bytes
  typedef struct _CI_CmdHeader
  {
    char sMagic[4];       // CMD\0
    uint8_t c4HeaderSize; // this must be at offset 4, and generator can add
                          // more data than CI_CmdHeader
    uint8_t parallelNdx;  // commands with same parallel index before index
                          // change, can be executed in parallel
    int8_t auxAct; // if a primary operator has auxilary activation, 0 = none
                   // (linear)
    uint8_t rsvd[1];
    uint32_t ofsInModel;
    CICmd_u cmd;
    //----------------------------
    CI_CmdParam prm; // 40 bytes
    CI_BufDesc bufDescs[CI_MAX_BUF_PER_CMD];
    uint32_t weitLen; // in bytes
    uint32_t biasLen; // in bytes
  } CI_CmdHeader;

  typedef struct _CI_CmdString
  {
    char sMagic[4];       // CMD\0
    uint8_t c4HeaderSize; // this must be at offset 4, and generator can add
                          // more data than CI_CmdHeader
    uint8_t parallelNdx;  // commands with same parallel index before index
                          // change, can be executed in parallel
    int8_t auxAct; // if a primary operator has auxilary activation, 0 = none
                   // (linear)
    uint8_t rsvd[1];
    uint32_t ofsInModel;
    CICmd_u cmd; // must be (CIOP_STRING<<8)
    //----------------------------
    char sz[112];
  } CI_CmdString;

  typedef struct _CI_Context
  {
    uint32_t inuse:1;
    uint32_t isMTSafe:1;
    uint32_t mcuID:1;
    uint32_t isReturnOutptBuffer:1;
    uint32_t isSkipPostProc:1;
    uint32_t cnt;
    const void* pcvIn;
    void* pvOut;
    uint32_t modelHandle;
    pfnReadData_t pfnReadData;
    const CI_ModelHeader* pModel;
    int cmdNdx;
    uint32_t ofsNextCmd;
    const char* pszStr;
    CI_Buf colBufs[1];
    CI_Buf bufs[CI_MAX_ALLOC_BUF];
    CI_CmdHeader cmdHdr;
    int8_t* pPair0;
    uint32_t wtSizeAcc;
    CI_OutMetricInfo outMetric;
  } CI_Context, *CI_Handle;

#define CIENC_CHW 1
#define CIENC_XY (1 << 1)
#define CIENC_MAX (1 << 3)
#define CIENC_16B (1 << 4)
#define CIENC_FAST (2 << 6)
#define CIENC_RGB (1 << 6)

// CIOP_XXX is the "op1" field ([15:08]) of 32-bit command 
#define CIOP_CNV 0
#define CIOP_CNVDW 1
#define CIOP_FC 2
#define CIOP_POOL 3
#define CIOP_RELU 4
#define CIOP_SOFTMAX 5
#define CIOP_RELU6 6
#define CIOP_ADD2 7
#define CIOP_CNVPW 8
#define CIOP_BN 11
#define CIOP_CONCAT 12
#define CIOP_STRING 254
#define CIOP_INPUT 255

#define CICMD_HWCADD208B 1792
#define CICMD_HWCADD216B 1808
#define CICMD_HWCAVEPOOLXX08B 768
#define CICMD_HWCAVEPOOLXX16B 784
#define CICMD_HWCAVEPOOLXY08B 770
#define CICMD_HWCAVEPOOLXY16B 786
#define CICMD_HWCBATCHNORM08B 2816
#define CICMD_HWCBATCHNORM16B 2832
#define CICMD_HWCCNVDWXXFAST08B 384
#define CICMD_HWCCNVDWXXFAST16B 400
#define CICMD_HWCCNVDWXYFAST08B 386
#define CICMD_HWCCNVDWXYFAST16B 402
#define CICMD_HWCCNVPWFAST08B 2176
#define CICMD_HWCCNVPWFAST16B 2192
#define CICMD_HWCCNVXX08B 0
#define CICMD_HWCCNVXX16B 16
#define CICMD_HWCCNVXXFAST08B 128
#define CICMD_HWCCNVXXFAST16B 144
#define CICMD_HWCCNVXXRGB08B 64
#define CICMD_HWCCNVXXRGB16B 80
#define CICMD_HWCCNVXY08B 2
#define CICMD_HWCCNVXY16B 18
#define CICMD_HWCCNVXYFAST08B 130
#define CICMD_HWCCNVXYFAST16B 146
#define CICMD_HWCCONCAT08B 3072
#define CICMD_HWCCONCAT16B 3088
#define CICMD_HWCFC08B 512
#define CICMD_HWCFC16B 528
#define CICMD_HWCMAXPOOLXX08B 776
#define CICMD_HWCMAXPOOLXX16B 792
#define CICMD_HWCMAXPOOLXY08B 778
#define CICMD_HWCMAXPOOLXY16B 794
#define CICMD_HWCRELU08B 1024
#define CICMD_HWCRELU16B 1040
#define CICMD_HWCRELU408B 4864
#define CICMD_HWCRELU416B 4880
#define CICMD_HWCRELU608B 1536
#define CICMD_HWCRELU616B 1552
#define CICMD_HWCRELU808B 4608
#define CICMD_HWCRELU816B 4624
#define CICMD_HWCSIGMOID08B 2304
#define CICMD_HWCSIGMOID16B 2320
#define CICMD_HWCSOFTMAX08B 1280
#define CICMD_HWCSOFTMAX16B 1296
#define CICMD_HWCTANH08B 2560
#define CICMD_HWCTANH16B 2576

#define CICMD_STRING (CIOP_STRING << 8)

// 以下工具宏用于提取各算子中的不同参数
#define IB (q7_t*)(pCtx->bufs[p->ib].pv)
#define OB (q7_t*)(pCtx->bufs[p->ob].pv)
#define IB0 (q7_t*)(pCtx->bufs[p->ibs[0]].pv)
#define IB1 (q7_t*)(pCtx->bufs[p->ibs[1]].pv)
#define IBN(n) (q7_t*)(pCtx->bufs[p->ibs[(n)]].pv)
#define OB0 (q7_t*)(pCtx->bufs[p->obs[0]].pv)
#define OB1 (q7_t*)(pCtx->bufs[p->obs[1]].pv)

#define IBW (q15_t*)(pCtx->bufs[p->ib].pv)
#define OBW (q15_t*)(pCtx->bufs[p->ob].pv)
#define IB0W (q15_t*)(pCtx->bufs[p->ibs[0]].pv)
#define IB1W (q15_t*)(pCtx->bufs[p->ibs[1]].pv)
#define OB0W (q15_t*)(pCtx->bufs[p->obs[0]].pv)
#define OB1W (q15_t*)(pCtx->bufs[p->obs[1]].pv)

#define IX (p->ix)
#define OX (p->ox)
#define IY (p->iy)
#define OY (p->oy)
#define IC (p->ic)
#define ICNT ((uint32_t)p->ix * p->iy * p->ic)
#define OC (p->oc)
#define OCNT ((uint32_t)p->ox * p->oy * p->oc)

#define KX (p->kx)
#define KY (p->ky)
#define SX (p->sx)
#define SY (p->sy)
#define PX (p->px)
#define PY (p->py)
#define DX (p->dx)
#define DY (p->dy)
#define AI (p->ai)
#define SW (p->sw)

#define AA (p->aa)

#define SB (p->sb)
#define SO (p->so)
#define AI0 (p->sis[0])
#define AI1 (p->sis[1])
#define COL ((q7_t*)(pCtx->colBufs[0].pv))
#define COLW ((q15_t*)(pCtx->colBufs[0].pv))

#define WB ((q7_t*)pWB)
#define WT ((q7_t*)pWT)
#define WBW ((q15_t*)pWB)
#define WTW ((q15_t*)pWT)

#define MAKE_CTX()                                                             \
  do {                                                                         \
    CI_ASSERT(sizeof(CI_Context) <= sizeof(CI_UserContext));                   \
    CI_Context* pCtx = (CI_Context*)pUserCtx;                                  \
  } while (0)

#define MAKE_HDR()                                                             \
  do {                                                                         \
    CI_ASSERT(sizeof(CI_Context) <= sizeof(CI_UserContext));                   \
    CI_Context* pCtx = (CI_Context*)pUserCtx;                                  \
  } while (0)


/*
 ***************************************************************************************
*/

// Specify a (fast RAM) cache for weight, if set, NNCU will copy weight data to this cache,
// whenever weit size is smaller than "capacity". When weight is in SDRAM, this can greatly
// increase performance, at up to more than 3 times!
void CI_SetWeitCache(void *pvAddr, size_t capacity);

/* Get model information: string info and output fraction bits*/
int CI_GetModelInfoXIP(const void* pvModel, ModelInfo_t* pInf);
/* 2nd version of CI_GetModelInfoXIP, adding the metrics of both input tensor and output tensor*/
int CI_GetModelInfoXIP_V2(const void* pvModel, ModelInfo_V2_t * pInf);

int CI_RunModelXIP(const void *pvModel, const void *pcvIn, void *pvOutBuf);
int CI_RunModelXIP_NoCopyOutput(const void* pvModel, const void* pcvIn, void** ppvOutBuf);
int CI_RunModelXIP_MTSafe(const void *pvModel, const void *pcvIn, void *pvOutBuf);
const CI_OutMetricInfo* CI_GetOutMetricInfo(const CI_UserContext* pUserCtx);
const char* CI_GetModelStringXIP(const void* pvModel);

/* enable/disable post processing.if pCtx is NULL, then use the default context,
 this is the case if you use CI_RunModelXXX family APIs*/
void CI_ConfigPostProcess(CI_Context * pCtx, uint8_t isEnabled);

// 以下是比较低级或内部使用的API
// modelHandle: A user-interpreted value to identify the model to run, usually useful for non-xip models to identify which model.
// pUserCtx: pointer to context of this CI engine. If zero, CI will try to use its default context, but only OK if the context is not inuse
// pfnReadData: pointer to a function that read data from non-xip models. If zero, will use the default reader in the port.
int CI_zzCreateContextNonXIP(uint32_t modelHandle, CI_UserContext* pUserCtx, void *pvOutBuf, pfnReadData_t pfnReadData);

// modelHandle: A user-interpreted value to identify the model to run, usually useful for non-xip models to identify which model.
// pCtx: pointer to context of this CI engine. If zero, CI will try to use its default context, but only OK if the context is not inuse
// pvModel: pointer to the model data, always starts with the model header structure
int CI_zzCreateContextXIP(uint32_t modelHandle, CI_UserContext*pUserCtx, void *pvOutBuf, const void *pvModel);

// >>>>>>>>>
// 从模型里申请和释放权重、偏置。对于XIP类型（模型存储在可寻址的地址空间中），申请和释放只是获取指向模型中相关数据的指针
int AllocWeits(CI_Context *pCtx, CI_CmdHeader *pCmd, void* *ppWt, void* *ppWb, uint32_t bits);
int FreeWeits(CI_Context* pCtx, CI_CmdHeader* pCmd, void** ppWt, void** ppWb, uint32_t bits);

int CI_zzStep(CI_UserContext* pUserCtx);
int CI_zzRunWithContext(CI_UserContext*pCtx, const void *pcvIn);

// declare default names of model and test vectors in C array form.
extern const unsigned char test1[];
extern const unsigned char model1[];

// Below functions need to port. They all have default weak implementations
// However, for non-XIP model storage, usually you need to implement "CIPort_DefaultReadData"
extern void* CIPort_Malloc(size_t size);
extern void CIPort_Free(void *p);
extern int CIPort_DefaultReadData(uint32_t modelHandle, const CI_UserModelHeader *p, uint32_t ofs, size_t bytes, void *pvBuf);
extern int CIPort_EnterCritical(void);
extern int CIPort_LeaveCritical(void);

// 以下函数主要用于辅助调试神经网络
extern /*__WEAK*/ void CI_DebugRecoverFloat_q7(const q7_t* pcIn,
                                    float rcvr[],
                                    size_t rcvrBufSize,
                                    uint32_t shf,
                                    uint32_t h,
                                    uint32_t w,
                                    uint32_t c,
                                    uint32_t n);

extern /*__WEAK*/ void CI_DebugRecoverFloat_q15(const q15_t* pcIn,
                                     float rcvr[],
                                     size_t rcvrBufSize,
                                     uint32_t shf,
                                     uint32_t h,
                                     uint32_t w,
                                     uint32_t c,
                                     uint32_t n);
typedef enum
{
  kRecover_out,
  kRecover_weit,
  kRecover_bias,
} RecoverType_enum;

// 这个函数会返回动态分配的buffer。必须调用CIPort_Free()以释放！
extern float* CI_RecoverOutputToFloat(CI_Context* pCtx, CI_CmdHeader* pCmd);
extern float* CI_RecoverWeitToFloat(CI_Context* pCtx, CI_CmdHeader* pCmd);

// 以下钩子函数的默认实现是弱链接的。用户可以重新实现以覆盖默认实现。默认实现不做任何处理，一般会被优化掉。
// 注：重新实现这些钩子函数时不要再使用“__WEAK”修饰！
// 与GenericKernel有关的钩子函数是无论如何都会调用的。也就是说即使是特殊的算子(Concat, ElementwiseOp)，也会调用。
// 调用顺序：
//    对于"Before"类钩子，先调用CI_HookBeforeGenericKernel再调用更加专用的钩子
//    对于"After"类钩子，先调用更加专用的钩子后最后调用CI_HookBeforeGenericKernel
typedef enum
{
    hookresult_none,
    // skip this operator, only used with CI_HookBeforeXXX
    hookresult_skip_op, 
    // skip this operator and attached activation, only used with CI_HookBeforeXXX
    // MUST be after hookresult_skip_op
    hookresult_skip_op_and_aux_act, 
}HookResult_enum;
extern /*__WEAK*/ HookResult_enum CI_HookBeforeGenericKernel(CI_Context* pCtx, CI_CmdHeader* pCmd, void* pvWeit, void* pvBias, void* pvIn,void* pvOut);
extern /*__WEAK*/ HookResult_enum CI_HookAfterGenericKernel(CI_Context* pCtx, CI_CmdHeader* pCmd, void* pvWeit, void* pvBias, void* pvIn,void* pvOut);
extern /*__WEAK*/ HookResult_enum CI_HookBeforeConcat(CI_Context* pCtx, int quantBits, int axis, int tnsCnt, int axisCnt, const CI_TnsDim_t* pcTnss, void* pvOut, int outFracBits);
extern /*__WEAK*/ HookResult_enum CI_HookAfterConcat(CI_Context* pCtx, int quantBits, int axis, int tnsCnt, int axisCnt, const CI_TnsDim_t* pcTnss, void* pvOut, int outFracBits);
extern /*__WEAK*/ HookResult_enum CI_HookBeforeElementwiseOp(q7_t* outBuf,uint32_t cnt, uint32_t so, q7_t* in1, uint32_t si1, q7_t* in2, uint32_t si2);
extern /*__WEAK*/ HookResult_enum CI_HookAfterElementwiseOp(q7_t* outBuf,uint32_t cnt, uint32_t so, q7_t* in1, uint32_t si1, q7_t* in2, uint32_t si2);

extern /*__WEAK*/ HookResult_enum CI_HookAfterAllCmds(CI_Context* pCtx, CI_CmdHeader* pCmd, void* pvOut);

//extern int CIPort_Printf(const char *fmt_s, ...);
#define CIPort_Printf PRINTF

#if defined(__cplusplus)
}
#endif
