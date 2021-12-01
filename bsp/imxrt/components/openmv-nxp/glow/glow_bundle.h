#include "stdint.h"

#define GLOW_ENTRY_OFFSET 68
typedef void(*func_ptr)(uint8_t*, uint8_t*,uint8_t*);

typedef struct {
	uint32_t input_offset;     // 0:4
	uint32_t output_offset;    // 4:8
	uint32_t weights_offset;   // 8:12
	uint32_t entry_offset;     // 12 : 16
	
	uint32_t mut_mem_size;     // 16:20
	uint32_t act_mem_size;     // 20:24
	uint32_t const_mem_size;   // 24:28
	
	uint32_t shape[4];         // 28:32 32：36 36：40 40：44
	uint32_t out_shape[4];   // 44 : 60	
	func_ptr entry;            // 60 : 64 
	
}glow_engine_t;



#define INPUT_OFFSET (*(uint32_t*)(model_buffer))
#define OUTPUT_OFFSET (*(uint32_t*)(model_buffer + 4))
#define WEIGHTS_OFFSET (*(uint32_t*)(model_buffer + 8))
#define ENTRY_OFFSET (*(uint32_t*)(model_buffer + 12))
#define MUT_MEM_SIZE (*(uint32_t*)(model_buffer + 16))
#define ACT_MEM_SIZE (*(uint32_t*)(model_buffer + 20))
#define CONST_MEM_SIZE (*(uint32_t*)(model_buffer + 24))
#define SHAPE_N (*(uint32_t*)(model_buffer + 28))
#define SHAPE_H (*(uint32_t*)(model_buffer + 32))
#define SHAPE_W (*(uint32_t*)(model_buffer + 36))	
#define SHAPE_C (*(uint32_t*)(model_buffer + 40))	
#define SHAPE_ON (*(uint32_t*)(model_buffer + 44))
#define SHAPE_OH (*(uint32_t*)(model_buffer + 48))
#define SHAPE_OW (*(uint32_t*)(model_buffer + 52))	
#define SHAPE_OC (*(uint32_t*)(model_buffer + 56))
#define ZI_POS (*(uint32_t*)(model_buffer + 60))
#define ZI_OFFSET (*(uint32_t*)(model_buffer + 64))

#define INIT_GLOW_ENGINE(glow_engine_ptr, model) \
	model_buffer = (uint32_t)model; \
	(glow_engine_ptr)->input_offset = INPUT_OFFSET; \
	(glow_engine_ptr)->output_offset = OUTPUT_OFFSET; \
	(glow_engine_ptr)->weights_offset = WEIGHTS_OFFSET; \
	(glow_engine_ptr)->entry_offset = ENTRY_OFFSET; \
	(glow_engine_ptr)->mut_mem_size = MUT_MEM_SIZE; \
	(glow_engine_ptr)->act_mem_size = ACT_MEM_SIZE; \
	(glow_engine_ptr)->const_mem_size = CONST_MEM_SIZE; \
	(glow_engine_ptr)->shape[0] = SHAPE_N; \
	(glow_engine_ptr)->shape[1] = SHAPE_H; \
	(glow_engine_ptr)->shape[2] = SHAPE_W; \
	(glow_engine_ptr)->shape[3] = SHAPE_C; \
	(glow_engine_ptr)->out_shape[0] = SHAPE_ON; \
	(glow_engine_ptr)->out_shape[1] = SHAPE_OH; \
	(glow_engine_ptr)->out_shape[2] = SHAPE_OW; \
	(glow_engine_ptr)->out_shape[3] = SHAPE_OC; \
	/* relocate the ZI_SEC */ \
	*((uint32_t*)(model_buffer + ZI_POS)) = (uint32_t)(model_buffer + ZI_OFFSET)
	

static uint32_t model_buffer;