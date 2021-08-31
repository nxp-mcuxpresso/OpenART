#ifndef __WEIGHT_CACHE_BUNDLE_H__
#define __WEIGHT_CACHE_BUNDLE_H__
#include "stdint.h"
#ifdef __cplusplus
extern "C"{
#endif
	void* alloc_weight_cache(uint32_t size);
	void* alloc0_weight_cache(uint32_t size);
	void free_weight_cache();
	void init_weigth_cache(void* tail, uint32_t size);
	void deinit_weight_cache();
#ifdef   __cplusplus
}
#endif
#endif // __WEIGHT_CACHE_BUNDLE_H__