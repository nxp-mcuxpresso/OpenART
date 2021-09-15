/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
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