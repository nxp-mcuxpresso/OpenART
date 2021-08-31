#include <mp.h>
#include "py_helper.h"
#include "py_image.h"
#include "ff_wrapper.h"
#include "nncie.h"
#include "rtconfig.h"


// nncu Model Object
typedef struct py_nncu_model_obj {
    mp_obj_base_t base;
    unsigned char *model_data;
    unsigned int model_data_len;
	unsigned int height, width, channels;
} py_nncu_model_obj_t;

STATIC void py_nncu_model_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    py_nncu_model_obj_t *self = self_in;
    mp_printf(print,
              "{\"height\":%d, \"width\":%d, \"channels\":%d}",
              self->height,
              self->width,
              self->channels);
}

// nncu Classification Object
#define py_nncu_classification_obj_size 5
typedef struct py_nncu_classification_obj {
    mp_obj_base_t base;
    mp_obj_t x, y, w, h, output;
} py_nncu_classification_obj_t;

STATIC void py_nncu_classification_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    py_nncu_classification_obj_t *self = self_in;
    mp_printf(print,
              "{\"x\":%d, \"y\":%d, \"w\":%d, \"h\":%d, \"output\":",
              mp_obj_get_int(self->x),
              mp_obj_get_int(self->y),
              mp_obj_get_int(self->w),
              mp_obj_get_int(self->h));
    mp_obj_print_helper(print, self->output, kind);
    mp_printf(print, "}");
}

STATIC mp_obj_t py_nncu_classification_subscr(mp_obj_t self_in, mp_obj_t index, mp_obj_t value)
{
    if (value == MP_OBJ_SENTINEL) { // load
        py_nncu_classification_obj_t *self = self_in;
        if (MP_OBJ_IS_TYPE(index, &mp_type_slice)) {
            mp_bound_slice_t slice;
            if (!mp_seq_get_fast_slice_indexes(py_nncu_classification_obj_size, index, &slice)) {
                nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "only slices with step=1 (aka None) are supported"));
            }
            mp_obj_tuple_t *result = mp_obj_new_tuple(slice.stop - slice.start, NULL);
            mp_seq_copy(result->items, &(self->x) + slice.start, result->len, mp_obj_t);
            return result;
        }
        switch (mp_get_index(self->base.type, py_nncu_classification_obj_size, index, false)) {
            case 0: return self->x;
            case 1: return self->y;
            case 2: return self->w;
            case 3: return self->h;
            case 4: return self->output;
        }
    }
    return MP_OBJ_NULL; // op not supported
}

mp_obj_t py_nncu_classification_rect(mp_obj_t self_in)
{
    return mp_obj_new_tuple(4, (mp_obj_t []) {((py_nncu_classification_obj_t *) self_in)->x,
                                              ((py_nncu_classification_obj_t *) self_in)->y,
                                              ((py_nncu_classification_obj_t *) self_in)->w,
                                              ((py_nncu_classification_obj_t *) self_in)->h});
}

mp_obj_t py_nncu_classification_x(mp_obj_t self_in) { return ((py_nncu_classification_obj_t *) self_in)->x; }
mp_obj_t py_nncu_classification_y(mp_obj_t self_in) { return ((py_nncu_classification_obj_t *) self_in)->y; }
mp_obj_t py_nncu_classification_w(mp_obj_t self_in) { return ((py_nncu_classification_obj_t *) self_in)->w; }
mp_obj_t py_nncu_classification_h(mp_obj_t self_in) { return ((py_nncu_classification_obj_t *) self_in)->h; }
mp_obj_t py_nncu_classification_output(mp_obj_t self_in) { return ((py_nncu_classification_obj_t *) self_in)->output; }

STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_nncu_classification_rect_obj, py_nncu_classification_rect);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_nncu_classification_x_obj, py_nncu_classification_x);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_nncu_classification_y_obj, py_nncu_classification_y);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_nncu_classification_w_obj, py_nncu_classification_w);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_nncu_classification_h_obj, py_nncu_classification_h);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_nncu_classification_output_obj, py_nncu_classification_output);

STATIC const mp_rom_map_elem_t py_nncu_classification_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_rect), MP_ROM_PTR(&py_nncu_classification_rect_obj) },
    { MP_ROM_QSTR(MP_QSTR_x), MP_ROM_PTR(&py_nncu_classification_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_y), MP_ROM_PTR(&py_nncu_classification_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_w), MP_ROM_PTR(&py_nncu_classification_w_obj) },
    { MP_ROM_QSTR(MP_QSTR_h), MP_ROM_PTR(&py_nncu_classification_h_obj) },
    { MP_ROM_QSTR(MP_QSTR_output), MP_ROM_PTR(&py_nncu_classification_output_obj) }
};

STATIC MP_DEFINE_CONST_DICT(py_nncu_classification_locals_dict, py_nncu_classification_locals_dict_table);

static const mp_obj_type_t py_nncu_classification_type = {
    { &mp_type_type },
    .name  = MP_QSTR_nncu_classification,
    .print = py_nncu_classification_print,
    .subscr = py_nncu_classification_subscr,
    .locals_dict = (mp_obj_t) &py_nncu_classification_locals_dict
};

static const mp_obj_type_t py_nncu_model_type;
extern char Image$$WEIT_CACHE_AREA$$Base[];
#define weit_cache_area Image$$WEIT_CACHE_AREA$$Base
void py_model_helper_args_tuple(py_nncu_model_obj_t *arg_model, mp_obj_t arg){
	mp_obj_tuple_t *dims_tuple = arg;
	if(mp_obj_is_type(dims_tuple, &mp_type_tuple)){
		mp_obj_t *items;
		unsigned int len = 0;
		mp_obj_tuple_get(dims_tuple, &len, &items);
		uint32_t w = 1, h = 1, c = 1;
		switch(len){
			case 3:
				w = mp_obj_get_int(items[len-3]);
			case 2:
				h = mp_obj_get_int(items[len-2]);
			case 1:
				c = mp_obj_get_int(items[len-1]);
				break;
			default:
				nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Tuple length Must as (1, 2, 3)"));
		}
		arg_model->width = w; arg_model->height = h; arg_model->channels = c;
	}else{
		nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "tuple only"));
	}
}	
STATIC mp_obj_t int_py_nncu_load(mp_obj_t path_obj, bool alloc_mode, bool weit_cache, bool load_once)
{
    const char *path = mp_obj_str_get_str(path_obj);
    py_nncu_model_obj_t *nncu_model = m_new_obj(py_nncu_model_obj_t);
    nncu_model->base.type = &py_nncu_model_type;
	
	FIL fp;
	file_read_open(&fp, path);
	nncu_model->model_data_len = f_size(&fp);
	nncu_model->model_data = alloc_mode
		? fb_alloc(nncu_model->model_data_len, FB_ALLOC_PREFER_SIZE)
		: xalloc(nncu_model->model_data_len);
	read_data(&fp, nncu_model->model_data, nncu_model->model_data_len);
	
	file_close(&fp);
	if(weit_cache) CI_SetWeitCache(weit_cache_area, WEIT_CACHE_SIZE);
	
	ModelInfo_V2_t ModelInfo;
	CI_GetModelInfoXIP_V2(nncu_model->model_data, &ModelInfo);

	if(!(ModelInfo.quantBits >= 8))
		nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "only support 8+bits quant"));
	
	CI_MetricInfo *input = &ModelInfo.inMetric;
	CI_MetricInfo *output = &ModelInfo.outMetric;	
	
	nncu_model->height = input->h;
	nncu_model->width = input->w;
	nncu_model->channels = input->c;
	
    // In this mode we leave the model allocated on the frame buffer.
    // py_nncu_free_from_fb() must be called to free the model allocated on the frame buffer.
    // On error everything is cleaned because of fb_alloc_mark().

    if (alloc_mode && load_once) {
        fb_alloc_mark_permanent(); // nncu_model->model_data will not be popped on exception.
    }

    return nncu_model;
}

STATIC mp_obj_t py_nncu_load(uint n_args, const mp_obj_t *args, mp_map_t *kw_args)
{
    return int_py_nncu_load(args[0], py_helper_keyword_int(n_args, args, 1, kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_load_to_fb), false), py_helper_keyword_int(n_args, args, 2, kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_weit_cache), true), true);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(py_nncu_load_obj, 1, py_nncu_load);

STATIC mp_obj_t py_nncu_free_from_fb()
{
    fb_alloc_free_till_mark_past_mark_permanent();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_nncu_free_from_fb_obj, py_nncu_free_from_fb);

STATIC py_nncu_model_obj_t *py_nncu_load_alloc(mp_obj_t path_obj)
{
    if (MP_OBJ_IS_TYPE(path_obj, &py_nncu_model_type)) {
        return (py_nncu_model_obj_t *) path_obj;
    } else {
        return (py_nncu_model_obj_t *) int_py_nncu_load(path_obj, true, true, false);
    }
}

typedef struct py_nncu_input_data_callback_data {
    image_t *img;
    rectangle_t *roi;
	int offset, scale;
} py_nncu_input_data_callback_data_t;



STATIC void py_nncu_input_data_callback(void *callback_data,
                                      void *model_input,
                                      const unsigned int input_height,
                                      const unsigned int input_width,
                                      const unsigned int input_channels)
{
    py_nncu_input_data_callback_data_t *arg = (py_nncu_input_data_callback_data_t *) callback_data;

    float xscale = input_width / ((float) arg->roi->w);
    float yscale = input_height / ((float) arg->roi->h);
    // MAX == KeepAspectRationByExpanding - MIN == KeepAspectRatio
    float scale = IM_MAX(xscale, yscale), scale_inv = 1 / scale;
    float x_offset = ((arg->roi->w * scale) - input_width) / 2;
    float y_offset = ((arg->roi->h * scale) - input_height) / 2;
	
	int8_t shift = -128;

    switch (arg->img->bpp) {
        case IMAGE_BPP_BINARY: {
            for (int y = 0, yy = input_height; y < yy; y++) {
                uint32_t *row_ptr = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(arg->img, fast_floorf((y + y_offset) * scale_inv) + arg->roi->y);
                int row = input_width * y;
                for (int x = 0, xx = input_width; x < xx; x++) {
                    int pixel = IMAGE_GET_BINARY_PIXEL_FAST(row_ptr, fast_floorf((x + x_offset) * scale_inv) + arg->roi->x);
                    int index = row + x;
                    switch (input_channels) {
                        case 1: {
                            ((uint8_t *) model_input)[index] = COLOR_BINARY_TO_GRAYSCALE(pixel) ^ shift;
                            break;
                        }
                        case 3: {
                            int index_3 = index * 3;
                            pixel = COLOR_BINARY_TO_RGB565(pixel);
							((uint8_t *) model_input)[index_3 + 0] = COLOR_RGB565_TO_R8(pixel) ^ shift;
							((uint8_t *) model_input)[index_3 + 1] = COLOR_RGB565_TO_G8(pixel) ^ shift;
							((uint8_t *) model_input)[index_3 + 2] = COLOR_RGB565_TO_B8(pixel) ^ shift;
                            break;
                        }
                        default: {
                            break;
                        }
                    }
                }
            }
            break;
        }
        case IMAGE_BPP_GRAYSCALE: {
            for (int y = 0, yy = input_height; y < yy; y++) {
                uint8_t *row_ptr = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(arg->img, fast_floorf((y + y_offset) * scale_inv) + arg->roi->y);
                int row = input_width * y;
                for (int x = 0, xx = input_width; x < xx; x++) {
                    int pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(row_ptr, fast_floorf((x + x_offset) * scale_inv) + arg->roi->x);
                    int index = row + x;
                    switch (input_channels) {
                        case 1: {
                            ((uint8_t *) model_input)[index] = pixel ^ shift;
                            break;
                        }
                        case 3: {
                            int index_3 = index * 3;
                            pixel = COLOR_GRAYSCALE_TO_RGB565(pixel);
							((uint8_t *) model_input)[index_3 + 0] = COLOR_RGB565_TO_R8(pixel) ^ shift;
							((uint8_t *) model_input)[index_3 + 1] = COLOR_RGB565_TO_G8(pixel) ^ shift;
							((uint8_t *) model_input)[index_3 + 2] = COLOR_RGB565_TO_B8(pixel) ^ shift;
                            break;
                        }
                        default: {
                            break;
                        }
                    }
                }
            }
            break;
        }
        case IMAGE_BPP_RGB565: {
            for (int y = 0, yy = input_height; y < yy; y++) {
                uint16_t *row_ptr = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(arg->img, fast_floorf((y + y_offset) * scale_inv) + arg->roi->y);
                int row = input_width * y;
                for (int x = 0, xx = input_width; x < xx; x++) {
                    int pixel = IMAGE_GET_RGB565_PIXEL_FAST(row_ptr, fast_floorf((x + x_offset) * scale_inv) + arg->roi->x);
                    int index = row + x;
                    switch (input_channels) {
                        case 1: {
							((uint8_t *) model_input)[index] = COLOR_RGB565_TO_GRAYSCALE(pixel) ^ shift;
                            break;
                        }
                        case 3: {
                            int index_3 = index * 3;
							((uint8_t *) model_input)[index_3 + 0] = COLOR_RGB565_TO_R8(pixel) ^ shift;
							((uint8_t *) model_input)[index_3 + 1] = COLOR_RGB565_TO_G8(pixel) ^ shift;
							((uint8_t *) model_input)[index_3 + 2] = COLOR_RGB565_TO_B8(pixel) ^ shift;
                            break;
                        }
                        default: {
                            break;
                        }
                    }
                }
            }
            break;
        }
        default: {
            break;
        }
    }
}

typedef struct py_nncu_classify_output_data_callback_data {
    mp_obj_t out;
} py_nncu_classify_output_data_callback_data_t;

typedef enum {
	QUANT_8 = 0,
	QUANT_16,
} quant_type;

void SUM(uint32_t *src, uint32_t len){
	__asm volatile (
		"loop: \n"
		"	ldrh r1, [r0], #2 \n"
		"	add r2, #1 \n"
		"	"
		"	subs r1, #1 \n"
		"	bne loop \n"
	);
}

STATIC void py_nncu_classify_output_data_callback(void *callback_data,
                                                void *model_output,
                                                const unsigned int output_height,
                                                const unsigned int output_width,
                                                const unsigned int output_channels,
												const unsigned int output_fracs,
												const bool enable_post,
												const quant_type quant_bits)
{
    py_nncu_classify_output_data_callback_data_t *arg = (py_nncu_classify_output_data_callback_data_t *) callback_data;

	int channels = enable_post ? 1 : output_height * output_width * output_channels;
    arg->out = mp_obj_new_list(channels, NULL);
	uint32_t base = 0;
	if(!enable_post){
		for(int i=0;i<channels;i++){
			switch(quant_bits){
				case QUANT_8:
					base += ((int8_t *) model_output)[i];
					break;
				case QUANT_16:
					base += ((int16_t *) model_output)[i];
					break;
				default:
					break;
			}
		}
	}
    for (unsigned int i = 0; i < channels; i++) {
		mp_obj_t value;
		switch(quant_bits){
			case QUANT_8:
				if(enable_post)
					value = mp_obj_new_int((((int8_t *) model_output)[i]));
				else
					value = mp_obj_new_float((((int8_t *) model_output)[i]) * 1.0 / base);
				break;
			case QUANT_16:
				if(enable_post)
					value = mp_obj_new_int((((int16_t *) model_output)[i]));
				else
					value = mp_obj_new_float((((int16_t *) model_output)[i]) * 1.0 / base);			
		}
		((mp_obj_list_t *) arg->out)->items[i] = value;
    }
}


STATIC mp_obj_t py_nncu_classify(uint n_args, const mp_obj_t *args, mp_map_t *kw_args)
{

    fb_alloc_mark();

    py_nncu_model_obj_t *arg_model = py_nncu_load_alloc(args[0]);
    image_t *arg_img = py_helper_arg_to_image_mutable(args[1]);

    rectangle_t roi;
    py_helper_keyword_rectangle_roi(arg_img, n_args, args, 2, kw_args, &roi);

    float arg_min_scale = py_helper_keyword_float(n_args, args, 3, kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_min_scale), 1.0f);
    PY_ASSERT_TRUE_MSG((0.0f < arg_min_scale) && (arg_min_scale <= 1.0f), "0 < min_scale <= 1");

    float arg_scale_mul = py_helper_keyword_float(n_args, args, 4, kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_scale_mul), 0.5f);
    PY_ASSERT_TRUE_MSG((0.0f <= arg_scale_mul) && (arg_scale_mul < 1.0f), "0 <= scale_mul < 1");

    float arg_x_overlap = py_helper_keyword_float(n_args, args, 5, kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_x_overlap), 0.0f);
    PY_ASSERT_TRUE_MSG(((0.0f <= arg_x_overlap) && (arg_x_overlap < 1.0f)) || (arg_x_overlap == -1.0f), "0 <= x_overlap < 1");

    float arg_y_overlap = py_helper_keyword_float(n_args, args, 6, kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_y_overlap), 0.0f);
    PY_ASSERT_TRUE_MSG(((0.0f <= arg_y_overlap) && (arg_y_overlap < 1.0f)) || (arg_y_overlap == -1.0f), "0 <= y_overlap < 1");
	
	// post processing or not
	bool enable_post = py_helper_keyword_int(n_args, args, 7, kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_enable), false);
	CI_ConfigPostProcess(NULL, enable_post);
    
	mp_obj_t objects_list = mp_obj_new_list(0, NULL);
	int tick;
	void* outputData;
	uint32_t in_w = arg_model->width, in_h = arg_model->height, in_c = arg_model->channels;
	void* inputData = fb_alloc(in_w * in_h * in_c, FB_ALLOC_PREFER_SIZE);
	
    for (float scale = 1.0f; scale >= arg_min_scale; scale *= arg_scale_mul) {
        // Either provide a subtle offset to center multiple detection windows or center the only detection window.
        for (int y = roi.y + ((arg_y_overlap != -1.0f) ? (fmodf(roi.h, (roi.h * scale)) / 2.0f) : ((roi.h - (roi.h * scale)) / 2.0f));
            // Finish when the detection window is outside of the ROI.
            (y + (roi.h * scale)) <= (roi.y + roi.h);
            // Step by an overlap amount accounting for scale or just terminate after one iteration.
            y += ((arg_y_overlap != -1.0f) ? (roi.h * scale * (1.0f - arg_y_overlap)) : roi.h)) {
            // Either provide a subtle offset to center multiple detection windows or center the only detection window.
            for (int x = roi.x + ((arg_x_overlap != -1.0f) ? (fmodf(roi.w, (roi.w * scale)) / 2.0f) : ((roi.w - (roi.w * scale)) / 2.0f));
                // Finish when the detection window is outside of the ROI.
                (x + (roi.w * scale)) <= (roi.x + roi.w);
                // Step by an overlap amount accounting for scale or just terminate after one iteration.
                x += ((arg_x_overlap != -1.0f) ? (roi.w * scale * (1.0f - arg_x_overlap)) : roi.w)) {

                rectangle_t new_roi;
                rectangle_init(&new_roi, x, y, roi.w * scale, roi.h * scale);

                if (rectangle_overlap(&roi, &new_roi)) { // Check if new_roi is null...
					
                    py_nncu_input_data_callback_data_t py_nncu_input_data_callback_data;
                    py_nncu_input_data_callback_data.img = arg_img;
                    py_nncu_input_data_callback_data.roi = &new_roi;

                    py_nncu_classify_output_data_callback_data_t py_nncu_classify_output_data_callback_data;
					
					tick = rt_tick_get();											
					py_nncu_input_data_callback(&py_nncu_input_data_callback_data, inputData, in_h, in_w, in_c);
                    CI_RunModelXIP_NoCopyOutput(arg_model->model_data, inputData, &outputData);
					mp_printf(&mp_plat_print, "NNCU Inference during %dms\r\n",rt_tick_get()-tick);
					ModelInfo_V2_t ModelInfo;
					CI_GetModelInfoXIP_V2(arg_model->model_data, &ModelInfo);
					
					const CI_OutMetricInfo* outInfo = &ModelInfo.outMetric;
                    py_nncu_classification_obj_t *o = m_new_obj(py_nncu_classification_obj_t);
                    o->base.type = &py_nncu_classification_type;
                    o->x = mp_obj_new_int(new_roi.x);
                    o->y = mp_obj_new_int(new_roi.y);
                    o->w = mp_obj_new_int(new_roi.w);
                    o->h = mp_obj_new_int(new_roi.h);
					
					quant_type type = ModelInfo.quantBits == 8 ? QUANT_8 : QUANT_16; 
					py_nncu_classify_output_data_callback(&py_nncu_classify_output_data_callback_data, 
														  outputData, outInfo->h, outInfo->w, outInfo->c, outInfo->fracBits, enable_post, type);
                    o->output = py_nncu_classify_output_data_callback_data.out;
                    mp_obj_list_append(objects_list, o);
					
                }
            }
        }
    }

    fb_alloc_free_till_mark();

    return objects_list;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(py_nncu_classify_obj, 2, py_nncu_classify);

mp_obj_t py_nncu_height(mp_obj_t self_in) { return mp_obj_new_int(((py_nncu_model_obj_t *) self_in)->height); }
mp_obj_t py_nncu_width(mp_obj_t self_in) { return mp_obj_new_int(((py_nncu_model_obj_t *) self_in)->width); }
mp_obj_t py_nncu_channels(mp_obj_t self_in) { return mp_obj_new_int(((py_nncu_model_obj_t *) self_in)->channels); }

STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_nncu_height_obj, py_nncu_height);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_nncu_width_obj, py_nncu_width);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_nncu_channels_obj, py_nncu_channels);

STATIC const mp_rom_map_elem_t locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_height), MP_ROM_PTR(&py_nncu_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_width), MP_ROM_PTR(&py_nncu_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_channels), MP_ROM_PTR(&py_nncu_channels_obj) },
    { MP_ROM_QSTR(MP_QSTR_classify), MP_ROM_PTR(&py_nncu_classify_obj) },
};

STATIC MP_DEFINE_CONST_DICT(locals_dict, locals_dict_table);

STATIC const mp_obj_type_t py_nncu_model_type = {
    { &mp_type_type },
    .name  = MP_QSTR_nncu_model,
    .print = py_nncu_model_print,
    .locals_dict = (mp_obj_t) &locals_dict
};


STATIC const mp_rom_map_elem_t globals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_nncu) },
    { MP_ROM_QSTR(MP_QSTR_load),            MP_ROM_PTR(&py_nncu_load_obj) },
    { MP_ROM_QSTR(MP_QSTR_free_from_fb),    MP_ROM_PTR(&py_nncu_free_from_fb_obj) },
    { MP_ROM_QSTR(MP_QSTR_classify),        MP_ROM_PTR(&py_nncu_classify_obj) },
};

STATIC MP_DEFINE_CONST_DICT(globals_dict, globals_dict_table);

const mp_obj_module_t nncu_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_t) &globals_dict
};