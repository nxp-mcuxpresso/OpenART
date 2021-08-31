#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "fsl_lpi2c.h"

#include "rtthread.h"
#include "touch.h"
#include "lvgl.h"
rt_device_t tp_dev = NULL;
/* Will be called by the library to read the touchpad */
bool DEMO_ReadTouch(lv_indev_drv_t *drv, lv_indev_data_t *data, uint32_t lvgl_w, uint32_t lvgl_h)
{
    static int tp_pos[2];
	int buffer[2];
	if (tp_dev == NULL)
	{
		tp_dev = rt_device_find("touch");
		if (tp_dev == NULL)
			return false;
		
		rt_device_control(tp_dev, RT_TOUCH_CTRL_SET_X_RANGE, &lvgl_w);
		rt_device_control(tp_dev, RT_TOUCH_CTRL_SET_Y_RANGE, &lvgl_h);
	}
	
	if (rt_device_read(tp_dev,0,tp_pos, sizeof(tp_pos)) >0)
    {
        data->state = LV_INDEV_STATE_PR;
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
	
	data->point.x = tp_pos[0];
	data->point.y = tp_pos[1];
	

    /*Return `false` because we are not buffering and no more data to read*/
    return false;
}


