# cifar10 with lvgl example
#

import sensor, image, time, machine, pyb, os, tf,gc
import lvgl as lv
import lvgl_helper
import micropython

sensor.open()
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)


sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

#Initialize LVGL
lv.init()
#lvgl task hander called in timer
def timer_callback(self):
    lv.tick_inc(10)
    lv.task_handler()
    #pyb.mdelay(5)

timer = machine.Timer(1)
timer.init(50)
timer.callback(timer_callback)
#initialize display interface
lv_disp_buf = lv.disp_buf_t()
LVGL_W,LVGL_H = lvgl_helper.get_display_size()

buf1_1 = lvgl_helper.alloc(LVGL_W, LVGL_H)
buf1_2 = lvgl_helper.alloc(LVGL_W, LVGL_H)
lv_disp_buf.init(buf1_1, buf1_2, LVGL_W*LVGL_H)
disp_drv = lv.disp_drv_t()
disp_drv.init()
disp_drv.buffer = lv_disp_buf
disp_drv.flush_cb = lvgl_helper.flush

disp_drv.hor_res = LVGL_W
disp_drv.ver_res = LVGL_H
disp_drv.register()

# declare the screen to show
scr = lv.obj() 
#declare image to show picture from camera
img = lv.img(lv.scr_act())
img.align(lv.scr_act(), lv.ALIGN.CENTER, 0, 0)
#declare a label to show the result
label = lv.label(lv.scr_act())
label.set_text("Type")
label.set_align(lv.ALIGN.CENTER)
style = lv.style_t()


def send2flush(image):
    w = 100
    h = 80
    image1 = image.resize(w,h)
    img_dsc = lv.img_dsc_t(
        {
            "header": {"always_zero": 0, "w": w, "h": h, "cf": lv.img.CF.TRUE_COLOR},
            "data_size": w*h*2,
            "data": lvgl_helper.get_ptr(image1),
        }
    )
    img.align(lv.scr_act(), lv.ALIGN.CENTER, 0, 0)
    
    img.set_src(img_dsc)
    

#initialize cifar10 model
mobilenet = "cifar10_quant.tflite"
model_labels = [line.rstrip('\n') for line in open("/sd/labels.txt")]
net = tf.load(mobilenet)
clock = time.clock()

while(1):
    image = sensor.snapshot()
    clock.tick()
    send2flush(image)
    for obj in tf.classify(net, image, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
        #print("**********\nTop 5 Detections at [x=%d,y=%d,w=%d,h=%d]" % obj.rect())
        
        # This combines the labels and confidence values into a list of tuples
        # and then sorts that list by the confidence values.
        sorted_list = sorted(zip(model_labels, obj.output()), key = lambda x: x[1], reverse = True)
        top_score = 0
        top_idx = 0
        for i in range(5):
            if sorted_list[i][1] > top_score :
                top_score = sorted_list[i][1]
                top_idx = i
        string = sorted_list[top_idx][0].replace('\n', '').replace('\r', '') + " = %d"%(sorted_list[top_idx][1]*100) + "%"
        string += " fps : %d" % clock.fps()
        label.set_text(string)    
        gc.collect()



lvgl_helper.free()
scr.delete()
lv.deinit()
timer.deinit()



