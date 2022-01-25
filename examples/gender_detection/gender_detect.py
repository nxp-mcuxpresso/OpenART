#
# Copyright (c) 2006-2018, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
# gender detection with lvgl example
#

#import modules
import sensor, image, time, machine, pyb,tf,gc
import lvgl as lv
import lvgl_helper
from imagetools import get_png_info, open_png
#Initialize camera sensor
sensor.reset()
sensor.set_auto_gain(True)
sensor.set_auto_exposure(True)
sensor.set_auto_whitebal(True)
sensor.set_brightness(2)
sensor.set_contrast(1)
sensor.set_gainceiling(8)
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)          # Wait for settings take effect.
sensor.set_auto_gain(False)
clock = time.clock()                # Create a clock object to track the FPS.
#Initialize LVGL
lv.init()
#lvgl task hander called in timer
def timer_callback(self):
    lv.tick_inc(10)
    lv.task_handler()
    pyb.mdelay(5)

timer = machine.Timer(1)
timer.init(50)
timer.callback(timer_callback)
#initialize display interface
lv_disp_buf = lv.disp_buf_t()
LVGL_W,LVGL_H = lvgl_helper.get_display_size()
if LVGL_W > 640 and LGVL_H > 480:
    LVGL_W = 640 #reduce the display res size
    LVGL_H = 480
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

indev_drv = lv.indev_drv_t()
indev_drv.init()
indev_drv.type = lv.INDEV_TYPE.POINTER
indev_drv.read_cb = lvgl_helper.capture

# declare the screen to show
scr = lv.obj()
#declare image to show picture from camera
img = lv.img(lv.scr_act())
img.align(lv.scr_act(), lv.ALIGN.CENTER, 0, 0)
#declare a label to show the result
label = lv.label(lv.scr_act())
label.set_text("")
label.set_align(lv.ALIGN.CENTER)
style = lv.style_t()


ui_screen_image_camera = lv.img(lv.scr_act(), None)
style_screen_image_camera_main = lv.style_t()
style_screen_image_camera_main.init()
style_screen_image_camera_main.set_image_recolor(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_image_camera_main.set_image_recolor_opa(lv.STATE.DEFAULT, 0)
style_screen_image_camera_main.set_image_opa(lv.STATE.DEFAULT, 255)
ui_screen_image_camera.add_style(ui_screen_image_camera.PART.MAIN, style_screen_image_camera_main)
ui_screen_image_camera.set_pos(80, 15)
ui_screen_image_camera.set_size(256, 192)

# Register PNG image decoder
decoder = lv.img.decoder_create()
decoder.info_cb = get_png_info
decoder.open_cb = open_png
#declare png buttons
with open('/sd/detection.png','rb') as f:
  img_data_sexdetect_None = f.read()
  f.close()

img_dsc_sexdetect_None = lv.img_dsc_t({
    'data_size': len(img_data_sexdetect_None),
    'data': img_data_sexdetect_None
})

with open('/sd/btn_female.png','rb') as f:
  img_data_female = f.read()
  f.close()

img_dsc_female = lv.img_dsc_t({
    "header": {"always_zero": 0, "w": 64, "h": 64, "cf": lv.img.CF.TRUE_COLOR},
    'data_size': len(img_data_female),
    'data': img_data_female
})

with open('/sd/btn_male.png','rb') as f:
  img_data_male = f.read()
  f.close()

img_dsc_male = lv.img_dsc_t({
    "header": {"always_zero": 0, "w": 64, "h": 64, "cf": lv.img.CF.TRUE_COLOR},
    'data_size': len(img_data_male),
    'data': img_data_male
})

with open('/sd/btn_sexsel.png','rb') as f:
  img_data_sexdetect = f.read()
  f.close()

img_dsc_sexdetect = lv.img_dsc_t({
    "header": {"always_zero": 0, "w": 64, "h": 64, "cf": lv.img.CF.TRUE_COLOR},
    'data_size': len(img_data_sexdetect),
    'data': img_data_sexdetect
})
img_sexdetect = lv.img(lv.scr_act())
lv.img.cache_set_size(2)
img_sexdetect.set_src(img_dsc_sexdetect)
img_sexdetect.set_pos(0, 50)

def show_camera_image(image,w,h):
    img_dsc = lv.img_dsc_t(
        {
            "header": {"always_zero": 0, "w": w, "h": h, "cf": lv.img.CF.TRUE_COLOR},
            "data_size": w*h*2,
            "data": lvgl_helper.get_ptr(image),
        }
    )
    ui_screen_image_camera.set_src(img_dsc)

#declare camera image
ui_screen_image_preview = lv.img(lv.scr_act(), None)
style_screen_image_preview_main = lv.style_t()
style_screen_image_preview_main.init()
style_screen_image_preview_main.set_image_recolor(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_image_preview_main.set_image_recolor_opa(lv.STATE.DEFAULT, 0)
style_screen_image_preview_main.set_image_opa(lv.STATE.DEFAULT, 255)
ui_screen_image_preview.add_style(ui_screen_image_preview.PART.MAIN, style_screen_image_preview_main)
ui_screen_image_preview.set_pos(80, 45)
ui_screen_image_preview.set_size(64, 64)

def send2flushoutput_male(w=64, h=64):
    global img_dsc_male
    global img_sexdetect

    img_sexdetect.set_src(img_dsc_male)


def send2flushoutput_sexdetect(w=64, h=64):
    global img_dsc_sexdetect
    global img_sexdetect

    img_sexdetect.set_src(img_dsc_sexdetect)


def send2flushoutput_female(w=64, h=64):
    global img_dsc_female
    global img_sexdetect

    img_sexdetect.set_src(img_dsc_female)

def send2flushpreview(image, w=64, h=64):

    img_dsc = lv.img_dsc_t(
        {
            "header": {"always_zero": 0, "w": w, "h": h , "cf": lv.img.CF.TRUE_COLOR},
            "data_size": w*h*2,
            "data": lvgl_helper.get_ptr(image),
        }
    )
    ui_screen_image_preview.set_src(img_dsc)


# Load Haar Cascade
# By default this will use all stages, lower satges is faster but less accurate.
face_cascade = image.HaarCascade("frontalface", stages=25)
#initialize gender detection model
mobilenet = "96_sex_cnn_quant.tflite"
net = tf.load(mobilenet)
clock = time.clock()
list_sex=["female","male"]

icon_to = 0
camera_fps = 0
preview_icon_reflesh = 0
detect_icon_relfesh = 0
while(1):
    icon_to = icon_to + 1
    image = sensor.snapshot()
    clock.tick()
    show_camera_image(image,image.width(),image.height())
    objects = image.find_features(face_cascade, threshold=0.90, scale_factor=1.35)
    #find faces
    if(not len(objects)):
        if(icon_to > 20 and not detect_icon_relfesh):
            img_sexdetect.set_src(img_dsc_sexdetect)
            detect_icon_relfesh = 1
        if (not preview_icon_reflesh):
            ui_screen_image_preview.set_src(img_dsc_sexdetect_None)
            preview_icon_reflesh = 1
    for r in objects:
        print("found %d:%d"%(r[2],r[3]))
        detect_icon_relfesh = 0
        preview_icon_reflesh =0
        if(r[2] >= 80 and r[3] >= 80):
            img = image.copy(r)
            result_img = img.resize(96,96)
            print("found face:%d,%d" % (img.width(),img.height()))
            icon_to = 0
            send2flushpreview(result_img,result_img.width(),result_img.height())
            #model classify the faces
            for obj in tf.classify(net, result_img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
                print(obj.output())
                list = obj.output()

                if (float(list[0]))>0.5625:
                    model_labels = list_sex[0]
                    send2flushoutput_female(w=64, h=64)
                if (float(list[0]))<0.4375:
                    model_labels = list_sex[1]
                    send2flushoutput_male(w=64, h=64)
                string = ("%s fps: %d" % (model_labels,camera_fps))
                label.set_text(string)
                gc.collect()
    camera_fps = clock.fps()
    print(camera_fps)

lvgl_helper.free()
scr.delete()
lv.deinit()
timer.deinit()



