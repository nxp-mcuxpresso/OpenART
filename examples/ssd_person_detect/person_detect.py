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
if LVGL_W > 640 and LVGL_H > 480:
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

def flush_capture(image):
    global img

    img_dsc = lv.img_dsc_t(
        {
            "header": {"always_zero": 0, "w": 320, "h": 240, "cf": lv.img.CF.TRUE_COLOR},
            "data_size": 320*240*2,
            "data": lvgl_helper.get_ptr(image),
        }
    )
    img.align(lv.scr_act(), lv.ALIGN.CENTER, 0, 0)
    #img.set_angle(1800) # roate 180 degree to sync with display
    img.set_src(img_dsc)

model = "person_detect.tflite"
net = tf.load(model)

style_line = lv.style_t()
style_line.init()
style_line.set_line_width(lv.STATE.DEFAULT, 8)
style_line.set_line_color(lv.STATE.DEFAULT, lv.color_make(0x00,0x00,0xff))
style_line.set_line_rounded(lv.STATE.DEFAULT, True)
line1 = lv.line(lv.scr_act())

def draw_person_rect(x1,y1,x2,y2):
    global line1
    line_points = [
        {"x":x1,"y":y1},
        {"x":x2,"y":y1},
        {"x":x2,"y":y2},
        {"x":x1,"y":y2},
        {"x":x1,"y":y1},
    ]

    line1.set_points(line_points, len(line_points))      # Set the points
    line1.add_style(lv.line.PART.MAIN, style_line)
    line1.align(None, lv.ALIGN.CENTER, 0, 0)

while(1):
    pic = sensor.snapshot()

    flush_capture(pic)
    count = 0
    objs  = tf.detect(net,pic,bgr=1)
    for obj in objs:
        x1,y1,x2,y2,lab,scores = obj
        if scores > 0.4:
            x1 = int(x1*pic.width() + 0.5)
            y1 = int(y1*pic.height() + 0.5)
            x2 = int(x2*pic.width() + 0.5)
            y2 = int(y2*pic.height() + 0.5)
            draw_person_rect(x1,y1,x2,y2)
            count = count +1
    string = "Found %d person in picture"%count
    label.set_text(string)
    if(count == 0):
        draw_person_rect(0,0,0,0)





