import sensor, image, time, machine, pyb, os, tf,gc
import lvgl as lv
import lvgl_helper
import micropython
import math
import cmath
from FaceUtils import find_faces,load_db,clear_faces,save_face,findFaceinList,get_faces_cnt,ModelCulculateArray

sensor.reset()
sensor.set_auto_gain(True)
sensor.set_auto_exposure(True)
sensor.set_auto_whitebal(True)
sensor.set_brightness(2)
sensor.set_contrast(1)
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.set_windowing((240, 200))       # Set 240x240 window.
sensor.skip_frames(time = 2000)          # Wait for settings take effect.
sensor.set_auto_gain(False)
clock = time.clock()

angle_threadhold = 45
angle_sensitive = 10
#mobile face net functions


#load saved face from sdcard
face_count = load_db()
print("find %d faces saved"%face_count)
#Initialize LVGL
lv.init()
#lvgl task hander called in timer
def timer_callback(self):
    lv.tick_inc(10)
    lv.task_handler()


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

indev_drv = lv.indev_drv_t()
indev_drv.init()
indev_drv.type = lv.INDEV_TYPE.POINTER
indev_drv.read_cb = lvgl_helper.capture
#show cursor of touch panel
tp = indev_drv.register()
cursor=lv.img(lv.scr_act())
cursor.set_src(lv.SYMBOL.CLOSE)
tp.set_cursor(cursor)
##############################
scr = lv.obj()
#add face btn
add_face_enable = False
def add_face_cb(obj = None, event=-1 ):
    global add_face_enable
    if event == lv.EVENT.CLICKED:
        if not add_face_enable:
            add_face_enable = True
            style_screen_add_face_main.set_bg_color(lv.STATE.DEFAULT, lv.color_make(0xff, 0x0, 0x0))
            ui_screen_add_face.add_style(ui_screen_add_face.PART.MAIN, style_screen_add_face_main)

        else:
            add_face_enable = False
            style_screen_add_face_main.set_bg_color(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
            ui_screen_add_face.add_style(ui_screen_add_face.PART.MAIN, style_screen_add_face_main)

        print("add face :%d" % add_face_enable)

def show_Cam_Image(img,w,h):
    img_dsc = lv.img_dsc_t(
        {
            "header": {"always_zero": 0, "w": w, "h": h, "cf": lv.img.CF.TRUE_COLOR},
            "data_size": w*h*2,
            "data": lvgl_helper.get_ptr(img),
        }
    )
    ui_screen_image_camera.set_src(img_dsc)

def clearResultImage_cb(obj):
    ui_screen_image_result.set_src(lv.SYMBOL.DUMMY)
    #print("image clear task")
    camera_label.set_text("Finding Face")

def showResultImage(idx):
    global clear_res_task

    fname = '/sd/mobilefacenet/%d'%idx + '.bmp'
    try:
        img = image.Image(fname)
        img_dsc = lv.img_dsc_t(
            {
                "header": {"always_zero": 0, "w": 64, "h": 64 , "cf": lv.img.CF.TRUE_COLOR},
                "data_size": 64*64*2,
                "data": lvgl_helper.get_ptr(img),
            }
        )
        ui_screen_image_result.set_src(img_dsc)
        #print("show %s"%fname)
    except:
        print("show %s failed"%fname)

    clear_res_task.reset()
    clear_res_task.set_repeat_count(0)
    clear_res_task = lv.task_create_basic()
    clear_res_task.set_cb(clearResultImage_cb)
    clear_res_task.set_repeat_count(1)
    clear_res_task.set_period(2000)

def msg_box_callback(obj,event):
    global add_face_enable
    global snapstop
    global ui_screen_add_face
    global ui_screen_image_result
    global face_image

    if event == lv.EVENT.VALUE_CHANGED:
        print("save faces:%d %s"% (obj.get_active_btn(),obj.get_active_btn_text()))
        obj.start_auto_close(0)
        if obj.get_active_btn() == 0:
            img_dsc = lv.img_dsc_t(
                {
                    "header": {"always_zero": 0, "w": face_image.width(), "h": face_image.height() , "cf": lv.img.CF.TRUE_COLOR},
                    "data_size": 64*64*2,
                    "data": lvgl_helper.get_ptr(face_image),
                }
            )
            add_face_enable = 0
            style_screen_add_face_main.set_bg_color(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
            ui_screen_add_face.add_style(ui_screen_add_face.PART.MAIN, style_screen_add_face_main)
            ui_screen_image_result.set_src(img_dsc)
            arr = ModelCulculateArray(face_image)
            save_face(arr,face_image)

            ui_screen_image_preview.set_src(lv.SYMBOL.DUMMY)
        snapstop = 0

def show_Face_Image(img, w=64, h=64):
    global snapstop
    global face_image
    global Face_DB
    face_count = get_faces_cnt()
    img_dsc = lv.img_dsc_t(
        {
            "header": {"always_zero": 0, "w": w, "h": h , "cf": lv.img.CF.TRUE_COLOR},
            "data_size": w*h*2,
            "data": lvgl_helper.get_ptr(img),
        }
    )
    ui_screen_image_preview.set_src(img_dsc)

    if add_face_enable:
        snapstop = 1
        face_image = img.copy()
        mbox = lv.msgbox(lv.scr_act(), None)
        btn_string = ['Yes','No','']
        mbox.add_btns(btn_string)
        mbox.set_text("Are you sure to add this face ?")
        mbox.set_event_cb(msg_box_callback)
    elif face_count > 0:
        millis = pyb.millis()
        arr = ModelCulculateArray(img)
        idx,angle = findFaceinList(arr,angle_threadhold,angle_sensitive)
        if(idx >= 0 and idx < face_count) :
            showResultImage(idx)
            camera_label.set_text("Found match face angle:%d"%angle)
        else :
            ui_screen_image_result.set_src(lv.SYMBOL.DUMMY)
            camera_label.set_text("Mismatch angle:%d"%angle)
        ui_screen_image_preview.set_src(lv.SYMBOL.DUMMY)

        print("find %d:%d during:%d" % (idx,angle,pyb.millis()-millis))

def reduce_sens_cb(obj,evt):
    global angle_sensitive
    if evt == lv.EVENT.CLICKED:
        if (angle_sensitive > 1):
            angle_sensitive = angle_sensitive -1
            camera_label.set_text("Threshold is %d"%(angle_sensitive+angle_threadhold))

def inc_sens_cb(obj, evt):
    global angle_sensitive
    if evt == lv.EVENT.CLICKED:
        if (angle_sensitive < 20):
            angle_sensitive = angle_sensitive +1
            camera_label.set_text("Threshold is %d"%(angle_sensitive+angle_threadhold))

def reset_sens_cb(obj, evt):
    global angle_sensitive
    if evt == lv.EVENT.CLICKED:
        angle_sensitive = 10
        camera_label.set_text("Threshold is %d"%(angle_sensitive+angle_threadhold))

def clear_msg_box_callback(obj,event):
    global snapstop
    global load_db
    if event == lv.EVENT.VALUE_CHANGED:
        print("save faces:%d %s"% (obj.get_active_btn(),obj.get_active_btn_text()))
        obj.start_auto_close(0)
        if obj.get_active_btn() == 0:
            clear_faces()
            load_db = 0

        snapstop = 0

def clear_face_cb(obj = None, event=-1 ):
    global snapstop
    if event == lv.EVENT.CLICKED:
        snapstop = 1
        mbox = lv.msgbox(lv.scr_act(), None)
        btn_string = ['Yes','No','']
        mbox.add_btns(btn_string)
        mbox.set_text("Are you sure to clear all faces ?")
        mbox.set_event_cb(clear_msg_box_callback)
#draw UI
style_screen_add_face_main = lv.style_t()
style_screen_add_face_main.init()
style_screen_add_face_main.set_radius(lv.STATE.DEFAULT, 50)
style_screen_add_face_main.set_bg_color(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_add_face_main.set_bg_grad_color(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_add_face_main.set_bg_grad_dir(lv.STATE.DEFAULT, lv.GRAD_DIR.VER)
style_screen_add_face_main.set_bg_opa(lv.STATE.DEFAULT, 255)
style_screen_add_face_main.set_border_color(lv.STATE.DEFAULT, lv.color_make(0x01, 0xa2, 0xb1))
style_screen_add_face_main.set_border_width(lv.STATE.DEFAULT, 2)
style_screen_add_face_main.set_border_opa(lv.STATE.DEFAULT, 255)
style_screen_add_face_main.set_outline_color(lv.STATE.DEFAULT, lv.color_make(0xd4, 0xd7, 0xd9))
style_screen_add_face_main.set_outline_opa(lv.STATE.DEFAULT, 255)

camera_label = lv.label(lv.scr_act())
camera_label.align(lv.scr_act(),lv.ALIGN.IN_TOP_MID,0,0)
camera_label.set_text("Not match")

ui_screen_image_camera = lv.img(lv.scr_act(), None)
style_screen_image_camera_main = lv.style_t()
style_screen_image_camera_main.init()
style_screen_image_camera_main.set_image_recolor(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_image_camera_main.set_image_recolor_opa(lv.STATE.DEFAULT, 0)
style_screen_image_camera_main.set_image_opa(lv.STATE.DEFAULT, 255)
ui_screen_image_camera.add_style(ui_screen_image_camera.PART.MAIN, style_screen_image_camera_main)
ui_screen_image_camera.set_pos(120, 40)
ui_screen_image_camera.set_size(240, 200)

ui_screen_add_face = lv.btn(lv.scr_act(), None)
ui_screen_add_face.add_style(ui_screen_add_face.PART.MAIN, style_screen_add_face_main)
ui_screen_add_face.set_pos(16, 28)
ui_screen_add_face.set_size(80, 40)
ui_screen_add_face_label = lv.label(ui_screen_add_face, None)
ui_screen_add_face_label.set_text("Add Face")
ui_screen_add_face_label.set_style_local_text_color(ui_screen_add_face_label.PART.MAIN, lv.STATE.DEFAULT, lv.color_make(0x00, 0x00, 0x00))
ui_screen_add_face.set_event_cb(add_face_cb)

ui_screen_image_result = lv.img(lv.scr_act(), None)
style_screen_image_result_main = lv.style_t()
style_screen_image_result_main.init()
style_screen_image_result_main.set_image_recolor(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_image_result_main.set_image_recolor_opa(lv.STATE.DEFAULT, 0)
style_screen_image_result_main.set_image_opa(lv.STATE.DEFAULT, 255)
ui_screen_image_result.add_style(ui_screen_image_result.PART.MAIN, style_screen_image_result_main)
ui_screen_image_result.set_pos(20, 162)
ui_screen_image_result.set_size(64, 64)
ui_screen_image_result.set_src(lv.SYMBOL.DUMMY)

clear_res_task = lv.task_create_basic()
clear_res_task.set_cb(clearResultImage_cb)
clear_res_task.set_repeat_count(1)
clear_res_task.set_period(2000)

ui_screen_image_preview = lv.img(lv.scr_act(), None)
style_screen_image_preview_main = lv.style_t()
style_screen_image_preview_main.init()
style_screen_image_preview_main.set_image_recolor(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_image_preview_main.set_image_recolor_opa(lv.STATE.DEFAULT, 0)
style_screen_image_preview_main.set_image_opa(lv.STATE.DEFAULT, 255)
ui_screen_image_preview.add_style(ui_screen_image_preview.PART.MAIN, style_screen_image_preview_main)
ui_screen_image_preview.set_pos(296, 40)
ui_screen_image_preview.set_size(64, 64)
ui_screen_image_preview.set_src(lv.SYMBOL.DUMMY)
snapstop = 0
face_image = sensor.snapshot()
ui_screen_sens_inc = lv.btn(lv.scr_act(), None)
style_screen_sens_inc_main = lv.style_t()
style_screen_sens_inc_main.init()
style_screen_sens_inc_main.set_radius(lv.STATE.DEFAULT, 50)
style_screen_sens_inc_main.set_bg_color(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_sens_inc_main.set_bg_grad_color(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_sens_inc_main.set_bg_grad_dir(lv.STATE.DEFAULT, lv.GRAD_DIR.VER)
style_screen_sens_inc_main.set_bg_opa(lv.STATE.DEFAULT, 255)
style_screen_sens_inc_main.set_border_color(lv.STATE.DEFAULT, lv.color_make(0x01, 0xa2, 0xb1))
style_screen_sens_inc_main.set_border_width(lv.STATE.DEFAULT, 2)
style_screen_sens_inc_main.set_border_opa(lv.STATE.DEFAULT, 255)
style_screen_sens_inc_main.set_outline_color(lv.STATE.DEFAULT, lv.color_make(0xd4, 0xd7, 0xd9))
style_screen_sens_inc_main.set_outline_opa(lv.STATE.DEFAULT, 255)
ui_screen_sens_inc.add_style(ui_screen_sens_inc.PART.MAIN, style_screen_sens_inc_main)
ui_screen_sens_inc.set_pos(382, 92)
ui_screen_sens_inc.set_size(80, 40)
ui_screen_sens_inc.set_event_cb(inc_sens_cb)
ui_screen_sens_inc_label = lv.label(ui_screen_sens_inc, None)
ui_screen_sens_inc_label.set_text("Sens +")
ui_screen_sens_inc_label.set_style_local_text_color(ui_screen_sens_inc_label.PART.MAIN, lv.STATE.DEFAULT, lv.color_make(0x00, 0x00, 0x00))
# illegal var: lv_font_simsun_12
#ui_screen_sens_inc_label.set_style_local_text_font(ui_screen_sens_inc_label.PART.MAIN, lv.STATE.DEFAULT, &lv_font_simsun_12)
ui_screen_sens_reduce = lv.btn(lv.scr_act(), None)
style_screen_sens_reduce_main = lv.style_t()
style_screen_sens_reduce_main.init()
style_screen_sens_reduce_main.set_radius(lv.STATE.DEFAULT, 50)
style_screen_sens_reduce_main.set_bg_color(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_sens_reduce_main.set_bg_grad_color(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_sens_reduce_main.set_bg_grad_dir(lv.STATE.DEFAULT, lv.GRAD_DIR.VER)
style_screen_sens_reduce_main.set_bg_opa(lv.STATE.DEFAULT, 255)
style_screen_sens_reduce_main.set_border_color(lv.STATE.DEFAULT, lv.color_make(0x01, 0xa2, 0xb1))
style_screen_sens_reduce_main.set_border_width(lv.STATE.DEFAULT, 2)
style_screen_sens_reduce_main.set_border_opa(lv.STATE.DEFAULT, 255)
style_screen_sens_reduce_main.set_outline_color(lv.STATE.DEFAULT, lv.color_make(0xd4, 0xd7, 0xd9))
style_screen_sens_reduce_main.set_outline_opa(lv.STATE.DEFAULT, 255)
ui_screen_sens_reduce.add_style(ui_screen_sens_reduce.PART.MAIN, style_screen_sens_reduce_main)
ui_screen_sens_reduce.set_pos(383, 162)
ui_screen_sens_reduce.set_size(80, 40)
ui_screen_sens_reduce_label = lv.label(ui_screen_sens_reduce, None)
ui_screen_sens_reduce_label.set_text("Sens -")
ui_screen_sens_reduce_label.set_style_local_text_color(ui_screen_sens_reduce_label.PART.MAIN, lv.STATE.DEFAULT, lv.color_make(0x00, 0x00, 0x00))
ui_screen_sens_reduce.set_event_cb(reduce_sens_cb)
# illegal var: lv_font_simsun_12
#ui_screen_sens_reduce_label.set_style_local_text_font(ui_screen_sens_reduce_label.PART.MAIN, lv.STATE.DEFAULT, &lv_font_simsun_12)
ui_screen_sens_reset = lv.btn(lv.scr_act(), None)
style_screen_sens_reset_main = lv.style_t()
style_screen_sens_reset_main.init()
style_screen_sens_reset_main.set_radius(lv.STATE.DEFAULT, 50)
style_screen_sens_reset_main.set_bg_color(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_sens_reset_main.set_bg_grad_color(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_sens_reset_main.set_bg_grad_dir(lv.STATE.DEFAULT, lv.GRAD_DIR.VER)
style_screen_sens_reset_main.set_bg_opa(lv.STATE.DEFAULT, 255)
style_screen_sens_reset_main.set_border_color(lv.STATE.DEFAULT, lv.color_make(0x01, 0xa2, 0xb1))
style_screen_sens_reset_main.set_border_width(lv.STATE.DEFAULT, 2)
style_screen_sens_reset_main.set_border_opa(lv.STATE.DEFAULT, 255)
style_screen_sens_reset_main.set_outline_color(lv.STATE.DEFAULT, lv.color_make(0xd4, 0xd7, 0xd9))
style_screen_sens_reset_main.set_outline_opa(lv.STATE.DEFAULT, 255)
ui_screen_sens_reset.add_style(ui_screen_sens_reset.PART.MAIN, style_screen_sens_reset_main)
ui_screen_sens_reset.set_pos(382, 28)
ui_screen_sens_reset.set_size(80, 40)
ui_screen_sens_reset.set_event_cb(reset_sens_cb)
ui_screen_sens_reset_label = lv.label(ui_screen_sens_reset, None)
ui_screen_sens_reset_label.set_text("Sens reset")
ui_screen_sens_reset_label.set_style_local_text_color(ui_screen_sens_reset_label.PART.MAIN, lv.STATE.DEFAULT, lv.color_make(0x00, 0x00, 0x00))
# illegal var: lv_font_simsun_12
#ui_screen_sens_reset_label.set_style_local_text_font(ui_screen_sens_reset_label.PART.MAIN, lv.STATE.DEFAULT, &lv_font_simsun_12)
ui_screen_del_face = lv.btn(lv.scr_act(), None)
style_screen_del_face_main = lv.style_t()
style_screen_del_face_main.init()
style_screen_del_face_main.set_radius(lv.STATE.DEFAULT, 50)
style_screen_del_face_main.set_bg_color(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_del_face_main.set_bg_grad_color(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_del_face_main.set_bg_grad_dir(lv.STATE.DEFAULT, lv.GRAD_DIR.VER)
style_screen_del_face_main.set_bg_opa(lv.STATE.DEFAULT, 255)
style_screen_del_face_main.set_border_color(lv.STATE.DEFAULT, lv.color_make(0x01, 0xa2, 0xb1))
style_screen_del_face_main.set_border_width(lv.STATE.DEFAULT, 2)
style_screen_del_face_main.set_border_opa(lv.STATE.DEFAULT, 255)
style_screen_del_face_main.set_outline_color(lv.STATE.DEFAULT, lv.color_make(0xd4, 0xd7, 0xd9))
style_screen_del_face_main.set_outline_opa(lv.STATE.DEFAULT, 255)

ui_screen_del_face.add_style(ui_screen_del_face.PART.MAIN, style_screen_del_face_main)
ui_screen_del_face.set_pos(16, 92)
ui_screen_del_face.set_size(80, 40)
ui_screen_del_face_label = lv.label(ui_screen_del_face, None)
ui_screen_del_face_label.set_text("Clear Faces")
ui_screen_del_face_label.set_style_local_text_color(ui_screen_del_face_label.PART.MAIN, lv.STATE.DEFAULT, lv.color_make(0x00, 0x00, 0x00))
ui_screen_del_face.set_event_cb(clear_face_cb)
# illegal var: lv_font_simsun_12
#ui_screen_del_face_label.set_style_local_text_font(ui_screen_del_face_label.PART.MAIN, lv.STATE.DEFAULT, &lv_font_simsun_12)
lv.scr_load(lv.scr_act())

while(1):
    if snapstop:
        pyb.mdelay(10)
        continue
    else :
        img = sensor.snapshot()
        show_Cam_Image(img,img.width(),img.height())
        # Find objects.
        # Note: Lower scale factor scales-down the image more and detects smaller objects.
        # Higher threshold results in a higher detection rate, with more false positives.
        objects = find_faces(img)
        for r in objects:
            img_face = img.copy(r)
            print("find face:%d:%d"%(img_face.width(),img_face.height()))
            img_face = img_face.resize(64,64)
            show_Face_Image(img_face,img_face.width(),img_face.height())
            gc.collect()
    pyb.mdelay(2)





