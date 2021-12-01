# 0. need to init the drv_dev & inv_dev & disp buf
# 1. must use lv.img_dsc_t({"header:"...}) to init a var:xx as the parameter of the img.set_src(xx)
# 2. must use lv.font_t({"font":xx}) to init a var:xx, if you want to use set_style_local_text_font()
# 3. can not support move animation now
# 4. can not support cb now
# 5. lvgl update: LV_ROLLER_MODE_INIFINITE -> LV_ROLLER_MODE_INFINITE
# 6. if you want to use text area, create by yourself. the text inside guider including the keyboard
# 7. define the lv.calendar_date_t({"year":x,"month":x,"day":x}) by yourself if you use calendar, showed_data: cur visible month in calendat
import sensor, image, time, machine, pyb, os, tf,gc
import lvgl as lv
import lvgl_helper
import micropython
import math
import cmath


sensor.reset()
sensor.set_auto_gain(True)
sensor.set_auto_exposure(True)
sensor.set_auto_whitebal(True)
sensor.set_brightness(2)
sensor.set_contrast(1)
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.HQVGA)   # Set frame size to QVGA (320x240)
#sensor.set_windowing((320, 240))       # Set 240x240 window.
sensor.set_framerate(0<<9 | 1<<11)
sensor.skip_frames(time = 2000)          # Wait for settings take effect.
sensor.set_auto_gain(False)
clock = time.clock()

mobilenet = "mfn_drop_320_best_quant.tflite"
net = tf.load(mobilenet)
angle_threadhold = 45
angle_sensitive = 10
#mobile face net functions
def ModelCulculateArray(img):
    global net

    array = []
    for obj in tf.classify(net, img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
        array = obj.output()
    gc.collect()
    return array
'''
calculating float is too slow in python , use c fucntion instead
calculate during almost 1000 ms
def Calc_Angle(arrin,arrout):
    iM1 = 0
    iM2 = 0
    iDot = 0
    fM1 = 0.0
    fM2 = 0.0
    fDot = 0.0
    during = pyb.millis()
    i=0
    while (i <128):
        int_in = int(arrin[i]*128)
        int_out = int(arrout[i]*128)
        iDot = iDot + int_in*int_out
        iM1 = iM1 + int_in*int_in
        iM2 = iM2 + int_out*int_out
        i = i +1
    print("float calc during:%d"%(pyb.millis()-during))
    fDot = float(iDot)
    fM1 = math.sqrt(iM1)
    fM2 = math.sqrt(iM2)
    cosval = fDot / (fM1*fM2)
    angle = math.acos(cosval) * 180 / 3.141592654
    print("Angle:%f duing:%d"%(angle,pyb.millis()-during))
    return angle
'''
def findFaceinList(arrlist, arr):
    global angle_threadhold
    global angle_sensitive

    min = 999.0
    i = 0
    index = -1
    for arrs in arrlist:
        min_temp = cmath.calc_angle_float(arr,arrs)#defined in cmath module
        if (min >= min_temp and min_temp <= angle_threadhold + angle_sensitive):
            min = min_temp
            index = i

        elif min >= min_temp:
            min = min_temp
        i = i + 1

    return index,min


#Initializa Face database
def loadFaceVectors():
    count = 0
    list1 = []
    try:
        fd = open('/sd/mobilefacenet/vectors.txt')
        for line in fd:
            array1 = line.strip('[]\n').split(',')
            try:
                arr = list(map(float,array1))
                if len(arr) == 128:
                    list1.append(arr)
                    count = count +1
            except:
                print("invalid face vector")
                count = 0
                list1 = []

        fd.close()

    except:
        print("no face found")

    return (list1,count)

def saveVectors(arrs):
    string1 = ''
    for arr in arrs:
        string1 = string1 + str(arr) + '\n'
    fd = open('/sd/mobilefacenet/vectors.txt','w')

    count = fd.write(string1)
    fd.close()
    print("write:%d"%count)

def clearFaces():
    global Face_count
    global Face_DB
    Face_count = 0

    for obj in Face_DB:
        Face_DB.remove(obj)

    try:
        fd = open('/sd/mobilefacenet/vectors.txt','w')
        c= fd.write('\n')
        fd.close()
        print("clear vectors%d"%c)
    except:
        print("no vectors")

def saveFace(img):
    global Face_count
    global Face_DB

    fname = '/sd/mobilefacenet/%d'%Face_count+'.bmp'
    img.save(fname)
    Face_count = Face_count +1
    newFaceArr = ModelCulculateArray(img)
    Face_DB.append(newFaceArr)
    saveVectors(Face_DB)

#load saved face from sdcard
Face_DB = []
Face_DB, Face_count = loadFaceVectors()
print("find %d faces saved"%Face_count)
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
            obj.set_state(obj.STATE.PRESSED)
        else:
            add_face_enable = False
            obj.set_state(obj.STATE.RELEASED)

        print("add face :%d" % add_face_enable)

ui_screen_add_face = lv.btn(lv.scr_act(), None)
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
ui_screen_add_face.add_style(ui_screen_add_face.PART.MAIN, style_screen_add_face_main)
ui_screen_add_face.set_pos(16, 28)
ui_screen_add_face.set_size(80, 40)
ui_screen_add_face_label = lv.label(ui_screen_add_face, None)
ui_screen_add_face_label.set_text("Add Face")
ui_screen_add_face_label.set_style_local_text_color(ui_screen_add_face_label.PART.MAIN, lv.STATE.DEFAULT, lv.color_make(0x00, 0x00, 0x00))
ui_screen_add_face.set_event_cb(add_face_cb)

ui_screen_image_camera = lv.img(lv.scr_act(), None)
style_screen_image_camera_main = lv.style_t()
style_screen_image_camera_main.init()
style_screen_image_camera_main.set_image_recolor(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_image_camera_main.set_image_recolor_opa(lv.STATE.DEFAULT, 0)
style_screen_image_camera_main.set_image_opa(lv.STATE.DEFAULT, 255)
ui_screen_image_camera.add_style(ui_screen_image_camera.PART.MAIN, style_screen_image_camera_main)
ui_screen_image_camera.set_pos(111, 45)
ui_screen_image_camera.set_size(256, 192)
camera_label = lv.label(lv.scr_act())
camera_label.align(lv.scr_act(),lv.ALIGN.IN_BOTTOM_MID,0,0)
camera_label.set_text("Not match")

def show_Cam_Image(img,w,h):
    img_dsc = lv.img_dsc_t(
        {
            "header": {"always_zero": 0, "w": w, "h": h, "cf": lv.img.CF.TRUE_COLOR},
            "data_size": w*h*2,
            "data": lvgl_helper.get_ptr(img),
        }
    )
    ui_screen_image_camera.set_src(img_dsc)

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
def clearResultImage(obj):
    ui_screen_image_result.set_src(lv.SYMBOL.DUMMY)
    print("image clear task")
    camera_label.set_text("Finding Face")

clear_res_task = lv.task_create_basic()
clear_res_task.set_cb(clearResultImage)
clear_res_task.set_repeat_count(1)
clear_res_task.set_period(2000)

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
        print("show %s"%fname)
    except:
        print("show %s failed"%fname)

        clear_res_task.reset()
        clear_res_task.set_repeat_count(0)
        clear_res_task = lv.task_create_basic()
        clear_res_task.set_cb(clearResultImage)
        clear_res_task.set_repeat_count(1)
        clear_res_task.set_period(2000)




ui_screen_image_preview = lv.img(lv.scr_act(), None)
style_screen_image_preview_main = lv.style_t()
style_screen_image_preview_main.init()
style_screen_image_preview_main.set_image_recolor(lv.STATE.DEFAULT, lv.color_make(0xff, 0xff, 0xff))
style_screen_image_preview_main.set_image_recolor_opa(lv.STATE.DEFAULT, 0)
style_screen_image_preview_main.set_image_opa(lv.STATE.DEFAULT, 255)
ui_screen_image_preview.add_style(ui_screen_image_preview.PART.MAIN, style_screen_image_preview_main)
ui_screen_image_preview.set_pos(308, 45)
ui_screen_image_preview.set_size(64, 64)
ui_screen_image_preview.set_src(lv.SYMBOL.DUMMY)
snapstop = 0
face_image = sensor.snapshot()
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
                    "header": {"always_zero": 0, "w": 64, "h": 64 , "cf": lv.img.CF.TRUE_COLOR},
                    "data_size": 64*64*2,
                    "data": lvgl_helper.get_ptr(face_image),
                }
            )
            add_face_enable = 0
            ui_screen_add_face.set_state(ui_screen_add_face.STATE.RELEASED)
            ui_screen_image_result.set_src(img_dsc)
            saveFace(face_image)
            ui_screen_image_preview.set_src(lv.SYMBOL.DUMMY)
        snapstop = 0

def show_Face_Image(img, w=64, h=64):
    global snapstop
    global face_image
    global Face_DB
    global Face_count
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
    elif Face_count > 0:
        millis = pyb.millis()
        arr = ModelCulculateArray(img)
        idx,angle = findFaceinList(Face_DB,arr)
        if(idx >= 0):
            showResultImage(idx)
            camera_label.set_text("Found match face angle:%d"%angle)
        else :
            ui_screen_image_result.set_src(lv.SYMBOL.DUMMY)
            camera_label.set_text("Mismatch angle:%d"%angle)
        ui_screen_image_preview.set_src(lv.SYMBOL.DUMMY)

        print("find %d:%d during:%d" % (idx,angle,pyb.millis()-millis))


sens_label = lv.label(lv.scr_act())
sens_label.align(lv.scr_act(),lv.ALIGN.IN_TOP_MID,0,0)
sens_label.set_text("")
def reduce_sens_cb(obj,evt):
    global angle_sensitive
    if evt == lv.EVENT.CLICKED:
        if (angle_sensitive > 1):
            angle_sensitive = angle_sensitive -1
            sens_label.set_text("sensitivity is %d"%angle_sensitive)


def inc_sens_cb(obj, evt):
    global angle_sensitive
    if evt == lv.EVENT.CLICKED:
        if (angle_sensitive < 20):
            angle_sensitive = angle_sensitive +1
            sens_label.set_text("sensitivity is %d"%angle_sensitive)

def reset_sens_cb(obj, evt):
    global angle_sensitive
    if evt == lv.EVENT.CLICKED:
        angle_sensitive = 10
        sens_label.set_text("sensitivity is %d"%angle_sensitive)

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


def clear_msg_box_callback(obj,event):
    global snapstop

    if event == lv.EVENT.VALUE_CHANGED:
        print("save faces:%d %s"% (obj.get_active_btn(),obj.get_active_btn_text()))
        obj.start_auto_close(0)
        if obj.get_active_btn() == 0:
            clearFaces()

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


# Load Haar Cascade
# By default this will use all stages, lower satges is faster but less accurate.
face_cascade = image.HaarCascade("frontalface", stages=25)

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
        objects = img.find_features(face_cascade, threshold=0.90, scale_factor=1.55)
        for r in objects:
            if(r[2] >= 80 and r[3] >= 80):
                img_face = img.copy(r)
                result_img = img_face.resize(64,64)
                print("found face:%d,%d" % (img_face.width(),img_face.height()))
                show_Face_Image(result_img,result_img.width(),result_img.height())
    pyb.mdelay(5)





