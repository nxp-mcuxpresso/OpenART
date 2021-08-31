import time, machine, pyb, os, gc
import lvgl as lv
import lvgl_helper

#Initialize LVGL
lv.init()
#lvgl task hander called in timer
def timer_callback(self):
    lv.tick_inc(20)
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
indev_drv = lv.indev_drv_t()
indev_drv.init()
indev_drv.type = lv.INDEV_TYPE.POINTER
indev_drv.read_cb = lvgl_helper.capture
#show cursor of touch panel
tp = indev_drv.register()
cursor=lv.img(lv.scr_act())
cursor.set_src(lv.SYMBOL.GPS)
tp.set_cursor(cursor)
# declare the screen to show
scr = lv.obj()

def LV_DPX(n):
    if n == 0:
        n = 0
    else:
        dpi = lv.disp_t.get_dpi(None)
        if int((dpi*n+80)/160) > 1 :
            n = int((dpi*n+80)/160)
        else:
            n = 1
    return n
def LV_MATH_MAX(m,n):
    if m>n:
        return m
    else :
        return n
def LV_MATH_MIN(m,n):
    if m>n:
        return n
    else :
        return m

LV_DPI = 100

theme_flag = lv.THEME_MATERIAL_FLAG.LIGHT
def color_chg_event_cb(obj,evt):
    global theme_flag
    if evt == lv.EVENT.VALUE_CHANGED:
        if (theme_flag == lv.THEME_MATERIAL_FLAG.LIGHT):
            theme_flag = lv.THEME_MATERIAL_FLAG.DARK
        else :
            theme_flag = lv.THEME_MATERIAL_FLAG.LIGHT
        lv.theme_material_init(lv.theme_get_color_primary(), lv.theme_get_color_secondary(),theme_flag,
        lv.theme_get_font_small(),lv.theme_get_font_normal(), lv.theme_get_font_subtitle(), lv.theme_get_font_title())

tv = lv.tabview(lv.scr_act())
tv.set_style_local_pad_left(tv.PART.TAB_BG, lv.STATE.DEFAULT, int(LVGL_W/2))
sw = lv.switch(lv.scr_act())
sw.set_pos(10,10)
sw.set_style_local_value_str(sw.PART.BG, lv.STATE.DEFAULT, "Dark")
sw.set_style_local_value_align(sw.PART.BG, lv.STATE.DEFAULT, lv.ALIGN.OUT_RIGHT_MID)
sw.set_style_local_value_ofs_x(sw.PART.BG, lv.STATE.DEFAULT, int(100/35));
sw.set_event_cb(color_chg_event_cb)

t1 = tv.add_tab("Controls")
t2 = tv.add_tab("Visuals")
t3 = tv.add_tab("Selectors")
#Create Controls tab
lv.page.set_scrl_layout(t1,lv.LAYOUT.PRETTY_TOP)
dis_size = lv.disp_t.get_size_category(None)
if (dis_size <= lv.DISP_SIZE.SMALL):
    div = 1
else:
    div = 2
grid_w = lv.page.get_width_grid(t1,div,1)
h = lv.cont(t1)
h.set_layout(lv.LAYOUT.PRETTY_MID)
h_main = lv.style_t()
h_main.init()
h.add_style(h.PART.MAIN,h_main)
h.set_drag_parent(True)
h.set_style_local_value_str(h.PART.MAIN, lv.STATE.DEFAULT, "Basics")
h.set_style_local_value_align(h.PART.MAIN, lv.STATE.DEFAULT, lv.ALIGN.IN_TOP_LEFT)
h.set_fit2(lv.FIT.NONE,lv.FIT.TIGHT)
h.set_width(grid_w)
btn = lv.btn(h)
btn.set_fit2(lv.FIT.NONE,lv.FIT.TIGHT)
btn.set_width(h.get_width_grid(div,1))
label=lv.label(btn)
label.set_text("Button")
lv.switch(h)
lv.checkbox(h)
fit_w = h.get_width_fit()

def slider_event_cb(obj,evt):
    if evt == lv.EVENT.VALUE_CHANGED:
        value = obj.get_value()
        obj.set_style_local_value_str(obj.PART.KNOB,lv.STATE.DEFAULT,str(value))

slider = lv.slider(h)
slider.set_value(40,lv.ANIM.OFF)
slider.set_event_cb(slider_event_cb)
slider.set_width_margin(fit_w)
slider.set_style_local_margin_top(slider.PART.BG,lv.STATE.DEFAULT,LV_DPX(25))
slider.set_style_local_value_font(slider.PART.KNOB,lv.STATE.DEFAULT,lv.theme_get_font_small())
slider.set_style_local_value_ofs_y(slider.PART.KNOB, lv.STATE.FOCUSED,- LV_DPX(25))
slider.set_style_local_value_opa(slider.PART.KNOB, lv.STATE.DEFAULT, lv.OPA.TRANSP)
slider.set_style_local_value_opa(slider.PART.KNOB, lv.STATE.FOCUSED, lv.OPA.COVER)
slider.set_style_local_transition_time(slider.PART.KNOB, lv.STATE.DEFAULT, 300)
slider.set_style_local_transition_prop_5(slider.PART.KNOB, lv.STATE.DEFAULT, lv.STYLE.VALUE_OFS_Y)
slider.set_style_local_transition_prop_6(slider.PART.KNOB, lv.STATE.DEFAULT, lv.STYLE.VALUE_OPA)

slider = lv.slider(h,slider)
slider.set_type(slider.TYPE.RANGE)
slider.set_value(70,lv.ANIM.OFF)
slider.set_left_value(30,lv.ANIM.OFF)
slider.set_style_local_value_ofs_y(slider.PART.INDIC, lv.STATE.DEFAULT, - LV_DPX(25))
slider.set_style_local_value_font(slider.PART.INDIC, lv.STATE.DEFAULT, lv.theme_get_font_small())
slider.set_style_local_value_opa(slider.PART.INDIC, lv.STATE.DEFAULT, lv.OPA.COVER)
slider.set_event_cb(slider_event_cb)
lv.event_send(slider,lv.EVENT.VALUE_CHANGED,None)
if slider.get_width() > fit_w:
    slider.get_width(fit_w)

h = lv.cont(t1, h)
h.set_fit(lv.FIT.NONE)
h.set_style_local_value_str(h.PART.MAIN, lv.STATE.DEFAULT, "Text input")
def ta_event_cb(obj,evt):
    if(evt == lv.EVENT.DEFOCUSED):
        print("ta")

ta = lv.textarea(h)
lv.cont.set_fit2(ta,lv.FIT.PARENT,lv.FIT.NONE)
ta.set_text("")
ta.set_placeholder_text("E-mail address")
ta.set_one_line(True)
ta.set_cursor_hidden(True)
ta.set_event_cb(ta_event_cb)

ta = lv.textarea(h,ta)
ta.set_pwd_mode(True)
ta.set_placeholder_text("Password")
ta.set_event_cb(ta_event_cb)

ta = lv.textarea(h)
lv.cont.set_fit2(ta,lv.FIT.PARENT, lv.FIT.NONE)
ta.set_text("")
ta.set_placeholder_text("Message")
ta.set_cursor_hidden(True)
ta.set_event_cb(ta_event_cb)
lv.cont.set_fit4(ta,lv.FIT.PARENT, lv.FIT.PARENT, lv.FIT.NONE, lv.FIT.PARENT)

#create visuals
lv.page.set_scrl_layout(t2,lv.LAYOUT.PRETTY_TOP)
disp_size = lv.disp_t.get_size_category(None)
grid_h_chart = lv.page.get_height_grid(t2, 1, 1)
if disp_size <= lv.DISP_SIZE.LARGE:
    div = 1
else:
    div = 2
grid_w_chart = lv.page.get_width_grid(t2, div, 1)
chart = lv.chart(t2)
style_box = lv.style_t()
style_box.init()
chart.add_style(chart.PART.BG, style_box)
if(disp_size <= lv.DISP_SIZE.SMALL) :
        chart.set_style_local_text_font(chart.PART.SERIES_BG, lv.STATE.DEFAULT, lv.theme_get_font_small())
chart.set_drag_parent(True)
chart.set_style_local_value_str(lv.cont.PART.MAIN, lv.STATE.DEFAULT, "Line chart")
chart.set_style_local_value_align(chart.PART.BG, lv.STATE.DEFAULT, lv.ALIGN.IN_TOP_LEFT)
chart.set_width_margin(grid_w_chart)
chart.set_height_margin(grid_h_chart)
chart.set_div_line_count(3, 0)
chart.set_point_count(8)
chart.set_type(chart.TYPE.LINE)
if(disp_size > lv.DISP_SIZE.SMALL) :
    chart.set_style_local_pad_left(chart.PART.BG, lv.STATE.DEFAULT, int(4 * (LV_DPI / 10)))
    chart.set_style_local_pad_bottom(chart.PART.BG, lv.STATE.DEFAULT, int(3 * (LV_DPI / 10)))
    chart.set_style_local_pad_right(chart.PART.BG, lv.STATE.DEFAULT, int(2 * (LV_DPI / 10)))
    chart.set_style_local_pad_top(chart.PART.BG, lv.STATE.DEFAULT, int(2 * (LV_DPI / 10)))
    chart.set_y_tick_length(0, 0)
    chart.set_x_tick_length(0, 0)
    chart.set_y_tick_texts("600\n500\n400\n300\n200", 0, chart.AXIS.DRAW_LAST_TICK)
    chart.set_x_tick_texts("Jan\nFeb\nMar\nApr\nMay\nJun\nJul\nAug", 0, chart.AXIS.DRAW_LAST_TICK)

color_read = lv.color16_t()
color_blue = lv.color16_t()
s1 = chart.add_series(color_read)
s2 = chart.add_series(color_blue)
chart.set_next(s1, 10)
chart.set_next(s1, 90)
chart.set_next(s1, 30)
chart.set_next(s1, 60)
chart.set_next(s1, 10)
chart.set_next(s1, 90)
chart.set_next(s1, 30)
chart.set_next(s1, 60)
chart.set_next(s1, 10)
chart.set_next(s1, 90)

chart.set_next(s2, 32)
chart.set_next(s2, 66)
chart.set_next(s2, 5)
chart.set_next(s2, 47)
chart.set_next(s2, 32)
chart.set_next(s2, 32)
chart.set_next(s2, 66)
chart.set_next(s2, 5)
chart.set_next(s2, 47)
chart.set_next(s2, 66)
chart.set_next(s2, 5)
chart.set_next(s2, 47)

chart2 = lv.chart(t2,chart)
chart2.set_type(chart2.TYPE.COLUMN)
chart2.set_style_local_value_str(lv.cont.PART.MAIN, lv.STATE.DEFAULT, "Column chart")
chart2.set_style_local_value_align(chart2.PART.BG, lv.STATE.DEFAULT, lv.ALIGN.IN_TOP_LEFT)
s1 = chart2.add_series(color_read)
s2 = chart2.add_series(color_blue)
chart2.set_next(s1, 10)
chart2.set_next(s1, 90)
chart2.set_next(s1, 30)
chart2.set_next(s1, 60)
chart2.set_next(s1, 10)
chart2.set_next(s1, 90)
chart2.set_next(s1, 30)
chart2.set_next(s1, 60)
chart2.set_next(s1, 10)
chart2.set_next(s1, 90)

chart2.set_next(s2, 32)
chart2.set_next(s2, 66)
chart2.set_next(s2, 5)
chart2.set_next(s2, 47)
chart2.set_next(s2, 32)
chart2.set_next(s2, 32)
chart2.set_next(s2, 66)
chart2.set_next(s2, 5)
chart2.set_next(s2, 47)
chart2.set_next(s2, 66)
chart2.set_next(s2, 5)
chart2.set_next(s2, 47)

if disp_size <= lv.DISP_SIZE.SMALL:
    grid_w_meter = lv.page.get_width_grid(t2, 1, 1)
elif disp_size <= lv.DISP_SIZE.MEDIUM:
    grid_w_meter = lv.page.get_width_grid(t2, 2, 1)
else:
    grid_w_meter = lv.page.get_width_grid(t2, 3, 1)
meter_h = lv.page.get_height_fit(t2)
if grid_w_meter>meter_h:
    meter_size = meter_h
else:
    meter_size = grid_w_meter
lmeter = lv.linemeter(t2)
lmeter.set_drag_parent(True)
lmeter.set_value(50)
lmeter.set_size(meter_size,meter_size)
lmeter.add_style(lmeter.PART.MAIN, style_box)
lmeter.set_style_local_value_str(lmeter.PART.MAIN, lv.STATE.DEFAULT, "")

label = lv.label(lmeter)
label.align(lmeter,lv.ALIGN.CENTER,0,0)
label.set_text('Line meter')
label.set_style_local_text_font(label.PART.MAIN, lv.STATE.DEFAULT, lv.theme_get_font_title())

def linemeter_anim(obj,value):
    obj.set_value(value)
    label = obj.get_child(None)
    label.set_text(str(value))
    label.align(obj,lv.ALIGN.CENTER,0,0)

a = lv.anim_t()
a.init()
a.set_custom_exec_cb(linemeter_anim)
a.set_custom_var(lmeter)
a.set_values(0, 100)
a.set_time(4000)
a.set_playback_time(1000)
a.set_repeat_count(4000)

lv.anim_t.start(a)


def gauge_anim(obj,value):
    obj.set_value(0,value)
    label = obj.get_child(None)
    label.set_text(str(value))
    label.align(obj,lv.ALIGN.CENTER,0,0)

gauge = lv.gauge(t2)
gauge.set_drag_parent(True)
gauge.set_size(meter_size, meter_size)
gauge.add_style(gauge.PART.MAIN, style_box)
gauge.set_style_local_value_str(gauge.PART.MAIN, lv.STATE.DEFAULT, "")
gauge.set_style_local_value_align(gauge.PART.MAIN, lv.STATE.DEFAULT, lv.ALIGN.IN_TOP_LEFT)
label = lv.label(gauge)
label.align(gauge,lv.ALIGN.CENTER,0,int(grid_w_meter/3))
label.set_text('gauge')

a = lv.anim_t()
a.init()
a.set_custom_exec_cb(gauge_anim)
a.set_custom_var(gauge)
a.set_values(0, 100)
a.set_time(4000)
a.set_playback_time(1000)
a.set_repeat_count(4000)


lv.anim_t.start(a)

def arc_anim(obj,value):
    obj.set_end_angle(value)
    label = obj.get_child(None)
    label.set_text(str(value))
    label.align(obj,lv.ALIGN.CENTER,0,0)

arc = lv.arc(t2)
arc.set_drag_parent(True)
arc.set_bg_angles(0, 360)
arc.set_rotation(270)
arc.set_angles(0, 0)
arc.set_size(meter_size, meter_size)
arc.add_style(arc.PART.BG, style_box)
arc.set_style_local_value_str(arc.PART.BG, lv.STATE.DEFAULT, "")
label = lv.label(arc)
label.align(arc,lv.ALIGN.CENTER,0,0)
label.set_text("arc")


a = lv.anim_t()
a.init()
a.set_custom_exec_cb(arc_anim)
a.set_custom_var(arc)
a.set_time(4000)
a.set_playback_time(1000)
a.set_repeat_count(4000)
a.set_values(0,360)

lv.anim_t.start(a)

bar_h = lv.cont(t2)
bar_h.set_fit2(lv.FIT.NONE,lv.FIT.TIGHT)
bar_h.add_style(bar_h.PART.MAIN, style_box)
bar_h.set_style_local_value_str(bar_h.PART.MAIN, lv.STATE.DEFAULT, "Bar")
bar_h.set_style_local_value_align(bar_h.PART.MAIN, lv.STATE.DEFAULT, lv.ALIGN.IN_BOTTOM_MID)
if(disp_size <= lv.DISP_SIZE.SMALL):
    bar_h.set_width(lv.page.get_width_grid(t2, 1, 1))
elif(disp_size <=lv.DISP_SIZE.MEDIUM):
    bar_h.set_width(lv.page.get_width_grid(t2, 2, 1))
else :
    bar_h.set_width(lv.page.get_width_grid(t2, 3, 2))

bar_anim_x = 0
def bar_anim(task):
    global bar_anim_x
    bar = task.get_user_data()
    text = "Copy %d/%d"%(bar_anim_x,bar.get_max_value())
    bar.set_style_local_value_str(bar.PART.BG, lv.STATE.DEFAULT, text)
    bar.set_value(bar, bar_anim_x, lv.ANIM.OFF)
    bar_anim_x = bar_anim_x +1
    if(bar_anim_x > bar.get_max_value()):
        bar_anim_x = 0

bar = lv.bar(bar_h)
bar.set_width(bar_h.get_width_fit())
bar.set_style_local_value_font(bar.PART.BG, lv.STATE.DEFAULT, lv.theme_get_font_small())
bar.set_style_local_value_align(bar.PART.BG, lv.STATE.DEFAULT, lv.ALIGN.OUT_BOTTOM_MID)
bar.set_style_local_value_ofs_y(bar.PART.BG, lv.STATE.DEFAULT, int(LV_DPI / 20))
bar.set_style_local_margin_bottom(bar.PART.BG, lv.STATE.DEFAULT, int(LV_DPI / 7))
bar.align(None, lv.ALIGN.CENTER, 0, 0)

led_h = lv.cont(t2)
led_h.set_layout(lv.LAYOUT.PRETTY_MID)
if(disp_size <= lv.DISP_SIZE.SMALL):
    led_h.set_width(lv.page.get_width_grid(t2, 1, 1))
elif(disp_size <=lv.DISP_SIZE.MEDIUM):
    led_h.set_width(lv.page.get_width_grid(t2, 2, 1))
else :
    led_h.set_width(lv.page.get_width_grid(t2, 3, 1))

led_h.set_height(bar_h.get_height())
led_h.add_style(led_h.PART.MAIN, style_box)
led_h.set_drag_parent(True)
led_h.set_style_local_value_str(led_h.PART.MAIN, lv.STATE.DEFAULT, "LEDs")
led_h.set_style_local_value_align(led_h.PART.MAIN, lv.STATE.DEFAULT, lv.ALIGN.IN_BOTTOM_MID)
led = lv.led(led_h)
led_size = led_h.get_height_fit()
led.set_size(led_size, led_size)
led.off()
led = lv.led(led_h, led)
led.set_bright(int((255 - 120) / 2 + 120))
led = lv.led(led_h, led)
led.on()

if(disp_size == lv.DISP_SIZE.MEDIUM) :
    led_h.add_protect(lv.PROTECT.POS)
    led_h.align(bar_h, lv.ALIGN.OUT_BOTTOM_MID, 0, led_h.get_style_margin_top(led_h.PART.MAIN) + t2.get_style_pad_inner(lv.page.PART.SCROLLABLE))

#lv.task_create(bar_anim, 100, lv.TASK_PRIO.LOW, bar)

###########
lv.page.set_scrl_layout(t3, lv.LAYOUT.PRETTY_MID)
disp_size = lv.disp_t.get_size_category(None)
grid_h = lv.page.get_height_grid(t3, 1, 1)
if(disp_size <= lv.DISP_SIZE.SMALL) :
    grid_w = lv.page.get_width_grid(t3, 1, 1)
else:
    grid_w = lv.page.get_width_grid(t3, 2, 1)
cal = lv.calendar(t3)
cal.set_drag_parent(True)
if(disp_size > lv.DISP_SIZE.MEDIUM):
    if  grid_h > grid_w:
        grid = grid_w
    else:
        grid = grid_h
    cal.set_size(grid, grid)
else:
    cal.set_size(grid_w, grid_w)
    if(disp_size <= lv.DISP_SIZE.SMALL) :
        cal.set_style_local_text_font(cal.PART.BG, lv.STATE.DEFAULT, lv.theme_get_font_small())
h = lv.cont(t3)
h.set_drag_parent(True)
if(disp_size <= lv.DISP_SIZE.SMALL) :
    h.set_fit2(lv.FIT.NONE, lv.FIT.TIGHT)
    h.set_width(lv.page.get_width_fit(t3))
    h.set_layout(lv.LAYOUT.COLUMN_MID)
elif(disp_size <= lv.DISP_SIZE.MEDIUM):
    h.set_size(cal.get_width(), cal.get_height())
    h.set_layout(lv.LAYOUT.PRETTY_TOP)
else:
    h.set_click(False)
    h.set_style_local_bg_opa(h.PART.MAIN, lv.STATE.DEFAULT, lv.OPA.TRANSP)
    h.set_style_local_border_opa(h.PART._MAIN, lv.STATE.DEFAULT, lv.OPA.TRANSP)
    h.set_style_local_pad_left(h.PART._MAIN, lv.STATE.DEFAULT, 0)
    h.set_style_local_pad_right(h.PART._MAIN, lv.STATE.DEFAULT, 0)
    h.set_style_local_pad_top(h.PART._MAIN, lv.STATE.DEFAULT, 0)
    if(grid_h > grid_w):
        grid = grid_w
    else :
        grid = grid_h
    h.set_size(h, grid,grid)
    h.set_layout(h, lv.LAYOUT.PRETTY_TOP)

roller = lv.roller(h)
roller.add_style(h.PART.MAIN, style_box)
roller.set_style_local_value_str(h.PART.MAIN, lv.STATE.DEFAULT, "Roller")
roller.set_auto_fit(False)
roller.set_align(lv.label.ALIGN.CENTER)
roller.set_visible_row_count(4)
if(disp_size <= lv.DISP_SIZE.SMALL):
    div = 1
else:
    div = 2

roller.set_width(h.get_width_grid(div, 1))
dd = lv.dropdown(h)
dd.add_style(h.PART.MAIN, style_box)
dd.set_style_local_value_str(h.PART.MAIN, lv.STATE.DEFAULT, "")
dd.set_width(h.get_width_grid(div, 1))
dd.set_options("Alpha\nBravo\nCharlie\nDelta\nEcho\nFoxtrot\nGolf\nHotel\nIndia\nJuliette\nKilo\nLima\nMike\nNovember\n"
            "Oscar\nPapa\nQuebec\nRomeo\nSierra\nTango\nUniform\nVictor\nWhiskey\nXray\nYankee\nZulu")
list1 = lv.list(t3)
list1.set_scroll_propagation(True)
list1.set_size(grid_w, grid_h)

btn_texts_img = [lv.SYMBOL.SAVE,lv.SYMBOL.CUT,lv.SYMBOL.COPY,lv.SYMBOL.OK,lv.SYMBOL.EDIT,
            lv.SYMBOL.WIFI,lv.SYMBOL.BLUETOOTH,lv.SYMBOL.GPS ,lv.SYMBOL.USB,
            lv.SYMBOL.SD_CARD,lv.SYMBOL.CLOSE
            ]
btn_texts_txt = ["Save","Cut","Copy","OK","Edit",
            "WIFI","BlueTooth","GPS","USB",
            "SDCard","Close"
            ]

for i in range(len(btn_texts_img)):
    btn = list1.add_btn(btn_texts_img[i],btn_texts_txt[i])
    #btn.set_checkable(True)

h1 = lv.calendar_date_t()
h1.year=2020
h1.month=2
h1.day=14
cal.set_highlighted_dates(h1,1)

style_cell4 = lv.style_t()
style_cell4.init()
style_cell4.set_bg_opa(lv.STATE.DEFAULT,lv.OPA._50)
page = lv.page(t3)
page.set_size(grid_w, grid_h)
table_w_max = page.get_width_fit()
page.set_scroll_propagation(True)
def table_event_cb(obj,e):
    if(e == lv.EVENT.CLICKED):
        r,row,col = obj.get_pressed_cell()
        print("cell:",obj.get_cell_value(row,col))


table1 = lv.table(page)
table1.add_style(table1.PART.CELL4, style_cell4)
table1.clean_style_list(table1.PART.BG)
table1.set_drag_parent(True)
table1.set_event_cb(table_event_cb)
table1.set_col_cnt(2)
if(disp_size > lv.DISP_SIZE.SMALL):
        table1.set_col_width(0, LV_MATH_MAX(30, int(1 * table_w_max  / 5)))
        table1.set_col_width(1, LV_MATH_MAX(60, int(2 * table_w_max / 5)))
        table1.set_col_width(2, LV_MATH_MAX(60, int(2 * table_w_max / 5)))
else:
        table1.set_col_width(0, LV_MATH_MAX(30, int(1 * table_w_max  / 4 - 1)))
        table1.set_col_width(1, LV_MATH_MAX(60, int(3 * table_w_max / 4 - 1)))

table1.set_cell_value(0, 0, "#")
table1.set_cell_value(1, 0, "1")
table1.set_cell_value(2, 0, "2")
table1.set_cell_value(3, 0, "3")
table1.set_cell_value(4, 0, "4")
table1.set_cell_value(5, 0, "5")
table1.set_cell_value(6, 0, "6")
table1.set_cell_value(0, 1, "NAME")
table1.set_cell_value(1, 1, "Mark")
table1.set_cell_value(2, 1, "Jacob")
table1.set_cell_value(3, 1, "John")
table1.set_cell_value(4, 1, "Emily")
table1.set_cell_value(5, 1, "Ivan")
table1.set_cell_value(6, 1, "George")

if(disp_size > lv.DISP_SIZE.SMALL) :
    table1.set_cell_value(0, 2, "CITY")
    table1.set_cell_value(1, 2, "Moscow")
    table1.set_cell_value(2, 2, "New York")
    table1.set_cell_value(3, 2, "Oslo")
    table1.set_cell_value(4, 2, "London")
    table1.set_cell_value(5, 2, "Texas")
    table1.set_cell_value(6, 2, "Athen")



while(1):
    pyb.mdelay(50)



