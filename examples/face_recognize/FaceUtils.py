import tf,os,image,ujson,cmath

model = "/sd/face_detect_64x64.tflite"
threadhold = 0.90
net = tf.load(model)
mobilenet = "/sd/mfn_drop_320_best_quant.tflite"
mobilenet = tf.load(mobilenet)
db_file = "/sd/mobilefacenet/db.json"
db_list = []
db_count = 0

def clear_faces():
    global db_count
    global db_list

    for dict in db_list:
        idx = dict['idx']
        fname = "/sd/mobilefacenet/%d.bmp"%idx
        os.remove(fname)

    os.remove(db_file)
    db_count = 0
    db_list.clear()

def load_db():
    global db_count
    global db_list

    try:
        os.stat('/sd/mobilefacenet')
    except:
        os.mkdir('/sd/mobilefacenet')
    try:
        f = open(db_file,'r')
        db_str = f.read()
        db_list = ujson.loads(db_str)
        f.close()
    except:
        print("no face found")
    db_count = len(db_list)
    return len(db_list)

def get_faces_cnt():
    return len(db_list)

def save_db():
    global db_list

    try:
        os.remove(db_file)
    except:
        print("no db file")
    db_string = str(db_list)
    db_string = db_string.replace("'",'"')
    f = open(db_file,'w')
    f.write(db_string)
    f.close()

def save_face(arrs, img):
    global db_count
    global db_list

    dict = {"idx":0,"file":"","vector":1}
    fname = '/sd/mobilefacenet/%d'%db_count+'.bmp'
    dict['idx'] = db_count
    dict['file'] = fname
    dict['vector'] = arrs
    img.save(fname)
    db_count = db_count +1
    db_list.append(dict)
    save_db()
    print("save image: %s"%fname)

def ModelCulculateArray(img):
    global net

    array = []
    for obj in tf.classify(mobilenet, img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
        array = obj.output()
    return array

def findFaceinList(arr, angle_threadhold,angle_sensitive):
    global db_list

    min = 999.0
    i = 0
    index = -1
    for dict in db_list:
        min_temp = cmath.calc_angle_float(arr,dict['vector'])
        #defined in cmath module
        if (min >= min_temp and min_temp <= angle_threadhold + angle_sensitive):
            min = min_temp
            index = dict['idx']

        elif min >= min_temp:
            min = min_temp
        i = i + 1

    return index,min

def find_faces(img):
    global net
    global threadhold

    objs = tf.detect(model,img,bgr=1)
    obj_list = []
    for obj in objs:
        x1,y1,x2,y2,label,scores = obj
        if (scores > threadhold):
            w = x2- x1
            h = y2 - y1
            x1 = int(x1*img.width() + 0.5)
            y1 = int(y1*img.height() + 0.5)
            w = int(w*img.width() + 0.5)
            h = int(h*img.height() + 0.5)
            obj_list.append((x1,y1,w,h))

    return obj_list

