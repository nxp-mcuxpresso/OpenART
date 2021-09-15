#
# Copyright (c) 2006-2018, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
def unittest(data_path, temp_path):
    import image
    img = image.Image("/sd/unittest/data/shapes.ppm", copy_to_fb=True)
    rects = img.find_rects(threshold = 50000)
    return len(rects) == 1 and rects[0][0:] == (23, 39, 35, 36, 146566)
