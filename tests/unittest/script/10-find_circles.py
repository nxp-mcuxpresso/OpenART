#
# Copyright (c) 2006-2018, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
def unittest(data_path, temp_path):
    import image
    img = image.Image("/sd/unittest/data/shapes.ppm", copy_to_fb=True)
    circles = img.find_circles(threshold = 5000, x_margin = 30, y_margin = 30, r_margin = 30)
    return len(circles) == 1 and circles[0][0:] == (118, 56, 22, 5829)
