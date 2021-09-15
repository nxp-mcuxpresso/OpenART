#
# Copyright (c) 2006-2018, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
def unittest(data_path, temp_path):
    import image
    img = image.Image("/sd/unittest/data/eye.pgm", copy_to_fb=True)
    iris = img.find_eye((100, 70, 250, 100))
    return iris == (159, 114)
