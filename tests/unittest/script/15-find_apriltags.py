#
# Copyright (c) 2006-2018, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
def unittest(data_path, temp_path):
    import image
    img = image.Image("/sd/unittest/data/apriltags.pgm", copy_to_fb=True)
    tags = img.find_apriltags()
    return len(tags) == 1 and tags[0][0:8] == (45, 27, 69, 69, 255, 16, 80, 61)
