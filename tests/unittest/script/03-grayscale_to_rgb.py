#
# Copyright (c) 2006-2018, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
def unittest(data_path, temp_path):
    import image
    rgb = image.grayscale_to_rgb(182)
    return  (rgb[0] == 181 and rgb[1] == 182 and rgb[2] == 181)
