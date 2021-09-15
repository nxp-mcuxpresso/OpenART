#
# Copyright (c) 2006-2018, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
def unittest(data_path, temp_path):
    import image
    rgb = image.lab_to_rgb((74, -38, 30))
    return  (rgb[0] == 123 and rgb[1] == 198 and rgb[2] == 123)
