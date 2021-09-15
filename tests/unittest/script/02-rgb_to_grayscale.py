#
# Copyright (c) 2006-2018, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
def unittest(data_path, temp_path):
    import image
    gs = image.rgb_to_grayscale((120, 200, 120))
    return  (gs == 167)
