#
# Copyright (c) 2006-2018, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
from machine import I2C
i2c=I2C(1)
i2c.scan()
i2c.writeto(24,'Hello Micropython')
i2c.readfrom(24,17)