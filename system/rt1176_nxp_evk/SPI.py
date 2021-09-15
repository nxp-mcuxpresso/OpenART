#
# Copyright (c) 2006-2018, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
from machine import SPI
spi=SPI(40)
spi.write("hello")
spi.read(5)