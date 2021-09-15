#
# Copyright (c) 2006-2018, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
from machine import UART
uart = UART(1)
uart.write("UART WRITE TEST")
uart.read(4)