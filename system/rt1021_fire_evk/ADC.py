#
# Copyright (c) 2006-2018, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
from machine import ADC
adc = ADC(1,1) #ADC 1 , chl 1
adc.read()