import pyb
from pyb import LED

print('Start test LED\r\n')
red = LED(1)
green = LED(2)
blue = LED(3)
blue2 = LED(4)

red.on()
pyb.mdelay(500)
red.off()
pyb.mdelay(500)
red.toggle()
pyb.mdelay(500)
red.toggle()

green.on()
pyb.mdelay(500)
green.off()
pyb.mdelay(500)
green.toggle()
pyb.mdelay(500)
green.toggle()


blue.on()
pyb.mdelay(500)
blue.off()
pyb.mdelay(500)
blue.toggle()
pyb.mdelay(500)
blue.toggle()

blue2.on()
pyb.mdelay(500)
blue2.off()
pyb.mdelay(500)
blue2.toggle()
pyb.mdelay(500)
blue2.toggle()
print('LED test end\r\n')