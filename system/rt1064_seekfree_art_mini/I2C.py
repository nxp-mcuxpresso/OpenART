from machine import I2C
i2c=I2C(1)
i2c.scan()
i2c.writeto(24,'Hello Micropython')
i2c.readfrom(24,17)