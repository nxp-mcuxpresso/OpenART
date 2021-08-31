from machine import SPI
spi=SPI(40)
spi.write("hello")
spi.read(5)