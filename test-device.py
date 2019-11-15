"""Run these tests at the command line/terminal via

    $ python3 tests/manual_tests.py
"""
import usbiss
from usbiss.spi import SPI
import opc
from time import sleep

#spi = SPI("/dev/cu.usbmodem000452891") # Saleae probed
spi = SPI("/dev/cu.usbmodem000464721")

spi.mode = 1
spi.max_speed_hz = 300000

alpha = opc.OPCN3(spi, debug=False)

# turn on
print ("Turning ON: {}".format(alpha.on()))
sleep(1)

for i in range(10):
    print ("Reading histogram: run {}".format(i + 1))
    data = alpha.histogram()
    if i > 0: # discard first as it will contain invalid data 
        print (data)
        sleep(2)

# and off again
print ("Turning off: {}".format(alpha.off()))
