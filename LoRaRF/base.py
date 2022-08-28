import spidev
import RPi.GPIO

class NotCompatibleError(Exception):
    pass
    

class BaseLoRa(object):

    def __init__(self, *args, **kwargs):
        self.spi = spidev.SpiDev()
        self.gpio = RPi.GPIO
        self.gpio.setmode(RPi.GPIO.BCM)
        self.gpio.setwarnings(False)

    def begin(self):
        raise NotImplementedError

    def end(self):
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError

            
        

