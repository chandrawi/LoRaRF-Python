import RPi.GPIO

DEF_GPIO = 0
RPi_GPIO = 0
BB_GPIO = 1

class LoRaIO :

    GPIO = RPi.GPIO

    def __init__(self, GPIO_lib = DEF_GPIO) :
        if GPIO_lib == RPi_GPIO :
            GPIO = RPi.GPIO
            GPIO.setmode(RPi.GPIO.BCM)
            GPIO.setwarnings(False)
        elif GPIO_lib == BB_GPIO :
            print("Board currently not supported")
        else :
            print("Board currently not supported")
