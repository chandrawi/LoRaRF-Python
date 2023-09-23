import spidev
import gpiod
from typing import Iterable


class LoRaSpi():

    SPI_SPEED = 8000000

    def __init__(self, bus: int, cs: int, speed: int = SPI_SPEED):
        self.bus = bus
        self.cs = cs
        self.speed = speed

    def transfer(self, buf: Iterable) -> tuple:
        spi = spidev.SpiDev()
        spi.open(self.bus, self.cs)
        spi.lsbfirst = False
        spi.mode = 0
        spi.max_speed_hz = self.speed
        ret = spi.xfer2(buf)
        spi.close()
        return ret


class LoRaGpio:

    LOW = 0
    HIGH = 1

    def __init__(self, chip: int, offset: int):
        self.chip = "gpiochip" + str(chip)
        self.offset = offset

    def output(self, value: int):
        chip = gpiod.Chip(self.chip)
        line = chip.get_line(self.offset)
        try:
            line.request(consumer="LoRaGpio", type=gpiod.LINE_REQ_DIR_OUT)
            line.set_value(value)
        except: return
        finally:
            line.release()
            chip.close()

    def input(self) -> int:
        chip = gpiod.Chip(self.chip)
        line = chip.get_line(self.offset)
        try:
            line.request(consumer="LoRaGpio", type=gpiod.LINE_REQ_DIR_IN)
            value = line.get_value()
        except: return -1
        finally:
            line.release()
            chip.close()
        return value

    def monitor(self, callback, timeout: float):
        seconds = int(timeout)
        chip = gpiod.Chip(self.chip)
        line = chip.get_line(self.offset)
        try:
            line.request(consumer="LoRaGpio", type=gpiod.LINE_REQ_EV_RISING_EDGE)
            if line.event_wait(seconds, int((timeout - seconds) * 1000000000)):
                callback()
        except: return
        finally:
            line.release()
            chip.close()

    def monitor_continuous(self, callback, timeout: float):
        seconds = int(timeout)
        while True:
            chip = gpiod.Chip(self.chip)
            line = chip.get_line(self.offset)
            try:
                line.request(consumer="LoRaGpio", type=gpiod.LINE_REQ_EV_RISING_EDGE)
                if line.event_wait(seconds, int((timeout - seconds) * 1000000000)):
                    callback()
            except: continue
            finally:
                line.release()
                chip.close()


class BaseLoRa :

    def begin(self):
        raise NotImplementedError

    def end(self):
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError

    def beginPacket(self):
        raise NotImplementedError

    def endPacket(self, timeout: int)-> bool:
        raise NotImplementedError

    def write(self, data, length: int):
        raise NotImplementedError

    def request(self, timeout: int)-> bool:
        raise NotImplementedError

    def available(self):
        raise NotImplementedError

    def read(self, length: int):
        raise NotImplementedError

    def wait(self, timeout: int)-> bool:
        raise NotImplementedError

    def status(self):
        raise NotImplementedError
