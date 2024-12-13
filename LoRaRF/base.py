import spidev
import gpiod
from gpiod.line import Direction, Value, Edge
from typing import Iterable
import time


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

    LOW = Value.INACTIVE
    HIGH = Value.ACTIVE

    def __init__(self, chip: int, offset: int):
        self.chip = "/dev/gpiochip" + str(chip)
        self.offset = offset
        self.seqno = 0

    def output(self, value: Value):
        with gpiod.request_lines(
            self.chip,
            consumer="LoRaGpio",
            config={self.offset: gpiod.LineSettings(direction=Direction.OUTPUT)}
        ) as request:
            request.set_value(self.offset, value)

    def input(self):
        with gpiod.request_lines(
            self.chip,
            consumer="LoRaGpio",
            config={self.offset: gpiod.LineSettings(direction=Direction.INPUT)}
        ) as request:
            return request.get_value(self.offset)

    def monitor(self, callback, timeout: float):
        t = time.time()
        with gpiod.request_lines(
            self.chip,
            consumer="LoRaGpio",
            config={self.offset: gpiod.LineSettings(edge_detection=Edge.RISING)}
        ) as request:
            while (time.time() - t) < timeout or timeout == 0:
                for event in request.read_edge_events():
                    if event.line_seqno != self.seqno:
                        self.seqno = event.line_seqno
                        callback()
                        return


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
