import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX127x, LoRaSpi, LoRaGpio
import time

# Begin LoRa radio with connected SPI bus and IO pins (cs and reset) on GPIO
# SPI is defined by bus ID and cs ID and IO pins defined by chip and offset number
spi = LoRaSpi(0, 0)
cs = LoRaGpio(0, 8)
reset = LoRaGpio(0, 24)
LoRa = SX127x(spi, cs, reset)
print("Begin LoRa radio")
if not LoRa.begin() :
    raise Exception("Something wrong, can't begin LoRa radio")

# Set frequency to 915 Mhz
print("Set frequency to 915 Mhz")
LoRa.setFrequency(915000000)

# Set RX gain. RX gain option are power saving gain or boosted gain
print("Set RX gain to power saving gain")
LoRa.setRxGain(LoRa.RX_GAIN_POWER_SAVING, LoRa.RX_GAIN_AUTO)    # AGC on, Power saving gain

# Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
# Receiver must have same SF and BW setting with transmitter to be able to receive LoRa packet
print("Set modulation parameters:\n\tSpreading factor = 7\n\tBandwidth = 125 kHz\n\tCoding rate = 4/5")
LoRa.setSpreadingFactor(7)                                      # LoRa spreading factor: 7
LoRa.setBandwidth(125000)                                       # Bandwidth: 125 kHz
LoRa.setCodeRate(5)                                             # Coding rate: 4/5

# Configure packet parameter including header type, preamble length, payload length, and CRC type
# The explicit packet includes header contain CR, number of byte, and CRC type
# Receiver can receive packet with different CR and packet parameters in explicit header mode
print("Set packet parameters:\n\tExplicit header type\n\tPreamble length = 12\n\tPayload Length = 15\n\tCRC on")
LoRa.setHeaderType(LoRa.HEADER_EXPLICIT)                        # Explicit header mode
LoRa.setPreambleLength(12)                                      # Set preamble length to 12
LoRa.setPayloadLength(15)                                       # Initialize payloadLength to 15
LoRa.setCrcEnable(True)                                         # Set CRC enable

# Set syncronize word for public network (0x34)
print("Set syncronize word to 0x34")
LoRa.setSyncWord(0x34)

print("\n-- LoRa Receiver --\n")

# Receive message continuously
while True :

    # Request for receiving new LoRa packet
    LoRa.request()
    # Wait for incoming LoRa packet
    LoRa.wait()

    # Put received packet to message and counter variable
    # read() and available() method must be called after request() or listen() method
    message = ""
    # available() method return remaining received payload length and will decrement each read() or get() method called
    while LoRa.available() > 1 :
        message += chr(LoRa.read())
    counter = LoRa.read()

    # Print received message and counter in serial
    print(f"{message}  {counter}")

    # Print packet/signal status including RSSI, SNR, and signalRSSI
    print("Packet status: RSSI = {0:0.2f} dBm | SNR = {1:0.2f} dB".format(LoRa.packetRssi(), LoRa.snr()))

    # Show received status in case CRC or header error occur
    status = LoRa.status()
    if status == LoRa.STATUS_CRC_ERR : print("CRC error")
    elif status == LoRa.STATUS_HEADER_ERR : print("Packet header error")
