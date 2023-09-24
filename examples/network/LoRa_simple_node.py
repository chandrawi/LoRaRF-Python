import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x, LoRaSpi, LoRaGpio
import time
import struct
import random

# Begin LoRa radio with connected SPI bus and IO pins (cs and reset) on GPIO
# SPI is defined by bus ID and cs ID and IO pins defined by chip and offset number
spi = LoRaSpi(0, 0)
cs = LoRaGpio(0, 8)
reset = LoRaGpio(0, 24)
busy = LoRaGpio(0, 23)
LoRa = SX126x(spi, cs, reset, busy)
print("Begin LoRa radio")
if not LoRa.begin() :
    raise Exception("Something wrong, can't begin LoRa radio")

# Configure LoRa to use TCXO with DIO3 as control
LoRa.setDio3TcxoCtrl(LoRa.DIO3_OUTPUT_1_8, LoRa.TCXO_DELAY_10)
print("Set RF module to use TCXO as clock reference")

# Set frequency to 915 Mhz
LoRa.setFrequency(915000000)
print("Set frequency to 915 Mhz")

# Set TX power to +22 dBm
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
print("Set TX power to +22 dBm")

# Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
sf = 7
bw = 125000
cr = 5
LoRa.setLoRaModulation(sf, bw, cr)
print("Set modulation parameters:\n\tSpreading factor = 7\n\tBandwidth = 125 kHz\n\tCoding rate = 4/5")

# Configure packet parameter including header type, preamble length, payload length, and CRC type
headerType = LoRa.HEADER_IMPLICIT
preambleLength = 12
payloadLength = 12
crcType = True
LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType)
print(f"Set packet parameters:\n\tImplicit header type\n\tPreamble length = 12\n\tPayload Length = 12\n\tCRC on")

# Set syncronize word for public network (0x3444)
LoRa.setSyncWord(0x3444)
print("Set syncronize word to 0x3444")

print("\n-- LoRa Node --\n")

# IDs and message format to transmit
gatewayId = 0xCC
nodeId = 0x77
messageId = 0
format = 'BBHIi'
length = struct.calcsize(format)

# Transmit message continuously
while True :

    # Structured message
    structure = [
        gatewayId,
        nodeId,
        messageId,
        round(time.time()),
        random.randrange(-1073741824, 1073741824)
    ]
    message = struct.pack(format, *structure)
    messageId = (messageId + 1) % 65536

    # Transmit structured message
    # Set transmit timeout to 250 ms
    LoRa.beginPacket()
    LoRa.put(message)
    LoRa.endPacket(250)
    LoRa.wait()

    # Print structured message
    print("Gateway ID    : 0x{0:02X}".format(gatewayId))
    print("Node ID       : 0x{0:02X}".format(nodeId))
    print(f"Message ID    : {structure[2]}")
    print(f"Time          : {structure[3]}")
    print(f"Data          : {structure[4]}")

    # Print transmit time
    print("Transmit time : {0:0.2f} ms\n".format(LoRa.transmitTime()))

    # Print status timeout when transmit process terminated due to timeout
    if LoRa.status() == LoRa.STATUS_TX_TIMEOUT : print("Transmit timeout")

    # Put RF module to sleep in a few seconds
    LoRa.sleep()
    time.sleep(5)
    LoRa.wake()
