import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
from LoRaRF import SX126x
from LoRaRF import LoRaIO
import time
import struct
import random

# Begin LoRa radio and set NSS, reset, busy, IRQ, txen, and rxen pin with connected Raspberry Pi gpio pins
# IRQ pin not used in this example (set to -1). Set txen and rxen pin to -1 if RF module doesn't have one
busId = 1; csId = 0
board = LoRaIO.RPi_GPIO; resetPin = 22; busyPin = 23; irqPin = -1; txenPin = 26; rxenPin = 25
LoRa = SX126x(busId, csId, LoRaIO.RPi_GPIO, resetPin, busyPin, irqPin, txenPin, rxenPin)
print("Begin LoRa radio")
if not LoRa.begin() :
    raise Exception("Something wrong, can't begin LoRa radio")

# Configure LoRa to use TCXO with DIO3 as control
LoRa.setDio3TcxoCtrl(LoRa.DIO3_OUTPUT_1_8, LoRa.TCXO_DELAY_10)
print("Set RF module to use TCXO as clock reference")

# Set frequency to 915 Mhz
LoRa.setFrequency(915000000)
print("Set frequency to 915 Mhz")

# Set TX power, default power for SX1262 and SX1268 are +22 dBm and for SX1261 is +14 dBm
# This function will set PA config with optimal setting for requested TX power
LoRa.setTxPower(LoRa.TX_POWER_SX1262_22)
print("Set TX power to +22 dBm")

# Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
# Receiver must have same SF and BW setting with transmitter to be able to receive LoRa packet
sf = 7
bw = LoRa.LORA_BW_125
cr = LoRa.LORA_CR_4_5
LoRa.setLoRaModulation(sf, bw, cr)
print("Set modulation parameters:\n\tSpreading factor = 7\n\tBandwidth = 125 kHz\n\tCoding rate = 4/5")

# Configure packet parameter including header type, preamble length, payload length, and CRC type
# The explicit packet includes header contain CR, number of byte, and CRC type
# Receiver can receive packet with different CR and packet parameters in explicit header mode
headerType = LoRa.LORA_HEADER_IMPLICIT
preambleLength = 12
payloadLength = 12
crcType = LoRa.LORA_CRC_ON
LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType)
print(f"Set packet parameters:\n\tImplicit header type\n\tPreamble length = 12\n\tPayload Length = 12\n\tCRC on")

# Set syncronize word for private network (0x1424)
LoRa.setLoRaSyncWord(0x1424)
print("Set syncronize word to 0x1424")

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
    messageId += 1

    # Transmit structured message
    # Set transmit timeout to 250 ms
    LoRa.beginPacket()
    LoRa.put(message)
    LoRa.endPacket(250)

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
