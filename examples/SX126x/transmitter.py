import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x
import time

# Begin LoRa radio and set NSS, reset, busy, IRQ, txen, and rxen pin with connected Raspberry Pi gpio pins
# IRQ pin not used in this example (set to -1). Set txen and rxen pin to -1 if RF module doesn't have one
busId = 1; csId = 0
resetPin = 22; busyPin = 23; irqPin = -1; txenPin = 5; rxenPin = 25
LoRa = SX126x()
print("Begin LoRa radio")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin) :
    raise Exception("Something wrong, can't begin LoRa radio")

# Configure LoRa to use TCXO with DIO3 as control
LoRa.setDio3TcxoCtrl(LoRa.DIO3_OUTPUT_1_8, LoRa.TCXO_DELAY_10)
print("Set RF module to use TCXO as clock reference")

# Set frequency to 915 Mhz
LoRa.setFrequency(915000000)
print("Set frequency to 915 Mhz")

# Set TX power, default power for SX1262 and SX1268 are +22 dBm and for SX1261 is +14 dBm
# This function will set PA config with optimal setting for requested TX power
LoRa.setTxPower(17, LoRa.TX_POWER_SX1262)
print("Set TX power to +17 dBm")

# Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
# Receiver must have same SF and BW setting with transmitter to be able to receive LoRa packet
sf = 7
bw = 125000
cr = 5
LoRa.setLoRaModulation(sf, bw, cr)
print("Set modulation parameters:\n\tSpreading factor = 7\n\tBandwidth = 125 kHz\n\tCoding rate = 4/5")

# Configure packet parameter including header type, preamble length, payload length, and CRC type
# The explicit packet includes header contain CR, number of byte, and CRC type
# Receiver can receive packet with different CR and packet parameters in explicit header mode
headerType = LoRa.HEADER_EXPLICIT
preambleLength = 12
payloadLength = 15
crcType = True
LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType)
print("Set packet parameters:\n\tExplicit header type\n\tPreamble length = 12\n\tPayload Length = 15\n\tCRC on")

# Set syncronize word for public network (0x3444)
LoRa.setSyncWord(0x3444)
print("Set syncronize word to 0x3444")

print("\n-- LoRa Transmitter --\n")

# Message to transmit
message = "HeLoRa World!\0"
messageList = list(message)
for i in range(len(messageList)) : messageList[i] = ord(messageList[i])
counter = 0

# Transmit message continuously
while True :

    # Transmit message and counter
    # write() method must be placed between beginPacket() and endPacket()
    LoRa.beginPacket()
    LoRa.write(messageList, len(messageList))
    LoRa.write([counter], 1)
    LoRa.endPacket()

    # Print message and counter
    print(f"{message}  {counter}")

    # Wait until modulation process for transmitting packet finish
    LoRa.wait()

    # Print transmit time and data rate
    print("Transmit time: {0:0.2f} ms | Data rate: {1:0.2f} byte/s".format(LoRa.transmitTime(), LoRa.dataRate()))

    # Don't load RF module with continous transmit
    time.sleep(5)
    counter = (counter + 1) % 256
