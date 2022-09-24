import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX127x
import time

# Begin LoRa radio and set NSS, reset, busy, IRQ, txen, and rxen pin with connected Raspberry Pi gpio pins
# IRQ pin not used in this example (set to -1). Set txen and rxen pin to -1 if RF module doesn't have one
busId = 1; csId = 0
resetPin = 22; irqPin = -1; txenPin = -1; rxenPin = -1
LoRa = SX127x()
print("Begin LoRa radio")
if not LoRa.begin(busId, csId, resetPin, irqPin, txenPin, rxenPin) :
    raise Exception("Something wrong, can't begin LoRa radio")

# Set frequency to 915 Mhz
print("Set frequency to 915 Mhz")
LoRa.setFrequency(915000000)

# Set TX power, this function will set PA config with optimal setting for requested TX power
print("Set TX power to +17 dBm")
LoRa.setTxPower(17, LoRa.TX_POWER_PA_BOOST)                     # TX power +17 dBm using PA boost pin

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
