# import os, sys
# currentdir = os.path.dirname(os.path.realpath(__file__))
# parentdir = os.path.dirname(currentdir)
# sys.path.append(parentdir)
from LoRaRF import SX126x
from LoRaRF import LoRaIO
import time
import struct

# Begin LoRa radio and set NSS, reset, busy, IRQ, txen, and rxen pin with connected Raspberry Pi gpio pins
# IRQ pin not used in this example (set to -1). Set txen and rxen pin to -1 if RF module doesn't have one
busId = 1; csId = 0
board = LoRaIO.RPi_GPIO; resetPin = 22; busyPin = 23; irqPin = -1; txenPin = 5; rxenPin = 25
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

# Set RX gain. RX gain option are power saving gain or boosted gain
LoRa.setRxGain(LoRa.RX_GAIN_POWER_SAVING)
print("Set RX gain to power saving gain")

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

print("\n-- LoRa Gateway --\n")

# IDs and message format from received message
gatewayId = 0xCC
format = 'BBHIi'
length = struct.calcsize(format)

# Transmit message continuously
while True :

    # Set RF module to listen mode with 20 ms RX mode and 10 ms sleep
    # Some LoRa packet will not be received if sleep period too long or preamble length too short
    rxPeriod = 20
    sleepPeriod = 10
    LoRa.listen(rxPeriod, sleepPeriod)
    # Wait for incoming LoRa packet
    LoRa.wait()

    # Get received structured message
    message = LoRa.get(length)
    structure = struct.unpack(format, message)

    # Check gateway ID from received message
    if structure[0] == gatewayId :

        # Print structured message
        print("Gateway ID    : 0x{0:02X}".format(gatewayId))
        print("Node ID       : 0x{0:02X}".format(structure[1]))
        print(f"Message ID    : {structure[2]}")
        print(f"Time          : {structure[3]}")
        print(f"Data          : {structure[4]}")

        # Print packet status
        print("Packet status : RSSI = {0:0.2f} dBm | SNR = {1:0.2f} dB\n".format(LoRa.rssi(), LoRa.snr()))

    else :

        # Print error message
        print("Received message with wrong gateway ID (0x{0:02X})".format(structure[0]))

    # Show received status in case CRC or header error occur
    status = LoRa.status()
    if status == LoRa.STATUS_CRC_ERR : print("CRC error")
    if status == LoRa.STATUS_HEADER_ERR : print("Packet header error")
