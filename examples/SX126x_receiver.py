# import os, sys
# currentdir = os.path.dirname(os.path.realpath(__file__))
# parentdir = os.path.dirname(currentdir)
# sys.path.append(parentdir)
from LoRaRF import SX126x
from LoRaRF import LoRaIO
import time

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
headerType = LoRa.LORA_HEADER_EXPLICIT
preambleLength = 12
payloadLength = 15
crcType = LoRa.LORA_CRC_ON
LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType)
print("Set packet parameters:\n\tExplicit header type\n\tPreamble length = 12\n\tPayload Length = 15\n\tCRC on")

# Set syncronize word for public network (0x3444)
LoRa.setLoRaSyncWord(0x3444)
print("Set syncronize word to 0x3444")

print("\n-- LoRa Receiver --\n")

# Receive message continuously
while True :

    # Request for receiving new LoRa packet
    LoRa.request()

    # Put received packet to message and counter variable
    # read() and available() method must be called after request() or listen() method
    length = LoRa.available() - 1
    message = ""
    # available() method return remaining received payload length and will decrement each read() or get() method called
    while LoRa.available() > 1 :
        message += chr(LoRa.read())
    counter = LoRa.read()

    # Print received message and counter in serial
    print(f"{message}  {counter}")

    # Print packet/signal status including RSSI, SNR, and signalRSSI
    print("Packet status: RSSI = {0:0.2f} dBm | SNR = {1:0.2f} dB".format(LoRa.rssi(), LoRa.snr()))

    # Show received status in case CRC or header error occur
    status = LoRa.status()
    if status == LoRa.STATUS_CRC_ERR : print("CRC error")
    if status == LoRa.STATUS_HEADER_ERR : print("Packet header error")
