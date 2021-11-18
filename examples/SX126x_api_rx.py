# import os, sys
# currentdir = os.path.dirname(os.path.realpath(__file__))
# parentdir = os.path.dirname(currentdir)
# sys.path.append(parentdir)
from LoRaRF import SX126x
from LoRaRF import LoRaIO
import time

busId = 1; csId = 0
board = LoRaIO.RPi_GPIO; resetPin = 22; busyPin = 23; irqPin = 24; txenPin = 5; rxenPin = 25

LoRa = SX126x(busId, csId, LoRaIO.RPi_GPIO, resetPin, busyPin)
GPIO = LoRaIO.LoRaIO(LoRaIO.DEF_GPIO).GPIO

# TCXO control setting
dio3Voltage = LoRa.DIO3_OUTPUT_1_8
tcxoDelay = LoRa.TCXO_DELAY_10
# RF frequency setting
rfFrequency = 915000000
# RX gain setting
gain = LoRa.RX_GAIN_POWER_SAVING
# Define modulation parameters setting
sf = LoRa.LORA_SF_7
bw = LoRa.LORA_BW_125
cr = LoRa.LORA_CR_4_5
ldro = LoRa.LORA_LDRO_OFF
# Define packet parameters setting
preambleLength = 0x0C
headerType = LoRa.LORA_HEADER_EXPLICIT
payloadLength = 0x40
crcType = LoRa.LORA_CRC_ON
invertIq = LoRa.LORA_IQ_STANDARD
# SyncWord setting
sw = [0x34, 0x44]
# Receive flag
received = False
intSet = False

def setting() :
    print("-- SETTING FUNCTION --")
    # SPI bus setting
    LoRa.setSpi(busId, csId)
    # GPIO Pins setting
    print("Setting pins")
    LoRa.setPins(LoRaIO.RPi_GPIO, resetPin, busyPin, irqPin, txenPin, rxenPin)
    # Reset RF module by setting resetPin to LOW and begin SPI communication
    LoRa.reset()
    # Optionally configure TCXO or XTAL used in RF module
    print("Set RF module to use TCXO as clock reference")
    LoRa._setDio3AsTcxoCtrl(dio3Voltage, tcxoDelay)
    # Set to standby mode and set packet type to LoRa
    print("Going to standby mode")
    LoRa._setStandby(LoRa.STANDBY_RC)
    print("Set packet type to LoRa")
    LoRa._setPacketType(LoRa.LORA_MODEM)
    # Set frequency to selected frequency (rfFrequency = rfFreq * 32000000 / 2 ^ 25)
    print(f"Set frequency to {rfFrequency/1000000} Mhz")
    rfFreq = int(rfFrequency * 33554432 / 32000000)
    LoRa._setRfFrequency(rfFreq)
    # Set rx gain to selected gain
    if gain == LoRa.RX_GAIN_BOOSTED : gainMsg = "boosted gain"
    else : gainMsg = "power saving gain"
    print(f"Set RX gain to {gainMsg} dBm")
    LoRa._writeRegister(LoRa.REG_RX_GAIN, [gain], 1)
    # Configure modulation parameter with predefined spreading factor, bandwidth, coding rate, and low data rate optimize setting
    print("Set modulation with predefined parameters")
    LoRa._setModulationParamsLoRa(sf, bw, cr, ldro)
    # Configure packet parameter with predefined preamble length, header mode type, payload length, crc type, and invert iq option
    LoRa._setPacketParamsLoRa(preambleLength, headerType, payloadLength, crcType, invertIq)
    # Set predefined syncronize word
    print("Set syncWord to 0x{0:02X}{1:02X}".format(sw[0], sw[1]))
    LoRa._writeRegister(LoRa.REG_LORA_SYNC_WORD_MSB, sw, 2)

def checkReceiveDone(channel) :
    global received
    received = True

def receive(message, timeout) :
    print("\n-- RECEIVE FUNCTION --")
    # Activate interrupt when receive done on DIO1
    print("Set RX done, timeout, and CRC error IRQ on DIO1")
    mask = LoRa.IRQ_RX_DONE | LoRa.IRQ_TIMEOUT | LoRa.IRQ_CRC_ERR
    LoRa._setDioIrqParams(mask, mask, LoRa.IRQ_NONE, LoRa.IRQ_NONE)
    # Calculate timeout (timeout duration = timeout * 15.625 us)
    tOut = timeout * 64
    # Set RF module to RX mode to receive message
    print("Receiving LoRa packet within predefined timeout")
    LoRa._setRx(tOut)
    # Attach irqPin to DIO1
    print(f"Attach interrupt on pin {irqPin} (irqPin)")
    global intSet
    if not intSet :
        GPIO.setup(irqPin, GPIO.IN)
        GPIO.add_event_detect(irqPin, GPIO.RISING, callback=checkReceiveDone, bouncetime=100)
        intSet = True
    # Set rxen and txen pin state for receiving packet
    GPIO.output(rxenPin, GPIO.HIGH)
    GPIO.output(txenPin, GPIO.LOW)
    # Wait for RX done interrupt
    print("Wait for RX done interrupt")
    global received
    while not received : pass
    # Clear transmit interrupt flag
    received = False
    # Clear the interrupt status
    irqStat = []
    LoRa._getIrqStatus(irqStat)
    print("Clear IRQ status")
    LoRa._clearIrqStatus(irqStat[0])
    GPIO.output(rxenPin, GPIO.LOW)
    # Exit function if timeout reached
    if irqStat[0] & LoRa.IRQ_TIMEOUT :
        return irqStat[0]
    print("Packet received!")
    # Get last received length and buffer base address
    print("Get received length and buffer base address")
    payloadLengthRx = []; rxStartBufferPointer = []
    LoRa._getRxBufferStatus(payloadLengthRx, rxStartBufferPointer)
    # Get and display packet status
    print("Get received packet status")
    rssiPkt = []; snrPkt = []; signalRssiPkt = []
    LoRa._getPacketStatus(rssiPkt, snrPkt, signalRssiPkt)
    rssi = rssiPkt[0] / -2
    snr = snrPkt[0] / 4
    signalRssi = signalRssiPkt[0] / -2
    print(f"Packet status: RSSI = {rssi} | SNR = {snr} | signalRSSI = {signalRssi}")
    # Read message from buffer
    print("Read message from buffer")
    LoRa._readBuffer(rxStartBufferPointer[0], message, payloadLengthRx[0])
    # Return interrupt status
    return irqStat[0]

# Seetings for LoRa communication
setting()

while True :

    # Receive message
    messageList = []
    timeout = 5000
    status = receive(messageList, timeout)
    message = ""
    for i in range(len(messageList)) : message += chr(messageList[i])
    # Display status if error happen
    if status & LoRa.IRQ_RX_DONE :
        print(f"Message: \'{message}\'")
    elif status & LoRa.IRQ_TIMEOUT :
        print("Receive timeout")
    elif status & LoRa.IRQ_CRC_ERR :
        print("CRC error")
