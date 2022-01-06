import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x
import RPi.GPIO
import time

busId = 1; csId = 0
resetPin = 22; busyPin = 23; irqPin = 26; txenPin = 5; rxenPin = 25

LoRa = SX126x()
GPIO = RPi.GPIO

# TCXO control setting
dio3Voltage = LoRa.DIO3_OUTPUT_1_8
tcxoDelay = LoRa.TCXO_DELAY_10
# RF frequency setting
rfFrequency = 915000000
# RX gain setting
gain = LoRa.RX_GAIN_POWER_SAVING
# Define modulation parameters setting
sf = 7
bw = LoRa.BW_125000
cr = LoRa.CR_4_5
ldro = LoRa.LDRO_OFF
# Define packet parameters setting
preambleLength = 0x0C
headerType = LoRa.HEADER_EXPLICIT
payloadLength = 0x40
crcType = LoRa.CRC_ON
invertIq = LoRa.IQ_STANDARD
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
    LoRa.setPins(resetPin, busyPin, irqPin, txenPin, rxenPin)
    # Reset RF module by setting resetPin to LOW and begin SPI communication
    LoRa.reset()
    # Optionally configure TCXO or XTAL used in RF module
    print("Set RF module to use TCXO as clock reference")
    LoRa.setDio3AsTcxoCtrl(dio3Voltage, tcxoDelay)
    # Set to standby mode and set packet type to LoRa
    print("Going to standby mode")
    LoRa.setStandby(LoRa.STANDBY_RC)
    print("Set packet type to LoRa")
    LoRa.setPacketType(LoRa.LORA_MODEM)
    # Set frequency to selected frequency (rfFrequency = rfFreq * 32000000 / 2 ^ 25)
    print(f"Set frequency to {rfFrequency/1000000} Mhz")
    rfFreq = int(rfFrequency * 33554432 / 32000000)
    LoRa.setRfFrequency(rfFreq)
    # Set rx gain to selected gain
    if gain == LoRa.RX_GAIN_BOOSTED : gainMsg = "boosted gain"
    else : gainMsg = "power saving gain"
    print(f"Set RX gain to {gainMsg} dBm")
    LoRa.writeRegister(LoRa.REG_RX_GAIN, [gain], 1)
    # Configure modulation parameter with predefined spreading factor, bandwidth, coding rate, and low data rate optimize setting
    print("Set modulation with predefined parameters")
    LoRa.setModulationParamsLoRa(sf, bw, cr, ldro)
    # Configure packet parameter with predefined preamble length, header mode type, payload length, crc type, and invert iq option
    LoRa.setPacketParamsLoRa(preambleLength, headerType, payloadLength, crcType, invertIq)
    # Set predefined syncronize word
    print("Set syncWord to 0x{0:02X}{1:02X}".format(sw[0], sw[1]))
    LoRa.writeRegister(LoRa.REG_LORA_SYNC_WORD_MSB, sw, 2)

def checkReceiveDone(channel) :
    global received
    received = True

def receive(message, timeout) :
    print("\n-- RECEIVE FUNCTION --")
    # Activate interrupt when receive done on DIO1
    print("Set RX done, timeout, and CRC error IRQ on DIO1")
    mask = LoRa.IRQ_RX_DONE | LoRa.IRQ_TIMEOUT | LoRa.IRQ_CRC_ERR
    LoRa.setDioIrqParams(mask, mask, LoRa.IRQ_NONE, LoRa.IRQ_NONE)
    # Calculate timeout (timeout duration = timeout * 15.625 us)
    tOut = timeout * 64
    # Set RF module to RX mode to receive message
    print("Receiving LoRa packet within predefined timeout")
    LoRa.setRx(tOut)
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
    irqStat = LoRa.getIrqStatus()
    print("Clear IRQ status")
    LoRa.clearIrqStatus(irqStat)
    GPIO.output(rxenPin, GPIO.LOW)
    # Exit function if timeout reached
    if irqStat & LoRa.IRQ_TIMEOUT :
        return irqStat
    print("Packet received!")
    # Get last received length and buffer base address
    print("Get received length and buffer base address")
    payloadLengthRx = 0; rxStartBufferPointer = 0
    (payloadLengthRx, rxStartBufferPointer) = LoRa.getRxBufferStatus()
    # Get and display packet status
    print("Get received packet status")
    rssiPkt = 0; snrPkt = 0; signalRssiPkt = 0
    (rssiPkt, snrPkt, signalRssiPkt) = LoRa.getPacketStatus()
    rssi = rssiPkt / -2
    snr = snrPkt / 4
    signalRssi = signalRssiPkt / -2
    print(f"Packet status: RSSI = {rssi} | SNR = {snr} | signalRSSI = {signalRssi}")
    # Read message from buffer
    print("Read message from buffer")
    buffer = LoRa.readBuffer(rxStartBufferPointer, payloadLengthRx)
    for buf in buffer : message.append(buf)
    # Return interrupt status
    return irqStat

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
