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
# PA and TX power setting
paDutyCycle = 0x02
hpMax = 0x03
deviceSel = 0x00
power = 0x16
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
# Transmit flag
transmitted = False
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
    # Set tx power to selected TX power
    print(f"Set TX power to {power} dBm")
    LoRa._setPaConfig(paDutyCycle, hpMax, deviceSel, 0x01)
    LoRa._setTxParams(power, LoRa.PA_RAMP_200U)
    # Configure modulation parameter with predefined spreading factor, bandwidth, coding rate, and low data rate optimize setting
    print("Set modulation with predefined parameters")
    LoRa._setModulationParamsLoRa(sf, bw, cr, ldro)
    # Configure packet parameter with predefined preamble length, header mode type, payload length, crc type, and invert iq option
    LoRa._setPacketParamsLoRa(preambleLength, headerType, payloadLength, crcType, invertIq)
    # Set predefined syncronize word
    print("Set syncWord to 0x{0:02X}{1:02X}".format(sw[0], sw[1]))
    LoRa._writeRegister(LoRa.REG_LORA_SYNC_WORD_MSB, sw, 2)

def checkTransmitDone(channel) :
    global transmitted
    transmitted = True

def transmit(message, timeout) :
    print("\n-- TRANSMIT FUNCTION --")
    # Set buffer base address
    print("Mark a pointer in buffer for transmit message")
    LoRa._setBufferBaseAddress(0x00, 0x80)
    # Write the message to buffer
    length = len(message)
    messageString = ""
    for i in range(len(message)) : messageString += chr(message[i])
    print(f"Write message \"{messageString}\" in buffer")
    LoRa._writeBuffer(0x00, message, length)
    # Set payload length same as message length
    print(f"Set payload length same as message length ({length})")
    LoRa._setPacketParamsLoRa(preambleLength, headerType, length, crcType, invertIq)
    # Activate interrupt when transmit done on DIO1
    print("Set TX done and timeout IRQ on DIO1")
    mask = LoRa.IRQ_TX_DONE | LoRa.IRQ_TIMEOUT
    LoRa._setDioIrqParams(mask, mask, LoRa.IRQ_NONE, LoRa.IRQ_NONE)
    # Calculate timeout (timeout duration = timeout * 15.625 us)
    tOut = timeout * 64
    # Set RF module to TX mode to transmit message
    print("Transmitting LoRa packet")
    LoRa._setTx(tOut)
    tStart = time.time()
    tTrans = 0
    # Attach irqPin to DIO1
    print(f"Attach interrupt on pin {irqPin} (irqPin)")
    global intSet
    if not intSet :
        GPIO.setup(irqPin, GPIO.IN)
        GPIO.add_event_detect(irqPin, GPIO.RISING, callback=checkTransmitDone, bouncetime=100)
        intSet = True
    # Set rxen and txen pin state for transmitting packet
    GPIO.output(rxenPin, GPIO.LOW)
    GPIO.output(txenPin, GPIO.HIGH)
    # Wait for TX done interrupt and calcualte transmit time
    print("Wait for RX done interrupt")
    global transmitted
    while not transmitted : pass
    tTrans = time.time() - tStart
    # Clear transmit interrupt flag
    transmitted = False
    print("Packet transmitted!")
    # Display transmit time
    print(f"Transmit time = {tTrans*1000} ms")
    # Clear the interrupt status
    irqStat = []
    LoRa._getIrqStatus(irqStat)
    print("Clear IRQ status")
    LoRa._clearIrqStatus(irqStat[0])
    GPIO.output(txenPin, GPIO.LOW)
    # Return interrupt status
    return irqStat[0]

# Seetings for LoRa communication
setting()

while True :

    # Message to transmit
    message = "HeLoRa World!\0"
    messageList = list(message)
    for i in range(len(messageList)) : messageList[i] = ord(messageList[i])
    # Transmit message
    timeout = 1000
    status = transmit(messageList, timeout)
    # Display status if error happen
    if status & LoRa.IRQ_TIMEOUT :
        print("Transmit timeout")
    # Don't load RF module with continous transmit
    time.sleep(10)
