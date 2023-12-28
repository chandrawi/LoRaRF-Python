import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX127x, LoRaSpi, LoRaGpio
import time
from threading import Thread

# Begin LoRa radio with connected SPI bus and IO pins (cs, reset and busy) on GPIO
spi = LoRaSpi(0, 0)
cs = LoRaGpio(0, 8)
reset = LoRaGpio(0, 24)
irq = LoRaGpio(0, 17)
txen = LoRaGpio(0, 5)
rxen = LoRaGpio(0, 25)
LoRa = SX127x(spi, cs, reset)

# RF frequency setting
frequency = 915000000

# RX gain setting
boost = LoRa.RX_GAIN_POWER_SAVING
level = LoRa.RX_GAIN_AUTO

# Define modulation parameters setting
sf = 7
bw = 7                  # 125 khz
cr = 1                  # 5/4

# Define packet parameters setting
headerType = LoRa.HEADER_EXPLICIT
preambleLen = 12
crcEn = 1

# SyncWord setting
syncword = 0x12

# Receive flag
received = False

def checkReceiveDone(channel) :
    global received
    received = True

def settingFunction() :

    print("-- SETTING FUNCTION --")

    # Reset RF module by setting resetPin to LOW and begin SPI communication
    LoRa.reset()
    version = LoRa.readRegister(LoRa.REG_VERSION)
    if version == 0x12 or version == 0x22 :
        print("Resetting RF module")
    else :
        print("Something wrong, can't reset LoRa radio")

    # Set modem type to LoRa and put device to standby mode
    LoRa.writeRegister(LoRa.REG_OP_MODE, LoRa.MODE_SLEEP)
    LoRa.writeRegister(LoRa.REG_OP_MODE, LoRa.LONG_RANGE_MODE)
    LoRa.writeRegister(LoRa.REG_OP_MODE, LoRa.LONG_RANGE_MODE | LoRa.MODE_STDBY)
    print("Going to standby mode")
    print("Set packet type to LoRa")

    # Set frequency
    frf = int((frequency << 19) / 32000000)
    LoRa.writeRegister(LoRa.REG_FRF_MSB, (frf >> 16) & 0xFF)
    LoRa.writeRegister(LoRa.REG_FRF_MID, (frf >> 8) & 0xFF)
    LoRa.writeRegister(LoRa.REG_FRF_LSB, frf & 0xFF)
    print(f"Set frequency to {frequency / 1000000} Mhz")

    # Set rx gain to selected gain
    gainMsg = "power saving gain"
    if boost == LoRa.RX_GAIN_BOOSTED : gainMsg = "boosted gain"
    print(f"Set RX gain to {gainMsg}")
    LnaBoostHf = 0x00
    AgcOn = 0x00
    if boost : LnaBoostHf = 0x03
    if level == LoRa.RX_GAIN_AUTO : AgcOn = 0x01
    LoRa.writeRegister(LoRa.REG_LNA, LnaBoostHf | (level << 5))
    LoRa.writeBits(LoRa.REG_MODEM_CONFIG_3, AgcOn, 2, 1)

    # Set modulation param and packet param
    print("Set modulation with predefined parameters")
    print("Set packet with predefined parameters")
    LoRa.writeBits(LoRa.REG_MODEM_CONFIG_2, sf, 4, 4)
    LoRa.writeBits(LoRa.REG_MODEM_CONFIG_1, bw, 4, 4)
    LoRa.writeBits(LoRa.REG_MODEM_CONFIG_1, cr, 1, 3)
    LoRa.writeBits(LoRa.REG_MODEM_CONFIG_1, headerType, 0, 1)
    LoRa.writeBits(LoRa.REG_MODEM_CONFIG_2, crcEn, 2, 1)
    LoRa.writeRegister(LoRa.REG_PREAMBLE_MSB, (preambleLen >> 8) & 0xFF)
    LoRa.writeRegister(LoRa.REG_PREAMBLE_LSB, preambleLen & 0xFF)

    # Show modulation param and packet param registers
    reg = LoRa.readRegister(LoRa.REG_MODEM_CONFIG_1)
    print("Modem config 1 : 0x{:02X}".format(reg))
    reg = LoRa.readRegister(LoRa.REG_MODEM_CONFIG_2)
    print("Modem config 2 : 0x{:02X}".format(reg))
    reg = LoRa.readRegister(LoRa.REG_PREAMBLE_MSB)
    reg_ = LoRa.readRegister(LoRa.REG_PREAMBLE_LSB)
    print("Preamble length : 0x{:02X}".format(reg * 256 + reg_))

    # Set synchronize word
    LoRa.writeRegister(LoRa.REG_SYNC_WORD, syncword)
    reg = LoRa.readRegister(LoRa.REG_SYNC_WORD)
    print("Set syncWord to 0x{:02X}".format(reg))

def receiveFunction(message: list) :

    print("\n-- RECEIVE FUNCTION --")

    # Activate interrupt when transmit done on DIO0
    print("Set RX done and timeout IRQ on DIO0")
    LoRa.writeRegister(LoRa.REG_DIO_MAPPING_1, LoRa.DIO0_RX_DONE)
    # Attach irqPin to DIO0
    print(f"Attach interrupt on IRQ pin")
    monitoring = Thread(target=irq.monitor, args=(checkReceiveDone, 0.1))
    monitoring.start()

    # Set txen and rxen pin state for receiving packet
    if txen != None and rxen != None :
        txen.output(LoRaGpio.LOW)
        rxen.output(LoRaGpio.HIGH)

    # Receive message
    print("Receiving message...")
    LoRa.writeRegister(LoRa.REG_OP_MODE, LoRa.LONG_RANGE_MODE | LoRa.MODE_RX_CONTINUOUS)

    # Wait for RX done interrupt
    print("Wait for RX done interrupt")
    global received
    while not received : pass
    monitoring.join()
    # Clear transmit interrupt flag
    received = False
    print("Receive done")

    # Set mode to standby to end RX mode
    LoRa.writeBits(LoRa.REG_OP_MODE, LoRa.MODE_STDBY, 0, 3)
    print("Going to standby mode")

    # Clear the interrupt status
    irqStat = LoRa.readRegister(LoRa.REG_IRQ_FLAGS)
    LoRa.writeRegister(LoRa.REG_IRQ_FLAGS, 0xFF)
    print("Clear IRQ status")
    if rxen != None :
        rxen.output(LoRaGpio.LOW)

    # Get FIFO address of received message and configure address pointer
    reg = LoRa.readRegister(LoRa.REG_FIFO_RX_CURRENT_ADDR)
    LoRa.writeRegister(LoRa.REG_FIFO_ADDR_PTR, reg)
    print("Set FIFO address pointer to FIFO RX base address (0x{:02X})".format(reg))

    # Get payload length
    length = LoRa.readRegister(LoRa.REG_RX_NB_BYTES)
    print(f"Get message length ({length})")

    # Get and display packet status
    print("Get received packet status")
    rssi = (LoRa.readRegister(LoRa.REG_PKT_RSSI_VALUE) - LoRa.RSSI_OFFSET_HF)
    snr = LoRa.readRegister(LoRa.REG_PKT_SNR_VALUE) / 4.0
    print(f"Packet status: RSSI = {rssi} | SNR = {snr}")

    # Read message from buffer
    for i in range(length) :
        reg = LoRa.readRegister(LoRa.REG_FIFO)
        message.append(reg)
    print(f"Message in bytes : {message}")

    # return interrupt status
    return irqStat

# Settings for LoRa communication
settingFunction()

while True :

    # Receive message
    message = []
    status = receiveFunction(message)

    # Display message if receive success or display status if error
    if status & LoRa.IRQ_RX_DONE :
        msg = ""
        for i in range(len(message)) :
            msg += chr(message[i])
        print(f"Message: \'{msg}\'")
    if status & LoRa.IRQ_CRC_ERR :
        print("CRC error")
