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

# PA and TX power setting
paConfig = 0xC0
txPower = 17
paPin = LoRa.TX_POWER_PA_BOOST

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

# Transmit flag
transmitted = False

def checkTransmitDone(channel) :
    global transmitted
    transmitted = True

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

    # Set tx power to selected TX power
    print(f"Set TX power to {txPower} dbm")
    outputPower = txPower - 2
    paDac = 0x04
    if txPower > 17 : paDac = 0x07
    LoRa.writeRegister(LoRa.REG_PA_DAC, paDac)
    LoRa.writeRegister(LoRa.REG_PA_CONFIG, paConfig | outputPower)

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

def transmitFunction(message: list) :

    print("\n-- TRANSMIT FUNCTION --")

    # Configure FIFO address and address pointer for TX operation
    LoRa.writeRegister(LoRa.REG_FIFO_TX_BASE_ADDR, 0x00)
    reg = LoRa.readRegister(LoRa.REG_FIFO_TX_BASE_ADDR)
    LoRa.writeRegister(LoRa.REG_FIFO_ADDR_PTR, 0x00)
    reg_ = LoRa.readRegister(LoRa.REG_FIFO_ADDR_PTR)
    print("Set FIFO TX base address and address pointer (0x{:02X} | 0x{:02X})".format(reg, reg_))

    # Write message to FIFO
    msg = ""
    for i in range(len(message)):
        LoRa.writeRegister(LoRa.REG_FIFO, message[i])
        msg += chr(message[i])
    print(f"Write message \"{msg}\" in buffer")
    print(f"Message in bytes: {message}")

    # Set payload length
    LoRa.writeRegister(LoRa.REG_PAYLOAD_LENGTH, len(message))
    reg = LoRa.readRegister(LoRa.REG_PAYLOAD_LENGTH)
    print(f"Set payload length same as message length ({reg})")

    # Activate interrupt when transmit done on DIO0
    print("Set TX done and timeout IRQ on DIO0")
    LoRa.writeRegister(LoRa.REG_DIO_MAPPING_1, LoRa.DIO0_TX_DONE)
    # Attach irqPin to DIO0
    print(f"Attach interrupt on IRQ pin")
    monitoring = Thread(target=irq.monitor, args=(checkTransmitDone, 0.1))
    monitoring.start()

    # Set txen and rxen pin state for transmitting packet
    if txen != None and rxen != None :
        txen.output(LoRaGpio.HIGH)
        rxen.output(LoRaGpio.LOW)

    # Transmit message
    print("Transmitting message...")
    LoRa.writeRegister(LoRa.REG_OP_MODE, LoRa.LONG_RANGE_MODE | LoRa.MODE_TX)
    tStart = time.time()

    # Wait for TX done interrupt and calcualte transmit time
    print("Wait for TX done interrupt")
    global transmitted
    while not transmitted : pass
    monitoring.join()
    tTrans = (time.time() - tStart) * 1000
    # Clear transmit interrupt flag
    transmitted = False
    print("Transmit done")

    # Display transmit time
    print(f"Transmit time = {tTrans} ms")

    # Show IRQ flag and Clear interrupt
    irqStat = LoRa.readRegister(LoRa.REG_IRQ_FLAGS)
    LoRa.writeRegister(LoRa.REG_IRQ_FLAGS, 0xFF)
    print("Clear IRQ status")
    if txen != None :
        txen.output(LoRaGpio.LOW)

    # return interrupt status
    return irqStat

# Settings for LoRa communication
settingFunction()

while True :

    # Message to transmit
    message = "HeLoRa World\0"
    msg = []
    for i in range(len(message)) :
        msg.append(ord(message[i]))

    # Transmit message
    status = transmitFunction(msg)

    # Don't load RF module with continous transmit
    time.sleep(10)
