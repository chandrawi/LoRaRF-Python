import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX127x

LoRa = SX127x()

# Frequency, modulation param, packet param, and synchronize word setting
frequency = 915000000
sf = 7
bw = 7                              # 125 khz
cr = 1                              # 5/4
headerType = LoRa.HEADER_EXPLICIT
preambleLen = 12
crcEn = 1
syncword = 0x12

# Received message buffer
message = ""
fifoAddress = 0

# Set SPI and GPIO pins
print("Setting SPI and GPIO pins")
LoRa.setSpi(1, 0, 16000000)
LoRa.setPins(22, -1)

# Perform device reset
if LoRa.reset():
    print("Reset device")
else:
    print("Something wrong, can't reset device")
    sys.exit()

# Set modem type to LoRa and put device to standby mode
LoRa.writeRegister(LoRa.REG_OP_MODE, LoRa.MODE_SLEEP)
LoRa.writeRegister(LoRa.REG_OP_MODE, LoRa.LONG_RANGE_MODE)
LoRa.writeRegister(LoRa.REG_OP_MODE, LoRa.LONG_RANGE_MODE | LoRa.MODE_STDBY)
print("Set modem type to LoRa")

# Set frequency
frf = int((frequency << 19) / 32000000)
LoRa.writeRegister(LoRa.REG_FRF_MSB, frf >> 16)
LoRa.writeRegister(LoRa.REG_FRF_MID, frf >> 8)
LoRa.writeRegister(LoRa.REG_FRF_LSB, frf)
print(f"Set frequency to {frequency / 1000000} Mhz")

# Set modulation param and packet param
LoRa.writeBits(LoRa.REG_MODEM_CONFIG_2, sf, 4, 4)
LoRa.writeBits(LoRa.REG_MODEM_CONFIG_1, bw, 4, 4)
LoRa.writeBits(LoRa.REG_MODEM_CONFIG_1, cr, 1, 3)
LoRa.writeBits(LoRa.REG_MODEM_CONFIG_1, headerType, 0, 1)
LoRa.writeBits(LoRa.REG_MODEM_CONFIG_2, crcEn, 2, 1)
LoRa.writeRegister(LoRa.REG_PREAMBLE_MSB, preambleLen >> 8)
LoRa.writeRegister(LoRa.REG_PREAMBLE_LSB, preambleLen)

# Show modulation param and packet param registers
reg = LoRa.readRegister(LoRa.REG_MODEM_CONFIG_1)
print("Modem config 1 : 0x{:02X}".format(reg))
reg = LoRa.readRegister(LoRa.REG_MODEM_CONFIG_2)
print("Modem config 2 : 0x{:02X}".format(reg))
reg = LoRa.readRegister(LoRa.REG_PREAMBLE_MSB)
print("Preamble MSB : 0x{:02X}".format(reg))
reg = LoRa.readRegister(LoRa.REG_PREAMBLE_LSB)
print("Preamble LSB : 0x{:02X}".format(reg))

# Set synchronize word
LoRa.writeRegister(LoRa.REG_SYNC_WORD, syncword)
reg = LoRa.readRegister(LoRa.REG_SYNC_WORD)
print("Synchronize word : 0x{:02X}".format(reg))

# Receive message
print("Receiving message...")
LoRa.writeRegister(LoRa.REG_OP_MODE, LoRa.LORA_MODEM | LoRa.MODE_RX_CONTINUOUS)
reg = 0x00
while ((reg & (LoRa.IRQ_RX_DONE | LoRa.IRQ_CRC_ERR)) == 0):
    reg = LoRa.readRegister(LoRa.REG_IRQ_FLAGS)
print("Receive done")

# Set mode to standby to end RX mode
LoRa.writeRegister(LoRa.REG_OP_MODE, LoRa.LORA_MODEM | LoRa.MODE_STDBY)
print("Standby mode")

# Show IRQ flag and Clear interrupt
print("IRQ flag status : 0x{:02X}".format(reg))
LoRa.writeRegister(LoRa.REG_IRQ_FLAGS, LoRa.IRQ_TX_DONE)
print("Clear IRQ")

# Get FIFO address of received message and configure address pointer
reg = LoRa.readRegister(LoRa.REG_FIFO_RX_CURRENT_ADDR)
print("FIFO RX base address : 0x{:02X}".format(reg))
LoRa.writeRegister(LoRa.REG_FIFO_ADDR_PTR, reg)
print("Set FIFO address pointer to FIFO RX base address")

# Get payload length
payloadLength = LoRa.readRegister(LoRa.REG_RX_NB_BYTES)
print(f"Message length : {payloadLength}")

# Show received message
messageBytes = []
for i in range(payloadLength):
    reg = LoRa.readRegister(LoRa.REG_FIFO)
    messageBytes.append(reg)
    message += chr(reg)
print(f"Message in bytes: {messageBytes}")
print(f"Received transmit: \"{message}\"")
