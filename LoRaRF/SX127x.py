from .base import BaseLoRa
import spidev
import RPi.GPIO
import time

spi = spidev.SpiDev()
gpio = RPi.GPIO
gpio.setmode(RPi.GPIO.BCM)
gpio.setwarnings(False)

class SX127x(BaseLoRa) :
    """Class for SX1276/77/78/79 LoRa chipsets from Semtech"""

    # SX127X LoRa Mode Register Map
    REG_FIFO                               = 0x00
    REG_OP_MODE                            = 0x01
    REG_FRF_MSB                            = 0x06
    REG_FRF_MID                            = 0x07
    REG_FRF_LSB                            = 0x08
    REG_PA_CONFIG                          = 0x09
    REG_PA_RAMP                            = 0x0A
    REG_OCP                                = 0x0B
    REG_LNA                                = 0x0C
    REG_FIFO_ADDR_PTR                      = 0x0D
    REG_FIFO_TX_BASE_ADDR                  = 0x0E
    REG_FIFO_RX_BASE_ADDR                  = 0x0F
    REG_FIFO_RX_CURRENT_ADDR               = 0x10
    REG_IRQ_FLAGS_MASK                     = 0x11
    REG_IRQ_FLAGS                          = 0x12
    REG_RX_NB_BYTES                        = 0x13
    REG_RX_HEADR_CNT_VALUE_MSB             = 0x14
    REG_RX_HEADR_CNT_VALUE_LSB             = 0x15
    REG_RX_PKT_CNT_VALUE_MSB               = 0x16
    REG_RX_PKT_CNT_VALUE_LSB               = 0x17
    REG_MODEB_STAT                         = 0x18
    REG_PKT_SNR_VALUE                      = 0x19
    REG_PKT_RSSI_VALUE                     = 0x1A
    REG_RSSI_VALUE                         = 0x1B
    REG_HOP_CHANNEL                        = 0x1C
    REG_MODEM_CONFIG_1                     = 0x1D
    REG_MODEM_CONFIG_2                     = 0x1E
    REG_SYMB_TIMEOUT_LSB                   = 0x1F
    REG_PREAMBLE_MSB                       = 0x20
    REG_PREAMBLE_LSB                       = 0x21
    REG_PAYLOAD_LENGTH                     = 0x22
    REG_MAX_PAYLOAD_LENGTH                 = 0x23
    REG_HOP_PERIOD                         = 0x24
    REG_FIFO_RX_BYTE_ADDR                  = 0x25
    REG_MODEM_CONFIG_3                     = 0x26
    REG_FREQ_ERROR_MSB                     = 0x28
    REG_FREQ_ERROR_MID                     = 0x29
    REG_FREQ_ERROR_LSB                     = 0x2A
    REG_RSSI_WIDEBAND                      = 0x2C
    REG_FREQ1                              = 0x2F
    REG_FREQ2                              = 0x30
    REG_DETECTION_OPTIMIZE                 = 0x31
    REG_INVERTIQ                           = 0x33
    REG_HIGH_BW_OPTIMIZE_1                 = 0x36
    REG_DETECTION_THRESHOLD                = 0x37
    REG_SYNC_WORD                          = 0x39
    REG_HIGH_BW_OPTIMIZE_2                 = 0x3A
    REG_INVERTIQ2                          = 0x3B
    REG_DIO_MAPPING_1                      = 0x40
    REG_DIO_MAPPING_2                      = 0x41
    REG_VERSION                            = 0x42
    REG_TCXO                               = 0x4B
    REG_PA_DAC                             = 0x4D
    REG_FORMER_TEMP                        = 0x5B
    REG_AGC_REF                            = 0x61
    REG_AGC_THRESH_1                       = 0x62
    REG_AGC_THRESH_2                       = 0x63
    REG_AGC_THRESH_3                       = 0x64
    REG_PLL                                = 0x70

    # Modem options
    FSK_MODEM                              = 0x00 # GFSK packet type
    LORA_MODEM                             = 0x01 # LoRa packet type
    OOK_MODEM                              = 0x02 # OOK packet type

    # Long range mode and modulation type
    LONG_RANGE_MODE                        = 0x80 # GFSK packet type
    MODULATION_OOK                         = 0x20 # OOK packet type
    MODULATION_FSK                         = 0x00 # LoRa packet type

    # Devices modes
    MODE_SLEEP                             = 0x00 # sleep
    MODE_STDBY                             = 0x01 # standby
    MODE_TX                                = 0x03 # transmit
    MODE_RX_CONTINUOUS                     = 0x05 # continuous receive
    MODE_RX_SINGLE                         = 0x06 # single receive
    MODE_CAD                               = 0x07 # channel activity detection (CAD)

    # Rx operation mode
    RX_SINGLE                              = 0x000000    # Rx timeout duration: no timeout (Rx single mode)
    RX_CONTINUOUS                          = 0xFFFFFF    #                      infinite (Rx continuous mode)

    # TX power options
    TX_POWER_RFO                           = 0x00        # output power is limited to +14 dBm
    TX_POWER_PA_BOOST                      = 0x80        # output power is limited to +20 dBm

    # RX gain options
    RX_GAIN_POWER_SAVING                   = 0x00        # gain used in Rx mode: power saving gain (default)
    RX_GAIN_BOOSTED                        = 0x01        #                       boosted gain
    RX_GAIN_AUTO                           = 0x00        # option enable auto gain controller (AGC)

    # Header type
    HEADER_EXPLICIT                        = 0x00        # explicit header mode
    HEADER_IMPLICIT                        = 0x01        # implicit header mode

    # LoRa syncword
    SYNCWORD_LORAWAN                       = 0x34        # reserved LoRaWAN syncword

    # Oscillator options
    OSC_CRYSTAL                            = 0x00        # crystal oscillator with external crystal
    OSC_TCXO                               = 0x10        # external clipped sine TCXO AC-connected to XTA pin

    # DIO mapping
    DIO0_RX_DONE                           = 0x00        # set DIO0 interrupt for: RX done
    DIO0_TX_DONE                           = 0x40        #                         TX done
    DIO0_CAD_DONE                          = 0x80        #                         CAD done

    # IRQ flags
    IRQ_CAD_DETECTED                       = 0x01        # Valid Lora signal detected during CAD operation
    IRQ_FHSS_CHANGE                        = 0x02        # FHSS change channel interrupt
    IRQ_CAD_DONE                           = 0x04        # channel activity detection finished
    IRQ_TX_DONE                            = 0x08        # packet transmission completed
    IRQ_HEADER_VALID                       = 0x10        # valid LoRa header received
    IRQ_CRC_ERR                            = 0x20        # wrong CRC received
    IRQ_RX_DONE                            = 0x40        # packet received
    IRQ_RX_TIMEOUT                         = 0x80        # waiting packet received timeout

    # Rssi offset
    RSSI_OFFSET_LF                         = 164         # low band frequency RSSI offset
    RSSI_OFFSET_HF                         = 157         # high band frequency RSSI offset
    RSSI_OFFSET                            = 139         # frequency RSSI offset for SX1272
    BAND_THRESHOLD                         = 525E6       # threshold between low and high band frequency

    # TX and RX operation status
    STATUS_DEFAULT                         = 0           # default status (false)
    STATUS_TX_WAIT                         = 1
    STATUS_TX_TIMEOUT                      = 2
    STATUS_TX_DONE                         = 3
    STATUS_RX_WAIT                         = 4
    STATUS_RX_CONTINUOUS                   = 5
    STATUS_RX_TIMEOUT                      = 6
    STATUS_RX_DONE                         = 7
    STATUS_HEADER_ERR                      = 8
    STATUS_CRC_ERR                         = 9
    STATUS_CAD_WAIT                        = 10
    STATUS_CAD_DETECTED                    = 11
    STATUS_CAD_DONE                        = 12

    # SPI and GPIO pin setting
    _bus = 0
    _cs = 0
    _reset = 22
    _irq = -1
    _txen = -1
    _rxen = -1
    _spiSpeed = 7800000
    _txState = gpio.LOW
    _rxState = gpio.LOW

    # LoRa setting
    _dio = 1
    _modem = LORA_MODEM
    _frequency = 915000000
    _sf = 7
    _bw = 125000
    _cr = 5
    _ldro = False
    _headerType = HEADER_EXPLICIT
    _preambleLength = 12
    _payloadLength = 32
    _crcType = False
    _invertIq = False

    # Operation properties
    _payloadTxRx = 32
    _statusWait = STATUS_DEFAULT
    _statusIrq = STATUS_DEFAULT
    _transmitTime = 0.0

    # callback functions
    _onTransmit = None
    _onReceive = None

### COMMON OPERATIONAL METHODS ###

    def begin(self, bus: int = _bus, cs: int = _cs, reset: int = _reset, irq: int = _irq, txen: int = _txen, rxen: int = _rxen) -> bool :

        # set spi and gpio pins
        self.setSpi(bus, cs)
        self.setPins(reset, irq, txen, rxen)

        # perform device reset
        if not self.reset() :
            return False

        # set modem to LoRa
        self.setModem(self.LORA_MODEM)
        # set tx power and rx gain
        self.setTxPower(17, self.TX_POWER_PA_BOOST)
        self.setRxGain(self.RX_GAIN_BOOSTED, self.RX_GAIN_AUTO)
        return True

    def end(self) :

        self.sleep()
        spi.close()
        gpio.cleanup()

    def reset(self) :

        # put reset pin to low then wait 5 ms
        gpio.output(self._reset, gpio.LOW)
        time.sleep(0.001)
        gpio.output(self._reset, gpio.HIGH)
        time.sleep(0.005)
        # wait until device connected, return false when device too long to respond
        t = time.time()
        version = 0x00
        while version != 0x12 and version !=0x22 :
            version = self.readRegister(self.REG_VERSION)
            if time.time() - t > 1 :
                return False
        return True

    def sleep(self) :

        # put device in sleep mode
        self.writeRegister(self.REG_OP_MODE, self._modem | self.MODE_SLEEP)

    def wake(self) :

        # wake device by put in standby mode
        self.writeRegister(self.REG_OP_MODE, self._modem | self.MODE_STDBY)

    def standby(self) :

        self.writeRegister(self.REG_OP_MODE, self._modem | self.MODE_STDBY)

### HARDWARE CONFIGURATION METHODS ###

    def setSpi(self, bus: int, cs: int, speed: int = _spiSpeed) :

        self._bus = bus
        self._cs = cs
        self._spiSpeed = speed
        # open spi line and set bus id, chip select, and spi speed
        spi.open(bus, cs)
        spi.max_speed_hz = speed
        spi.lsbfirst = False
        spi.mode = 0

    def setPins(self, reset: int, irq: int = -1, txen: int = -1, rxen: int = -1) :

        self._reset = reset
        self._irq = irq
        self._txen = txen
        self._rxen = rxen
        # set pins as input or output
        gpio.setup(reset, gpio.OUT)
        if irq != -1 : gpio.setup(irq, gpio.IN)
        if txen != -1 : gpio.setup(txen, gpio.OUT)
        if rxen != -1 : gpio.setup(rxen, gpio.OUT)

    def setCurrentProtection(self, current: int) :

        # calculate ocp trim
        ocpTrim = 27
        if current <= 120 :
            ocpTrim = int((current - 45) / 5)
        elif current <= 240 :
            ocpTrim = int((current + 30) / 10)
        # set over current protection config
        self.writeRegister(self.REG_OCP, 0x20 | ocpTrim)

    def setOscillator(self, option: int) :
        
        cfg = self.OSC_CRYSTAL
        if option == self.OSC_TCXO :
            cfg = self.OSC_TCXO
        self.writeRegister(self.REG_TCXO, cfg)

### MODEM, MODULATION PARAMETER, AND PACKET PARAMETER SETUP METHODS ###

    def setModem(self, modem: int) :

        if modem == self.LORA_MODEM :
            self._modem = self.LONG_RANGE_MODE
        elif modem == self.FSK_MODEM :
            self._modem = self.MODULATION_FSK
        else :
            self._modem = self.MODULATION_OOK
        self.sleep()
        self.writeRegister(self.REG_OP_MODE, self._modem | self.MODE_STDBY)

    def setFrequency(self, frequency: int) :

        self._frequency = frequency
        # calculate frequency
        frf = int((frequency << 19) / 32000000)
        self.writeRegister(self.REG_FRF_MSB, (frf >> 16) & 0xFF)
        self.writeRegister(self.REG_FRF_MID, (frf >> 8) & 0xFF)
        self.writeRegister(self.REG_FRF_LSB, frf & 0xFF)

    def setTxPower(self, txPower: int, paPin: int) :

        # maximum TX power is 20 dBm and 14 dBm for RFO pin
        if txPower > 20 : txPower = 20
        elif txPower > 14 and paPin == self.TX_POWER_RFO : txPower = 14

        paConfig = 0x00
        outputPower = 0x00
        if paPin == self.TX_POWER_RFO :
            # txPower = Pmax - (15 - outputPower)
            if txPower == 14 :
                # max power (Pmax) 14.4 dBm
                paConfig = 0x60
                outputPower = txPower + 1
            else :
                # max power (Pmax) 13.2 dBm
                paConfig = 0x40
                outputPower = txPower + 2
        else :
            paConfig = 0xC0
            paDac = 0x04
            # txPower = 17 - (15 - outputPower)
            if txPower > 17 :
                outputPower = 15
                paDac = 0x07
                self.setCurrentProtection(100)  # max current 100 mA
            else :
                if txPower < 2 : txPower = 2
                outputPower = txPower - 2
                self.setCurrentProtection(140)  # max current 140 mA
            # enable or disable +20 dBm option on PA_BOOST pin
            self.writeRegister(self.REG_PA_DAC, paDac)

        # set PA config
        self.writeRegister(self.REG_PA_CONFIG, paConfig | outputPower)

    def setRxGain(self, boost: int, level: int) :

        # valid RX gain level 0 - 6 (0 -> AGC on)
        if level > 6 : level = 6
        # boost LNA and automatic gain controller config
        LnaBoostHf = 0x00
        if boost : LnaBoostHf = 0x03
        AgcOn = 0x00
        if level == self.RX_GAIN_AUTO : AgcOn = 0x01

        # set gain and boost LNA config
        self.writeRegister(self.REG_LNA, LnaBoostHf | (level << 5))
        # enable or disable AGC
        self.writeBits(self.REG_MODEM_CONFIG_3, AgcOn, 2, 1)

    def setLoRaModulation(self, sf: int, bw: int, cr: int, ldro: bool = False) :

        self.setSpreadingFactor(sf)
        self.setBandwidth(bw)
        self.setCodeRate(cr)
        self.setLdroEnable(ldro)

    def setLoRaPacket(self, headerType: int, preambleLength: int, payloadLength: int, crcType: bool = False, invertIq: bool = False) :

        self.setHeaderType(headerType)
        self.setPreambleLength(preambleLength)
        self.setPayloadLength(payloadLength)
        self.setCrcEnable(crcType)
        # self.setInvertIq(invertIq)

    def setSpreadingFactor(self, sf: int) :

        self._sf = sf
        # valid spreading factor is 6 - 12
        if sf < 6 : sf = 6
        elif sf > 12 : sf = 12
        # set appropriate signal detection optimize and threshold
        optimize = 0x03
        threshold = 0x0A
        if sf == 6 : 
            optimize = 0x05
            threshold = 0x0C
        self.writeRegister(self.REG_DETECTION_OPTIMIZE, optimize)
        self.writeRegister(self.REG_DETECTION_THRESHOLD, threshold)
        # set spreading factor config
        self.writeBits(self.REG_MODEM_CONFIG_2, sf, 4, 4)

    def setBandwidth(self, bw: int) :

        self._bw = bw
        bwCfg = 9                       # 500 kHz
        if bw < 9100 : bwCfg = 0        # 7.8 kHz
        elif bw < 13000 : bwCfg = 1     # 10.4 kHz
        elif bw < 18200 : bwCfg = 2     # 15.6 kHz
        elif bw < 26000 : bwCfg = 3     # 20.8 kHz
        elif bw < 36500 : bwCfg = 4     # 31.25 kHz
        elif bw < 52100 : bwCfg = 5     # 41.7 kHz
        elif bw < 93800 : bwCfg = 6     # 62.5 kHz
        elif bw < 187500 : bwCfg = 7    # 125 kHz
        elif bw < 375000 : bwCfg = 8    # 250 kHz
        self.writeBits(self.REG_MODEM_CONFIG_1, bwCfg, 4, 4)

    def setCodeRate(self, cr: int) :

        # valid code rate denominator is 5 - 8
        if cr < 5 : cr = 4
        elif cr > 8 : cr = 8
        crCfg = cr - 4
        self.writeBits(self.REG_MODEM_CONFIG_1, crCfg, 1, 3)

    def setLdroEnable(self, ldro: bool) :

        ldroCfg = 0x00
        if ldro : ldroCfg = 0x01
        self.writeBits(self.REG_MODEM_CONFIG_3, ldroCfg, 3, 1)

    def setHeaderType(self, headerType: int) :

        self._headerType = headerType
        headerTypeCfg = self.HEADER_EXPLICIT
        if headerType == self.HEADER_IMPLICIT : headerTypeCfg = self.HEADER_IMPLICIT
        self.writeBits(self.REG_MODEM_CONFIG_1, headerTypeCfg, 0, 1)

    def setPreambleLength(self, preambleLength: int) :

        self.writeRegister(self.REG_PREAMBLE_MSB, (preambleLength >> 8) & 0xFF)
        self.writeRegister(self.REG_PREAMBLE_LSB, preambleLength & 0xFF)

    def setPayloadLength(self, payloadLength: int) :

        self._payloadLength = payloadLength
        self.writeRegister(self.REG_PAYLOAD_LENGTH, payloadLength)

    def setCrcEnable(self, crcType: bool) :

        crcTypeCfg = 0x00
        if crcType : crcTypeCfg = 0x01
        self.writeBits(self.REG_MODEM_CONFIG_2, crcTypeCfg, 2, 1)

    def setInvertIq(self, invertIq: bool) :

        invertIqCfg1 = 0x00
        invertIqCfg2 = 0x1D
        if invertIq :
            invertIqCfg1 = 0x01
            invertIqCfg2 = 0x19
        self.writeBits(self.REG_INVERTIQ, invertIqCfg1, 0, 1)
        self.writeBits(self.REG_INVERTIQ, invertIqCfg1, 6, 1)
        self.writeRegister(self.REG_INVERTIQ2, invertIqCfg2)

    def setSyncWord(self, syncWord: int) :

        sw = syncWord
        # keep compatibility between 1 and 2 bytes synchronize word
        if syncWord > 0xFF :
            sw = ((syncWord >> 8) & 0xF0) | (syncWord & 0x0F)
        self.writeRegister(self.REG_SYNC_WORD, sw)

### TRANSMIT RELATED METHODS ###

    def beginPacket(self) :

        # reset TX buffer base address, FIFO address pointer and payload length
        self.writeRegister(self.REG_FIFO_TX_BASE_ADDR, self.readRegister(self.REG_FIFO_ADDR_PTR))
        self._payloadTxRx = 0

        # save current txen and rxen pin state and set txen pin to high and rxen pin to low
        if self._txen != -1 and self._rxen != -1 :
            self._txState = gpio.input(self._txen)
            self._rxState = gpio.input(self._rxen)
            gpio.output(self._txen, gpio.HIGH)
            gpio.output(self._rxen, gpio.LOW)

    def endPacket(self, timeout: int = 0) -> bool :

        # skip to enter TX mode when previous TX operation incomplete
        if self.readRegister(self.REG_OP_MODE) & 0x07 == self.MODE_TX :
            return False

        # clear IRQ flag from last TX or RX operation
        self.writeRegister(self.REG_IRQ_FLAGS, 0xFF)

        # set packet payload length
        self.writeRegister(self.REG_PAYLOAD_LENGTH, self._payloadTxRx)

        # set status to TX wait
        self._statusWait = self.STATUS_TX_WAIT
        self._statusIrq = 0x00

        # set device to transmit mode
        self.writeRegister(self.REG_OP_MODE, self._modem | self.MODE_TX)
        self._transmitTime = time.time()

        # set TX done interrupt on DIO0 and attach TX interrupt handler
        if self._irq != -1 :
            self.writeRegister(self.REG_DIO_MAPPING_1, self.DIO0_TX_DONE)
            gpio.remove_event_detect(self._irq)
            gpio.add_event_detect(self._irq, gpio.RISING, callback=self._interruptTx, bouncetime=10)
        return True

    def write(self, data, length: int = 0) :

        # prepare data and data length to be transmitted
        if type(data) is list or type(data) is tuple :
            if length == 0 or length > len(data) : length = len(data)
        elif type(data) is int or type(data) is float :
            length = 1
            data = (int(data),)
        else :
            raise TypeError("input data must be list, tuple, integer or float")

        # write data to buffer and update payload
        for i in range(length) :
            self.writeRegister(self.REG_FIFO, int(data[i]))
        self._payloadTxRx += length

    def put(self, data) :

        # prepare bytes or bytearray to be transmitted
        if type(data) is bytes or type(data) is bytearray :
            dataList = tuple(data)
            length = len(dataList)
        else : raise TypeError("input data must be bytes or bytearray")

        # write data to buffer and update payload
        for i in range(length) :
            self.writeRegister(self.REG_FIFO, int(data[i]))
        self._payloadTxRx += length

### RECEIVE RELATED METHODS ###

    def request(self, timeout: int = 0) -> bool :

        # skip to enter RX mode when previous RX operation incomplete
        rxMode = self.readRegister(self.REG_OP_MODE) & 0x07
        if rxMode == self.MODE_RX_SINGLE or rxMode == self.MODE_RX_CONTINUOUS:
            return False

        # clear IRQ flag from last TX or RX operation
        self.writeRegister(self.REG_IRQ_FLAGS, 0xFF)

        # save current txen and rxen pin state and set txen pin to low and rxen pin to high
        if self._txen != -1 and self._rxen != -1 :
            self._txState = gpio.input(self._txen)
            self._rxState = gpio.input(self._rxen)
            gpio.output(self._txen, gpio.LOW)
            gpio.output(self._rxen, gpio.HIGH)

        # set status to RX wait
        self._statusWait = self.STATUS_RX_WAIT
        self._statusIrq = 0x00

        # select RX mode to RX continuous mode for RX single and continuos operation
        rxMode = self.MODE_RX_CONTINUOUS
        if timeout == self.RX_CONTINUOUS :
            self._statusWait = self.STATUS_RX_CONTINUOUS
        elif timeout > 0 :
            # Select RX mode to single mode for RX operation with timeout
            rxMode = self.MODE_RX_SINGLE
            # calculate and set symbol timeout
            symbTimeout = int(timeout * self._bw / 1000) >> self._sf   # devided by 1000, ms to s
            self.writeBits(self.REG_MODEM_CONFIG_2, (symbTimeout >> 8) & 0x03, 0, 2)
            self.writeRegister(self.REG_SYMB_TIMEOUT_LSB, symbTimeout & 0xFF)

        # set device to receive mode
        self.writeRegister(self.REG_OP_MODE, self._modem | rxMode)

        # set RX done interrupt on DIO0 and attach RX interrupt handler
        if self._irq != -1 :
            self.writeRegister(self.REG_DIO_MAPPING_1, self.DIO0_RX_DONE)
            gpio.remove_event_detect(self._irq)
            if timeout == self.RX_CONTINUOUS :
                gpio.add_event_detect(self._irq, gpio.RISING, callback=self._interruptRxContinuous, bouncetime=10)
            else :
                gpio.add_event_detect(self._irq, gpio.RISING, callback=self._interruptRx, bouncetime=10)
        return True

    def available(self) :

        # get size of package still available to read
        return self._payloadTxRx

    def read(self, length: int = 0) :

        # single or multiple bytes read
        single = False
        if length == 0 :
            length = 1
            single = True

        # calculate actual read length and remaining payload length
        if self._payloadTxRx > length :
            self._payloadTxRx -= length
        else :
            self._payloadTxRx = 0
        # read multiple bytes of received package in FIFO buffer
        data = tuple()
        for i in range(length) :
            data = data + (self.readRegister(self.REG_FIFO),)

        # return single byte or tuple
        if single : return data[0]
        else : return data

    def get(self, length: int = 1) -> bytes :

        # calculate actual read length and remaining payload length
        if self._payloadTxRx > length :
            self._payloadTxRx -= length
        else :
            self._payloadTxRx = 0
        # read data from FIFO buffer and update payload length
        data = tuple()
        for i in range(length) :
            data = data + (self.readRegister(self.REG_FIFO),)

        # return array of bytes
        return bytes(data)

    def purge(self, length: int = 0) :

        # subtract or reset received payload length
        if (self._payloadTxRx > length) and length :
            self._payloadTxRx = self._payloadTxRx - length
        else :
            self._payloadTxRx = 0

### WAIT, OPERATION STATUS, AND PACKET STATUS METHODS ###

    def wait(self, timeout: int = 0) -> bool :

        # immediately return when currently not waiting transmit or receive process
        if self._statusIrq : return True

        # wait transmit or receive process finish by checking interrupt status or IRQ status
        irqFlag = 0x00
        irqFlagMask = self.IRQ_RX_DONE | self.IRQ_RX_TIMEOUT | self.IRQ_CRC_ERR
        if self._statusWait == self.STATUS_TX_WAIT :
            irqFlagMask = self.IRQ_TX_DONE
        t = time.time()
        while not (irqFlag & irqFlagMask) and self._statusIrq == 0x00 :
            # only check IRQ status register for non interrupt operation
            if self._irq == -1 : irqFlag = self.readRegister(self.REG_IRQ_FLAGS)
            # return when timeout reached
            if time.time() - t > timeout and timeout > 0 : return False

        if self._statusIrq :
            # immediately return when interrupt signal hit
            return True

        elif self._statusWait == self.STATUS_TX_WAIT :
            # calculate transmit time and set back txen and rxen pin to previous state
            self._transmitTime = time.time() - self._transmitTime
            if self._txen != -1 and self._rxen != -1 :
                gpio.output(self._txen, self._txState)
                gpio.output(self._rxen, self._rxState)

        elif self._statusWait == self.STATUS_RX_WAIT :
            # terminate receive mode by setting mode to standby
            self.standby()
            # set pointer to RX buffer base address and get packet payload length
            self.writeRegister(self.REG_FIFO_ADDR_PTR, self.readRegister(self.REG_FIFO_RX_CURRENT_ADDR))
            self._payloadTxRx = self.readRegister(self.REG_RX_NB_BYTES)
            # set back txen and rxen pin to previous state
            if self._txen != -1 and self._rxen != -1 :
                gpio.output(self._txen, self._txState)
                gpio.output(self._rxen, self._rxState)

        elif self._statusWait == self.STATUS_RX_CONTINUOUS :
            # set pointer to RX buffer base address and get packet payload length
            self.writeRegister(self.REG_FIFO_ADDR_PTR, self.readRegister(self.REG_FIFO_RX_CURRENT_ADDR))
            self._payloadTxRx = self.readRegister(self.REG_RX_NB_BYTES)
            # clear IRQ flag
            self.writeRegister(self.REG_IRQ_FLAGS, 0xFF)

        # store IRQ status
        self._statusIrq = irqFlag
        return True

    def status(self) -> int :

        # set back status IRQ for RX continuous operation
        statusIrq = self._statusIrq
        if self._statusWait == self.STATUS_RX_CONTINUOUS :
            self._statusIrq = 0x0000

        # get status for transmit and receive operation based on status IRQ
        if statusIrq & self.IRQ_RX_TIMEOUT : return self.STATUS_RX_TIMEOUT
        elif statusIrq & self.IRQ_CRC_ERR : return self.STATUS_CRC_ERR
        elif statusIrq & self.IRQ_TX_DONE : return self.STATUS_TX_DONE
        elif statusIrq & self.IRQ_RX_DONE : return self.STATUS_RX_DONE

        # return TX or RX wait status
        return self._statusWait

    def transmitTime(self) -> float :

        # get transmit time in millisecond (ms)
        return self._transmitTime * 1000

    def dataRate(self) -> float :

        # get data rate last transmitted package in kbps
        return self._payloadTxRx / self._transmitTime

    def packetRssi(self) -> float :

        # get relative signal strength index (RSSI) of last incoming package
        offset = self.RSSI_OFFSET_HF
        if self._frequency < self.BAND_THRESHOLD :
            offset = self.RSSI_OFFSET_LF
        if self.readRegister(self.REG_VERSION) == 0x22 :
            offset = self.RSSI_OFFSET
        return self.readRegister(self.REG_PKT_RSSI_VALUE) - offset

    def rssi(self) -> float :

        offset = self.RSSI_OFFSET_HF
        if self._frequency < self.BAND_THRESHOLD :
            offset = self.RSSI_OFFSET_LF
        if self.readRegister(self.REG_VERSION) == 0x22 :
            offset = self.RSSI_OFFSET
        return self.readRegister(self.REG_RSSI_VALUE) - offset

    def snr(self) -> float :

        # get signal to noise ratio (SNR) of last incoming package
        return self.readRegister(self.REG_PKT_SNR_VALUE) / 4.0

### INTERRUPT HANDLER METHODS ###

    def _interruptTx(self, channel) :

        # calculate transmit time
        self._transmitTime = time.time() - self._transmitTime

        # store IRQ status as TX done
        self._statusIrq = self.IRQ_TX_DONE

        # set back txen and rxen pin to previous state
        if self._txen != -1 and self._rxen != -1 :
            gpio.output(self._txen, self._txState)
            gpio.output(self._rxen, self._rxState)

        # call onTransmit function
        if callable(self._onTransmit) :
            self._onTransmit()

    def _interruptRx(self, channel) :

        # store IRQ status
        self._statusIrq = self.readRegister(self.REG_IRQ_FLAGS)
        # set IRQ status to RX done when interrupt occured before register updated
        if not self._statusIrq & 0xF0 :
            self._statusIrq = self.IRQ_RX_DONE

        # terminate receive mode by setting mode to standby
        self.writeBits(self.REG_OP_MODE, self.MODE_STDBY, 0, 3)

        # set back txen and rxen pin to previous state
        if self._txen != -1 and self._rxen != -1 :
            gpio.output(self._txen, self._txState)
            gpio.output(self._rxen, self._rxState)

        # set pointer to RX buffer base address and get packet payload length
        self.writeRegister(self.REG_FIFO_ADDR_PTR, self.readRegister(self.REG_FIFO_RX_CURRENT_ADDR))
        self._payloadTxRx = self.readRegister(self.REG_RX_NB_BYTES)

        # call onReceive function
        if callable(self._onReceive) :
            self._onReceive()

    def _interruptRxContinuous(self, channel) :

        # store IRQ status
        self._statusIrq = self.readRegister(self.REG_IRQ_FLAGS)
        # set IRQ status to RX done when interrupt occured before register updated
        if not self._statusIrq & 0xF0 :
            self._statusIrq = self.IRQ_RX_DONE

        # clear IRQ flag from last TX or RX operation
        self.writeRegister(self.REG_IRQ_FLAGS, 0xFF)

        # set pointer to RX buffer base address and get packet payload length
        self.writeRegister(self.REG_FIFO_ADDR_PTR, self.readRegister(self.REG_FIFO_RX_CURRENT_ADDR))
        self._payloadTxRx = self.readRegister(self.REG_RX_NB_BYTES)

        # call onReceive function
        if callable(self._onReceive) :
            self._onReceive()

    def onTransmit(self, callback) :

        # register onTransmit function to call every transmit done
        self._onTransmit = callback

    def onReceive(self, callback) :

        # register onReceive function to call every receive done
        self._onReceive = callback

### SX127X DRIVER: UTILITIES ###

    def writeBits(self, address: int, data: int, position: int, length: int) :

        read = self._transfer(address & 0x7F, 0x00)
        mask = (0xFF >> (8 - length)) << position
        write = (data << position) | (read & ~mask)
        self._transfer(address | 0x80, write)

    def writeRegister(self, address: int, data: int) :

        self._transfer(address | 0x80, data)

    def readRegister(self, address: int) ->int:

        return self._transfer(address & 0x7F, 0x00)

    def _transfer(self, address: int, data: int) ->int:

        buf = [address, data]
        feedback = spi.xfer2(buf)
        if (len(feedback) == 2) :
            return int(feedback[1])
        return -1
