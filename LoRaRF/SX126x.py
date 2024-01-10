from .base import LoRaSpi, LoRaGpio, BaseLoRa
from typing import Optional
import time
from threading import Thread

class SX126x(BaseLoRa) :
    """Class for SX1261/62/68 and LLCC68 LoRa chipsets from Semtech"""

    # SX126X register map
    REG_FSK_WHITENING_INITIAL_MSB          = 0x06B8
    REG_FSK_CRC_INITIAL_MSB                = 0x06BC
    REG_FSK_SYNC_WORD_0                    = 0x06C0
    REG_FSK_NODE_ADDRESS                   = 0x06CD
    REG_IQ_POLARITY_SETUP                  = 0x0736
    REG_LORA_SYNC_WORD_MSB                 = 0x0740
    REG_TX_MODULATION                      = 0x0889
    REG_RX_GAIN                            = 0x08AC
    REG_TX_CLAMP_CONFIG                    = 0x08D8
    REG_OCP_CONFIGURATION                  = 0x08E7
    REG_RTC_CONTROL                        = 0x0902
    REG_XTA_TRIM                           = 0x0911
    REG_XTB_TRIM                           = 0x0912
    REG_EVENT_MASK                         = 0x0944

    # SetSleep
    SLEEP_COLD_START                       = 0x00        # sleep mode: cold start, configuration is lost (default)
    SLEEP_WARM_START                       = 0x04        #             warm start, configuration is retained
    SLEEP_COLD_START_RTC                   = 0x01        #             cold start and wake on RTC timeout
    SLEEP_WARM_START_RTC                   = 0x05        #             warm start and wake on RTC timeout

    # SetStandby
    STANDBY_RC                             = 0x00        # standby mode: using 13 MHz RC oscillator
    STANDBY_XOSC                           = 0x01        #               using 32 MHz crystal oscillator

    # SetTx
    TX_SINGLE                              = 0x000000    # Tx timeout duration: no timeout (Rx single mode)

    # SetRx
    RX_SINGLE                              = 0x000000    # Rx timeout duration: no timeout (Rx single mode)
    RX_CONTINUOUS                          = 0xFFFFFF    #                      infinite (Rx continuous mode)

    # SetRegulatorMode
    REGULATOR_LDO                          = 0x00        # set regulator mode: LDO (default)
    REGULATOR_DC_DC                        = 0x01        #                     DC-DC

    # CalibrateImage
    CAL_IMG_430                            = 0x6B        # ISM band: 430-440 Mhz
    CAL_IMG_440                            = 0x6F
    CAL_IMG_470                            = 0x75        #           470-510 Mhz
    CAL_IMG_510                            = 0x81
    CAL_IMG_779                            = 0xC1        #           779-787 Mhz
    CAL_IMG_787                            = 0xC5
    CAL_IMG_863                            = 0xD7        #           863-870 Mhz
    CAL_IMG_870                            = 0xDB
    CAL_IMG_902                            = 0xE1        #           902-928 Mhz
    CAL_IMG_928                            = 0xE9

    # SetPaConfig
    TX_POWER_SX1261                        = 0x01        # device version for TX power: SX1261
    TX_POWER_SX1262                        = 0x02        #                              SX1262
    TX_POWER_SX1268                        = 0x08        #                              SX1268

    # SetRxTxFallbackMode
    FALLBACK_FS                            = 0x40        # after Rx/Tx go to: FS mode
    FALLBACK_STDBY_XOSC                    = 0x30        #                    standby mode with crystal oscillator
    FALLBACK_STDBY_RC                      = 0x20        #                    standby mode with RC oscillator (default)

    # SetDioIrqParams
    IRQ_TX_DONE                            = 0x0001      # packet transmission completed
    IRQ_RX_DONE                            = 0x0002      # packet received
    IRQ_PREAMBLE_DETECTED                  = 0x0004      # preamble detected
    IRQ_SYNC_WORD_VALID                    = 0x0008      # valid sync word detected
    IRQ_HEADER_VALID                       = 0x0010      # valid LoRa header received
    IRQ_HEADER_ERR                         = 0x0020      # LoRa header CRC error
    IRQ_CRC_ERR                            = 0x0040      # wrong CRC received
    IRQ_CAD_DONE                           = 0x0080      # channel activity detection finished
    IRQ_CAD_DETECTED                       = 0x0100      # channel activity detected
    IRQ_TIMEOUT                            = 0x0200      # Rx or Tx timeout
    IRQ_ALL                                = 0x03FF      # all interrupts
    IRQ_NONE                               = 0x0000      # no interrupts

    # SetDio2AsRfSwitch
    DIO2_AS_IRQ                            = 0x00        # DIO2 configuration: IRQ
    DIO2_AS_RF_SWITCH                      = 0x01        #                     RF switch control

    # SetDio3AsTcxoCtrl
    DIO3_OUTPUT_1_6                        = 0x00        # DIO3 voltage output for TCXO: 1.6 V
    DIO3_OUTPUT_1_7                        = 0x01        #                               1.7 V
    DIO3_OUTPUT_1_8                        = 0x02        #                               1.8 V
    DIO3_OUTPUT_2_2                        = 0x03        #                               2.2 V
    DIO3_OUTPUT_2_4                        = 0x04        #                               2.4 V
    DIO3_OUTPUT_2_7                        = 0x05        #                               2.7 V
    DIO3_OUTPUT_3_0                        = 0x06        #                               3.0 V
    DIO3_OUTPUT_3_3                        = 0x07        #                               3.3 V
    TCXO_DELAY_2_5                         = 0x0140      # TCXO delay time: 2.5 ms
    TCXO_DELAY_5                           = 0x0280      #                  5 ms
    TCXO_DELAY_10                          = 0x0560      #                  10 ms

    # SetRfFrequency
    RF_FREQUENCY_XTAL                      = 32000000    # XTAL frequency used for RF frequency calculation
    RF_FREQUENCY_NOM                       = 33554432    # used for RF frequency calculation

    # SetPacketType
    FSK_MODEM                              = 0x00        # GFSK packet type
    LORA_MODEM                             = 0x01        # LoRa packet type

    # SetTxParams
    PA_RAMP_10U                            = 0x00        # ramp time: 10 us
    PA_RAMP_20U                            = 0x01        #            20 us
    PA_RAMP_40U                            = 0x02        #            40 us
    PA_RAMP_80U                            = 0x03        #            80 us
    PA_RAMP_200U                           = 0x04        #            200 us
    PA_RAMP_800U                           = 0x05        #            800 us
    PA_RAMP_1700U                          = 0x06        #            1700 us
    PA_RAMP_3400U                          = 0x07        #            3400 us

    # SetModulationParams
    BW_7800                                = 0x00        # LoRa bandwidth: 7.8 kHz
    BW_10400                               = 0x08        #                 10.4 kHz
    BW_15600                               = 0x01        #                 15.6 kHz
    BW_20800                               = 0x09        #                 20.8 kHz
    BW_31250                               = 0x02        #                 31.25 kHz
    BW_41700                               = 0x0A        #                 41.7 kHz
    BW_62500                               = 0x03        #                 62.5 kHz
    BW_125000                              = 0x04        #                 125.0 kHz
    BW_250000                              = 0x05        #                 250.0 kHz
    BW_500000                              = 0x06        #                 500.0 kHz
    CR_4_4                                 = 0x00        # LoRa coding rate: 4/4 (no coding rate)
    CR_4_5                                 = 0x01        #                   4/5
    CR_4_6                                 = 0x01        #                   4/6
    CR_4_7                                 = 0x01        #                   4/7
    CR_4_8                                 = 0x01        #                   4/8
    LDRO_OFF                               = 0x00        # LoRa low data rate optimization: disabled
    LDRO_ON                                = 0x01        #                                  enabled

    # SetModulationParams for FSK packet type
    PULSE_NO_FILTER                        = 0x00        # FSK pulse shape: no filter applied
    PULSE_GAUSSIAN_BT_0_3                  = 0x08        #                  Gaussian BT 0.3
    PULSE_GAUSSIAN_BT_0_5                  = 0x09        #                  Gaussian BT 0.5
    PULSE_GAUSSIAN_BT_0_7                  = 0x0A        #                  Gaussian BT 0.7
    PULSE_GAUSSIAN_BT_1                    = 0x0B        #                  Gaussian BT 1
    BW_4800                                = 0x1F        # FSK bandwidth: 4.8 kHz DSB
    BW_5800                                = 0x17        #                5.8 kHz DSB
    BW_7300                                = 0x0F        #                7.3 kHz DSB
    BW_9700                                = 0x1E        #                9.7 kHz DSB
    BW_11700                               = 0x16        #                11.7 kHz DSB
    BW_14600                               = 0x0E        #                14.6 kHz DSB
    BW_19500                               = 0x1D        #                19.5 kHz DSB
    BW_23400                               = 0x15        #                23.4 kHz DSB
    BW_29300                               = 0x0D        #                29.3 kHz DSB
    BW_39000                               = 0x1C        #                39 kHz DSB
    BW_46900                               = 0x14        #                46.9 kHz DSB
    BW_58600                               = 0x0C        #                58.6 kHz DSB
    BW_78200                               = 0x1B        #                78.2 kHz DSB
    BW_93800                               = 0x13        #                93.8 kHz DSB
    BW_117300                              = 0x0B        #                117.3 kHz DSB
    BW_156200                              = 0x1A        #                156.2 kHz DSB
    BW_187200                              = 0x12        #                187.2 kHz DSB
    BW_234300                              = 0x0A        #                232.3 kHz DSB
    BW_312000                              = 0x19        #                312 kHz DSB
    BW_373600                              = 0x11        #                373.6 kHz DSB
    BW_467000                              = 0x09        #                476 kHz DSB

    # SetPacketParams
    HEADER_EXPLICIT                        = 0x00        # LoRa header mode: explicit
    HEADER_IMPLICIT                        = 0x01        #                   implicit
    CRC_OFF                                = 0x00        # LoRa CRC mode: disabled
    CRC_ON                                 = 0x01        #                enabled
    IQ_STANDARD                            = 0x00        # LoRa IQ setup: standard
    IQ_INVERTED                            = 0x01        #                inverted

    # SetPacketParams for FSK packet type
    PREAMBLE_DET_LEN_OFF                   = 0x00        # FSK preamble detector length: off
    PREAMBLE_DET_LEN_8                     = 0x04        #                               8-bit
    PREAMBLE_DET_LEN_16                    = 0x05        #                               16-bit
    PREAMBLE_DET_LEN_24                    = 0x06        #                               24-bit
    PREAMBLE_DET_LEN_32                    = 0x07        #                               32-bit
    ADDR_COMP_OFF                          = 0x00        # FSK address filtering: off
    ADDR_COMP_NODE                         = 0x01        #                        filtering on node address
    ADDR_COMP_ALL                          = 0x02        #                        filtering on node and broadcast address
    PACKET_KNOWN                           = 0x00        # FSK packet type: the packet length known on both side
    PACKET_VARIABLE                        = 0x01        #                  the packet length on variable size
    CRC_0                                  = 0x01        # FSK CRC type: no CRC
    CRC_1                                  = 0x00        #               CRC computed on 1 byte
    CRC_2                                  = 0x02        #               CRC computed on 2 byte
    CRC_1_INV                              = 0x04        #               CRC computed on 1 byte and inverted
    CRC_2_INV                              = 0x06        #               CRC computed on 2 byte and inverted
    WHITENING_OFF                          = 0x00        # FSK whitening: no encoding
    WHITENING_ON                           = 0x01        #                whitening enable

    # SetCadParams
    CAD_ON_1_SYMB                          = 0x00        # number of symbols used for CAD: 1
    CAD_ON_2_SYMB                          = 0x01        #                                 2
    CAD_ON_4_SYMB                          = 0x02        #                                 4
    CAD_ON_8_SYMB                          = 0x03        #                                 8
    CAD_ON_16_SYMB                         = 0x04        #                                 16
    CAD_EXIT_STDBY                         = 0x00        # after CAD is done, always exit to STDBY_RC mode
    CAD_EXIT_RX                            = 0x01        # after CAD is done, exit to Rx mode if activity is detected

    # GetStatus
    STATUS_DATA_AVAILABLE                  = 0x04        # command status: packet received and data can be retrieved
    STATUS_CMD_TIMEOUT                     = 0x06        #                 SPI command timed out
    STATUS_CMD_ERROR                       = 0x08        #                 invalid SPI command
    STATUS_CMD_FAILED                      = 0x0A        #                 SPI command failed to execute
    STATUS_CMD_TX_DONE                     = 0x0C        #                 packet transmission done
    STATUS_MODE_STDBY_RC                   = 0x20        # current chip mode: STDBY_RC
    STATUS_MODE_STDBY_XOSC                 = 0x30        #                    STDBY_XOSC
    STATUS_MODE_FS                         = 0x40        #                    FS
    STATUS_MODE_RX                         = 0x50        #                    RX
    STATUS_MODE_TX                         = 0x60        #                    TX

    # GetDeviceErrors
    RC64K_CALIB_ERR                        = 0x0001      # device errors: RC64K calibration failed
    RC13M_CALIB_ERR                        = 0x0002      #                RC13M calibration failed
    PLL_CALIB_ERR                          = 0x0004      #                PLL calibration failed
    ADC_CALIB_ERR                          = 0x0008      #                ADC calibration failed
    IMG_CALIB_ERR                          = 0x0010      #                image calibration failed
    XOSC_START_ERR                         = 0x0020      #                crystal oscillator failed to start
    PLL_LOCK_ERR                           = 0x0040      #                PLL failed to lock
    PA_RAMP_ERR                            = 0x0100      #                PA ramping failed

    # LoraSyncWord
    LORA_SYNC_WORD_PUBLIC                  = 0x3444      # LoRa SyncWord for public network
    LORA_SYNC_WORD_PRIVATE                 = 0x0741      # LoRa SyncWord for private network (default)

    # RxGain
    RX_GAIN_POWER_SAVING                   = 0x00        # gain used in Rx mode: power saving gain (default)
    RX_GAIN_BOOSTED                        = 0x01        #                       boosted gain
    POWER_SAVING_GAIN                      = 0x94        # power saving gain register value
    BOOSTED_GAIN                           = 0x96        # boosted gain register value

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
    _busyTimeout = 5000
    _irqTimeout = 10000
    _txState = LoRaGpio.LOW
    _rxState = LoRaGpio.LOW

    # LoRa setting
    _dio = 1
    _modem = LORA_MODEM
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
    _monitoring = None
    _bufferIndex = 0
    _payloadTxRx = 32
    _statusWait = STATUS_DEFAULT
    _statusIrq = STATUS_DEFAULT
    _transmitTime = 0.0

    # callback functions
    _onTransmit = None
    _onReceive = None

    def __init__(self, spi: LoRaSpi, cs: LoRaGpio, reset: LoRaGpio, busy: LoRaGpio, irq: Optional[LoRaGpio]=None, txen: Optional[LoRaGpio]=None, rxen: Optional[LoRaGpio]=None):

        self._spi = spi
        self._cs = cs
        self._reset = reset
        self._busy = busy
        self._irq = irq
        self._txen = txen
        self._rxen = rxen

### COMMON OPERATIONAL METHODS ###

    def begin(self) -> bool :

        # perform device reset
        self.reset()

        # check if device connect and set modem to LoRa
        self.setStandby(self.STANDBY_RC)
        if self.getMode() != self.STATUS_MODE_STDBY_RC :
            return False
        self.setPacketType(self.LORA_MODEM)
        self._fixResistanceAntenna()
        return True

    def end(self) :

        self.sleep(self.SLEEP_COLD_START)

    def reset(self) -> bool :

        # put reset pin to low then wait busy pin to low
        self._reset.output(LoRaGpio.LOW)
        time.sleep(0.001)
        self._reset.output(LoRaGpio.HIGH)
        return not self.busyCheck()

    def sleep(self, option = SLEEP_WARM_START) :

        # put device in sleep mode, wait for 500 us to enter sleep mode
        self.standby()
        self.setSleep(option)
        time.sleep(0.0005)

    def wake(self) :

        # wake device by set wake pin (cs pin) to low before spi transaction and put device in standby mode
        self._cs.output(LoRaGpio.LOW)
        time.sleep(0.0005)
        self.setStandby(self.STANDBY_RC)
        self._fixResistanceAntenna()

    def standby(self, option = STANDBY_RC) :

        self.setStandby(option)

    def busyCheck(self, timeout: int = _busyTimeout) :

        # wait for busy pin to LOW or timeout reached
        t = time.time()
        while self._busy.input() == LoRaGpio.HIGH :
            if (time.time() - t) > (timeout / 1000) : return True
        return False

    def setFallbackMode(self, fallbackMode) :

        self.setRxTxFallbackMode(fallbackMode)

    def getMode(self) -> int :

        return self.getStatus() & 0x70

### HARDWARE CONFIGURATION METHODS ###

    def setSpiSpeed(self, speed: int) :

        self._spi.speed = speed

    def setRfIrqPin(self, dioPinSelect: int) :

        if dioPinSelect == 2 or dioPinSelect == 3 : self._dio = dioPinSelect
        else : self._dio = 1

    def setDio2RfSwitch(self, enable: bool = True) :

        if enable : self.setDio2AsRfSwitchCtrl(self.DIO2_AS_RF_SWITCH)
        else : self.setDio2AsRfSwitchCtrl(self.DIO2_AS_IRQ)

    def setDio3TcxoCtrl(self, tcxoVoltage, delayTime) :

        self.setDio3AsTcxoCtrl(tcxoVoltage, delayTime)
        self.setStandby(self.STANDBY_RC)
        self.calibrate(0xFF)

    def setXtalCap(self, xtalA, xtalB) :

        self.setStandby(self.STANDBY_XOSC)
        self.writeRegister(self.REG_XTA_TRIM, (xtalA, xtalB), 2)
        self.setStandby(self.STANDBY_RC)
        self.calibrate(0xFF)

    def setRegulator(self, regMode) :

        self.setRegulatorMode(regMode)

    def setCurrentProtection(self, level) :
        #avoid wrap-around of OCP register, which has 6bits
        if level > 63 : level = 63
        self.writeRegister(self.REG_OCP_CONFIGURATION, (level,), 1)

### MODEM, MODULATION PARAMETER, AND PACKET PARAMETER SETUP METHODS ###

    def setModem(self, modem) :

        self._modem = modem
        self.setStandby(self.STANDBY_RC)
        self.setPacketType(modem)

    def setFrequency(self, frequency: int) :

        # perform image calibration before set frequency
        if frequency < 446000000 :
            calFreqMin = self.CAL_IMG_430
            calFreqMax = self.CAL_IMG_440
        elif frequency < 734000000 :
            calFreqMin = self.CAL_IMG_470
            calFreqMax = self.CAL_IMG_510
        elif frequency < 828000000 :
            calFreqMin = self.CAL_IMG_779
            calFreqMax = self.CAL_IMG_787
        elif frequency < 877000000 :
            calFreqMin = self.CAL_IMG_863
            calFreqMax = self.CAL_IMG_870
        else :
            calFreqMin = self.CAL_IMG_902
            calFreqMax = self.CAL_IMG_928
        self.calibrateImage(calFreqMin, calFreqMax)

        # calculate frequency and set frequency setting
        rfFreq = int(frequency * 33554432 / 32000000)
        self.setRfFrequency(rfFreq)

    def setTxPower(self, txPower: int, version = TX_POWER_SX1262) :

        #  maximum TX power is 22 dBm and 15 dBm for SX1261
        if txPower > 22 : txPower = 22
        elif txPower > 15 and version == self.TX_POWER_SX1261 : txPower = 15

        paDutyCycle = 0x00
        hpMax = 0x00
        deviceSel = 0x00
        power = 0x0E
        if version == self.TX_POWER_SX1261 : deviceSel = 0x01
        # set parameters for PA config and TX params configuration
        if txPower == 22 :
            paDutyCycle = 0x04
            hpMax = 0x07
            power = 0x16
        elif txPower >= 20 :
            paDutyCycle = 0x03
            hpMax = 0x05
            power = 0x16
        elif txPower >= 17 :
            paDutyCycle = 0x02
            hpMax = 0x03
            power = 0x16
        elif txPower >= 14 and version == self.TX_POWER_SX1261 :
            paDutyCycle = 0x04
            hpMax = 0x00
            power = 0x0E
        elif txPower >= 14 and version == self.TX_POWER_SX1262 :
            paDutyCycle = 0x02
            hpMax = 0x02
            power = 0x16
        elif txPower >= 14 and version == self.TX_POWER_SX1268 :
            paDutyCycle = 0x04
            hpMax = 0x06
            power = 0x0F
        elif txPower >= 10 and version == self.TX_POWER_SX1261 :
            paDutyCycle = 0x01
            hpMax = 0x00
            power = 0x0D
        elif txPower >= 10 and version == self.TX_POWER_SX1268 :
            paDutyCycle = 0x00
            hpMax = 0x03
            power = 0x0F
        else : return

        # set power amplifier and TX power configuration
        self.setPaConfig(paDutyCycle, hpMax, deviceSel, 0x01)
        self.setTxParams(power, self.PA_RAMP_800U)

    def setRxGain(self, rxGain) :

        # set power saving or boosted gain in register
        gain = self.POWER_SAVING_GAIN
        if rxGain == self.RX_GAIN_BOOSTED :
            gain = self.BOOSTED_GAIN
            # set certain register to retain configuration after wake from sleep mode
            self.writeRegister(self.REG_RX_GAIN, (gain,), 1)
            self.writeRegister(0x029F, (0x01, 0x08, 0xAC), 3)
        else :
            self.writeRegister(self.REG_RX_GAIN, (gain,), 1)

    def setLoRaModulation(self, sf: int, bw: int, cr: int, ldro: bool = False) :

        self._sf = sf
        self._bw = bw
        self._cr = cr
        self._ldro = ldro

        # valid spreading factor is between 5 and 12
        if sf > 12 : sf = 12
        elif sf < 5 : sf = 5
        # select bandwidth options
        if bw < 9100 : bw = self.BW_7800
        elif bw < 13000 : bw = self.BW_10400
        elif bw < 18200 : bw = self.BW_15600
        elif bw < 26000 : bw = self.BW_20800
        elif bw < 36500 : bw = self.BW_31250
        elif bw < 52100 : bw = self.BW_41700
        elif bw < 93800 : bw = self.BW_62500
        elif bw < 187500 : bw = self.BW_125000
        elif bw < 375000 : bw = self.BW_250000
        else : bw = self.BW_500000
        # valid code rate denominator is between 4 and 8
        cr = cr - 4
        if cr > 4 : cr = 0
        # set low data rate option
        if ldro : ldro = self.LDRO_ON
        else : ldro = self.LDRO_OFF

        self.setModulationParamsLoRa(sf, bw, cr, ldro)

    def setLoRaPacket(self, headerType, preambleLength: int, payloadLength: int, crcType: bool = False, invertIq: bool = False) :

        self._headerType = headerType
        self._preambleLength = preambleLength
        self._payloadLength = payloadLength
        self._crcType = crcType
        self._invertIq = invertIq

        # filter valid header type config
        if headerType != self.HEADER_IMPLICIT : headerType = self.HEADER_EXPLICIT
        # set CRC and invert IQ option
        if crcType : crcType = self.CRC_ON
        else : crcType = self.CRC_OFF
        if invertIq : invertIq = self.IQ_INVERTED
        else : invertIq = self.IQ_STANDARD

        self.setPacketParamsLoRa(preambleLength, headerType, payloadLength, crcType, invertIq)
        self._fixInvertedIq(invertIq)

    def setSpreadingFactor(self, sf: int) :

        self.setLoRaModulation(sf, self._bw, self._cr, self._ldro)

    def setBandwidth(self, bw: int) :

        self.setLoRaModulation(self._sf, bw, self._cr, self._ldro)

    def setCodeRate(self, cr: int) :

        self.setLoRaModulation(self._sf, self._bw, cr, self._ldro)

    def setLdroEnable(self, ldro: bool = True) :

        self.setLoRaModulation(self._sf, self._bw, self._cr, ldro)

    def setHeaderType(self, headerType) :

        self.setLoRaPacket(self._preambleLength, headerType, self._payloadLength, self._crcType, self._invertIq)

    def setPreambleLength(self, preambleLength: int) :

        self.setLoRaPacket(preambleLength, self._headerType, self._payloadLength, self._crcType, self._invertIq)

    def setPayloadLength(self, payloadLength: int) :

        self.setLoRaPacket(self._preambleLength, self._headerType, payloadLength, self._crcType, self._invertIq)

    def setCrcEnable(self, crcType: bool = True) :

        self.setLoRaPacket(self._preambleLength, self._headerType, self._payloadLength, crcType, self._invertIq)

    def setInvertIq(self, invertIq: bool = True) :

        self.setLoRaPacket(self._preambleLength, self._headerType, self._payloadLength, self._crcType, invertIq)

    def setSyncWord(self, syncWord: int) :

        buf = (
            (syncWord >> 8) & 0xFF,
            syncWord & 0xFF
        )
        if syncWord <= 0xFF :
            buf = (
                (syncWord & 0xF0) | 0x04,
                (syncWord << 4) | 0x04
            )
        self.writeRegister(self.REG_LORA_SYNC_WORD_MSB, buf, 2)

    def setFskModulation(self, br: int, pulseShape: int, bandwidth: int, fdev: int) :

        self.setModulationParamsFsk(br, pulseShape, bandwidth, fdev)

    def setFskPacket(self, preambleLength: int, preambleDetector: int, syncWordLength: int, addrComp: int, packetType: int, payloadLength: int, crcType: int, whitening: int) :

        self.setPacketParamsFsk(preambleLength, preambleDetector, syncWordLength, addrComp, packetType, payloadLength, crcType, whitening)

    def setFskSyncWord(self, sw: tuple, swLen: int) :

        self.writeRegister(self.REG_FSK_SYNC_WORD_0, sw, swLen)

    def setFskAddress(self, nodeAddr: int, broadcastAddr: int) :

        self.writeRegister(self.REG_FSK_NODE_ADDRESS, (nodeAddr, broadcastAddr), 2)

    def setFskCrc(self, crcInit: int, crcPolynom: int) :

        buf = (crcInit >> 8, crcInit & 0xFF, crcPolynom >> 8, crcPolynom & 0xFF)
        self.writeRegister(self.REG_FSK_CRC_INITIAL_MSB, buf, 4)

    def setFskWhitening(self, whitening: int) :

        self.writeRegister(self.REG_FSK_WHITENING_INITIAL_MSB, (whitening >> 8, whitening & 0xFF), 2)

### TRANSMIT RELATED METHODS ###

    def beginPacket(self) :

        # reset payload length and buffer index
        self._payloadTxRx = 0
        self.setBufferBaseAddress(self._bufferIndex, (self._bufferIndex + 0xFF) % 0xFF)

        # save current txen and rxen pin state and set txen pin to high and rxen pin to low
        if self._txen != None and self._rxen != None :
            self._txState = self._txen.input()
            self._rxState = self._rxen.input()
            self._txen.output(LoRaGpio.HIGH)
            self._rxen.output(LoRaGpio.LOW)
        self._fixLoRaBw500(self._bw)

    def endPacket(self, timeout: int = TX_SINGLE) -> bool :

        # skip to enter TX mode when previous TX operation incomplete
        if self.getMode == self.STATUS_MODE_TX : return False

        # clear previous interrupt and set TX done, and TX timeout as interrupt source
        self._irqSetup(self.IRQ_TX_DONE | self.IRQ_TIMEOUT)
        # set packet payload length
        self.setPacketParamsLoRa(self._preambleLength, self._headerType, self._payloadTxRx, self._crcType, self._invertIq)

        # set status to TX wait
        self._statusWait = self.STATUS_TX_WAIT
        self._statusIrq = 0x0000
        # calculate TX timeout config
        txTimeout = timeout << 6
        if txTimeout > 0x00FFFFFF : txTimeout = self.TX_SINGLE

        # set device to transmit mode with configured timeout or single operation
        self.setTx(txTimeout)
        self._transmitTime = time.time()

        # set operation status to wait and attach TX interrupt handler
        if self._irq != None :
            if isinstance(self._monitoring, Thread):
                self._monitoring.join()
            to = self._irqTimeout/1000 if timeout == self.TX_SINGLE else timeout/1000
            self._monitoring = Thread(target=self._irq.monitor, args=(self._interruptTx, to))
            self._monitoring.start()
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
        # write data to buffer and update buffer index and payload
        self.writeBuffer(self._bufferIndex, data, length)
        self._bufferIndex = (self._bufferIndex + length) % 256
        self._payloadTxRx += length

    def put(self, data) :

        # prepare bytes or bytearray to be transmitted
        if type(data) is bytes or type(data) is bytearray :
            dataList = tuple(data)
            length = len(dataList)
        else : raise TypeError("input data must be bytes or bytearray")
        # write data to buffer and update buffer index and payload
        self.writeBuffer(self._bufferIndex, dataList, length)
        self._bufferIndex = (self._bufferIndex + length) % 256
        self._payloadTxRx += length

### RECEIVE RELATED METHODS ###

    def request(self, timeout: int = RX_SINGLE) -> bool :

        # skip to enter RX mode when previous RX operation incomplete
        if self.getMode() == self.STATUS_MODE_RX : return False

        # clear previous interrupt and set RX done, RX timeout, header error, and CRC error as interrupt source
        self._irqSetup(self.IRQ_RX_DONE | self.IRQ_TIMEOUT | self.IRQ_HEADER_ERR | self.IRQ_CRC_ERR)

        # set status to RX wait or RX continuous wait
        self._statusWait = self.STATUS_RX_WAIT
        self._statusIrq = 0x0000
        # calculate RX timeout config
        rxTimeout = timeout << 6
        if rxTimeout > 0x00FFFFFF : rxTimeout = self.RX_SINGLE
        if timeout == self.RX_CONTINUOUS :
            rxTimeout = self.RX_CONTINUOUS
            self._statusWait = self.STATUS_RX_CONTINUOUS

        # save current txen and rxen pin state and set txen pin to low and rxen pin to high
        if self._txen != None and self._rxen != None :
            self._txState = self._txen.input()
            self._rxState = self._rxen.input()
            self._txen.output(LoRaGpio.LOW)
            self._rxen.output(LoRaGpio.HIGH)

        # set device to receive mode with configured timeout, single, or continuous operation
        self.setRx(rxTimeout)

        # set operation status to wait and attach RX interrupt handler
        if self._irq != None :
            if isinstance(self._monitoring, Thread):
                self._monitoring.join()
            to = self._irqTimeout/1000 if timeout == self.RX_SINGLE else timeout/1000
            if timeout == self.RX_CONTINUOUS:
                self._monitoring = Thread(target=self._irq.monitor_continuous, args=(self._interruptRxContinuous, to))
                self._monitoring.setDaemon(True)
            else:
                self._monitoring = Thread(target=self._irq.monitor, args=(self._interruptRx, to))
            self._monitoring.start()
        return True

    def listen(self, rxPeriod: int, sleepPeriod: int) -> bool :

        # skip to enter RX mode when previous RX operation incomplete
        if self.getMode() == self.STATUS_MODE_RX : return False

        # clear previous interrupt and set RX done, RX timeout, header error, and CRC error as interrupt source
        self._irqSetup(self.IRQ_RX_DONE | self.IRQ_TIMEOUT | self.IRQ_HEADER_ERR | self.IRQ_CRC_ERR)

        # set status to RX wait or RX continuous wait
        self._statusWait = self.STATUS_RX_WAIT
        self._statusIrq = 0x0000
        # calculate RX period and sleep period config
        rxPeriod = rxPeriod << 6
        sleepPeriod = sleepPeriod << 6
        if rxPeriod > 0x00FFFFFF : rxPeriod = 0x00FFFFFF
        if sleepPeriod > 0x00FFFFFF : sleepPeriod = 0x00FFFFFF

        # save current txen and rxen pin state and set txen pin to low and rxen pin to high
        if self._txen != None and self._rxen != None :
            self._txState = self._txen.input()
            self._rxState = self._rxen.input()
            self._txen.output(LoRaGpio.LOW)
            self._rxen.output(LoRaGpio.HIGH)

        # set device to receive mode with configured receive and sleep period
        self.setRxDutyCycle(rxPeriod, sleepPeriod)

        # set operation status to wait and attach RX interrupt handler
        if self._irq != None :
            if isinstance(self._monitoring, Thread):
                self._monitoring.join()
            to = self._irqTimeout/1000 if rxPeriod == self.RX_SINGLE else rxPeriod/1000
            self._monitoring = Thread(target=self._irq.monitor, args=(self._interruptRx, to))
            self._monitoring.start()
        return True

    def available(self) -> int :

        # get size of package still available to read
        return self._payloadTxRx

    def read(self, length: int = 0) :

        # single or multiple bytes read
        single = False
        if length == 0 :
            length = 1
            single = True
        # read data from buffer and update buffer index and payload
        buf = self.readBuffer(self._bufferIndex, length)
        self._bufferIndex = (self._bufferIndex + length) % 256
        if self._payloadTxRx > length :
            self._payloadTxRx -= length
        else :
            self._payloadTxRx = 0
        # return single byte or tuple
        if single : return buf[0]
        else : return buf

    def get(self, length: int = 1) -> bytes :

        # read data from buffer and update buffer index and payload
        buf = self.readBuffer(self._bufferIndex, length)
        self._bufferIndex = (self._bufferIndex + length) % 256
        if self._payloadTxRx > length :
            self._payloadTxRx -= length
        else :
            self._payloadTxRx = 0
        # return array of bytes
        return bytes(buf)

    def purge(self, length: int = 0) :

        # subtract or reset received payload length
        if self._bufferIndex > length :
            self._payloadTxRx = self._payloadTxRx - length
        else :
            self._payloadTxRx = 0
        self._bufferIndex += self._payloadTxRx

### WAIT, OPERATION STATUS, AND PACKET STATUS METHODS ###

    def wait(self, timeout: int = 0) -> bool :

        # immediately return when currently not waiting transmit or receive process
        if self._statusIrq :
            return True

        # wait transmit or receive process finish by checking IRQ status
        irqStat = 0x0000
        t = time.time()
        while irqStat == 0x0000 and self._statusIrq == 0x0000 :
            # only check IRQ status register for non interrupt operation
            if self._irq == None : irqStat = self.getIrqStatus()
            # return when timeout reached
            if (time.time() - t) > timeout and timeout > 0 : return False

        if self._statusIrq :
            # immediately return when interrupt signal hit
            return True
        elif self._statusWait == self.STATUS_TX_WAIT :
            # for transmit, calculate transmit time and set back txen and rxen pin to previous state
            self._transmitTime = time.time() - self._transmitTime
            if self._txen != None and self._rxen != None :
                self._txen.output(self._txState)
                self._rxen.output(self._rxState)
        elif self._statusWait == self.STATUS_RX_WAIT :
            # for receive, get received payload length and buffer index and set back txen and rxen pin to previous state
            (self._payloadTxRx, self._bufferIndex) = self.getRxBufferStatus()
            if self._txen != None and self._rxen != None :
                self._txen.output(self._txState)
                self._rxen.output(self._rxState)
            self._fixRxTimeout()
        elif self._statusWait == self.STATUS_RX_CONTINUOUS :
            # for receive continuous, get received payload length and buffer index and clear IRQ status
            (self._payloadTxRx, self._bufferIndex) = self.getRxBufferStatus()
            self.clearIrqStatus(0x03FF)

        # store IRQ status
        self._statusIrq = irqStat
        return True

    def status(self) -> int :

        # set back status IRQ for RX continuous operation
        statusIrq = self._statusIrq
        if self._statusWait == self.STATUS_RX_CONTINUOUS :
            self._statusIrq = 0x0000

        # get status for transmit and receive operation based on status IRQ
        if statusIrq & self.IRQ_TIMEOUT :
            if self._statusWait == self.STATUS_TX_WAIT : return self.STATUS_TX_TIMEOUT
            else : return self.STATUS_RX_TIMEOUT
        elif statusIrq & self.IRQ_HEADER_ERR : return self.STATUS_HEADER_ERR
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
        (rssiPkt, snrPkt, signalRssiPkt) = self.getPacketStatus()
        return rssiPkt / -2.0

    def snr(self) -> float :

        # get signal to noise ratio (SNR) of last incoming package
        (rssiPkt, snrPkt, signalRssiPkt) = self.getPacketStatus()
        if snrPkt > 127 : snrPkt = snrPkt - 256
        return snrPkt / 4.0

    def signalRssi(self) -> float :

        (rssiPkt, snrPkt, signalRssiPkt) = self.getPacketStatus()
        return signalRssiPkt / -2.0

    def rssiInst(self) -> float :

        return self.getRssiInst() / -2.0

    def getError(self) -> int :
        error = self.getDeviceErrors()
        self.clearDeviceErrors()
        return error

### INTERRUPT HANDLER METHODS ###

    def _irqSetup(self, irqMask) :

        # clear IRQ status of previous transmit or receive operation
        self.clearIrqStatus(0x03FF)
        # set selected interrupt source
        dio1Mask = 0x0000
        dio2Mask = 0x0000
        dio3Mask = 0x0000
        if self._dio == 2 : dio2Mask = irqMask
        elif self._dio == 3 : dio3Mask = irqMask
        else : dio1Mask = irqMask
        self.setDioIrqParams(irqMask, dio1Mask, dio2Mask, dio3Mask)

    def _interruptTx(self) :

        # calculate transmit time
        self._transmitTime = time.time() - self._transmitTime
        # set back txen and rxen pin to previous state
        if self._txen != None and self._rxen != None :
            self._txen.output(self._txState)
            self._rxen.output(self._rxState)
        # store IRQ status
        self._statusIrq = self.getIrqStatus()

        # call onTransmit function
        if callable(self._onTransmit) :
            self._onTransmit()

    def _interruptRx(self) :

        # set back txen and rxen pin to previous state
        if self._txen != None and self._rxen != None :
            self._txen.output(self._txState)
            self._rxen.output(self._rxState)
        self._fixRxTimeout()
        # store IRQ status
        self._statusIrq = self.getIrqStatus()
        # get received payload length and buffer index
        (self._payloadTxRx, self._bufferIndex) = self.getRxBufferStatus()

        # call onReceive function
        if callable(self._onReceive) :
            self._onReceive()

    def _interruptRxContinuous(self) :

        # store IRQ status
        self._statusIrq = self.getIrqStatus()
        # clear IRQ status
        self.clearIrqStatus(0x03FF)
        # get received payload length and buffer index
        (self._payloadTxRx, self._bufferIndex) = self.getRxBufferStatus()

        # call onReceive function
        if callable(self._onReceive) :
            self._onReceive()

    def onTransmit(self, callback) :

        # register onTransmit function to call every transmit done
        self._onTransmit = callback

    def onReceive(self, callback) :

        # register onReceive function to call every receive done
        self._onReceive = callback

### SX126X API: OPERATIONAL MODES COMMANDS ###

    def setSleep(self, sleepConfig: int) :
        self._writeBytes(0x84, (sleepConfig,), 1)

    def setStandby(self, stbyConfig: int) :
        self._writeBytes(0x80, (stbyConfig,), 1)

    def setFs(self) :
        self._writeBytes(0xC1, (), 0)

    def setTx(self, timeout: int) :
        buf = (
            (timeout >> 16) & 0xFF,
            (timeout >> 8) & 0xFF,
            timeout & 0xFF
        )
        self._writeBytes(0x83, buf, 3)

    def setRx(self, timeout: int) :
        buf = (
            (timeout >> 16) & 0xFF,
            (timeout >> 8) & 0xFF,
            timeout & 0xFF
        )
        self._writeBytes(0x82, buf, 3)

    def setTimerOnPreamble(self, enable: int) :
        self._writeBytes(0x9F, (enable,), 1)

    def setRxDutyCycle(self, rxPeriod: int, sleepPeriod: int) :
        buf = (
            (rxPeriod >> 16) & 0xFF,
            (rxPeriod >> 8) & 0xFF,
            rxPeriod & 0xFF,
            (sleepPeriod >> 16) & 0xFF,
            (sleepPeriod >> 8) & 0xFF,
            sleepPeriod & 0xFF
        )
        self._writeBytes(0x94, buf, 6)

    def setCad(self) :
        self._writeBytes(0xC5, (), 0)

    def setTxContinuousWave(self) :
        self._writeBytes(0xD1, (), 0)

    def setTxInfinitePreamble(self) :
        self._writeBytes(0xD2, (), 0)

    def setRegulatorMode(self, modeParam: int) :
        self._writeBytes(0x96, (modeParam,), 1)

    def calibrate(self, calibParam: int) :
        self._writeBytes(0x89, (calibParam,), 1)

    def calibrateImage(self, freq1: int, freq2: int) :
        buf = (freq1, freq2)
        self._writeBytes(0x98, buf, 2)

    def setPaConfig(self, paDutyCycle: int, hpMax: int, deviceSel: int, paLut: int) :
        buf = (paDutyCycle, hpMax, deviceSel, paLut)
        self._writeBytes(0x95, buf, 4)

    def setRxTxFallbackMode(self, fallbackMode: int) :
        self._writeBytes(0x93, (fallbackMode,), 1)

### SX126X API: REGISTER AND BUFFER ACCESS COMMANDS ###

    def writeRegister(self, address: int, data: tuple, nData: int) :
        buf = (
            (address >> 8) & 0xFF,
            address & 0xFF
        ) + tuple(data)
        self._writeBytes(0x0D, buf, nData+2)

    def readRegister(self, address: int, nData: int) -> tuple :
        addr = (
            (address >> 8) & 0xFF,
            address & 0xFF
        )
        buf = self._readBytes(0x1D, nData+1, addr, 2)
        return buf[1:]

    def writeBuffer(self, offset: int, data: tuple, nData: int) :
        buf = (offset,) + tuple(data)
        self._writeBytes(0x0E, buf, nData+1)

    def readBuffer(self, offset: int, nData: int) -> tuple :
        buf = self._readBytes(0x1E, nData+1, (offset,), 1)
        return buf[1:]

### SX126X API: DIO AND IRQ CONTROL ###

    def setDioIrqParams(self, irqMask: int, dio1Mask: int, dio2Mask: int, dio3Mask: int) :
        buf = (
            (irqMask >> 8) & 0xFF,
            irqMask & 0xFF,
            (dio1Mask >> 8) & 0xFF,
            dio1Mask & 0xFF,
            (dio2Mask >> 8) & 0xFF,
            dio2Mask & 0xFF,
            (dio3Mask >> 8) & 0xFF,
            dio3Mask & 0xFF
        )
        self._writeBytes(0x08, buf, 8)

    def getIrqStatus(self) -> int :
        buf = self._readBytes(0x12, 3)
        return (buf[1] << 8) | buf[2]

    def clearIrqStatus(self, clearIrqParam: int) :
        buf = (
            (clearIrqParam >> 8) & 0xFF,
            clearIrqParam & 0xFF
        )
        self._writeBytes(0x02, buf, 2)

    def setDio2AsRfSwitchCtrl(self, enable: int) :
        self._writeBytes(0x9D, (enable,), 1)

    def setDio3AsTcxoCtrl(self, tcxoVoltage: int, delay: int) :
        buf = (
            tcxoVoltage & 0xFF,
            (delay >> 16) & 0xFF,
            (delay >> 8) & 0xFF,
            delay & 0xFF
        )
        self._writeBytes(0x97, buf, 4)

### SX126X API: RF, MODULATION, AND PACKET COMMANDS ###

    def setRfFrequency(self, rfFreq: int) :
        buf = (
            (rfFreq >> 24) & 0xFF,
            (rfFreq >> 16) & 0xFF,
            (rfFreq >> 8) & 0xFF,
            rfFreq & 0xFF
        )
        self._writeBytes(0x86, buf, 4)

    def setPacketType(self, packetType: int) :
        self._writeBytes(0x8A, (packetType,), 1)

    def getPacketType(self) -> int :
        buf = self._readBytes(0x11, 2)
        return buf[1]

    def setTxParams(self, power: int, rampTime: int) :
        buf = (power, rampTime)
        self._writeBytes(0x8E, buf, 2)

    def setModulationParamsLoRa(self, sf: int, bw: int, cr: int, ldro: int) :
        buf = (sf, bw, cr, ldro, 0, 0, 0, 0)
        self._writeBytes(0x8B, buf, 8)

    def setModulationParamsFsk(self, br: int, pulseShape: int, bandwidth: int, Fdev: int) :
        buf = (
            (br >> 16) & 0xFF,
            (br >> 8) & 0xFF,
            br & 0xFF,
            pulseShape,
            bandwidth,
            (br >> 16) & 0xFF,
            (br >> 8) & 0xFF,
            Fdev & 0xFF
        )
        self._writeBytes(0x8B, buf, 8)

    def setPacketParamsLoRa(self, preambleLength: int, headerType: int, payloadLength: int, crcType: int, invertIq: int) :
        buf = (
            (preambleLength >> 8) & 0xFF,
            preambleLength & 0xFF,
            headerType,
            payloadLength,
            crcType,
            invertIq,
            0,
            0,
            0
        )
        self._writeBytes(0x8C, buf, 9)

    def setPacketParamsFsk(self, preambleLength: int, preambleDetector: int, syncWordLength: int, addrComp: int, packetType: int, payloadLength: int, crcType: int, whitening: int) :
        buf = (
            (preambleLength >> 8) & 0xFF,
            preambleLength & 0xFF,
            preambleDetector,
            syncWordLength,
            addrComp,
            packetType,
            payloadLength,
            crcType,
            whitening
        )
        self._writeBytes(0x8C, buf, 9)

    def setCadParams(self, cadSymbolNum: int, cadDetPeak: int, cadDetMin: int, cadExitMode: int, cadTimeout: int) :
        buf = (
            cadSymbolNum,
            cadDetPeak,
            cadDetMin,
            cadExitMode,
            (cadTimeout >> 16) & 0xFF,
            (cadTimeout >> 8) & 0xFF,
            cadTimeout & 0xFF
        )
        self._writeBytes(0x88, buf, 7)

    def setBufferBaseAddress(self, txBaseAddress: int, rxBaseAddress: int) :
        buf = (txBaseAddress, rxBaseAddress)
        self._writeBytes(0x8F, buf, 2)

    def setLoRaSymbNumTimeout(self, symbnum: int) :
        self._writeBytes(0xA0, (symbnum,), 1)

### SX126X API: STATUS COMMANDS ###

    def getStatus(self) -> int :
        buf = self._readBytes(0xC0, 1)
        return buf[0]

    def getRxBufferStatus(self) -> tuple :
        buf = self._readBytes(0x13, 3)
        return buf[1:3]

    def getPacketStatus(self) -> tuple :
        buf = self._readBytes(0x14, 4)
        return buf[1:4]

    def getRssiInst(self) -> int :
        buf = self._readBytes(0x15, 2)
        return buf[1]

    def getStats(self) -> tuple :
        buf = self._readBytes(0x10, 7)
        return (
            (buf[1] >> 8) | buf[2],
            (buf[3] >> 8) | buf[4],
            (buf[5] >> 8) | buf[6]
        )

    def resetStats(self) :
        buf = (0, 0, 0, 0, 0, 0)
        self._writeBytes(0x00, buf, 6)

    def getDeviceErrors(self) -> int :
        buf = self._readBytes(0x17, 2)
        return buf[1]

    def clearDeviceErrors(self) :
        buf = (0, 0)
        self._writeBytes(0x07, buf, 2)

### SX126X API: WORKAROUND FUNCTIONS ###

    def _fixLoRaBw500(self, bw: int) :
        packetType = self.getPacketType()
        buf = self.readRegister(self.REG_TX_MODULATION, 1)
        value = buf[0] | 0x04
        if packetType == self.LORA_MODEM and bw == self.BW_500000 :
            value = buf[0] & 0xFB
        self.writeRegister(self.REG_TX_MODULATION, (value,), 1)

    def _fixResistanceAntenna(self) :
        buf = self.readRegister(self.REG_TX_CLAMP_CONFIG, 1)
        value = buf[0] | 0x1E
        self.writeRegister(self.REG_TX_CLAMP_CONFIG, (value,), 1)

    def _fixRxTimeout(self) :
        self.writeRegister(self.REG_RTC_CONTROL, (0,), 1)
        buf = self.readRegister(self.REG_EVENT_MASK, 1)
        value = buf[0] | 0x02
        self.writeRegister(self.REG_EVENT_MASK, (value,), 1)

    def _fixInvertedIq(self, invertIq: bool) :
        buf = self.readRegister(self.REG_IQ_POLARITY_SETUP, 1)
        value = buf[0] & 0xFB
        if invertIq :
            value = buf[0] | 0x04
        self.writeRegister(self.REG_IQ_POLARITY_SETUP, (value,), 1)

### SX126X API: UTILITIES ###

    def _writeBytes(self, opCode: int, data: tuple, nBytes: int) :
        if self.busyCheck() : return
        buf = [opCode]
        for i in range(nBytes) : buf.append(data[i])
        self._cs.output(LoRaGpio.LOW)
        self._spi.transfer(buf)
        self._cs.output(LoRaGpio.HIGH)

    def _readBytes(self, opCode: int, nBytes: int, address: tuple = (), nAddress: int = 0) -> tuple :
        if self.busyCheck() : return ()
        buf = [opCode]
        for i in range(nAddress) : buf.append(address[i])
        for i in range(nBytes) : buf.append(0x00)
        self._cs.output(LoRaGpio.LOW)
        feedback = self._spi.transfer(buf)
        self._cs.output(LoRaGpio.HIGH)
        return tuple(feedback[nAddress+1:])
