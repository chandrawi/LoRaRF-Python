import spidev
import RPi.GPIO
import time

spi = spidev.SpiDev()
gpio = RPi.GPIO
gpio.setmode(RPi.GPIO.BCM)
gpio.setwarnings(False)

class SX127x:
    """Class for SX1276/77/78/79 LoRa chipsets from Semtech"""

    # SX127X LoRa Mode Register Map
    REG_FIFO                               = 0x00
    REG_OPMODE                             = 0x01
    REG_FR_MSB                             = 0x06
    REG_FR_MID                             = 0x07
    REG_FR_LSB                             = 0x08
    REG_PA_CONFIG                          = 0x09
    REG_PA_RAMP                            = 0x0A
    REG_OCP                                = 0x0B
    REG_LNA                                = 0x0C
    REG_FIFO_ADDR_PTR                      = 0x0D
    REG_FIFO_TX_BASE_AD                    = 0x0E
    REG_FIFO_RX_BASE_AD                    = 0x0F
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
    REG_MODEM_CONFIG                       = 0x1D
    REG_MODEM_CONFIG2                      = 0x1E
    REG_SYMB_TIMEOUT_LSB                   = 0x1F
    REG_PREAMBLE_MSB                       = 0x20
    REG_PREAMBLE_LSB                       = 0x21
    REG_PAYLOAD_LENGTH                     = 0x22
    REG_MAX_PAYLOAD_LENGTH                 = 0x23
    REG_HOP_PERIOD                         = 0x24
    REG_FIFO_RX_BYTE_ADDR                  = 0x25
    REG_MODEM_CONFIG3                      = 0x26
    # Reserved                             = 0x27 
    REG_FEI_MSB                            = 0x28
    REG_FEI_MID                            = 0x29
    REG_FEI_LSB                            = 0x2A
    # Reserved                             = 0x2B 
    REG_RSSI_WIDEBAND                      = 0x2C
    # Reserved                             = 0x2D 
    # Reserved                             = 0x2E 
    REG_FREQ1                              = 0x2F
    REG_FREQ2                              = 0x30
    REG_DETECT_OPTIMIZE                    = 0x31
    # Reserved                             = 0x32 
    REG_INVERT_IQ                          = 0x33
    # Reserved                             = 0x34 
    # Reserved                             = 0x35 
    REG_HIGH_BW_OPTIMIZE1                  = 0x36
    REG_DETECTION_THRESHOLD                = 0x37
    # Reserved                             = 0x38 
    REG_SYNC_WORD                          = 0x39
    REG_HIGH_BW_OPTIMIZE2                  = 0x3A
    REG_INVERT_IQ2                         = 0x3B
    # Reserved                             = 0x3C 
    # Reserved                             = 0x3D 
    # Reserved                             = 0x3E 
    # Reserved                             = 0x3F 
    REG_DIO_MAPPING_1                      = 0x40
    REG_DIO_MAPPING_2                      = 0x41
    REG_VERSION                            = 0x42
    # Unused                               = 0x44 
    REG_TXCO                               = 0x4B
    REG_PA_DAC                             = 0x4D
    REG_FORMER_TEMP                        = 0x5B
    # Unused                               = 0x5D 
    REG_AGC_REF                            = 0x61
    REG_AGC_THRESH1                        = 0x62
    REG_AGC_THRESH2                        = 0x63
    REG_AGC_THRESH3                        = 0x64
    REG_PLL                                = 0x70
    
    # TODO all attributes here on down, refactor
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
    _bus = 0
    _cs = 0
    _reset = 22
    _busy = 23
    _irq = -1
    _txen = -1
    _rxen = -1
    _wake = -1
    _busyTimeout = 5000
    _spiSpeed = 7800000
    _txState = gpio.LOW
    _rxState = gpio.LOW

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
    _bufferIndex = 0
    _payloadTxRx = 32
    _statusWait = STATUS_DEFAULT
    _statusIrq = STATUS_DEFAULT
    _transmitTime = 0.0

    # callback functions
    _onTransmit = None
    _onReceive = None
