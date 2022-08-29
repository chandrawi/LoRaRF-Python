from .base import BaseLoRa
import spidev
import RPi.GPIO
import time

spi = spidev.SpiDev()
gpio = RPi.GPIO
gpio.setmode(RPi.GPIO.BCM)
gpio.setwarnings(False)

class SX127x(BaseLoRa):
    """Class for SX1276/77/78/79 LoRa chipsets from Semtech"""

    # SX127X LoRa Mode Register Map
    REG_FIFO                               = 0x00
    REG_OP_MODE                            = 0x01
    # Unused                               = 0x02
    # Unused                               = 0x03
    # Unused                               = 0x04
    # Unused                               = 0x05
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
    # Reserved                             = 0x27 
        # Reserved                             = 0x27 
    # Reserved                             = 0x27 
    REG_FREQ_ERROR_MSB                     = 0x28
    REG_FREQ_ERROR_MID                     = 0x29
    REG_FREQ_ERROR_LSB                     = 0x2A
    # Reserved                             = 0x2B 
        # Reserved                             = 0x2B 
    # Reserved                             = 0x2B 
    REG_RSSI_WIDEBAND                      = 0x2C
    # Reserved                             = 0x2D 
        # Reserved                             = 0x2D 
    # Reserved                             = 0x2D 
    # Reserved                             = 0x2E 
        # Reserved                             = 0x2E 
    # Reserved                             = 0x2E 
    REG_FREQ1                              = 0x2F
    REG_FREQ2                              = 0x30
    REG_DETECTION_OPTIMIZE                 = 0x31
    # Reserved                             = 0x32 
        # Reserved                             = 0x32 
    # Reserved                             = 0x32 
    REG_INVERTIQ                           = 0x33
    # Reserved                             = 0x34 
        # Reserved                             = 0x34 
    # Reserved                             = 0x34 
    # Reserved                             = 0x35 
        # Reserved                             = 0x35 
    # Reserved                             = 0x35 
    REG_HIGH_BW_OPTIMIZE_1                 = 0x36
    REG_DETECTION_THRESHOLD                = 0x37
    # Reserved                             = 0x38 
        # Reserved                             = 0x38 
    # Reserved                             = 0x38 
    REG_SYNC_WORD                          = 0x39
    REG_HIGH_BW_OPTIMIZE_2                 = 0x3A
    REG_INVERTIQ2                          = 0x3B
    # Reserved                             = 0x3C 
        # Reserved                             = 0x3C 
    # Reserved                             = 0x3C 
    # Reserved                             = 0x3D 
        # Reserved                             = 0x3D 
    # Reserved                             = 0x3D 
    # Reserved                             = 0x3E 
        # Reserved                             = 0x3E 
    # Reserved                             = 0x3E 
    # Reserved                             = 0x3F 
        # Reserved                             = 0x3F 
    # Reserved                             = 0x3F 
    REG_DIO_MAPPING_1                      = 0x40
    REG_DIO_MAPPING_2                      = 0x41
    REG_VERSION                            = 0x42
    # Unused                               = 0x44 
        # Unused                               = 0x44 
    # Unused                               = 0x44 
    REG_TCXO                               = 0x4B
    REG_PA_DAC                             = 0x4D
    REG_FORMER_TEMP                        = 0x5B
    # Unused                               = 0x5D 
        # Unused                               = 0x5D 
    # Unused                               = 0x5D 
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

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def begin(self, bus: int = _bus, cs: int = _cs, reset: int = _reset, irq: int = _irq, txen: int = _txen, rxen: int = _rxen) ->bool:

        # set spi and gpio pins
        self.setSpi(bus, cs)
        self.setPins(reset, irq, txen, rxen)

    def end(self):
        pass

    def setSpi(self, bus: int, cs: int, speed: int = _spiSpeed):

        self._bus = bus
        self._cs = cs
        self._spiSpeed = speed
        # open spi line and set bus id, chip select, and spi speed
        spi.open(bus, cs)
        spi.max_speed_hz = speed
        spi.lsbfirst = False
        spi.mode = 0

    def setPins(self, reset: int, irq: int = -1, txen: int = -1, rxen: int = -1):

        self._reset = reset
        self._irq = irq
        self._txen = txen
        self._rxen = rxen
        # set pins as input or output
        gpio.setup(reset, gpio.OUT)
        if irq != -1 : gpio.setup(irq, gpio.IN)
        if txen != -1 : gpio.setup(txen, gpio.OUT)
        if rxen != -1 : gpio.setup(rxen, gpio.OUT)
