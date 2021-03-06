"""@package DW1000Constants

This python module is used to store every constants relevant for the DW1000 module and scripts.
"""

# SPI operations
READ = 0x00
WRITE = 0x80
READ_SUB = 0x40
WRITE_SUB = 0xC0
RW_SUB_EXT = 0x80
JUNK = 0x00

# Delay
INIT_DELAY = 0.000005
PMSC_CONFIG_DELAY = 0.00015

# DW1000 TX/RX Modes
IDLE_MODE = 0x00
RX_MODE = 0x01
TX_MODE = 0x02

# Clocks
AUTO_CLOCK = 0x00
XTI_CLOCK = 0x01

# Channel of operation
CHANNEL_1 = 1
CHANNEL_2 = 2
CHANNEL_3 = 3
CHANNEL_4 = 4
CHANNEL_5 = 5
CHANNEL_7 = 7

# Preamble codes
PREAMBLE_CODE_16MHZ_1 = 1
PREAMBLE_CODE_16MHZ_2 = 2
PREAMBLE_CODE_16MHZ_3 = 3
PREAMBLE_CODE_16MHZ_4 = 4
PREAMBLE_CODE_16MHZ_5 = 5
PREAMBLE_CODE_16MHZ_6 = 6
PREAMBLE_CODE_16MHZ_7 = 7
PREAMBLE_CODE_16MHZ_8 = 8
PREAMBLE_CODE_64MHZ_9 = 9
PREAMBLE_CODE_64MHZ_10 = 10
PREAMBLE_CODE_64MHZ_11 = 11
PREAMBLE_CODE_64MHZ_12 = 12
PREAMBLE_CODE_64MHZ_17 = 17
PREAMBLE_CODE_64MHZ_18 = 18
PREAMBLE_CODE_64MHZ_19 = 19
PREAMBLE_CODE_64MHZ_20 = 20

# Transmission/reception bit rate
TRX_RATE_110KBPS = 0x00
TRX_RATE_850KBPS = 0x01
TRX_RATE_6800KBPS = 0x02
""" Only used for human readable printing """
DATA_RATE_DICT = {
    TRX_RATE_110KBPS: 110,
    TRX_RATE_850KBPS: 850,
    TRX_RATE_6800KBPS: 6800
}

# Transmission pulse frequency
TX_PULSE_FREQ_16MHZ = 0x01
TX_PULSE_FREQ_64MHZ = 0x02
""" Only used for human readable printing """
PFREQ_DICT = {
    TX_PULSE_FREQ_16MHZ: 16,
    TX_PULSE_FREQ_64MHZ: 64
}

# Preamble length
TX_PREAMBLE_LEN_64 = 0x01
TX_PREAMBLE_LEN_128 = 0x05
TX_PREAMBLE_LEN_256 = 0x09
TX_PREAMBLE_LEN_512 = 0x0D
TX_PREAMBLE_LEN_1024 = 0x02
TX_PREAMBLE_LEN_1536 = 0x06
TX_PREAMBLE_LEN_2048 = 0x0A
TX_PREAMBLE_LEN_4096 = 0x03
""" Only used for human readable printing """
PLEN_DICT = {
    TX_PREAMBLE_LEN_64: 64,
    TX_PREAMBLE_LEN_128: 128,
    TX_PREAMBLE_LEN_256: 256,
    TX_PREAMBLE_LEN_512: 512,
    TX_PREAMBLE_LEN_1024: 1024,
    TX_PREAMBLE_LEN_1536: 1536,
    TX_PREAMBLE_LEN_2048: 2048,
    TX_PREAMBLE_LEN_4096: 4096
}

# PAC size
PAC_SIZE_8 = 8
PAC_SIZE_16 = 16
PAC_SIZE_32 = 32
PAC_SIZE_64 = 64

# DW1000 Modes of operation
# [dataRate, pulseFrequency, pacSize, preambleLength, channel, preacode]
MODE_STANDARD = [TRX_RATE_110KBPS, TX_PULSE_FREQ_64MHZ, PAC_SIZE_64, TX_PREAMBLE_LEN_2048, CHANNEL_5,
                 PREAMBLE_CODE_64MHZ_9]

# Register file IDs     Description                                 R/W Octets
DEV_ID = 0x00  # Device Identifier                         RO  4
EUI = 0x01  # Extended Unique Identifier                RW  8
# Reserved 0x02
PANADR = 0x03  # PAN Identifier and Short Address          RW  4
SYS_CFG = 0x04  # System Configuration                      RW  4
# Reserved 0x05         
SYS_TIME = 0x06  # System Time Counter                       RO  5
# Reserved 0x07
TX_FCTRL = 0x08  # Transmit Frame Control                    RW  5
TX_BUFFER = 0x09  # Transmit Data Buffer                      RW  1024
DX_TIME = 0x0A  # Delayed Send or Receive Time              RW  5
# Reserved 0x0B
RX_FWTO = 0x0C  # Receive Frame Wait Timeout Period         RW  2
SYS_CTRL = 0x0D  # System Control Register                   SRW 4
SYS_MASK = 0x0E  # System Event Mask Register                RW  4
SYS_STATUS = 0x0F  # System Event Status Register              SRW 5
RX_FINFO = 0x10  # RX Frame Information Register             ROD 4
RX_BUFFER = 0x11  # RX Frame Buffer                           RWD 1024
RX_FQUAL = 0x12  # RX Frame Quality Information              ROD 8
RX_TTCKI = 0x13  # Receiver Time Tracking Interval           ROD 4
RX_TTCKO = 0x14  # Receiver Time Tracking Offset             ROD 5
RX_TIME = 0x15  # Receive Time Stamp                        ROD 14
# Reserved 0x16
TX_TIME = 0x17  # Transmit Time Stamp                       RO  10
TX_ANTD = 0x18  # Transmitter Antenna Delay                 RW  2
# Reserved 0x19
ACK_RESP_T = 0x1A  # Acknowledement Time and Response Time     RW  4
# Reserved 0x1B
# Reserved 0x1C
RX_SNIFF = 0x1D  # SNIFF Mode                                RW  4
TX_POWER = 0x1E  # Transmit Power Control                    RW  4
CHAN_CTRL = 0x1F  # Channel Control                           SRW 4
# Reserved 0x20
USR_SFD = 0x21  # User defined SFD sequence                 RW  41
# Reserved 0x22         
AGC_CTRL = 0x23  # AGC configuration and control             RW  32
EXT_SYNC = 0x24  # External Synchronisation Control          RW  12
ACC_MEM = 0x25  # Accumulator CIR memory                    RO  4064
GPIO_CTRL = 0x26  # GPIO control and status                   RW  44
DRX_CONF = 0x27  # Digital receiver configuration            -   -
RF_CONF = 0x28  # Analog RF configuration block             -   -
# Reserved 0x29
TX_CAL = 0x2A  # Transmitter Calibration block             -   -
FS_CTRL = 0x2B  # Frequency synthesiser control block       -   -
AON = 0x2C  # Always-on system control interface        -   -
OTP_IF = 0x2D  # OTP Memory Interface                      -   -
LDE_CTRL = 0x2E  # Leading Edge Detection Interface          -   -
DIG_DIAG = 0x2F  # Digital Diagnostics Interface             -   41
# Reserved 0x30
# Reserved 0x31
# Reserved 0x32
# Reserved 0x33
# Reserved 0x34
# Reserved 0x35
PMSC = 0x36  # Power Management and System Control       -   -
# Reserved 0x37
# Reserved 0x38
# Reserved 0x39
# Reserved 0x3A
# Reserved 0x3B
# Reserved 0x3C
# Reserved 0x3D
# Reserved 0x3E
# Reserved 0x3F

# Registers offset
NO_SUB = 0xFF
PMSC_CTRL0_SUB = 0x00
PMSC_CTRL1_SUB = 0x04
PMSC_RES1_SUB = 0x08
PMSC_SNOZT_SUB = 0x0C
PMSC_RES2_SUB = 0x10
PMSC_TXFSEQ_SUB = 0x26
PMSC_LEDC_SUB = 0x28
SFD_LENGTH_SUB = 0x00
# OTP_IF subregisters
OTP_ADDR_SUB = 0x04
OTP_CTRL_SUB = 0x06
OTP_RDAT_SUB = 0x0A
# AGC_CTRL subregisters
AGC_TUNE1_SUB = 0x04
AGC_TUNE2_SUB = 0x0C
AGC_TUNE3_SUB = 0x12
# DRX_CONF subregisters
DRX_TUNE0b_SUB = 0x02
DRX_TUNE1a_SUB = 0x04
DRX_TUNE1b_SUB = 0x06
DRX_TUNE2_SUB = 0x08
DRX_TUNE4H_SUB = 0x26
# LDE_IF subregisters
LDE_CFG1_SUB = 0x0806
LDE_RXANTD_SUB = 0x1804
LDE_CFG2_SUB = 0x1806
LDE_REPC_SUB = 0x2804
# RF_CONF subregisters
RF_RXCTRLH_SUB = 0x08
RF_TXCTRL_SUB = 0x0C
# TX_CAL subregisters
TC_PGDELAY_SUB = 0x0B
# FS_CTRL subregisters
FS_PLLCFG_SUB = 0x07
FS_PLLTUNE_SUB = 0x0B
FS_XTALT_SUB = 0x0E
# RX_TIME subregisters
FP_AMPL1_SUB = 0x07
RX_STAMP_SUB = 0x00
# TX_TIME subregisters
TX_STAMP_SUB = 0
# RX_FQUAL subregisters
STD_NOISE_SUB = 0x00
FP_AMPL2_SUB = 0x02
PP_AMPL3_SUB = 0x04
CIR_PWR_SUB = 0x06

OTP_XTAL_ADDRESS = 0x01E

# 0x05: System configuration register bits, see 7.2.6 of User Manual
FFEN_BIT = 0  # Frame Filtering Enable
FFBC_BIT = 1  # Frame Filtering Behave as a Coordinator
FFAB_BIT = 2  # Frame Filtering Allow Beacon frame reception
FFAD_BIT = 3  # Frame Filtering Allow Data frame reception
FFAA_BIT = 4  # Frame Filtering Allow Acknoledgment frame reception
FFAM_BIT = 5  # Frame Filtering Allow MAC command frame reception
FFAR_BIT = 6  # Frame Filtering Allow Reserved frame types
FFA4_BIT = 7  # Frame Filtering Allow frames with frame type field of 4 (0b100)
FFA5_BIT = 8  # Frame Filtering Allow frames with frame type field of 5 (0b101)
HIRQ_POL_BIT = 9  # Host interrupt polarity
SPI_EDGE_BIT = 10  # SPI data launch edge
DIS_FCE_BIT = 11  # Disable frame check error handling
DIS_DRXB_BIT = 12  # Disable Double RX Buffer
DIS_PHE_BIT = 13  # Disable receiver abort on PHR error
DIS_RSDE_BIT = 14  # Disable receiver abort on RSD error
FCS_INIT2F_BIT = 15  # Allow selection of initial seed value for FCS generation
PHR_MODE_BITS = (16, 17)  # Select PHR type (00 = Standard Frame mode, 11 = Long Frames mode)
DIS_STXP_BIT = 18  # Disable Smart TX Power Control
# Bits 19-21 are reserved, should always be set to 0
RXM110K_BIT = 22  # Receiver Mode 110 kpbs data rate
# Bits 23-27 are reserved, should always be set to 0
RXWTOE_BIT = 28  # Receive Wait Timeout Enable
RXAUTR_BIT = 29  # Receiver Auto-Re-enable
AUTOACK_BIT = 30  # Automatic Acknowledgement Enable
AACKPEND_BIT = 31  # Automatic Acknowledgement Pending bit control

# 0x0D: System control register bits, see 7.2.15 of User Manual
SFCST_BIT = 0  # Suppress auto-FCS Transmission
TXSTRT_BIT = 1  # Transmit Start
TXDLYS_BIT = 2  # Transmitter Delayed Sending
CANSFCS_BIT = 3  # Cancel Suppression of auto-FCS transmission
TRXOFF_BIT = 6  # Transceiver Off
WAIT4RESP_BIT = 7  # Wait for Response
RXENAB_BIT = 8  # Enable Receiver
RXDLYE_BIT = 9  # Receiver Delayed Enable
HRBPT_BIT = 24  # Host Side Receive Buffer Pointer Toggle

# 0x0E: System event mask register bits, see 7.2.16 of User Manual
MCPLOCK_BIT = 1  # Mask clock PLL lock event
MESYNCR_BIT = 2  # Mask external sync clock reset event
MAAT_BIT = 3  # Mask automatic acknowledge trigger event
MTXFRB_BIT = 4  # Mask transmit frame begins event
MTXPRS_BIT = 5  # Mask transmit preamble sent event
MTXPHS_BIT = 6  # Mask transmit PHY Header Sent event
MTXFRS_BIT = 7  # Mask transmit frame sent event
MRXPRD_BIT = 8  # Mask receiver preamble detection event
MRXSFDD_BIT = 9  # Mask receiver SFD detected event
MLDEDONE_BIT = 10  # Mask LDE processing done event
MRXPHD_BIT = 11  # Mask receiver PHY header detect event
MRXPHE_BIT = 12  # Mask receiver PHY header error event
MRXDFR_BIT = 13  # Mask receiver data frame ready event
MRXFCG_BIT = 14  # Mask receiver FCS good event
MRXFCE_BIT = 15  # Mask receiver FCS error event
MRXRFSL_BIT = 16  # Mask receiver Reed Solomon Frame Sync Loss event
MRXRFTO_BIT = 17  # Mask Receive Frame Wait Timeout event
MLDEERR_BIT = 18  # Mask leading edge detection processing error event
MRXOVRR_BIT = 20  # Mask Receiver Overrun event
MRXPTO_BIT = 21  # Mask Preamble detection timeout event
MGPIOIRQ_BIT = 22  # Mask GPIO interrupt event
MSLP2INIT_BIT = 23  # Mask SLEEP to INIT event
MRFPLLL_BIT = 24  # Mask RF PLL Losing Lock warning event
MCPLLLL_BIT = 25  # Mask Clock PLL Losing Lock warning event
MRXSFDTO_BIT = 26  # Mask Receive SFD timeout event
MHPDWARN_BIT = 27  # Mask Half Period Delay Warning event
MTXBERR_BIT = 28  # Mask Transmit Buffer Error event
MAFFREJ_BIT = 29  # Mask Automatic Frame Filtering rejection event

# 0x0F: System event status register bits, see 7.2.17 of User Manual
IRQS_BIT = 0  # Interrupt Request Status
CPLOCK_BIT = 1  # Clock PLL Lock
ESYNCR_BIT = 2  # External Sync Clock Reset
AAT_BIT = 3  # Automatic Acknowledge Trigger
TXFRB_BIT = 4  # Transmit Frame Begins
TXPRS_BIT = 5  # Transmit Preamble Sent
TXPHS_BIT = 6  # Transmit PHY Header Sent
TXFRS_BIT = 7  # Transmit Frame Sent
RXPRD_BIT = 8  # Receiver Preamble Detected status
RXSFDD_BIT = 9  # Receiver SFD Detected
LDEDONE_BIT = 10  # LDE processing done
RXPHD_BIT = 11  # Receiver PHY Header Detect
RXPHE_BIT = 12  # Receiver PHY Header Error
RXDFR_BIT = 13  # Receiver Data Frame Ready
RXFCG_BIT = 14  # Receiver FCS Good
RXFCE_BIT = 15  # Receiver FCS Error
RXRFSL_BIT = 16  # Receiver Reed Solomon Frame Sync Loss
RXRFTO_BIT = 17  # Receiver Frame Wait Timeout
LDEERR_BIT = 18  # Leading edge detection processing error
RXOVRR_BIT = 20  # Receiver Overrun
RXPTO_BIT = 21  # Preamble detection timeout
GPIOIRQ_BIT = 22  # GPIO interrupt
SLP2INIT_BIT = 23  # SLLEP to INIT
RFPLL_LL_BIT = 24  # RF PLL Losing Lock
CLKPLL_LL_BIT = 25  # Clock PLL Losing Lock
RXSFDTO_BIT = 26  # Receive SFD timeout
HPDWARN_BIT = 27  # Half Period Delay Warning
TXBERR_BIT = 28  # Transmit Buffer Error
AFFREJ_BIT = 29  # Automatic Frame Filtering rejection
HSRBP_BIT = 30  # Host Side Receive Buffer Pointer
ICRBP_BIT = 31  # IC side Receive Buffer Pointer
RXRSCS_BIT = 32  # Receiver Reed-Solomon Correction Status
RXPREJ_BIT = 33  # Receiver Preamble Rejection
TXPUTE_BIT = 34  # Transmit power up time error

SYS_STATUS_ALL_RX_GOOD = (RXDFR_BIT, RXFCG_BIT, RXPRD_BIT,
                          RXSFDD_BIT, RXPHD_BIT, LDEDONE_BIT)
SYS_STATUS_ALL_DBLBUFF = (RXDFR_BIT, RXFCG_BIT)
SYS_STATUS_ALL_RX_ERR = (RXPHE_BIT, RXFCE_BIT, RXRFSL_BIT,
                         AFFREJ_BIT, LDEERR_BIT)
SYS_STATUS_ALL_RX_TO = (RXRFTO_BIT, RXPTO_BIT)
SYS_STATUS_ALL_TX = (AAT_BIT, TXFRB_BIT, TXPRS_BIT,
                     TXPHS_BIT, TXFRS_BIT)

# 0x1F: Channel control register bits, see 7.2.32 of user manual
DWSFD_BIT = 17
TNSSFD_BIT = 20
RNSSFD_BIT = 21

# 0x26: GPIO control and status sub registers, see 7.2.39 of user manual
GPIO_MODE_SUB = 0x00  # GPIO Mode Control register        RW  4
GPIO_DIR_SUB = 0x08  # GPIO Direction control register   RW  4
GPIO_DOUT_SUB = 0x0C  # GPIO Data Output register         RW  4
GPIO_IRQE_SUB = 0x10  # GPIO Interrupt Enable             RW  4
GPIO_ISEN_SUB = 0x14  # GPIO Interrupt Sense Selection    RW  4
GPIO_IMODE_SUB = 0x18  # GPIO Interrupt Mode (Level/Edge)  RW  4
GPIO_IBES_SUB = 0x1C  # GPIO Interrupt "Both Edge" Select RW  4
GPIO_ICLR_SUB = 0x20  # GPIO Interrupt Latch Clear        RW  4
GPIO_IDBE_SUB = 0x24  # GPIO Interrupt De-bounce Enable   RW  4
GPIO_RAW_SUB = 0x28  # GPIO raw state                    RO  4

# Enable clock masks
ENABLE_CLOCK_MASK1 = 0xFE
ENABLE_CLOCK_MASK2 = 0xFC

# 0x36 0x4: PMSCCTRL0 bits and preconfigured bytes
PMSC_CTRL0_GPCE_BIT = 16
PMSC_CTRL0_GPRN_BIT = 17
PMSC_CTRL0_GPDCE_BIT = 18
PMSC_CTRL0_GPDRN_BIT = 19
# PMSC operation data bytes
# SOFTRESET operations, see SOFTRESET section in 7.2.50.1 of the user manual
SOFT_RESET_SYSCLKS = 0x01
SOFT_RESET_CLEAR = 0x00
SOFT_RESET_SET = 0xF0
SOFT_RESET_RX = 0xE0  # Assuming only 4th byte is written!

# 0x36 0x28: PMSCLEDC bits and preconfigured bytes
PMSC_LEDC_BLINKEN_BIT = 8
PMSC_LEDC_BLINK_TIM_BYTE = 0x20
PMSC_LEDC_BLINKNOW_BYTE = 0x00

# Register access operations to load LDE microcode, see table 4 in 2.5.5.10 of the user manual
LDE_L1STEP1 = 0x01
LDE_L1STEP2 = 0x03
LDE_L2STEP1 = 0x00
LDE_L2STEP2 = 0x80
LDE_L3STEP1 = 0x00
LDE_L3STEP2 = 0x02

# Read OTP memory operation data bytes
OTP_STEP2 = 0x03
OTP_STEP3 = 0x01
OTP_STEP5 = 0x00

# EnableMode masks
ENABLE_MODE_MASK1 = 0x83
ENABLE_MODE_MASK2 = 0xFC
ENABLE_MODE_MASK3 = 0xF3
ENABLE_MODE_MASK4 = 0xC3
# sfdLength
SFD_LENGTH_850KBPS = 0x10
SFD_LENGTH_6800KBPS = 0x08
SFD_LENGTH_OTHER = 0x40
# preacode masks
PREACODE_MASK1 = 0x1F
PREACODE_MASK2 = 0x3F
PREACODE_MASK3 = 0x07

# Set delay masks
SET_DELAY_MASK = 0xFE

# get data masks
GET_DATA_MASK = 0x03FF

# set data masks
SET_DATA_MASK1 = 0xE0
SET_DATA_MASK2 = 0x03

# Tune operations
# AGC_TUNE1, see 7.2.36.3 of the user manual
AGC_TUNE1_DEFAULT_OP = 0x889B
AGC_TUNE1_16MHZ_OP = 0x8870
# AGC_TUNE2, see 7.2.63.5 of the user manual
AGC_TUNE2_OP = 0x2502A907
# AGC_TUNE3, see 7.2.36.7 of the user manual
AGC_TUNE3_OP = 0x0035
# DRX_TUNE0b, see 7.2.40.2 of the user manual
DRX_TUNE0b_110KBPS_NOSTD_OP = 0x0016
DRX_TUNE0b_850KBPS_NOSTD_OP = 0x0006
DRX_TUNE0b_6800KBPS_STD_OP = 0x0001
# DRX_TUNE1a, see 7.2.40.3 of the user manual
DRX_TUNE1a_16MHZ_OP = 0x0087
DRX_TUNE1a_64MHZ_OP = 0x008D
# DRX_TUNE1b, see 7.2.40.4 of the user manual
DRX_TUNE1b_M1024 = 0x0064
DRX_TUNE1b_L1024 = 0x0020
DRX_TUNE1b_64 = 0x0010
# DRX_TUNE2, see 7.2.40.5 of the user manual
DRX_TUNE2_8_16MHZ = 0x311A002D
DRX_TUNE2_8_64MHZ = 0x313B006B
DRX_TUNE2_16_16MHZ = 0x331A0052
DRX_TUNE2_16_64MHZ = 0x333B00BE
DRX_TUNE2_32_16MHZ = 0x351A009A
DRX_TUNE2_32_64MHZ = 0x353B015E
DRX_TUNE2_64_16MHZ = 0x371A011D
DRX_TUNE2_64_64MHZ = 0x373B0296
# LDE_CFG2, see 7.2.47.6 of the user manual
LDE_CFG2_16MHZ_OP = 0x1607
LDE_CFG2_64MHZ_OP = 0x0607
# DRX_TUNE4H, see 7.2.40.10 of the user manual
DRX_TUNE4H_64 = 0x0010
DRX_TUNE4H_128 = 0x0028
# RF_RXCTRLH, see 7.2.41.3 of the user manual
RF_RXCTRLH_1235 = 0xD8
RF_RXCTRLH_147 = 0xBC
# RF_TXCTRL, see 7.2.41.4 of the user manual
RF_TXCTRL_1 = 0x00005C40
RF_TXCTRL_2 = 0x00045CA0
RF_TXCTRL_3 = 0x00086CC0
RF_TXCTRL_4 = 0x00045C80
RF_TXCTRL_5 = 0x001E3FE0
RF_TXCTRL_7 = 0x001E7DE0
# TC_PGDELAY, see 7.2.43.4 of the user manual
TC_PGDELAY_1 = 0xC9
TC_PGDELAY_2 = 0xC2
TC_PGDELAY_3 = 0xC5
TC_PGDELAY_4 = 0x95
TC_PGDELAY_5 = 0xC0
TC_PGDELAY_7 = 0x93
# FS_PLLCFG, see 7.2.44.2 of the user manual
FS_PLLCFG_1 = 0x09000407
FS_PLLCFG_24 = 0x08400508
FS_PLLCFG_3 = 0x08401009
FS_PLLCFG_57 = 0x0800041D
# FS_PLLTUNE, see 7.2.44.3 of the user manual
FS_PLLTUNE_1 = 0x1E
FS_PLLTUNE_24 = 0x26
FS_PLLTUNE_3 = 0x5E
FS_PLLTUNE_57 = 0xBE
# TX_POWER, see 7.2.31.3 table 19 of the user manual
TX_POWER_12_16MHZ = 0x75757575
TX_POWER_12_64MHZ = 0x67676767
TX_POWER_3_16MHZ = 0x6F6F6F6F
TX_POWER_3_64MHZ = 0x8B8B8B8B
TX_POWER_4_16MHZ = 0x5F5F5F5F
TX_POWER_4_64MHZ = 0x9A9A9A9A
TX_POWER_5_16MHZ = 0x48484848
TX_POWER_5_64MHZ = 0x85858585
TX_POWER_7_16MHZ = 0x92929292
TX_POWER_7_64MHZ = 0xD1D1D1D1
# LDE_CFG1, see 2.5.5.4 of the user manual
LDE_CFG1_OP = 0xD
# LDE_REPC, see 7.2.47.7 table 48 of the user manual
LDE_REPC_1AND2 = 0x5998
LDE_REPC_3AND8 = 0x51EA
LDE_REPC_4 = 0x428E
LDE_REPC_5 = 0x451E
LDE_REPC_6 = 0x2E14
LDE_REPC_7 = 0x8000
LDE_REPC_9 = 0x28F4
LDE_REPC_1017 = 0x3332
LDE_REPC_111321 = 0x3AE0
LDE_REPC_12 = 0x3D70
LDE_REPC_14161819 = 0x35C2
LDE_REPC_15 = 0x2B84
LDE_REPC_20 = 0x47AE
LDE_REPC_2224 = 0x3850
LDE_REPC_23 = 0x30A2
# Tune masks
TUNE_MASK1 = 0x1F
TUNE_MASK2 = 0x60
TUNE_OPERATION = 0x10

# signal power in the first path, see 4.7.1 of the user manual and receive signal power, see 4.7.2 of the user manual
A_16MHZ = 115.72
CORRFAC_16MHZ = 2.3334
A_64MHZ = 121.74
CORRFAC_64MHZ = 1.1667
TWOPOWER17 = 131072
PWR_COEFF = 88
PWR_COEFF2 = 10.0

# Time
MILLISECONDS = 1000
MICROSECONDS = 1
DEFAULT_TIMER_DELAY = 80
DEFAULT_REPLY_DELAY_TIME = 7000
ANTENNA_DELAY_RASPI = 16390
ANTENNA_DELAY = 16470
TIME_OVERFLOW = 0x10000000000
# time resolution in micro seconds of time based registers
TIME_RES = 0.000015650040064103
TIME_RES_INV = 63897.6

FC_1_BLINK = 0xC5
FC_1 = 0x41
FC_2 = 0x8C
FC_2_SHORT = 0x88
PAN_ID_1 = 0xCA
PAN_ID_2 = 0xDE

# Frame length
LONG_MAC_LEN = 15
SHORT_MAC_LEN = 9

# Message types
POLL = 0
POLL_ACK = 1
RANGE = 2
RANGE_REPORT = 3
BLINK = 4
RANGE_FAILED = 255

RESET_PERIOD = 250

# Bits/Bytes operation
MASK_LS_BYTE = 0xFF
MASK_LS_2BITS = 0x03
MASK_LS_2BYTES = 0xFFFF
MASK_NIBBLE = 0x0F

# Correct Time stamp
BIAS_500_16_ZERO = 10
BIAS_500_64_ZERO = 8
BIAS_900_16_ZERO = 7
BIAS_900_64_ZERO = 7
BIAS_500_16 = [198, 187, 179, 163, 143, 127, 109, 84, 59, 31, 0, 36, 65, 84, 97, 106, 110, 112]
BIAS_500_64 = [110, 105, 100, 93, 82, 69, 51, 27, 0, 21, 35, 42, 49, 62, 71, 76, 81, 86]
BIAS_900_16 = [137, 122, 105, 88, 69, 47, 25, 0, 21, 48, 79, 105, 127, 147, 160, 169, 178, 197]
BIAS_900_64 = [147, 133, 117, 99, 75, 50, 29, 0, 24, 45, 63, 76, 87, 98, 116, 122, 132, 142]
ADJUSTMENT_TIME_FACTOR = 0.001
# Speed of radio waves [m/s] * timestamp resolution [~15.65ps] of DW1000
DISTANCE_OF_RADIO = 0.0046917639786159
DISTANCE_OF_RADIO_INV = 213.139451293

# Operation mode bits
DATA_RATE_BIT = 0
PULSE_FREQUENCY_BIT = 1
PAC_SIZE_BIT = 2
PREAMBLE_LENGTH_BIT = 3
CHANNEL_BIT = 4
PREAMBLE_CODE_BIT = 5

RECEIVER = 0
TRANSMITTER = 1

# Messages used in the ranging protocol
# -- Message types
POLL = 0
POLL_ACK = 1
RANGE = 2
RANGE_REPORT = 3
RANGE_FAILED = 255
BLINK = 4
RANGING_INIT = 5

LEN_DATA = 90

# Default value
# in ms
DEFAULT_RESET_PERIOD = 200
# in us
DEFAULT_REPLY_DELAY_TIME = 7000

# Default timer delay
DEFAULT_TIMER_DELAY = 80
