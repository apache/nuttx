README
^^^^^^

README for NuttX port to the u-blox C027 dev board.

The board is intended for prototyping software for u-blox cellular modems (SARA
or LISA families) and the u-blox GPS receiver (MAX-7Q, MAX-M8Q, or other,
depending on board revision).

This board features the LPC1768 MCU and is related to the Embedded Artists' base
board with the NXP the LPCXpresso daughter board, see
configs/lpcxpresso-lpc1768/README.txt.

The board also features an embedded USB debugger.

Contents
^^^^^^^^

  u-blox C027 Board
  Configurations

u-blox C027 Board
^^^^^^^^^^^^^^^^^

Pin Description                  Connector On Board
-------------------------------- --------- --------------
P0[0]/RD1/TXD3/SDA1               D14       TXD3/SDA1
P0[1]/TD1/RXD3/SCL1               D15       RXD3/SCL1
P0[2]/TXD0/AD0[7]                           USBTXD
P0[3]/RXD0/AD0[6]                           USBRXD
P0[4]/I2SRX-CLK/RD2/CAP2.0        CANRD     CAN_RX2
P0[5]/I2SRX-WS/TD2/CAP2.1         CANTD     CAN_TX2
P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     CANS      SSEL1
P0[7]/I2STX_CLK/SCK1/MAT2[1]                MDMUSBDET
P0[8]/I2STX_WS/MISO1/MAT2[2]                MDMLVLOE
P0[9]/I2STX_SDA/MOSI1/MAT2[3]               MDMILVLOE
P0[10]/TXD2/SDA2                            GPSTXD
P0[11]/RXD2/SCL2                            GPSRXD
P0[15]/TXD1/SCK0/SCK                        MDMTXD
P0[16]/RXD1/SSEL0/SSEL                      MDMRXD
P0[17]/CTS1/MISO0/MISO                      MDMCTS
P0[18]/DCD1/MOSI0/MOSI                      MDMDCD
P0[19]/DSR1/SDA1                            MDMDCR
P0[20]/DTR1/SCL1                            MDMDTR
P0[21]/RI1/MCIPWR/RD1                       MDMRI
P0[22]/RTS1/TD1                             MDMRTS
P0[23]/AD0[0]/I2SRX_CLK/CAP3[0]   A0        I2S_CLK
P0[24]/AD0[1]/I2SRX_WS/CAP3[1]    A1        I2S_WS
P0[25]/AD0[2]/I2SRX_SDA/TXD3      A2        I2S_SDA
P0[26]/AD0[3]/AOUT/RXD3           A3        AD0.3/AOUT
P0[27]/SDA0/USB_SDA                         GPSSDA
P0[28]/SCL0                                 GPSSCL
P0[29]/USB_D+                               MDMUSBDP
P0[30]/USB_D-                               MDMUSBDM
P1[0]/ENET_TXD0                             ENET_TXD0
P1[1]/ENET_TXD1                             ENET_TXD1
P1[4]/ENET_TX_EN                            ENET_TX_EN
P1[8]/ENET_CRS                              ENET_CRS
P1[9]/ENET_RXD0                             ENET_RXD0
P1[10]/ENET_RXD1                            ENET_RXD1
P1[14]/ENET_RX_ER                           ENET_RX_ER
P1[15]/ENET_REF_CLK                         ENET_REF_CLK
P1[16]/ENET_MDC                             ENET_MDC
P1[17]/ENET_MDIO                            ENET_MDIO
P1[18]/USB_UP_LED/PWM1[1]/CAP1[0]           GPSRST
P1[19]/MC0A/USB_PPWR/N_CAP1.1               GPSPPS
P1[20]/MCFB0/PWM1.2/SCK0          D13       PWM1.2/SCK0
P1[21]/MCABORT/PWM1.3/SSEL0       D10       PWM1.3/SSEL0
P1[22]/MC0B/USB-PWRD/MAT1.0                 GPSINT
P1[23]/MCFB1/PWM1.4/MISO0         D12       PWM1.4/MISO0
P1[24]/MCFB2/PWM1.5/MOSI0         D11       PWM1.5/MOSI0
P1[25]/MC1A/MAT1.1                          ETH_LED_LNK
P1[26]/MC1B/PWM1.6/CAP0.0                   ETH_LED_SPD
P1[27]/CLKOUT/USB-OVRCR-N/CAP0.1            ETH_OSC_EN
P1[28]/MC2A/PCAP1.0/MAT0.0                  ETH_RST
P1[29]/MC2B/PCAP1.1/MAT0.1                  GPSEN
P1[30]/VBUS/AD0[4]                A4        AD0.4
P1[31]/SCK1/AD0[5]                A5        AD0.5
P2[0]/PWM1.1/TXD1                 D3        PWM1.1
P2[1]/PWM1.2/RXD1                 D5        PWM1.2
P2[2]/PWM1.3/CTS1/TRACEDATA[3]    D6        PWM1.3
P2[3]/PWM1.4/DCD1/TRACEDATA[2]    D9        PWM1.4
P2[4]/PWM1.5/DSR1/TRACEDATA[1]    D8        PWM1.5
P2[5]/PWM1[6]/DTR1/TRACEDATA[0]             MDMEN
P2[6]/PCAP1[0]/RI1/TRACECLK                 MDMPWRON
P2[7]/RD2/RTS1                              MDMGPIO1
P2[8]/TD2/TXD2                              MDMRST
P2[9]/USB_CONNECT/RXD2                      MDMUSBCON
P2[10]/EINT0/NMI                  ISP_PAD
P2[11]/EINT1/I2STX_CLK            D7        EINT1
P2[12]/EINT2/I2STX_WS             D4        EINT2
P2[13]/EINT3/I2STX_SDA            D2        EINT3
P3[25]/MAT0.0/PWM1.2                        LED
P4[28]/RX-MCLK/MAT2.0/TXD3        D0        TXD3
P4[29]/TX-MCLK/MAT2.1/RXD3        D1        RXD3

Configurations
^^^^^^^^^^^^^^

nsh
---

The nsh configuration allows to test the modem and the GPS receiver. The modem
can be tested in either the internal TCP/IP stack mode or with the NuttX TCP/IP
stack and pppd.

Modem starts in power-down mode and has to be switched on, e.g., using the nsh
command:

nsh> ubloxmodem on

To connect using the NuttX TCP/IP stack, use pppd. At the time of writing the
pppd app is located in apps/examples/pppd and requires setting correct login
details in the source code.

To connect using the internal u-blox TCP/IP stack, use the chat app. Its
configuration is done in configs/u-blox-c027/nsh/defconfig and can be adjusted
using 'make menuconfig'.

The NuttX u-blox modem driver is included like this:

CONFIG_MODEM=y
CONFIG_MODEM_U_BLOX=y

The nsh app that allows power control is included with the default configuration
as follows:

CONFIG_EXAMPLES_UBLOXMODEM=y
CONFIG_EXAMPLES_UBLOXMODEM_TTY_DEVNODE="/dev/ttyS1"
CONFIG_EXAMPLES_UBLOXMODEM_DEVNODE="/dev/ubxmdm"

The modem driver uses two device nodes. The TTY device is the modem terminal,
while the other device is used for general control via GPIO such as power
control. At the moment only one modem is supported, and the corresponding device
control node is /dev/ubxmdm.

The chat app allows to run scripts. A few such scripts are preset in the
defconfig:

CONFIG_EXAMPLES_CHAT_PRESET[0..3]:
0 - set up the provider context with username and password,
1 - connect to the provider,
2 - open a TCP socket to the u-blox test server,
3 - test the TCP socket server operation and close the socket.
