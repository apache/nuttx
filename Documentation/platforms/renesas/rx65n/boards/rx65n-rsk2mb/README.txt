README
======

This README file discusses the port of NuttX to the RX65N RSK2MB board. This board features the RX65N (R5F565NEHDFC 176pin)

Contents
========

  - Board Features
  - Status/Open Issues
  - Serial Console
  - LEDs
  - Networking
  - RTC
  - USB Device
  - RSPI
  - RIIC
  - DTC
  - USB Host
  - USB Host Hub
  - Debugging

Board Features
==============
- Mounted devices: RX65N (R5F565NEDDFC: No Encrypt Function, Code Flash 2MB, Pin Count 176-pin),
  or RX65N (R5F565NEHDFC: Supported Encrypt Function, Code Flash 2MB, Pin Count 176-pin)
- Mounts TFT Display. Graphic LCD controller can be evaluated
- 1 channel Ethernet can be evaluated
- RX65N builds in Trusted Secure IP. AES encryption function and robust key management can be evaluated (*)
- Mounts SD slot. If an optional Wireless LAN expansion board package for RSK (RTK0ZZZZZZP00000BR#WS) is used,
  Wireless LAN can evaluated
- 1 channel USB Function and 1 channel USB Host can be evaluated
- In addition, CAN, RSPI, QSPI, etc. can be evaluated

See the RX65N RSK2MB website for further information about this board:

  - https://www.renesas.com/br/en/products/software-tools/boards-and-kits/starter-kits/renesas-starter-kitplus-for-rx65n-2mb.html

Serial Console
==============
RX65N RSK2MB supports 12 serial ports (SCI0 - SCI12), however only 1 port can be tested(SCI8, which is the serial console). Only SCI8 port can be tested which is connected to USB Serial port.

Serial ports SCI1, SCI2, SCI9-SCI12, cannot be tested because they are multiplexed to other Rx65N controller interfaces.

Following SCI ports are configured w.r.t RX65N pin configuration
SCI1 Pin Configuration :
-----------
RX65N RSK2MB
  Function
-----------
  PF2   RXD1
  PF1   TXD1
------------

SCI2 Pin Configuration :
-----------
RX65N RSK2MB
  Function
-----------
  P52   RXD2
  P50   TXD2
------------
SCI8 Pin Configuration :
-----------
RX65N RSK2MB
  Function
-----------
  PJ1   RXD8
  PJ2   TXD8
------------

Serial Connection Configuration
-------------------------------
1. RSK2MB board needs to be connected to PC, using USB cable(One end of which is connected to PC, other end
connected to USB serial port on H/W board).
2. RSK USB Serial Driver needs to be downloaded on PC side.
3. Configure Teraterm to 115200 baud.

LEDs
====

  The RX65N RSK2MB board has 2 Power LED's(PowerLED5 LED_G, PowerLED3 LED_G) and 4 user LED's (LED_G, LED_O, LED_R, LED_R).

  If enabled 4 User LED's are simply turned on when the board boots
  successfully, and is blinking on panic / assertion failed.

Networking
==========

Ethernet Connections
--------------------

  ------        ---------
  RX65N
  RSK2MB        Ethernet
  Pin           Function
  ------        ---------
  PC4           ET0_TX_CLK
  P76           ET0_RX_CLK
  P80           ET0_TX_EN
  PC6           ET0_ETXD3
  PC5           ET0_ETXD2
  P82           ET0_ETXD1
  P81           ET0_ETXD0
  PC3           ET0_TX_ER
  PC2           ET0_RX_DV
  PC0           ET0_ERXD3
  PC1           ET0_ERXD2
  P74           ET0_ERXD1
  P75           ET0_ERXD0
  P77           ET0_RX_ER
  P83           ET0_CRS
  PC7           ET0_COL
  P72           ET0_MDC
  P71           ET0_MDIO
  P54           ET0_LINKSTA
  ------         ---------

USB Device
-----------

For the RX65N RSK2MB board, to be used as USB Device, the following Jumper settings need to be done

J7     Short Pin 2 & Pin 3
J16    Short Pin 1 & Pin 2

NuttX Configurations
--------------------
The following configurations, need to be enabled for network.

CONFIG_RX65N_EMAC=y    : Enable the EMAC Peripheral for RX65N
CONFIG_RX65N_EMAC0=y   : Enable the EMAC Peripheral for RX65N
CONFIG_RX65N_EMAC0_PHYSR=30 : Address of PHY status register
CONFIG_RX65N_EMAC0_PHYSR_100FD=0x18  : Needed for PHY CHIP
CONFIG_RX65N_EMAC0_PHYSR_100HD=0x08  : "    " " " "     "
CONFIG_RX65N_EMAC0_PHYSR_10FD=0x14   : "    " " " "     "
CONFIG_RX65N_EMAC0_PHYSR_10HD=0x04   : "    " " " "     "
CONFIG_RX65N_EMAC0_PHYSR_ALTCONFIG=y : "    " " " "     "
CONFIG_RX65N_EMAC0_PHYSR_ALTMODE=0x1c : "    " " " "     "
CONFIG_RX65N_EMAC0_RMII=y
CONFIG_RX65N_EMAC0_PHYADDR=0 :  PHY is at address 1

CONFIG_SCHED_WORKQUEUE=y : Work queue support is needed
CONFIG_SCHED_HPWORK=y    :  High Priority Work queue support
CONFIG_SCHED_LPWORK=y    :  Low Priority Work queue support

Using the network with NSH
--------------------------
The IP address is configured using DHCP, using the below mentioned configurations :

CONFIG_NETUTILS_DHCPC=y
CONFIG_NETUTILS_DHCPD=y
CONFIG_NSH_DHCPC=y
CONFIG_NETINIT_DHCPC=y

nsh> ifconfig
  eth0    HWaddr 00:e0:de:ad:be:ef at UP
          IPaddr:10.75.24.53 DRaddr:10.75.24.1 Mask:255.255.254.0

You can use ping to test for connectivity to the host (Careful,
Window firewalls usually block ping-related ICMP traffic).  On the
target side, you can:

  nsh> ping 10.75.24.250
  PING 10.75.24.250 56 bytes of data
  56 bytes from 10.75.24.250: icmp_seq=1 time=0 ms
  56 bytes from 10.75.24.250: icmp_seq=2 time=0 ms
  56 bytes from 10.75.24.250: icmp_seq=3 time=0 ms
  56 bytes from 10.75.24.250: icmp_seq=4 time=0 ms
  56 bytes from 10.75.24.250: icmp_seq=5 time=0 ms
  56 bytes from 10.75.24.250: icmp_seq=6 time=0 ms
  56 bytes from 10.75.24.250: icmp_seq=7 time=0 ms
  56 bytes from 10.75.24.250: icmp_seq=8 time=0 ms
  56 bytes from 10.75.24.250: icmp_seq=9 time=0 ms
  56 bytes from 10.75.24.250: icmp_seq=10 time=0 ms
  10 packets transmitted, 10 received, 0% packet loss, time 10100 ms

On the host side, you should also be able to ping the RX65N-RSK2MB:

  $ ping 10.75.24.53

Configure UDP blaster application as mentioned below :

CONFIG_EXAMPLES_UDPBLASTER_HOSTIP=0x0a4b1801  (10.75.24.1) ------> Gateway IP
CONFIG_EXAMPLES_UDPBLASTER_NETMASK=0xfffffe00 (255.255.254.0) --------> Netmask
CONFIG_EXAMPLES_UDPBLASTER_TARGETIP=0x0a4b189b (10.75.24.155) ---------> Target IP

RSPI
-----------

For RX65N RSK2MB board, Following pin is configured for all channels in JA3.
Channel0: Pin number 7 and 8 in JA3 is used for MOSIA and MISOA respectively
Channel1: Pin number 35 and 36 in JA3 is used for MOSIB and MISOB respectively
Channel2: Pin number 18 and 19 in JA3 is used for MOSIC and MISOC respectively
and for enabling these pin need to select DSW-SEL0 by making off SW4-4

USB Host
=============
For the RX65N RSK2MB board, to be used as USB Device, the following Jumper settings need to be done

J7     Short Pin 1 & Pin 2
J16    Short Pin 2 & Pin 3

USB Device
=============
For the RX65N RSK2MB board, to be used as USB Device, the following Jumper settings need to be done

J7     Short Pin 2 & Pin 3
J16    Short Pin 1 & Pin 2

RTC
==========

NuttX Configurations
---------------
The configurations listed in Renesas_RX65N_NuttX_RTC_Design.doc need to be enabled.

RTC Testing
------------------
The test cases mentioned in Renesas_RX65N_RTC_Test_Cases.xls are to be executed
as part of RTC testing.

The following configurations are to be enabled as part of testing RTC examples.
CONFIG_EXAMPLES_ALARM
CONFIG_EXAMPLES_PERIODIC
CONFIG_EXAMPLES_CARRY

USB Device Configurations
--------------------------
The following configurations need to be enabled for USB Device

CONFIG_USBDEV
CONFIG_CDCACM
CONFIG_STDIO_BUFFER_SIZE=64
CONFIG_STDIO_LINEBUFFER

USB Device Testing
------------------------
The following testing is executed as part of USB Device testing on RX65N target for GRROSE board

echo "This is a test for USB Device" > /dev/ttyACM0

xd 0 0x20000 > /dev/ttyACM0

The output of the commands mentioned above should be seen on the USB Device COM port on teraterm

RSPI Configurations
--------------------------
The following configurations need to be enabled for RSPI

CONFIG_SYSTEM_SPITOOL=y

RSPI Testing
------------------------
The following testing is executed as part of RSPI testing on RX65N target for RSK2MB board

On RSK2MB board, all three channels 0, 1 and 2 has been brought out and tested.

Following command can be used for testing RSPI communication to slave device.
spi exch -b 0 -x 4 aabbccdd
where b is bus number and x is Number of word to exchange.

RIIC Configurations
--------------------------
The following configurations need to be enabled for RIIC.

CONFIG_SYSTEM_I2CTOOL=y

RIIC Testing
------------------------
The following testing is executed as part of RIIC testing on RX65N target for RSK2MB board

On RSK2MB board only channel 0 can be tested.

Following command can be used for testing RIIC communication with slave device.
i2c set -b 0 -a 53 -r 0 10
where b is bus number, a is the slave address, r is the register address and 10 is the value to be written.

DTC Configurations
--------------------------
The following configurations need to be enabled for DTC.

CONFIG_SYSTEM_SPITOOL=y

DTC Testing
------------------------

DTC has been tested using RSPI driver.

USB Host Configurations
--------------------------
The following configurations need to be enabled for USB Host Mode driver to
support USB HID Keyboard class and MSC Class.

CONFIG_USBHOST=y
CONFIG_USBHOST_HIDKBD=y
CONFIG_FS_FAT=y
CONFIG_EXAMPLES_HIDKBD=y

USB Host Driver Testing
------------------------
The Following Class Drivers were tested as mentioned below :

- USB HID Keyboard Class
On the NuttX Console "hidkbd" application was executed

nsh> hidkbd
The characters typed from the keyboard were executed correctly.

- USB MSC Class

The MSC device is enumerated as sda in /dev directory.

The block device is mounted using the command as mentioned below :

mount -t vfat /dev/sda /mnt

The MSC device is mounted in /dev directory

The copy command is executed to test the Read/Write functionality

cp /mnt/<file.txt> /mnt/file_copy.txt

USB Host Hub Configurations
--------------------------
The following configurations need to be enabled for USB Host Mode driver to
support USB HID Keyboard class and MSC Class.

CONFIG_RX65N_USBHOST=y
CONFIG_USBHOST_HUB=y
CONFIG_USBHOST_ASYNCH=y
CONFIG_USBHOST=y
CONFIG_USBHOST_HIDKBD=y
CONFIG_FS_FAT=y
CONFIG_EXAMPLES_HIDKBD=y

USB Host Hub Driver Testing
------------------------
The Following Class Drivers were tested as mentioned below :

- USB HID Keyboard Class
On the NuttX Console "hidkbd" application was executed

nsh> hidkbd
The characters typed from the keyboard were executed correctly.

- USB MSC Class
The MSC device is enumerated as sda in /dev directory.

The block device is mounted using the command as mentioned below :

mount -t vfat /dev/sda /mnt

The MSC device is mounted in /dev directory

The copy command is executed to test the Read/Write functionality

cp /mnt/<file.txt> /mnt/file_copy.txt

Debugging
==========
1. NuttX needs to be compiled in Cygwin environment on Windows.

The following Configuration needs to be set, in order to do source level debugging.

CONFIG_DEBUG_SYMBOLS = y (Set this option, using menuconfig only, DO NOT Enable this as default configuration).

2. Download & Install Renesas e2studio IDE
3. Load the project(NuttX built on Cygwin) as Makefile project with existing code
4. Right click on the project, and select Debug Configurations
5. The binary(NuttX) needs to be loaded using E1/E2 Emulator
6. Select the Device name as R5F565NE and Emulator as E1/E2(whichever is being used)
7. Select Connection type as JTAG
8. Load and run the binary

Flashing NuttX
===============
Alternatively, NuttX binary can be flashed using Renesas flash programmer tool without using e2 studio/Cygwin

Below are the steps mentioned to flash NuttX binary using Renesas flash programmer tool(RFP).

1.In order to flash using Renesas flash programmer tool, nuttx.mot file should be generated.
2. Add the following lines in tools/Unix.mk file :
ifeq ($(CONFIG_MOTOROLA_SREC),y)
	@echo "CP: nuttx.mot"
	$(Q) $(OBJCOPY) $(OBJCOPYARGS) $(BIN) -O srec -I elf32-rx-be-ns nuttx.mot
endif
3. Add CONFIG_MOTOROLA_SREC=y in defconfig file or choose make menucofig->Build Setup-> Binary Output Format->
   Select Motorola SREC format.
4. Download Renesas flash programmer tool from https://www.renesas.com/in/en/products/software-tools/tools/programmer/renesas-flash-programmer-programming-gui.html#downloads
5. Refer to the user manual document, for steps to flash NuttX binary using RFP tool.
