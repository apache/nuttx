README
======

This README file discusses the port of NuttX to “GR-ROSE” board produced by Gadget Renesas.This board features the RX65N (R5F565NEHDFP 100pin QFP)

Contents
========

  - Board Features
  - Status/Open Issues
  - Serial Console
  - LEDs
  - Networking
  - Contents
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
- Micro controller - RX65N (R5F565NEHDFP 100pin QFP) RXv2 core [34 CoreMark/mA]
- ROM/RAM - 2MB/640KB
- Operating Frequency	- 120MHz(12MHz 10 Multiplication)
- RTC Clock - 32.768kHz
- Sensors - Temperature(inside MCU)
- ROS I/F	- Ethernet, USB(rosserial)
- Serial Servo I/F -	TTL x 4, RS-485 x 1
- Analog I/F	- ADC(12bit) x 6, DAC x 1
- Wireless - IEEE 802.11b/g/n
- PMOD I/F - 1 (I2C, SPI, UART)
- External power supply - USB VBUS or 4.5V～18V
- Supply to external - 3.3V, 5V

See the RX65N GRROSE website for further information about this board:

  - http://gadget.renesas.com/en/product/rose.html

Serial Console
==============
RX65N GRROSE supports 12 serial ports (SCI0 - SCI12), however only 5 ports can be tested(SCI0, SCI1, SCI2,
SCI5 & SCI6).

Please find the pin configurations for SCI0, SCI1, SCI2, SCI5 & SCI6

SCI0 Pin Configuration :

-----------
RX65N GRROSE
  Function
-----------
  P21   RXD0
  P20   TXD0
------------

SCI1 Pin Configuration :

-----------
RX65N GRROSE
  Function
-----------
  P30   RXD1
  P26   TXD1
------------

SCI2 Pin Configuration :

-----------
RX65N GRROSE
  Function
-----------
  P12   RXD2
  P13   TXD2
------------

SCI3 Pin Configuration :

-----------
RX65N GRROSE
  Function    (connected to WiFi module)
-----------
  P25   RXD3
  P23   TXD3
------------

SCI5 Pin Configuration :

-----------
RX65N GRROSE
  Function
-----------
  PC2   RXD5
  PC3   TXD5
------------

SCI6 Pin Configuration :

-----------
RX65N GRROSE
  Function
-----------
  P33   RXD6
  P32   TXD6
------------

SCI8 Pin Configuration :

-----------
RX65N GRROSE
  Function   (Half duplication mode with RS485 driver)
-----------
  PC6   RXD8
  PC7   TXD8
  PC5   Direction (L=TX, H=RX)

Serial Connection Configuration
--------------------------
1. GRROSE board needs to be connected to PC terminal, using USB to Serial Chip.
2. Connect TX of USB to serial chip to RX of SCIX(0,1,2,5,6)
3. Connect RX of USB to serial chip to TX of SCIX(0,1,2,5,6)
4. Connect GND to GND pin.
5. Configure Teraterm to 115200 baud.

LEDs
====

  The RX65N GRROSE board has 2 LED's, 1 Power LED(LED3) and 2 User LED's(LED1, LED2),which are enabled through software.

  If enabled the LED is simply turned on when the board boots
  successfully, and is blinking on panic / assertion failed.

Networking
==========

Ethernet Connections
-----------

  ------        ---------
  RX65N
  GRROSE        Ethernet
  Pin           Function
  ------        ---------
  PA4           ET0_MDC
  PA3           ET0_MDIO
  PB2           REF50CK0
  PB7           RMII0_CRS_DV
  PB1           RMII0_RXD0
  PB0           RMII0_RXD1
  PB3           RMII0_RX_ER
  PB5           RMII0_ETXD0
  PB6           RMII0_ETXD1
  PB4           RMII0_TXD_EN
  PA5           ET0_LINKSTA
  PA6_ET_RST    ETHER reset
  ------         ---------
NuttX Configurations
---------------
The following configurations, need to be enabled for network.

CONFIG_RX65N_EMAC=y    : Enable the EMAC Peripheral for RX65N
CONFIG_RX65N_EMAC0=y   : Enable the EMAC Peripheral for RX65N
CONFIG_RX65N_EMAC0_PHYSR=30 : Address of PHY status register on LAN8720A
CONFIG_RX65N_EMAC0_PHYSR_100FD=0x18  : Needed for LAN8720A
CONFIG_RX65N_EMAC0_PHYSR_100HD=0x08  : "    " " " "     "
CONFIG_RX65N_EMAC0_PHYSR_10FD=0x14   : "    " " " "     "
CONFIG_RX65N_EMAC0_PHYSR_10HD=0x04   : "    " " " "     "
CONFIG_RX65N_EMAC0_PHYSR_ALTCONFIG=y : "    " " " "     "
CONFIG_RX65N_EMAC0_PHYSR_ALTMODE=0x1c : "    " " " "     "
CONFIG_RX65N_EMAC0_RMII=y
CONFIG_RX65N_EMAC0_PHYADDR=0 : LAN8720A PHY is at address 1

CONFIG_SCHED_WORKQUEUE=y : Work queue support is needed
CONFIG_SCHED_HPWORK=y    :  High Priority Work queue support
CONFIG_SCHED_LPWORK=y    :  Low Priority Work queue support

Using the network with NSH
--------------------------
The IP address is configured using DHCP, using the below mentioned configurations :

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

On the host side, you should also be able to ping the RX65N-GRROSE:

  $ ping 10.75.24.53

Configure UDP blaster application as mentioned below :

CONFIG_EXAMPLES_UDPBLASTER_HOSTIP=0x0a4b1801  (10.75.24.1) ------> Gateway IP
CONFIG_EXAMPLES_UDPBLASTER_NETMASK=0xfffffe00 (255.255.254.0) --------> Netmask
CONFIG_EXAMPLES_UDPBLASTER_TARGETIP=0x0a4b189b (10.75.24.155) ---------> Target IP

RSPI
-----------

For GRROSE board only channel 1 can be tested since RSPI channel1 pinout is only brought out as
Pin number 2 and 3 in CN4 is used for MOSIB and MISOB respectively.

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
The following testing is executed as part of RSPI testing on RX65N target for GRROSE board

On GRROSE board only channel 1 can be tested since RSPI channel1 pinout is only brought out.

Following command can be used for testing RSPI communication to slave device.
spi exch -b 0 -x 4 aabbccdd
where b is bus number and x is Number of word to exchange.

RIIC Configurations
--------------------------
The following configurations need to be enabled for RIIC

CONFIG_SYSTEM_I2CTOOL=y

RIIC Testing
------------------------

On GRROSE board, none of the RIIC channel pins are brought out in the board so not tested for communication.

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

1. NuttX needs to be compiled in Cygwin.

The following Configuration needs to be set, in order to do source level debugging.

CONFIG_DEBUG_SYMBOLS = y (Set this option, using menuconfig only, DO NOT Enable this as default configuration).

2. Download & Install Renesas e2studio IDE.
3. Load the project(NuttX built on Cygwin) as Makefile project with existing code
4. Right click on the project, and select Debug Configurations.
5. The binary(NuttX) needs to be loaded using E1/E2 Emulator.
6. Select the Device name as R5F565NE and Emulator as E1/E2(whichever is being used)
7. Select Connection type as FINE.
8. Load and run the binary.

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
