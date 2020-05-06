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
  - IPv6 Integration
  - HTTP Server Integration on IPv4
  - DHCP Client Integration on IPv4
  - DHCP Server Integration on IPv4
  - FTP Server Integration on IPv4
  - FTP Client Integration on IPv4
  - TFTP Client Integration on IPv4
  - Telnet Server Integration on IPv4
  - Telnet Client Integration on IPv4
  - Ustream Socket Integration on IPv4
  - Udgram Socket Integration on IPv4
  - SMTP Client Integration on IPv4
  - Raw Socket Integration
  - Custom User Socket Integration
  - IGMPv2 Integration
  - Inherit telnet server Integration
  - VNC Server Integration
  - PPPD Integration
  - HTTP Client Integration
  - NTP Client Integration
  - NFS Client Integration
  - MLD Integration
  - ICMPv6 AutoConfig Integration
  - IP Forwarding Integration for IPv4
  - DNS Name Resolution Integration for IPv4
  - LINK MONITOR Integration
  - RTC
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

Status/Open Issues
==================
Ethernet
---------
1.Observed instability in Link Management, due to difference in hardware design.(No Separate Interrupt line for PHY)
2.Currently tested only ping and udpblaster application.
3. Executed long run ping and udpblaster stress test for 12 hrs. Code is able to execute for 12hrs without any breakage.

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
2. Add the following lines in tools/Makefile.unix file :
ifeq ($(CONFIG_MOTOROLA_SREC),y)
	@echo "CP: $(NUTTXNAME).mot"
	$(Q) $(OBJCOPY) $(OBJCOPYARGS) $(BIN) -O srec -I elf32-rx-be-ns $(NUTTXNAME).mot
endif
3. Add CONFIG_MOTOROLA_SREC=y in defconfig file or choose make menucofig->Build Setup-> Binary Output Format->
   Select Motorola SREC format.
4. Download Renesas flash programmer tool from https://www.renesas.com/in/en/products/software-tools/tools/programmer/renesas-flash-programmer-programming-gui.html#downloads
5. Refer to the user manual document, for steps to flash NuttX binary using RFP tool.
Changes Made in NuttX 8.2 Code
================================
1. In wd_start.c file, in function wd_expiration(), typecasting is done when the signal handler nxsig_timeout() is invoked.
2. In rtc.c, (drivers/timers/rtc.c) file, in function rtc_periodic_callback(), alarminfo->active = false is commented.
The reason being, periodic interrupt should not be disabled. Uncommenting the above mentioned line (alarminfo->active = false), will make the periodic interrupt come only once.
