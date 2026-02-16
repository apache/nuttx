=========
ne64badge
=========

.. tags:: vendor:freescale

This README discusses issues unique to NuttX configurations for the Future
Electronics Group NE64 /PoE Badge board based on the MC9S12NE64 hcs12 cpu.

See :doc:` </platforms/hc/m9s12/index>` for more information about the chip this
board uses.

Pin Mapping
===========

===  ===================  ==================  ======================
Pin  Name                 Board signal        Note
===  ===================  ==================  ======================
44   RESET                J3 RESET_L          Also to SW3
57   BKGD/MODC/TAGHI_B    BDM BKGD CON6A
85   PAD0                 VR1                 Potentiometer
86   PAD1                 J3 ANALOG_IN0       Not used on board
87   PAD2                 J3 ANALOG_IN1
88   PAD3                 J3 ANALOG_IN2
89   PAD4                 J3 ANALOG_IN3
70   PHY_TXP              J7 TD+              RJ45 connector
71   PHY_TXN              J7 TD-              RJ45 connector
73   PHY_RXP              J7 RD+              RJ45 connector
74   PHY_RXN              J7 RD-              RJ45 connector
60   PA0/ADDR8/DATA8      J3 ADDR_DATA8       Not used on board
61   PA1/ADDR9/DATA9      J3 ADDR_DATA9
62   PA2/ADDR10/DATA10    J3 ADDR_DATA10
63   PA3/ADDR11/DATA11    J3 ADDR_DATA11
77   PA4/ADDR12/DATA12    J3 ADDR_DATA12
78   PA5/ADDR13/DATA13    J3 ADDR_DATA13
79   PA6/ADDR14/DATA14    J3 ADDR_DATA14
80   PA7/ADDR15/DATA15    J3 ADDR_DATA15
10   PB0/ADDR0/DATA0      J3 ADDR_DATA0       Not used on board
11   PB1/ADDR1/DATA1      J3 ADDR_DATA1
12   PB2/ADDR2/DATA2      J3 ADDR_DATA2
13   PB3/ADDR3/DATA3      J3 ADDR_DATA3
16   PB4/ADDR4/DATA4      J3 ADDR_DATA4
17   PB5/ADDR5/DATA5      J3 ADDR_DATA5
18   PB6/ADDR6/DATA6      J3 ADDR_DATA6
19   PB7/ADDR7/DATA7      J3 ADDR_DATA7
56   PE0/XIRQ_B           BUTTON1             SW1
55   PE1/IRQ_B            J3 IRQ              Not used on board
54   PE2/R_W              J3 RW
53   PE3/LSTRB_B/TAGLO_B  J3 LSTRB
41   PE4/ECLK             J3 ECLK
40   PE5/IPIPE0/MODA      J3 MODA
39   PE6/IPIPE1/MODB      J3 MODB
38   PE7/NOACC/XCLKS_B    pulled low          pulled low
97   PK0/XADR14           N/C                 N/C
98   PK1/XADR15           N/C                 N/C
99   PK2/XADR16           N/C                 N/C
100  PK3/XADR17           N/C                 N/C
103  PK4/XADR18           N/C                 N/C
104  PK5/XADR19           N/C                 N/C
105  PK6/XCS_B            J3 XCS              Not used on board
106  PK7/ECS_B/ROMCTL     J3 ECS
110  PT4/IOC1_4           J3 GPIO8            Not used on board
109  PT5/IOC1_5           J3 GPIO9
108  PT6/IOC1_6           J3 GPIO10
107  PT7/IOC1_7           N/C                 N/C
30   PS0/RXD0             RS232_RX            Eventually maps to J2 RXD
31   PS1/TXD0             RS232_TX            Eventually maps to J2 TXD
32   PS2/RXD1             J3&J4 UART_RX       Not used on board
33   PS3/TXD1             J3&J4 UART_TX
34   PS4/MISO             J3 SPI_MISO
35   PS5/MOSI             J3 SPI_MOSI
36   PS6/SCK              J3 SPI_CLOCK
37   PS7/SS_B             J3 SPI_SS
22   PG0/RXD0/KWG0        J3 GPIO0            Not used on board
23   PG1/RXD1/KWG1        J3 GPIO1
24   PG2/RXD2/KWG2        J3 GPIO2
25   PG3/RXD3/KWG3        J3 GPIO3
26   PG4/RXCLK/KWG4       J3 GPIO4
27   PG5/RXDV/KWG5        J3 GPIO5
28   PG6/RXER/KWG6        J3 GPIO6
29   PG7/KWG7             J3 GPIO7
7    PH0/TXD0/KWH0        N/C                 N/C
6    PH1/TXD1/KWH1        N/C                 N/C
5    PH2/TXD2/KWH2        J4 XBEE_RESET       Not used on board
4    PH3/TXD3/KWH3        J4 XBEE_RSSI        Not used on board
3    PH4/TXCLK/KWH4       BUTTON2             SW2
2    PH5/TXDV/KWH5        J5 XBEE_LOAD_H      Not used on board
1    PH6/TXER/KWH6        J4 XBEE_LOAD_L      Not used on board
8    PJ0/MDC/KWJ0         LED1                D21, red
9    PJ1/MDIO/KWJ1        LED2                D22, red
20   PJ2/CRS/KWJ2         J3 SPI_CS           Not used on board
21   PJ3/COL/KWJ3         N/C
112  PJ6/SDA/KWJ6         J3 I2C_DATA         Not used on board
111  PJ7/SCL/KWJ7         J3 I2C_CLOCK
51   PL6/TXER/KWL6        N/C                 N/C
52   PL5/TXDV/KWL5        N/C                 N/C
58   PL4/COLLED           Collision LED       red
59   PL3/DUPLED           Full Duplex LED     Yellow
81   PL2/SPDLED           100Mbps Speed LED   Yellow
83   PL1/LNKLED           Link Good LED       Green
84   PL0/ACTLED           Activity LED        Yellow
===  ===================  ==================  ======================

Installation
============

.. note::

   Either Linux or Cygwin on Windows can be used for the development
   environment. The source has been built only using the GNU toolchain (see
   below). Other toolchains will likely cause problems.

A GNU GCC-based toolchain is assumed. The PATH environment variable should be
modified to point to the correct path to the HC12 GCC toolchain (if different
from the default in your PATH variable).

If you have no HC12 toolchain, one can be downloaded from the NuttX
Bitbucket download site (https://bitbucket.org/nuttx/buildroot/downloads/).
This GNU toolchain builds and executes in the Linux or Cygwin
environments.

1. You must have already configured NuttX in ``<some-dir>/nuttx``.

   .. code:: console

      $ tools/configure.sh ne64badge:<sub-dir>

2. Download the latest buildroot package into ``<some-dir>``

3. Unpack the buildroot tarball. The resulting directory may have versioning
   information on it like ``buildroot-x.y.z``. If so, rename
   ``<some-dir>/buildroot-x.y.z`` to ``<some-dir>/buildroot``.

   .. code:: console

      $ cd <some-dir>/buildroot
      $ cp boards/m9s12x-defconfig-3.3.6 .config
      $ make oldconfig
      $ make

   If the make fails because it can't find the file to download, you may
   have to locate the file on the internet and download it into the archives/
   directory manually. For example, ``binutils-2.18`` can be found here:
   http://ftp.gnu.org/gnu/binutils/

8. Make sure that the PATH variable includes the path to the newly built
   binaries.

Configurations
==============

This board's configurations can be selected using the following command, where
``<config>`` is one of the configurations listed below.

.. code:: console

   $ ./tools/configure.sh ne64badge:<config>

**GPIO Interrupts**

* ``CONFIG_HCS12_GPIOIRQ``: Enable general support for GPIO IRQs
* ``CONFIG_HCS12_PORTG_INTS``: Enable PortG IRQs
* ``CONFIG_HCS12_PORTH_INTS``: Enable PortH IRQs
* ``CONFIG_HCS12_PORTJ_INTS``: Enable PortJ IRQs

**HCS12 build options**

* ``CONFIG_HCS12_SERIALMON``: Indicates that the target systems uses the
  Freescale serial bootloader.

* ``CONFIG_HCS12_NONBANKED``: Indicates that the target systems does not support
  banking. Only short calls are made; one fixed page is presented in the paging
  window. Only 48Kb of FLASH is usable in this configuration: pages 3e, 3d, then
  3f will appear as a contiguous address space in memory.

**HCS12 Sub-system support**

* ``CONFIG_HCS12_SCI0``
* ``CONFIG_HCS12_SCI1``

**HCS12 specific device driver settings**

* ``CONFIG_SCIn_SERIAL_CONSOLE``: selects SCIn for the console and ttys0
  (default is the SCI0).

* ``CONFIG_SCIn_RXBUFSIZE``: Characters are buffered as received. This specific
  the size of the receive buffer

* ``CONFIG_SCIn_TXBUFSIZE``: Characters are buffered before being sent. This
  specific the size of the transmit buffer

* ``CONFIG_SCIn_BAUD``: The configure BAUD of the UART.

* ``CONFIG_SCIn_BITS``: The number of bits.  Must be either 7 or 8.

* ``CONFIG_SCIn_PARTIY``: 0=no parity, 1=odd parity, 2=even parity, 3=mark 1,
  4=space 0

* ``CONFIG_SCIn_2STOP``: Two stop bits

ostest
------

This configuration directory, performs a simple OS test using
``examples/ostest``.
