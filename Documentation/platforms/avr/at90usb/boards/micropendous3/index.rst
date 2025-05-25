==============
Micropendous 3
==============

.. tags:: arch:avr, chip:at90usb

This is the documentation page for the port of NuttX to the Micropendous 3
board. This board is developed by `opendous
<http://code.google.com/p/opendous/>`_. The Micropendous 3 is based on an Atmel
AT90USB646, 647, 1286 or 1287 MCU. NuttX was ported using the AT90USB647
version. As of this writing, documentation for the Micropendous board is
available `here <http://code.google.com/p/micropendous/wiki/Micropendous3>`_

Micropendous3 Features
======================

* Based on the 64-pin USB AVR Microcontrollers: AT90USB646, AT90USB647,
* AT90USB1286, or AT90USB1287.
* USB Full Speed (12Mbit/s)
* USB Device Mode (Host mode supported with AT90USBxx7 devices)
* 60kb (AT90USB64) or 120kb (AT90USB128) of available FLASH memory for
* your programs (4kb(AT90USB64)/8kb(AT90USB128) used by USB bootloader -
* stock Atmel or LUFA)
* 4 kbytes SRAM and 2 kbytes of EEPROM (AT90USB64) or 8 kbytes SRAM and 4
* kbytes of EEPROM (AT90USB128)
* External SRAM is possible.  Layout for CY7C1019D 1-Mbit SRAM (unpopulated)
* USB powered
* 16MHz crystal
* 48 General Purpose IO Pins (47 with external SRAM)
* Vcc=VBUS jumper selects whether USB VBUS or an external supply is used
* to power the board
* RESET and HWB buttons to enable firmware loading over USB (no external
* programmer required)
* HWB can be used as a user button
* USB-A Plug
* JTAG header
* Size LxWxH (including headers): 3.15" x 0.8" x 0.6" =~ 8cm x 2cm x 1.5cm
* Completely OpenHardware Design

Pin Usage
=========

AT90USB90128/64 TQFP64

=== ======================== =============================================
PIN SIGNAL                   BOARD CONNECTION
=== ======================== =============================================
1   (INT.6/AIN.0) PE6        J3-25 E6, CY7C1019D ^CE (Unpopulated)
2   (INT.7/AIN.1/UVcon) PE7  J3-26 E7, CY7C1019D A16 (Unpopulated)
3   UVcc
4   D-                       USB DP
5   D+                       USB DM
6   UGnd                     GND
7   UCap                     GND (via cap)
8   VBus                     USB VBUS
9   (IUID) PE3               J3-22 E3
10  (SS/PCINT0) PB0          J3-28 B0
11  (PCINT1/SCLK) PB1        J3-29 B1
12  (PDI/PCINT2/MOSI) PB2    J3-30 B2
13  (PDO/PCINT3/MISO) PB3    J3-31 B3
14  (PCINT4/OC.2A) PB4       J3-32 B4
15  (PCINT5/OC.1A) PB5       J3-33 B5
16  (PCINT6/OC.1B) PB6       J3-34 B6
17  (PCINT7/OC.0A/OC.1C) PB7 J3-35 B7
18  (INT4/TOSC1) PE4         J3-23 E4
19  (INT.5/TOSC2) PE5        J3-24 E5
20  RESET                    SW1
21  VCC                      VCC
22  GND                      GND
23  XTAL2                    X1
24  XTAL1                    X1
25  (OC0B/SCL/INT0) PD0      J3-36 D0
26  (OC2B/SDA/INT1) PD1      J3-37 D1
27  (RXD1/INT2) PD2          J3-38 D2
28  (TXD1/INT3) PD3          J3-39 D3
29  (ICP1) PD4               J3-40 D4
30  (XCK1) PD5               J3-41 D5
31  (T1) PD6                 J3-42 D6
32  (T0) PD7                 J3-43 D7
48  PA3 (AD3)                J3-14 A3, 74AHC573 D3, CY7C1019D O3 (Unpopulated)
47  PA4 (AD4)                J3-15 A4, 74AHC573 D4, CY7C1019D O4 (Unpopulated)
46  PA5 (AD5)                J3-16 A5, 74AHC573 D5, CY7C1019D O5 (Unpopulated)
45  PA6 (AD6)                J3-17 A6, 74AHC573 D6, CY7C1019D O6 (Unpopulated)
44  PA7 (AD7)                J3-18 A7, 74AHC573 D7, CY7C1019D O7 (Unpopulated)
43  PE2 (ALE/HWB)            SW-2 (pulled-up), J3-21 E2, 74AHC573 Cp
42  PC7 (A15/IC.3/CLKO)      J3-51 C7, CY7C1019D A15 (Unpopulated)
41  PC6 (A14/OC.3A)          J3-50 C6, CY7C1019D A14 (Unpopulated)
40  PC5 (A13/OC.3B)          J3-49 C5, CY7C1019D A13 (Unpopulated)
39  PC4 (A12/OC.3C)          J3-48 C4, CY7C1019D A12 (Unpopulated)
38  PC3 (A11/T.3)            J3-47 C3, CY7C1019D A11 (Unpopulated)
37  PC2 (A10)                J3-46 C2, CY7C1019D A10 (Unpopulated)
36  PC1 (A9)                 J3-45 C1, CY7C1019D A9  (Unpopulated)
35  PC0 (A8)                 J3-44 C0, CY7C1019D A8  (Unpopulated)
34  PE1 (RD)                 J3-20 E1, CY7C1019D ^OE (Unpopulated)
33  PE0 (WR)                 J3-19 E0, CY7C1019D ^WE (Unpopulated)
64  AVCC                     (Power circuitry)
63  GND                      GND
62  AREF                     J3-2 AREF, (Power circuitry)
61  PF0 (ADC0)               J3-3 F0
60  PF1 (ADC1)               J3-4 F1
59  PF2 (ADC2)               J3-5 F2
58  PF3 (ADC3)               J3-6 F3
57  PF4 (ADC4/TCK)           J3-7 F4, JTAG TCK
56  PF5 (ADC5/TMS)           J3-8 F5, JTAG TMS
55  PF6 (ADC6/TDO)           J3-9 F6, JTAG TD0
54  PF7 (ADC7/TDI)           J3-20 F7, JTAG TDI
53  GND                      GND
52  VCC                      VCC
51  PA0 (AD0)                J3-11 A0, 74AHC573 D0, CY7C1019D O0 (Unpopulated)
50  PA1 (AD1)                J3-12 A1, 74AHC573 D1, CY7C1019D O1 (Unpopulated)
49  PA2 (AD2)                J3-13 A2, 74AHC573 D2, CY7C1019D O2 (Unpopulated)
=== ======================== =============================================

Atmel AVRISP mkII Connection
============================

ISP6PIN Header

.. code:: text
   
         1  2
   MISO  o  o VCC
    SCK  o  o MOSI
   RESET o  o GND

Micropendous 3 JTAG (JTAG10PIN Connector)

.. code:: text

       1  2                 1  2
   TCK o  o GND         TCK o  o GND
   TDO o  o VCC         TDO o  o VTref
   TMS o  o RESET       TMS o  o nSRST
   VCC o  o N/C             o  o (nTRST)
   TDI o  o GND         TDI o  o GND

JTAGICE mkII Connection to 10-pin Header

==================== =====================
10PIN Header         6PIN Header
==================== =====================
Pin 1 TCK            Pin 3 SCK
Pin 2 GND            Pin 6 GND
Pin 3 TDO            Pin 1 MISO
Pin 4 VTref          Pin 2 Vcc
Pin 6 nSRT           Pin 5 Reset
Pin 9 TDI            Pin 4 MOSI
==================== =====================

DFU Bootloader
==============

There is also an DFU bootloader that resides in the upper 8Kb of FLASH (unless
you ERASE the flash with ICE). You can enter this bootloader (if it is in FLASH)
by:

Holding both the SW1 (RESET) and SW2, then releasing SW1 while continuing to
hold SW2. SW2 connects to the PE2/HWB signal and causes a reset into the
bootloader memory region.

Then you can use FLIP to load code into FLASH (available at the Atmel Web Site).
The DFU USB driver for the DFU bootload is available in the usb subdirectory in
the FLIP installation location.

Toolchains
==========

Read about the tool chains at :doc:`../../index`.

Serial Console
==============

A serial console is supported on an external MAX232/MAX3232 Connected on PD2 and
PD3:

**Port D, Bit 2: RXD1, Receive Data (Data input pin for the USART1).**

When the USART1 receiver is enabled this pin is configured as an input
regardless of the value of DDD2. When the USART forces this pin to be an input,
the pull-up can still be controlled by the PORTD2 bit.

**Port D, Bit 3: TXD1, Transmit Data (Data output pin for the USART1).**

When the USART1 Transmitter is enabled, this pin is configured as an output
regardless of the value of DDD3.

AT90USB90128/64 TQFP64

=== ======================= =============================================
PIN SIGNAL                  BOARD CONNECTION
=== ======================= =============================================
27  (RXD1/INT2) PD2           J3-38 D2
28  (TXD1/INT3) PD3           J3-39 D3
=== ======================= =============================================

Micropendous 3 Configuration Options
====================================

Individual subsystems can be enabled:

* ``CONFIG_AVR_INT0=n``
* ``CONFIG_AVR_INT1=n``
* ``CONFIG_AVR_INT2=n``
* ``CONFIG_AVR_INT3=n``
* ``CONFIG_AVR_INT4=n``
* ``CONFIG_AVR_INT5=n``
* ``CONFIG_AVR_INT6=n``
* ``CONFIG_AVR_INT7=n``
* ``CONFIG_AVR_USBHOST=n``
* ``CONFIG_AVR_USBDEV=n``
* ``CONFIG_AVR_WDT=n``
* ``CONFIG_AVR_TIMER0=n``
* ``CONFIG_AVR_TIMER1=n``
* ``CONFIG_AVR_TIMER2=n``
* ``CONFIG_AVR_TIMER3=n``
* ``CONFIG_AVR_SPI=n``
* ``CONFIG_AVR_USART1=y``
* ``CONFIG_AVR_ANACOMP=n``
* ``CONFIG_AVR_ADC=n``
* ``CONFIG_AVR_TWI=n``

If the watchdog is enabled, this specifies the initial timeout.  Default
is maximum supported value.

* ``CONFIG_WDTO_15MS``
* ``CONFIG_WDTO_30MS``
* ``CONFIG_WDTO_60MS``
* ``CONFIG_WDTO_120MS``
* ``CONFIG_WDTO_1250MS``
* ``CONFIG_WDTO_500MS``
* ``CONFIG_WDTO_1S``
* ``CONFIG_WDTO_2S``
* ``CONFIG_WDTO_4S``
* ``CONFIG_WDTO_8S``

AT90USB specific device driver settings:

* ``CONFIG_USARTn_SERIAL_CONSOLE``: selects the USARTn for the console and ttys0
  (default is no serial console).
* ``CONFIG_USARTn_RXBUFSIZE``: Characters are buffered as received. This
  specific the size of the receive buffer
* ``CONFIG_USARTn_TXBUFSIZE``: Characters are buffered before being sent.  This
  specific the size of the transmit buffer
* ``CONFIG_USARTn_BAUD``: The configure BAUD of the USART.  Must be
* ``CONFIG_USARTn_BITS``: The number of bits.  Must be either 7 or 8.
* ``CONFIG_USARTn_PARTIY``: 0=no parity, 1=odd parity, 2=even parity
* ``CONFIG_USARTn_2STOP``: Two stop bits

Configurations
==============

1. Each Micropendous3 configuration is maintained in a sub-directory and
   can be selected as follows:

   .. code::  console

      $ tools/configure.sh micropendous3:<subdir>

   Where ``<subdir>`` is one of the configuration sub-directories described in
   the following paragraph.

   .. note::

      You must also copy avr-libc header files, perhaps like:

      .. code:: console

         $ cp -a /cygdrive/c/WinAVR/include/avr include/.

2. These configurations use the mconf-based configuration tool. To change a
   configurations using that tool, you should:

   a. Build and install the kconfig-mconf tool. See ``nuttx/README.txt``
      see additional README.txt files in the NuttX tools repository.

   b. Execute ``make menuconfig`` in ``nuttx/`` in order to start the
      reconfiguration process.

3. By default, all configurations assume the NuttX Buildroot toolchain
   under Cygwin with Windows. This is easily reconfigured:

   * ``CONFIG_HOST_WINDOWS=y``
   * ``CONFIG_WINDOWS_CYGWIN=y``
   * ``CONFIG_AVR_BUILDROOT_TOOLCHAIN=y``

4. Build with GCC disables CONFIG_DEBUG_OPT_UNUSED_SECTIONS by default. This is
   because the linker script was not checked to determine if it properly
   prevents removal of sections which the linker considers unreferenced but
   which must be present in the binary.

hello
-----

The simple ``apps/examples/hello`` "Hello, World!" example.

FLASH/SRAM Requirements (as of 6/16/2011):

.. code:: console

   $ avr-nuttx-elf-size nuttx
    text    data     bss     dec     hex filename
   24816     978     308   26102    65f6 nuttx

Strings are in SRAM.
