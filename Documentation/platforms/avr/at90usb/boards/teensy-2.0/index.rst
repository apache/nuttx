==========
Teensy 2.0
==========

.. tags:: arch:avr, chip:at90usb

This is the documentation for the port of NuttX to the PJRC Teensy++ 2.0 board.
This board is developed by http://pjrc.com/teensy/. The Teensy++ 2.0 is based on
an Atmel AT90USB1286 MCU.

Teensy++ 2.0 Features
=====================

* Based on the 64-pin USB AVR Microcontroller AT90USB1286.
* USB Full Speed (12Mbit/s)
* USB Device Mode
* 120kbof available FLASH memory for programs.
* 8 kbytes SRAM and 4 kbytes of EEPROM
* USB powered
* 16MHz crystal
* 48 General Purpose IO Pins

Pin Usage
=========

AT90USB1286 TQFP64

=== ========================= =============================================
PIN SIGNAL                    BOARD CONNECTION
=== ========================= =============================================
1   (INT.6/AIN.0) PE6         Pad E6
2   (INT.7/AIN.1/UVcon) PE7   Pad E7
3   UVcc                      (Voltage circutry)
4   D-                        USB DP
5   D+                        USB DM
6   UGnd                      GND
7   UCap                      GND (via cap)
8   VBus                      USB VBUS
9   (IUID) PE3                N/C
10  (SS/PCINT0) PB0           Pad B0
11  (PCINT1/SCLK) PB1         Pad B1
12  (PDI/PCINT2/MOSI) PB2     Pad B2
13  (PDO/PCINT3/MISO) PB3     Pad B3
14  (PCINT4/OC.2A) PB4        Pad B4
15  (PCINT5/OC.1A) PB5        Pad B5
16  (PCINT6/OC.1B) PB6        Pad B6
17  (PCINT7/OC.0A/OC.1C) PB7  Pad B7
18  (INT4/TOSC1) PE4          Pad E4
19  (INT.5/TOSC2) PE5         Pad E5
20  RESET                     Switch pulls to ground
21  VCC                       VCC
22  GND                       GND
23  XTAL2                     XTAL (16MHz)
24  XTAL1                     XTAL (16MHz)
25  (OC0B/SCL/INT0) PD0       Pad D0
26  (OC2B/SDA/INT1) PD1       Pad D1
27  (RXD1/INT2) PD2           Pad D2
28  (TXD1/INT3) PD3           Pad D3
29  (ICP1) PD4                Pad D4
30  (XCK1) PD5                Pad D5
31  (T1) PD6                  Pad D6, LED
32  (T0) PD7                  Pad D7
48  PA3 (AD3)                 Pad A3
47  PA4 (AD4)                 Pad A4
46  PA5 (AD5)                 Pad A5
45  PA6 (AD6)                 Pad A6
44  PA7 (AD7)                 Pad A7
43  PE2 (ALE/HWB)             Pad ALE (Pulled down)
42  PC7 (A15/IC.3/CLKO)       Pad C7
41  PC6 (A14/OC.3A)           Pad C6
40  PC5 (A13/OC.3B)           Pad C5
39  PC4 (A12/OC.3C)           Pad C4
38  PC3 (A11/T.3)             Pad C3
37  PC2 (A10)                 Pad C2
36  PC1 (A9)                  Pad C1
35  PC0 (A8)                  Pad C0
34  PE1 (RD)                  Pad E1
33  PE0 (WR)                  Pad E0
64  AVCC                      VCC
63  GND                       GND
62  AREF                      Pad Ref (Capacitor to ground)
61  PF0 (ADC0)                Pad F0
60  PF1 (ADC1)                Pad F1
59  PF2 (ADC2)                Pad F2
58  PF3 (ADC3)                Pad F3
57  PF4 (ADC4/TCK)            Pad F4
56  PF5 (ADC5/TMS)            Pad F5
55  PF6 (ADC6/TDO)            Pad F6
54  PF7 (ADC7/TDI)            Pad F7
53  GND                       GND
52  VCC                       VCC
51  PA0 (AD0)                 Pad A0
50  PA1 (AD1)                 Pad A1
49  PA2 (AD2)                 Pad A2
=== ========================= =============================================

Halfkey Bootloader
==================

Download the Teensy application from http://pjrc.com/teensy/loader.html.
Instructions are available for your OS at that place as well.

Summary:

1. Start Teensy
2. Press button on the Teensy board
3. Select a HEX file (File menu)
4. Select "program" (Operations menu)
5. Reboot (Operations menu).

Serial Console
==============

A serial console is supported on an external MAX232/MAX3232 Connected
on PD2 and PD3:

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
27  (RXD1/INT2) PD2          Pad D2
28  (TXD1/INT3) PD3          Pad D3
=== ======================= =============================================

Plus power and ground. There are numerous ground points and both USB 5V and Vcc
are available.

SD Connection
=============

I have the SD-ADP SD/MMC Card Adaptor from www.gravitech.com
(http://www.gravitech.us/sdcaad.html). Features:

* On-board 3.3V regulator
* Connect directly to 3.3V or 5.0V microcontroller
* Card detect LED
* Includes 11-pin male header
* Board dimension: 2.0"x1.3"

SD-ADP Pinout / SD Connection
-----------------------------

== ==== ============ =======================================================
J2 NAME SD CARD      DESCRIPTION
== ==== ============ =======================================================
 1 VIN   (regulator) Input power to the SD card (3.3V to 6.0V)
 2 GND   3,6,12,13   Common (Connects to the housing of the SD socket)
 3 3V3   4 3.3V      Output voltage from the on-board 3.3V regulator (250mA)
 4 NC    9 NC        Connect to pin 9 on the SD card (not used in SPI mode)
 5 CS    1 DAT3/CS   Chip select (1*)
 6 DI    2 CMD/DI    Serial input data (1*)
 7 SCK   5 SCK       Serial clock (1*)
 8 DO    7 DAT0/DO   Serial output data
 9 IRQ   8 DAT1/IRQ  Interrupt request, connect to pin 8 on the SD card (not used in SPI mode)
10 CD   10 CD        Card detect (active low)
11 WP   11 WP        Write protect
== ==== ============ =======================================================

(1*) Via a 74LCX245 level translator / buff

Teensy SPI Connection
---------------------

== ==== === ========================= =======
J2 NAME PIN NAME                      PAD
== ==== === ========================= =======
 1 VIN  --  Connected to USB +5V
 2 GND  --  Connected to USB GND
 3 3V3  --  Not used                  ---
 4 NC   --  Not used
 5 CS   10  (SS/PCINT0) PB0           Pad B0
 6 DI   12  (PDI/PCINT2/MOSI) PB2     Pad B2
 7 SCK  11  (PCINT1/SCLK) PB1         Pad B1
 8 DO   13  (PDO/PCINT3/MISO) PB3     Pad B3
 9 IRQ  --  Not used                  ---
10 CD   14  (PCINT4/OC.2A) PB4        Pad B4
11 WP   15  (PCINT5/OC.1A) PB5        Pad B5
== ==== === ========================= =======

Toolchains
==========

Read about the tool chains at :doc:`../../index`.

Teensy++ Configuration Options
==============================

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
* ``CONFIG_USARTn_BAUD``: The configure BAUD of the USART.
* ``CONFIG_USARTn_BITS``: The number of bits. Must be either 7 or 8.
* ``CONFIG_USARTn_PARTIY``: 0=no parity, 1=odd parity, 2=even parity
* ``CONFIG_USARTn_2STOP``: Two stop bits

AT90USB specific USB device configuration:

* ``CONFIG_USB_DISABLE_PADREGULATOR``
* ``CONFIG_USB_LOWSPEED``
* ``CONFIG_USB_NOISYVBUS``

Configurations
==============

1. Each Teensy++ configuration is maintained in a sub-directory and can be
   selected as follow:

   .. code:: console

      $ tools/configure.sh teensy-2.0:<subdir>

   Where ``<subdir>`` is one of the configuration sub-directories described in
   the following paragraph.

   .. note::

      You must also copy avr-libc header files, perhaps like:

      .. code:: console

         $ cp -a /cygdrive/c/WinAVR/include/avr include/.

2. These configurations use the mconf-based configuration tool. To change a
   configurations using that tool, you should:

   a. Build and install the kconfig-mconf tool. See ``nuttx/README.txt`` see
      additional ``README.txt`` files in the NuttX tools repository.

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

nsh
---

This is a reduce NuttShell (NSH) configuration using ``apps/example/nsh``. The
serial console is provided on USART1 and can be accessed via an external RS-232
driver as described above under "Serial Console".

ostest
------

This configuration directory, performs a simple OS test using
``apps/examples/ostest``. 

.. warning::

   The OS test is quite large. In order to get it to fit within AVR memory
   constraints, it will probably be necessary to disable some OS features.

usbmsc
------

This configuration directory exercises the USB mass storage class driver at
apps/system/usbmsc. See apps/examples/README.txt for more information.

.. warning::
   
   THIS CONFIGURATION HAS NOT YET BEEN DEBUGGED AND DOES NOT WORK!!!
   ISSUES:

   1. THE SPI DRIVER IS UNTESTED
   2. THE USB DRIVER IS UNTESTED
   3. THE RAM USAGE MIGHT BE EXCESSIVE

   Update 7/11: (1) The SPI/SD driver has been verified, however, (2) I believe
   that the current teensy-2.0/usbmsc configuration uses too much SRAM for the
   system to behave sanely. A lower memory footprint version of the mass storage
   driver will be required before this can be debugged.
