=====
Amber
=====

.. tags:: chip:atmega128, arch:avr

This is the documentation for the `SoC Robotics Amber Web Server
<https://soc-robotics.com/product/Amber_Specs/Amber_Processor.html>`_ that is
based on the Atmel AVR ATMega128 MCU. There is not much there yet and what is
there is untested due to tool-related issues.

Documentation for the board is available here:

* http://www.soc-robotics.com/product/Amber_Specs/Amber_Processor.html
* http://www.soc-robotics.com/pdfs/Amber%201-5a%20Hardware%20Reference%20Guide.pdf

Features
========

* 17.56MHz ATmega128 Atmel 8bit AVR RISC Processor
* 128Kbyte Flash
* 64Kbyte RAM
* 10BaseT Ethernet Port
* High Speed Serial Port
* 8Ch 10bit Analog Input port
* 16 Digital IO ports
* Expansion bus for daughter cards
* LED status indicators
* ISP Programming port
* 7-14VDC input
* Power via Ethernet port

Pinout (PCB Rev 1.5a)
=====================

=== ================ =============================
Pin ID               Amber board connection
=== ================ =============================
 1  PEN              Pulled-up
 2  PE0 (RXD0/PDI)   MAX202ECWED T1IN or J7-1, ISP-PDI (via 74HC5053), J5-26
 3  PE1 (TXD0/PDO)   MAX202ECWED A1OUT or J7-9, ISP-PDO (via 74HC5053), J5-25
 4  PE2 (XCK0/AIN0)  MAX202ECWED T2IN, J5-24
 5  PE3 (OC3A/AIN1)  MAX202ECWED A2OUT, J5-23
 6  PE4 (OC3B/INT4)  J5-22
 7  PE5 (OC3C/INT5)  J5-21, RTL8019AS INT 0, TP5 PE5
 8  PE6 (T3/INT6)    J5-20
 9  PE7 (ICP3/INT7)  J5-19
10  PB0 (SS)         Pull up of SS SPI master
11  PB1 (SCK)        J7-7, ISP_SCK (via 74HC4053) and AT45D011 SCK, J5-17
12  PB2 (MOSI)       AT45D011 SI. J5-16
13  PB3 (MISO)       AT45D011 SO, J5-15
14  PB4 (OC0)        AT45D011 CS\, J5-14
15  PB5 (OC1A)       J5-13
16  PB6 (OC1B)       J5-12
17  PB7 (OC2/OC1C)   J5-11
18  PG3/TOSC2        32.768KHz XTAL
19  PG4/TOSC1        32.768KHz XTAL
20  RESET            RESET
21  VCC
22  GND              GND
23  XTAL2            14.7456MHz XTAL
24  XTAL1            14.7456MHz XTAL
25  PD0 (SCL/INT0)   J5-10
26  PD1 (SDA/INT1)   J5-9
27  PD2 (RXD1/INT2)  J5-8, MAX488CSA RO (RS-485)
28  PD3 (TXD1/INT3)  J5-7, MAX488CSA DI (RS-485)
29  PD4 (ICP1)       J5-6
30  PD5 (XCK1)       J5-5
31  PD6 (T1)         J5-4
32  PD7 (T2)         J5-3
48  PA3 (AD3)        J5-?, 74HC5730, 62246DLP-7, RTL8019AS
47  PA4 (AD4)        J5-?, 74HC5730, 62246DLP-7, RTL8019AS
46  PA5 (AD5)        J5-?, 74HC5730, 62246DLP-7, RTL8019AS
45  PA6 (AD6)        J5-?, 74HC5730, 62246DLP-7, RTL8019AS
44  PA7 (AD7)        J5-?, 74HC5730, 62246DLP-7, RTL8019AS
43  PG2 (ALE)        J5-1, 74HC5730, 62246DLP-7, RTL8019AS
42  PC7 (A15)        TP4 A15, J5-27, 74HC5730
41  PC6 (A14)        J5-28, 74HC5730, 62246DLP-7, RTL8019AS
40  PC5 (A13)        J5-29, 74HC5730, 62246DLP-7, RTL8019AS
39  PC4 (A12)        J5-30, 74HC5730, 62246DLP-7, RTL8019AS
38  PC3 (A11)        J5-31, 74HC5730, 62246DLP-7, RTL8019AS
37  PC2 (A10)        J5-32, 74HC5730, 62246DLP-7, RTL8019AS
36  PC1 (A9)         J5-33, 74HC5730, 62246DLP-7, RTL8019AS
35  PC0 (A8)         J5-34, 74HC5730, 62246DLP-7, RTL8019AS
34  PG1 (RD)         TP2 RD\, J5-52, 62246DLP-7, RTL8019AS
33  PG0 (WR)         TP3 WR\, J5-51, 62246DLP-7, RTL8019AS
64  AVCC
63  GND              GND
62  AREF             (analog supply)
61  PF0 (ADC0)       J6-5, PDV-P9 Light Sensor
60  PF1 (ADC1)       J6-7, Thermister
59  PF2 (ADC2)       J6-9, MXA2500GL Dual Axis Accesserometer, AOUTX
58  PF3 (ADC3)       J6-11, MXA2500GL Dual Axis Accesserometer, AOUTY
57  PF4 (ADC4/TCK)   J6-13, MXA2500GL Dual Axis Accesserometer, TOUT
56  PF5 (ADC5/TMS)   J6-15
55  PF6 (ADC6/TDO)   J6-17
54  PF7 (ADC7/TDI)   J6-19
53  GND              GND
52  VCC
51  PA0 (AD0)        J5-?, 74HC5730, 62246DLP-7, RTL8019AS
50  PA1 (AD1)        J5-?, 74HC5730, 62246DLP-7, RTL8019AS
49  PA2 (AD2)        J5-?, 74HC5730, 62246DLP-7, RTL8019AS
=== ================ =============================

Switches and Jumpers
--------------------

**ISP/UART0**

* JP1: DTE/DCE selection
* JP2
* JP5
* J11: STK500 Enable

**ADC**

* JP8
* JP9

**Networking**

* JP10

**RS-485**

* J8
* J9
* J10

Atmel AVRISP mkII Connection
----------------------------

**ISP6PIN Header**

.. code:: text

          1  2
    MISO  o  o VCC
     SCK  o  o MOSI
   RESET\ o  o GND

**ISP10PIN Connector**

.. code:: text

          1  2
     MOSI o  o Vcc   - ISP-PDI: PE0/PDI/RX0 via 74HC5053
      LED o  o GND   - ISP-PROG: J11/GND, to 74HC5053 and LED
   RESET\ o  o GND   - to 74HC505
     SCK  o  o GND   - ISP_SCK: SCK, PB0/SS\
     MISO o  o GND   - ISP-PDO: PE1/PD0/TX0 via 74HC5053

   Board Orientation

     |
     | +-----+
     | + O O |
     | + O O |
     | + O O
     | + O O |
     | + O x | PIN 1
     | +-----+
     |

**AVRISP mkII Connection to 10-pin Header**

10PIN Header:

====== ========
Pin    Function
====== ========
Pin 1  MOSI
Pin 2  Vcc
Pin 3  LED
Pin 4  GND
Pin 5  RESET\
Pin 6  GND
Pin 7  SCK
Pin 8  GND
Pin 9  MISO
Pin 10 GND
====== ========


6PIN Header:

===== ===============
Pin   Function
===== ===============
Pin 4 MOSI
Pin 2 Vcc
--    Controlled via J11
Pin 6 GND
Pin 5 RESET\
--    N/C
Pin 3 SCK
--    N/C
Pin 1 MISO
--    N/C
===== ===============

Installation
============

The toolchain may be selected using the kconfig-mconf tool (via ``make
menuconfig``), by editing the existing configuration file (defconfig), or by
overriding the toolchain on the make commandline with
``CONFIG_AVR_TOOLCHAIN=<toolchain>``.

The valid values for ``<toolchain>`` are BUILDROOT, CROSSPACK, LINUXGCC and
WINAVR.

Buildroot
---------

There is a DIY buildroot version for the AVR boards here:
http://bitbucket.org/nuttx/buildroot/downloads/. See the following section for
details on building this toolchain.

You may also have to modify the PATH environment variable if your make cannot
find the tools.

After configuring NuttX, make sure that ``CONFIG_AVR_BUILDROOT_TOOLCHAIN=y`` is
set in your ``.config`` file.

WinAVR
------

For Cygwin development environment on Windows machines, you can use
WinAVR: http://sourceforge.net/projects/winavr/files/

You may also have to modify the PATH environment variable if your make cannot
find the tools.

After configuring NuttX, make sure that ``CONFIG_AVR_WINAVR_TOOLCHAIN=y`` is set
in your ``.config`` file.

.. warning::

   There is an incompatible version of cygwin.dll in the WinAVR/bin
   directory!  Make sure that the path to the correct cygwin.dll file precedes
   the path to the WinAVR binaries!

Linux
-----

For Linux, there are widely available avr-gcc packages. On Ubuntu, use:

.. code:: console

   $ sudo apt-get install gcc-avr gdb-avr avr-libc

After configuring NuttX, make sure that ``CONFIG_AVR_LINUXGCC_TOOLCHAIN=y`` is
set in your ``.config`` file.

macOS
-----

For macOS, the CrossPack for AVR toolchain is available from:
http://www.obdev.at/products/crosspack/index.html

This toolchain is functionally equivalent to the Linux GCC toolchain.

Windows Native Toolchains
-------------------------

The WinAVR toolchain is a Windows native toolchain. There are several
limitations to using a Windows native toolchain in a Cygwin environment.
The three biggest are:

1. The Windows toolchain cannot follow Cygwin paths. Path conversions are
   performed automatically in the Cygwin makefiles using the 'cygpath'
   utility but you might easily find some new path problems. If so, check
   out ``cygpath -w``

2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic
   links are used in NuttX (e.g., include/arch).  The make system works
   around these  problems for the Windows tools by copying directories
   instead of linking them.  But this can also cause some confusion for
   you:  For example, you may edit a file in a "linked" directory and find
   that your changes had no effect. That is because you are building the
   copy of the file in the "fake" symbolic directory.  If you use a
   Windows toolchain, you should get in the habit of making like this:

   .. code:: console

      $ make clean_context all

   An alias in your ``.bashrc`` file might make that less painful.

An additional issue with the WinAVR toolchain, in particular, is that it
contains an incompatible version of the Cygwin DLL in its ``bin/`` directory.
You must take care that the correct Cygwin DLL is used.

NuttX buildroot Toolchain
-------------------------

If NuttX buildroot toolchain source tarball cne can be downloaded from the NuttX
Bitbucket download site (https://bitbucket.org/nuttx/nuttx/downloads/). This GNU
toolchain builds and executes in the Linux or Cygwin environment.

1. You must have already configured NuttX in ``<some-dir>/nuttx``.

   .. note::

      You also must copy avr-libc header files into the NuttX include directory
      with command perhaps like:

      .. code:: console

         $ cp -a /cygdrive/c/WinAVR/include/avr include/.

2. Download the latest buildroot package into ``<some-dir>``

3. Unpack the buildroot tarball. The resulting directory may have versioning
   information on it like 'buildroot-x.y.z'. If so, rename
   ``<some-dir>/buildroot-x.y.z`` to ``<some-dir>/buildroot``.

4. Run the following commands:

   .. code:: console

      $ cd <some-dir>/buildroot
      $ cp boards/avr-defconfig-4.5.2 .config
      $ make oldconfig
      $ make

5. Make sure that the PATH variable includes the path to the newly built
   binaries.

See the file boards/README.txt in the buildroot source tree. That has more
detailed PLUS some special instructions that you will need to follow if you are
building a toolchain for Cygwin under Windows.

``avr-libc``
------------

**Header Files**

In any case, header files from avr-libc are required:
http://www.nongnu.org/avr-libc/. A snapshot of avr-lib is included in the WinAVR
installation. For Linux development platforms, avr-libc package is readily
available (and would be installed in the apt-get command shown above).  But if
you are using the NuttX buildroot configuration on Cygwin, then you will have to
build get avr-libc from binaries.

**Header File Installation**

The NuttX build will required that the AVR header files be available via
the NuttX include directory. This can be accomplished by either copying
the avr-libc header files into the NuttX include directory:

.. code:: console

   $ cp -a <avr-libc-path>/include/avr <nuttx-path>/include/.

Or simply using a symbolic link:

.. code:: console

   $ ln -s <avr-libc-path>/include/avr <nuttx-path>/include/.

.. note::

   It may not be necessary to have a built version of avr-lib; only header files
   are required.  But if you choose to use the optimized library functions of
   the floating point library, then you may have to build avr-lib from sources.
   Below are instructions for building avr-lib from fresh sources:

   1. Download the avr-libc package from:
      http://savannah.nongnu.org/projects/avr-libc/. I am using
      avr-lib-1.7.1.tar.bz2

   2. Unpack the tarball and ``cd`` into it.

      .. code:: console

         $ tar jxf avr-lib-1.7.1.tar.bz2
         $ cd avr-lib-1.7.1

   3. Configure avr-lib. Assuming that WinAVR is installed at the following
      location:

      .. code:: console

         $ export PATH=/cygdrive/c/WinAVR/bin:$PATH
         $ ./configure --build=`./config.guess` --host=avr

      This takes a *long* time.

   4. Make avr-lib by running ``make``.

      This also takes a long time because it generates variants for nearly
      all AVR chips.

   5. Install avr-lib by running ``make install``.

Configurations
==============

Amber Web Server configurations can be selected as follows:

.. code:: console

   $ tools/configuresh amber:<config>

Where ``<config>`` i one of the configurations list below.

.. note::

   You must also cop avr-libc header files, perhaps like:

   .. code:: console

      $ cp -a /cygdrve/c/WinAVR/include/avr include/.

hello
-----

The simple "Hello, World!" example.
