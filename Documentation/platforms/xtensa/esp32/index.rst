===============
Espressif ESP32
===============

The ESP32 is a series of single and dual-core SoCs from Espressif
based on Harvard architecture Xtensa LX6 CPUs and with on-chip support
for Bluetooth and WiFi.

All embedded memory, external memory and peripherals are located on the
data bus and/or the instruction bus of these CPUs. With some minor
exceptions, the address mapping of two CPUs is symmetric, meaning they
use the same addresses to access the same memory. Multiple peripherals in
the system can access embedded memory via DMA.

On dual-core SoCs, the two CPUs are typically named "PRO_CPU" and "APP_CPU"
(for "protocol" and "application"), however for most purposes the
two CPUs are interchangeable.

Toolchain
=========

You can use the prebuilt `toolchain <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-tools.html#xtensa-esp32-elf>`__
for Xtensa architecture and `OpenOCD <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-tools.html#openocd-esp32>`__
for ESP32 by Espressif.

For flashing firmware, you will need to install ``esptool.py`` by running::

    pip install esptool

Building from source
--------------------

You can also build the toolchain yourself. The steps to
build the toolchain with crosstool-NG on Linux are as follows

.. code-block:: console

  $ git clone https://github.com/espressif/crosstool-NG.git
  $ cd crosstool-NG
  $ git checkout esp-2019r2
  $ git submodule update --init

  $ ./bootstrap && ./configure --enable-local && make

  $ ./ct-ng xtensa-esp32-elf
  $ ./ct-ng build

  $ chmod -R u+w builds/xtensa-esp32-elf

  $ export PATH="crosstool-NG/builds/xtensa-esp32-elf/bin:$PATH"

These steps are given in setup guide in
`ESP-IDF repository <https://docs.espressif.com/projects/esp-idf/en/latest/get-started/linux-setup-scratch.html>`_.

Flashing
========

Firmware for ESP32 is flashed via the USB/UART interface using the ``esptool.py`` tool. To flash your NuttX firmware simply run::

    make download ESPTOOL_PORT=<port>

where ``<port>`` is typically ``/dev/ttyUSB0`` or similar. You can change the baudrate by passing ``ESPTOOL_BAUD``.

Bootloader and partitions
-------------------------

ESP32 requires a bootloader to be flashed as well as a set of FLASH partitions. This is only needed the first time
(or any time you which to modify either of these). An easy way is to use prebuilt binaries for NuttX from `here <https://github.com/espressif/esp-nuttx-bootloader>`_. In there you will find instructions to rebuild these if necessary. 
Once you downloaded both binaries, you can flash them by adding an ``ESPTOOL_BINDIR`` parameter, pointing to the directory where these binaries were downloaded:

.. code-block:: console

   $ make download ESPTOOL_PORT=<port> ESPTOOL_BINDIR=<dir>

.. note:: It is recommended that if this is the first time you are using the board with NuttX that you perform a complete
   SPI FLASH erase.

   .. code-block:: console
       
      $ esptool.py erase_flash

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX:

========== ======= =====
Peripheral Support NOTES
========== ======= =====
GPIO         Yes       
UART         Yes
SPI          Yes       
I2C          Yes       
DMA          Yes       
Wifi         Yes       
Ethernet     Yes
SPIFLASH     Yes
SPIRAM       Yes
Timers       Yes
Watchdog     Yes
RTC          Yes
RNG          Yes
AES          Yes
eFuse        Yes
ADC          No
Bluetooth    No
SDIO         No
SD/MMC       No
I2S          No
LED_PWM      No
RMT          No
MCPWM        No
Pulse_CNT    No
SHA          No
RSA          No
========== ======= =====

Memory Map
==========

Address Mapping
---------------

================== ========== ========== =============== ===============
BUS TYPE           START      LAST       DESCRIPTION     NOTES
================== ========== ========== =============== ===============
                   0x00000000 0x3F3FFFFF                 Reserved
Data               0x3F400000 0x3F7FFFFF External Memory
Data               0x3F800000 0x3FBFFFFF External Memory
                   0x3FC00000 0x3FEFFFFF                 Reserved
Data               0x3FF00000 0x3FF7FFFF Peripheral
Data               0x3FF80000 0x3FFFFFFF Embedded Memory
Instruction        0x40000000 0x400C1FFF Embedded Memory
Instruction        0x400C2000 0x40BFFFFF External Memory
.                  0x40C00000 0x4FFFFFFF                 Reserved
Data / Instruction 0x50000000 0x50001FFF Embedded Memory

.                  0x50002000 0xFFFFFFFF                 Reserved
================== ========== ========== =============== ===============


Embedded Memory
---------------

=========== ========== ========== =============== ===============
BUS TYPE    START      LAST       DESCRIPTION     NOTES
=========== ========== ========== =============== ===============
Data        0x3ff80000 0x3ff81fff RTC FAST Memory PRO_CPU Only
.           0x3ff82000 0x3ff8ffff                 Reserved
Data        0x3ff90000 0x3ff9ffff Internal ROM 1
.           0x3ffa0000 0x3ffadfff                 Reserved
Data        0x3ffae000 0x3ffdffff Internal SRAM 2 DMA
Data        0x3ffe0000 0x3fffffff Internal SRAM 1 DMA
=========== ========== ========== =============== ===============

Boundary Address
---------------

====================== ========== ========== =============== ===============
BUS TYPE               START      LAST       DESCRIPTION     NOTES
====================== ========== ========== =============== ===============
Instruction            0x40000000 0x40007fff Internal ROM 0  Remap
Instruction            0x40008000 0x4005ffff Internal ROM 0
.                      0x40060000 0x4006ffff                 Reserved
Instruction            0x40070000 0x4007ffff Internal SRAM 0 Cache
Instruction            0x40080000 0x4009ffff Internal SRAM 0
Instruction            0x400a0000 0x400affff Internal SRAM 1
Instruction            0x400b0000 0x400b7FFF Internal SRAM 1 Remap
Instruction            0x400b8000 0x400bffff Internal SRAM 1
Instruction            0x400c0000 0x400c1FFF RTC FAST Memory PRO_CPU Only
Data / Instruction     0x50000000 0x50001fff RTC SLOW Memory

====================== ========== ========== =============== ===============

External Memory
---------------

=========== ========== ========== =============== ===============
BUS TYPE    START      LAST       DESCRIPTION     NOTES
=========== ========== ========== =============== ===============
Data        0x3f400000 0x3f7fffff External Flash  Read
Data        0x3f800000 0x3fbfffff External SRAM   Read and Write
=========== ========== ========== =============== ===============

Boundary Address
----------------

Instruction 0x400c2000 0x40bfffff 11512 KB External Flash Read

Linker Segments
---------------

+---------------------+------------+------------+------+------------------------------+
| DESCRIPTION         | START      | END        | ATTR | LINKER SEGMENT NAME          |
+=====================+============+============+======+==============================+
| FLASH mapped data:  | 0x3f400010 | 0x3fc00010 | R    | drom0_0_seg                  |
|     - .rodata       |            |            |      |                              |
|     - Constructors  |            |            |      |                              |
|       /destructors  |            |            |      |                              |
+---------------------+------------+------------+------+------------------------------+
| COMMON data RAM:    | 0x3ffb0000 | 0x40000000 | RW   | dram0_0_seg  (NOTE 1,2,3)    |
|  - .bss/.data       |            |            |      |                              |
+---------------------+------------+------------+------+------------------------------+
| IRAM for PRO cpu:   | 0x40080000 | 0x400a0000 | RX   | iram0_0_seg                  |
|  - Interrupt Vectors|            |            |      |                              |
|  - Low level        |            |            |      |                              |
|    handlers         |            |            |      |                              |
|  - Xtensa/Espressif |            |            |      |                              |
|    libraries        |            |            |      |                              |
+---------------------+------------+------------+------+------------------------------+
| RTC fast memory:    | 0x400c0000 | 0x400c2000 | RWX  | rtc_iram_seg (PRO_CPU only)  |
|  - .rtc.text        |            |            |      |                              |
|    (unused?)        |            |            |      |                              |
+---------------------+------------+------------+------+------------------------------+
| FLASH:              | 0x400d0018 | 0x40400018 | RX   | iram0_2_seg  (actually FLASH)|
|  - .text            |            |            |      |                              |
+---------------------+------------+------------+------+------------------------------+
| RTC slow memory:    | 0x50000000 | 0x50001000 | RW   | rtc_slow_seg (NOTE 4)        |
|  - .rtc.data/rodata |            |            |      |                              |
|    (unused?)        |            |            |      |                              |
+---------------------+------------+------------+------+------------------------------+

.. note::

  (1) Linker script will reserve space at the beginning of the segment
      for BT and at the end for trace memory.
  (2) Heap ends at the top of dram_0_seg.
  (3) Parts of this region is reserved for the ROM bootloader.
  (4) Linker script will reserve space at the beginning of the segment
      for co-processor reserve memory and at the end for ULP coprocessor
      reserve memory.

64-bit Timers
=============

ESP32 has 4 generic timers of 64 bits (2 from Group 0 and 2 from Group 1). They're
accessible as character drivers, the configuration along with a guidance on how
to run the example and the description of the application level interface
can be found :doc:`here </components/drivers/character/timer>`.

Watchdog Timers
===============

ESP32 has 3 WDTs. 2 MWDTS from the Timers Module and 1 RWDT from the RTC Module
(Currently not supported yet). They're accessible as character drivers,
The configuration along with a guidance on how to run the example and the description
of the application level interface can be found
:doc:`here </components/drivers/character/watchdog>`.

SMP
===

The ESP32 has 2 CPUs.  Support is included for testing an SMP configuration.
That configuration is still not yet ready for usage but can be enabled with
the following configuration settings,
in :menuselection:`RTOS Features --> Tasks and Scheduling`, with::

    CONFIG_SPINLOCK=y
    CONFIG_SMP=y
    CONFIG_SMP_NCPUS=2

Debug Tip:  During debug session, OpenOCD may mysteriously switch from one
CPU to another.  This behavior can be eliminated by uncommenting one of the
following in ``scripts/esp32.cfg``::

  # Only configure the PRO CPU
  #set ESP32_ONLYCPU 1
  # Only configure the APP CPU
  #set ESP32_ONLYCPU 2

Open Issues
-----------

  1. Cache Issues.  I have not thought about this yet, but certainly caching is
     an issue in an SMP system:

     - Cache coherency.  Are there separate caches for each CPU?  Or a single
       shared cache?  If the are separate then keep the caches coherent will
       be an issue.
     - Caching MAY interfere with spinlocks as they are currently implemented.
       Waiting on a cached copy of the spinlock may result in a hang or a
       failure to wait.

  2. Assertions.  On a fatal assertions, other CPUs need to be stopped.

WiFi
====

A standard network interface will be configured and can be initialized such as::

    ifup wlan0
    wapi psk wlan0 mypasswd 1
    wapi essid wlan0 myssid 1
    renew wlan0

In this case a connection to AP with SSID ``myssid`` is done, using ``mypasswd`` as
password. IP address is obtained via DHCP using ``renew`` command. You can check
the result by running ``ifconfig`` afterwards.

.. tip:: Boards usually expose a ``wapi`` defconfig which enables WiFi

Bluetooth
=========

Bluetooth is not currently supported.

Debugging with OpenOCD
======================

First you in need some debug environment which would be a JTAG emulator
and the ESP32 OpenOCD software which is available here:
https://github.com/espressif/openocd-esp32

OpenOCD Documentation
---------------------

There is on overview of the use of OpenOCD `here <https://dl.espressif.com/doc/esp-idf/latest/openocd.html>`.
This document is also available in `ESP-IDF source tree <https://github.com/espressif/esp-idf>`_
in ``docs`` directory.

OpenOCD Configuration File
--------------------------

A template ESP32 OpenOCD configuration file is provided in
ESP-IDF ``docs`` directory (``esp32.cfg``).  Since you are not using
FreeRTOS, you will need to uncomment the line::

  set ESP32_RTOS none

in the OpenOCD configuration file.  You will also need to change
the source line from::

  find interface/ftdi/tumpa.cfg

to reflect the physical JTAG adapter connected.

A copy of this OpenOCD configuration file available in the NuttX
source tree at ``nuttx/boards/xtensa/esp32/esp32-devkitc/scripts/esp32.cfg``.
It has these modifications:

  - The referenced "set ESP32_RTOS none" line has been uncommented
  - The "find interface/ftdi/tumpa.cfg" was removed. This means that you will
    need to specify the interface configuration file on the OpenOCD
    command line.

Another OpenOCD configuration file is available in the NuttX source tree at
``nuttx/boards/xtensa/esp32/esp32-devkitc/scripts/esp32-ft232h.cfg``.
It has been tested with:

  - `ESP32-DevKitC V4 <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html>`_

  - Akizukidenshi's FT232HL, a FT232H based JTAG adapter
    (http://akizukidenshi.com/catalog/g/gK-06503/) with JP3 and JP4 closed,
    and connected to ESP32 as:

    +------------------+-------------+
    | ESP32-DevKitC V4 | FT232HL     |
    +=======+==========+=============+
    | J2    |  J3      | J2          |
    +-------+----------+-------------+
    | IO13  |          | AD0   (TCK) |
    +-------+----------+-------------+
    | IO12  |          | AD1   (TDI) |
    +-------+----------+-------------+
    |       |  IO15    | AD2   (TDO) |
    +-------+----------+-------------+
    | IO14  |          | AD3   (TMS) |
    +-------+----------+-------------+
    | GND   |          | GND         |
    +-------+----------+-------------+

The following version of OpenOCD from ESP-IDF (macOS version)::

  % openocd --version
  Open On-Chip Debugger  v0.10.0-esp32-20191114 (2019-11-14-14:19)
  Licensed under GNU GPL v2
  For bug reports, read
          http://openocd.org/doc/doxygen/bugs.html

General OpenOCD build instructions
----------------------------------

Installing OpenOCD.  The sources for the ESP32-enabled variant of
OpenOCD are available from Espressifs GitHub. To download the source,
use the following commands:

.. code-block:: console

   $ git clone https://github.com/espressif/openocd-esp32.git
   $ cd openocd-esp32
   $ git submodule init
   $ git submodule update

Then look at the README and the docs/INSTALL.txt files in the
openocd-esp32 directory for further instructions.  There area
separate README files for Linux/Cygwin, macOS, and Windows.  Here
is what I ended up doing (under Linux):

.. code-block:: console

  $ cd openocd-esp32
  $ ./bootstrap
  $ ./configure
  $ make

If you do not do the install step, then you will have a localhost
version of the OpenOCD binary at ``openocd-esp32/src``.

Starting the OpenOCD Server
---------------------------

  - cd to openocd-esp32 directory
  - copy the modified esp32.cfg script to this directory

Then start OpenOCD by executing a command like the following.  Here
I assume that:

  - You did not install OpenOCD; binaries are available at
    openocd-esp32/src and interface scripts are in
    openocd-eps32/tcl/interface
  - I select the configuration for the Olimex ARM-USB-OCD
    debugger.

Then the command to start OpenOCD is:

.. code-block:: console

  $ ./src/openocd -s ./tcl -f tcl/interface/ftdi/olimex-arm-usb-ocd.cfg -f ./esp32.cfg

I then see::

  Open On-Chip Debugger 0.10.0-dev-g3098897 (2016-11-14-12:19)
  Licensed under GNU GPL v2
  For bug reports, read
          http://openocd.org/doc/doxygen/bugs.html
  adapter speed: 200 kHz
  force hard breakpoints
  Info : clock speed 200 kHz
  Info : JTAG tap: esp32.cpu0 tap/device found: 0x120034e5 (mfg: 0x272 (Tensilica), part: 0x2003, ver: 0x1)
  Info : JTAG tap: esp32.cpu1 tap/device found: 0x120034e5 (mfg: 0x272 (Tensilica), part: 0x2003, ver: 0x1)
  Info : esp32.cpu0: Debug controller was reset (pwrstat=0x5F, after clear 0x0F).
  Info : esp32.cpu0: Core was reset (pwrstat=0x5F, after clear 0x0F).

Connecting a debugger to OpenOCD
--------------------------------

OpenOCD should now be ready to accept gdb connections. If you have
compiled the ESP32 toolchain using Crosstool-NG, or if you have
downloaded a precompiled toolchain from the Espressif website, you
should already have xtensa-esp32-elf-gdb, a version of gdb that can
be used for this

First, make sure the project you want to debug is compiled and
flashed into the ESP32’s SPI flash. Then, in a different console
than OpenOCD is running in, invoke gdb. For example, for the
template app, you would do this like such::

.. code-block:: console

  $ cd nuttx
  $ xtensa-esp32-elf-gdb -ex 'target remote localhost:3333' nuttx

This should give you a gdb prompt.

Breakpoints
-----------

You can set up to 2 hardware breakpoints, which can be anywhere in the
address space. Also 2 hardware watchpoints.

The openocd esp32.cfg file currently forces gdb to use hardware
breakpoints, I believe because software breakpoints (or, at least, the
memory map for automatically choosing them) aren't implemented yet
(as of 2016-11-14).

JTAG Emulator
-------------

The documentation indicates that you need to use an external JTAG
like the TIAO USB Multi-protocol Adapter and the Flyswatter2.
The instructions at http://www.esp32.com/viewtopic.php?t=381 show
use of an FTDI C232HM-DDHSL-0 USB 2.0 high speed to MPSSE cable.

The ESP32 DevkitC v4 board has no on board JTAG connector.  It will
be necessary to make a cable or some other board to connect a JTAG
emulator.  Refer to http://www.esp32.com/viewtopic.php?t=381 "How
to debug ESP32 with JTAG / OpenOCD / GDB 1st part connect the
hardware."

Relevant pin-out:

========= =============
PIN LABEL JTAG FUNCTION
========= =============
IO14      TMS
IO12      TDI
GND       GND
IO13      TCK
x         x
IO15      TDO
========= =============

You can find the mapping of JTAG signals to ESP32 GPIO numbers in
"ESP32 Pin List" document found
`here <http://espressif.com/en/support/download/documents?keys=&field_type_tid%5B%5D=13>`_.

I put the ESP32 on a prototyping board and used a standard JTAG 20-pin
connector with an older Olimex JTAG that I had.  Here is how I wired
the 20-pin connector:

===================== ===============
20-PIN JTAG CONNECTOR ESP32 PIN LABEL
===================== ===============
 1 VREF  INPUT        3V3
 3 nTRST OUTPUT       N/C
 5 TDI   OUTPUT       IO12
 7 TMS   OUTPUT       IO14
 9 TCLK  OUTPUT       IO13
11 RTCK  INPUT        N/C
13 TDO   INPUT        IO15
15 RESET I/O          N/C
17 DBGRQ OUTPUT       N/C
19 5V    OUTPUT       N/C
 2 VCC   INPUT        3V3
 4 GND   N/A          GND
 6 GND   N/A          GND
 8 GND   N/A          GND
10 GND   N/A          GND
12 GND   N/A          GND
14 GND   N/A          GND
16 GND   N/A          GND
18 GND   N/A          GND
20 GND   N/A          GND
===================== ===============

Executing and Debugging from FLASH and IRAM
===========================================

FLASH
-----

OpenOCD currently doesn't have a FLASH driver for ESP32, so you can load
code into IRAM only via JTAG. FLASH-resident sections like .FLASH.rodata
will fail to load.  The bootloader in ROM doesn't parse ELF, so any image
which is bootloaded from FLASH has to be converted into a custom image
format first.

The tool esp-idf uses for flashing is a command line Python tool called
"esptool.py" which talks to a serial bootloader in ROM.  A version is
supplied in the esp-idf codebase in components/esptool_py/esptool, the
"upstream" for that tool is here and now supports ESP32::

  https://github.com/espressif/esptool/

To FLASH an ELF via the command line is a two step process, something like
this::

  esptool.py --chip esp32 elf2image --flash_mode dio --flash_size 4MB -o nuttx.bin nuttx
  esptool.py --chip esp32 --port COMx write_flash 0x1000 bootloader.bin 0x8000 partition_table.bin 0x10000 nuttx.bin

The first step converts an ELF image into an ESP32-compatible binary
image format, and the second step flashes it (along with bootloader image and
partition table binary.)
The offset for the partition table may vary, depending on ESP-IDF
configuration, ``CONFIG_PARTITION_TABLE_OFFSET``, which is by default 0x8000
as of writing this.

To put the ESP32 into serial flashing mode, it needs to be reset with IO0 held
low.  On the Core boards this can be accomplished by holding the button marked
"Boot" and pressing then releasing the button marked "EN".  Actually, esptool.py
can enter bootloader mode automatically (via RTS/DTR control lines), but
unfortunately a timing interaction between the Windows CP2012 driver and the
hardware means this doesn't currently work on Windows.

Secondary Boot Loader / Partition Table
---------------------------------------

See:

  - https://github.com/espressif/esp-idf/tree/master/components/bootloader
  - https://github.com/espressif/esp-idf/tree/master/components/partition_table .

The secondary boot loader by default programs a RTC watchdog timer.
As NuttX doesn't know the timer, it reboots every ~9 seconds. You can
disable the timer by tweaking sdkconfig CONFIG_BOOTLOADER_WDT_ENABLE
and rebuild the boot loader.

Running from IRAM with OpenOCD
------------------------------

Running from IRAM is a good debug option.  You should be able to load the
ELF directly via JTAG in this case, and you may not need the bootloader.

NuttX supports a configuration option, CONFIG_ESP32_DEVKITC_RUN_IRAM, that may be
selected for execution from IRAM.  This option simply selects the correct
linker script for IRAM execution.

Skipping the Secondary Bootloader
---------------------------------

It is possible to skip the secondary bootloader and run out of IRAM using
only the primary bootloader if your application of small enough (< 128KiB code,
<180KiB data), then you can simplify initial bring-up by avoiding second stage
bootloader. Your application will be loaded into IRAM using first stage
bootloader present in ESP32 ROM. To achieve this, you need two things:

  1. Have a linker script which places all code into IRAM and all data into
     IRAM/DRAM

  2. Use "esptool.py" utility to convert application .elf
     file into binary format which can be loaded by first stage bootloader.

Again you would need to link the ELF file and convert it to binary format suitable
for flashing into the board.  The command should to convert ELF file to binary
image looks as follows::

  esptool.py --chip esp32 elf2image --flash_mode "dio" --flash_freq "40m" --flash_size "2MB" -o nuttx.bin nuttx

To flash binary image to your development board, use the same esptool.py utility::

  esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 write_flash -z --flash_mode dio --flash_freq 40m --flash_size 2MB 0x1000 nuttx.bin

The argument before app.bin (0x1000) indicates the offset in flash where binary
will be written. ROM bootloader expects to find an application (or second stage
bootloader) image at offset 0x1000, so we are writing the binary there.

Sample OpenOCD Debug Steps
--------------------------

I did the initial bring-up using the IRAM configuration and OpenOCD.  Here
is a synopsis of my debug steps:

boards/xtensa/esp32/esp32-devkitc/configs/nsh with::

  CONFIG_DEBUG_ASSERTIONS=y
  CONFIG_DEBUG_FEATURES=y
  CONFIG_DEBUG_SYMBOLS=y
  CONFIG_ESP32_DEVKITC_RUN_IRAM=y

I also made this change configuration which will eliminate all attempts to
re-configure serial. It will just use the serial settings as they were left
by the bootloader::

  CONFIG_SUPPRESS_UART_CONFIG=y

Start OpenOCD::

  cd ../openocde-esp32
  cp ../nuttx/boards/xtensa/esp32/esp32-devkitc/scripts/esp32.cfg .
  sudo ./src/openocd -s ./tcl/ -f tcl/interface/ftdi/olimex-arm-usb-ocd.cfg -f ./esp32.cfg

Start GDB and load code::

  cd ../nuttx
  xtensa-esp32-elf-gdb -ex 'target remote localhost:3333' nuttx
  (gdb) load nuttx
  (gdb) mon reg pc [value report by load for entry point]
  (gdb) s

Single stepping works fine for me as do breakpoints::

  Breakpoint 1, up_timer_initialize () at chip/esp32_timerisr.c:172
  72 {
  (gdb) n
  esp32.cpu0: Target halted, pc=0x400835BF
  187 g_tick_divisor = divisor;
  (gdb) ...

Using QEMU
==========

First follow the instructions `here <https://github.com/espressif/qemu/wiki>`_ to build QEMU.
Enable the ESP32_QEMU_IMAGE config found in "Board Selection -> ESP32 binary image for QEMU".
Download the bootloader and the partition table from https://github.com/espressif/esp-nuttx-bootloader/releases
and place them in a directory, say ../esp-bins.
Build and generate the QEMU image: `make ESPTOOL_BINDIR=../esp-bins`
A new image "esp32_qemu_image.bin" will be created.  It can be run as::

 ~/PATH_TO_QEMU/qemu/build/xtensa-softmmu/qemu-system-xtensa -nographic \
    -machine esp32 \
    -drive file=esp32_qemu_image.bin,if=mtd,format=raw

Things to Do
============

1. Lazy co-processor save logic supported by Xtensa. That logic works like this:

   a. CPENABLE is set to zero on each context switch, disabling all co-
      processors.
   b. If/when the task attempts to use the disabled co-processor, an
      exception  occurs
   c. The co-processor exception handler re-enables the co-processor.

   Instead, the NuttX logic saves and restores CPENABLE on each context
   switch.  This has disadvantages in that (1) co-processor context will
   be saved and restored even if the co-processor was never used, and (2)
   tasks must explicitly enable and disable co-processors.

2. Currently the Xtensa port copies register state save information from
   the stack into the TCB.  A more efficient alternative would be to just
   save a pointer to a register state save area in the TCB.  This would
   add some complexity to signal handling and also also the
   up_initialstate().  But the performance improvement might be worth
   the effort.

3. See SMP-related issues above

4. See OpenOCD for the ESP32 above

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
