===============
Espressif ESP32
===============

The ESP32 is a series of single and dual-core SoCs from Espressif
based on Harvard architecture Xtensa LX6 CPUs and with on-chip support
for Bluetooth and Wi-Fi.

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

    $ pip install esptool

Building from source
--------------------

You can also build the toolchain yourself. The steps to
build the toolchain with crosstool-NG on Linux are as follows

.. code-block:: console

  $ git clone https://github.com/espressif/crosstool-NG.git
  $ cd crosstool-NG
  $ git checkout esp-2021r1
  $ git submodule update --init

  $ ./bootstrap && ./configure --enable-local && make

  $ ./ct-ng xtensa-esp32-elf
  $ ./ct-ng build

  $ chmod -R u+w builds/xtensa-esp32-elf

  $ export PATH="crosstool-NG/builds/xtensa-esp32-elf/bin:$PATH"

These steps are given in the setup guide in
`ESP-IDF documentation <https://docs.espressif.com/projects/esp-idf/en/latest/get-started/linux-setup-scratch.html>`_.

Flashing
========

Firmware for ESP32 is flashed via the USB/UART interface using the ``esptool.py`` tool. 
It's a two step process where the first converts the ELF file into a ESP32-compatible binary 
and the second flashes it to the board.  These steps are included into the build system and you can
flash your NuttX firmware simply by running::

    $ make flash ESPTOOL_PORT=<port>

where ``<port>`` is typically ``/dev/ttyUSB0`` or similar. You can change the baudrate by passing ``ESPTOOL_BAUD``.

Bootloader and partitions
-------------------------

ESP32 requires a bootloader to be flashed as well as a set of FLASH partitions. This is only needed the first time
(or any time you which to modify either of these). An easy way is to use prebuilt binaries for NuttX from `here <https://github.com/espressif/esp-nuttx-bootloader>`_. In there you will find instructions to rebuild these if necessary. 
Once you downloaded both binaries, you can flash them by adding an ``ESPTOOL_BINDIR`` parameter, pointing to the directory where these binaries were downloaded:

.. code-block:: console

   $ make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=<dir>

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

Wi-Fi
====

A standard network interface will be configured and can be initialized such as::

    nsh> ifup wlan0
    nsh> wapi psk wlan0 mypasswd 3
    nsh> wapi essid wlan0 myssid 1
    nsh> renew wlan0

In this case a connection to AP with SSID ``myssid`` is done, using ``mypasswd`` as
password. IP address is obtained via DHCP using ``renew`` command. You can check
the result by running ``ifconfig`` afterwards.

.. tip:: Boards usually expose a ``wapi`` defconfig which enables Wi-Fi

Wi-Fi SoftAP
===========

It is possible to use ESP32 as an Access Point (SoftAP). Actually there are some
boards with a ``sta_softap`` which enables this support.

If you are using this board config profile you can run these commands to be able
to connect your smartphone or laptop to your board::

    nsh> ifup wlan1
    nsh> dhcpd_start wlan1
    nsh> wapi psk wlan0 mypasswd 1
    nsh> wapi essid wlan1 nuttxap 1

In this case, you are creating the access point ``nuttxapp`` in your board and to
connect to it on your smartphone you will be required to type the password ``mypasswd``.
The ``dhcpd_start`` is necessary to let your board to associate an IP to your smartphone.

Bluetooth
=========

Bluetooth is not currently supported.

Using QEMU
==========

First follow the instructions `here <https://github.com/espressif/qemu/wiki>`_ to build QEMU.

Enable the ``ESP32_QEMU_IMAGE`` config found in :menuselection:`Board Selection --> ESP32 binary image for QEMU`.

Download the bootloader and the partition table from https://github.com/espressif/esp-nuttx-bootloader/releases
and place them in a directory, say ``../esp-bins``.

Build and generate the QEMU image::

 $ make ESPTOOL_BINDIR=../esp-bins

A QEMU-compatible ``nuttx.merged.bin`` binary image will be created. It can be run as::

 $ qemu-system-xtensa -nographic -machine esp32 -drive file=nuttx.merged.bin,if=mtd,format=raw

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
   add some complexity to signal handling and also to up_initialstate().
   But the performance improvement might be worth the effort.

3. See SMP-related issues above

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
