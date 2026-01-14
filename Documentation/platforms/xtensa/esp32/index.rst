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

ESP32 Toolchain
===============

The toolchain used to build ESP32 firmware can be either downloaded or built from the sources.

It is **highly** recommended to use (download or build) the same toolchain version that is being
used by the NuttX CI.

Please refer to the Docker
`container <https://github.com/apache/nuttx/tree/master/tools/ci/docker/linux/Dockerfile>`_ and
check for the current compiler version being used. For instance:

.. code-block::

  ###############################################################################
  # Build image for tool required by ESP32 builds
  ###############################################################################
  FROM nuttx-toolchain-base AS nuttx-toolchain-esp32
  # Download the latest ESP32, ESP32-S2 and ESP32-S3 GCC toolchain prebuilt by Espressif
  RUN mkdir -p xtensa-esp-elf-gcc && \
    curl -s -L "https://github.com/espressif/crosstool-NG/releases/download/esp-14.2.0_20241119/xtensa-esp-elf-14.2.0_20241119-x86_64-linux-gnu.tar.xz" \
    | tar -C xtensa-esp-elf-gcc --strip-components 1 -xJ

For ESP32, the toolchain version is based on GGC 14.2.0 (``xtensa-esp-elf-14.2.0_20241119``)

The prebuilt Toolchain (Recommended)
------------------------------------

First, create a directory to hold the toolchain:

.. code-block:: console

  $ mkdir -p /path/to/your/toolchain/xtensa-esp-elf-gcc

Download and extract toolchain:

.. code-block:: console

  $ curl -s -L "https://github.com/espressif/crosstool-NG/releases/download/esp-14.2.0_20241119/xtensa-esp-elf-14.2.0_20241119-x86_64-linux-gnu.tar.xz" \
  | tar -C xtensa-esp-elf-gcc --strip-components 1 -xJ

Add the toolchain to your `PATH`:

.. code-block:: console

  $ echo "export PATH=/path/to/your/toolchain/xtensa-esp-elf-gcc/bin:$PATH" >> ~/.bashrc

You can edit your shell's rc files if you don't use bash.

Building from source
--------------------

You can also build the toolchain yourself. The steps to
build the toolchain with crosstool-NG on Linux are as follows

.. code-block:: console

  $ git clone https://github.com/espressif/crosstool-NG.git
  $ cd crosstool-NG
  $ git submodule update --init

  $ ./bootstrap && ./configure --enable-local && make

  $ ./ct-ng xtensa-esp32-elf
  $ ./ct-ng build

  $ chmod -R u+w builds/xtensa-esp32-elf

  $ export PATH="crosstool-NG/builds/xtensa-esp32-elf/bin:$PATH"

These steps are given in the setup guide in
`ESP-IDF documentation <https://docs.espressif.com/projects/esp-idf/en/latest/get-started/linux-setup-scratch.html>`_.

Building and flashing NuttX
===========================

Installing esptool
------------------

First, make sure that ``esptool.py`` is installed and up-to-date.
This tool is used to convert the ELF to a compatible ESP32 image and to flash the image into the board.

It can be installed with: ``pip install esptool>=4.8.1``.

.. warning::
    Installing ``esptool.py`` may required a Python virtual environment on newer systems.
    This will be the case if the ``pip install`` command throws an error such as:
    ``error: externally-managed-environment``.

    If you are not familiar with virtual environments, refer to `Managing esptool on virtual environment`_ for instructions on how to install ``esptool.py``.


Bootloader and partitions
-------------------------

NuttX can boot the ESP32 directly using the so-called "Simple Boot". An externally-built
2nd stage bootloader is not required in this case as all functions required to boot the device
are built within NuttX. Simple boot does not require any specific configuration (it is selectable
by default if no other 2nd stage bootloader is used).

If other features are required, an externally-built 2nd stage bootloader is needed. The bootloader
is built using the ``make bootloader`` command. This command generates the firmware in the
``nuttx`` folder. The ``ESPTOOL_BINDIR`` is used in the ``make flash`` command to specify the path
to the bootloader. For compatibility among other SoCs and future options of 2nd stage bootloaders,
the commands ``make bootloader`` and the ``ESPTOOL_BINDIR`` option (for the ``make flash``) can be
used even if no externally-built 2nd stage bootloader is being built (they will be ignored if
Simple Boot is used, for instance)::

  $ make bootloader

.. note:: It is recommended that if this is the first time you are using the board with NuttX to
   perform a complete SPI FLASH erase.

    .. code-block:: console

      $ esptool.py erase_flash

Building and Flashing
---------------------

This is a two-step process where the first step converts the ELF file into an ESP32 compatible binary
and the second step flashes it to the board. These steps are included in the build system and it is
possible to build and flash the NuttX firmware simply by running::

    $ make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

where:

* ``ESPTOOL_PORT`` is typically ``/dev/ttyUSB0`` or similar.
* ``ESPTOOL_BINDIR=./`` is the path of the externally-built 2nd stage bootloader and the partition table (if applicable): when built using the ``make bootloader``, these files are placed into ``nuttx`` folder.
* ``ESPTOOL_BAUD`` is able to change the flash baud rate if desired.

Flashing NSH Example
--------------------

This example shows how to build and flash the ``nsh`` defconfig for the ESP32-DevKitC board::

    $ cd nuttx
    $ make distclean
    $ ./tools/configure.sh esp32-devkitc:nsh
    $ make -j$(nproc)

When the build is complete, the firmware can be flashed to the board using the command::

    $ make -j$(nproc) flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

where ``<port>`` is the serial port where the board is connected::

  $ make flash ESPTOOL_PORT=/dev/ttyUSB0 ESPTOOL_BINDIR=./
   CP: nuttx.hex
   MKIMAGE: ESP32 binary
   esptool.py -c esp32 elf2image --ram-only-header -fs 4MB -fm dio -ff 40m -o nuttx.bin nuttx
   esptool.py v4.8.1
   Creating esp32 image...
   Image has only RAM segments visible. ROM segments are hidden and SHA256 digest is not appended.
   Merged 1 ELF section
   Successfully created esp32 image.
   Generated: nuttx.bin
   esptool.py -c esp32 -p /dev/ttyUSB0 -b 921600  write_flash -fs detect -fm dio -ff 40m 0x1000 nuttx.bin
   esptool.py v4.8.1
   Serial port /dev/ttyUSB0
   Connecting.....
   Chip is ESP32-D0WD-V3 (revision v3.1)
   [...]
   Flash will be erased from 0x00001000 to 0x00032fff...
   Flash params set to 0x0230
   Compressed 203816 bytes to 74735...
   Wrote 203816 bytes (74735 compressed) at 0x00001000 in 2.2 seconds (effective 744.4 kbit/s)...
   Hash of data verified.

   Leaving...
   Hard resetting via RTS pin...


Now opening the serial port with a terminal emulator should show the NuttX console::

  $ picocom -b 115200 /dev/ttyUSB0
  NuttShell (NSH) NuttX-12.8.0
  nsh> uname -a
  NuttX 12.8.0 759d37b97c-dirty Mar  5 2025 20:31:15 xtensa esp32-devkitc

Debugging
=========

This section describes debugging techniques for the ESP32.

Debugging with ``openocd`` and ``gdb``
--------------------------------------

Espressif uses a specific version of OpenOCD to support ESP32: `openocd-esp32 <https://github.com/espressif/>`_.

Please check `Building OpenOCD from Sources <https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32/api-guides/jtag-debugging/index.html#jtag-debugging-building-openocd>`_
for more information on how to build OpenOCD for ESP32.

ESP32 has dedicated pins for JTAG debugging. The following pins are used for JTAG debugging:

============= ===========
ESP32 Pin     JTAG Signal
============= ===========
MTDO / GPIO15 TDO
MTDI / GPIO12 TDI
MTCK / GPIO13 TCK
MTMS / GPIO14 TMS
============= ===========

Some boards, like :ref:`ESP32-Ethernet-Kit V1.2 <platforms/xtensa/esp32/boards/esp32-ethernet-kit/index:ESP32-Ethernet-Kit V1.2>` and
:ref:`ESP-WROVER-KIT <platforms/xtensa/esp32/boards/esp32-wrover-kit/index:ESP-WROVER-KIT>`, have a built-in JTAG debugger.

Other boards that don't have any built-in JTAG debugger can be debugged using an external JTAG debugger, like the one
described for the :ref:`ESP32-DevKitC <platforms/xtensa/esp32/boards/esp32-devkitc/index:Debugging with OpenOCD>`.

.. note:: One must configure the USB drivers to enable JTAG communication. Please check
  `Configure USB Drivers <https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32/api-guides/jtag-debugging/configure-ft2232h-jtag.html#configure-usb-drivers>`_
  for configuring the JTAG adapter of the :ref:`ESP32-Ethernet-Kit V1.2 <platforms/xtensa/esp32/boards/esp32-ethernet-kit/index:ESP32-Ethernet-Kit V1.2>` and
  :ref:`ESP-WROVER-KIT <platforms/xtensa/esp32/boards/esp32-wrover-kit/index:ESP-WROVER-KIT>` boards and other FT2232-based JTAG adapters.

OpenOCD can then be used::

  openocd -s <tcl_scripts_path> -c 'set ESP_RTOS hwthread' -f board/esp32-wrover-kit-3.3v.cfg -c 'init; reset halt; esp appimage_offset 0x1000'

.. note::
  - ``appimage_offset`` should be set to ``0x1000`` when ``Simple Boot`` is used. For MCUboot, this value should be set to
    ``CONFIG_ESP32_OTA_PRIMARY_SLOT_OFFSET`` value (``0x10000`` by default).
  - ``-s <tcl_scripts_path>`` defines the path to the OpenOCD scripts. Usually set to `tcl` if running openocd from its source directory.
    It can be omitted if `openocd-esp32` were installed in the system with `sudo make install`.

Once OpenOCD is running, you can use GDB to connect to it and debug your application::

  xtensa-esp32-elf-gdb -x gdbinit nuttx

whereas the content of the ``gdbinit`` file is::

  target remote :3333
  set remote hardware-watchpoint-limit 2
  mon reset halt
  flushregs
  monitor reset halt
  thb nsh_main
  c

.. note:: ``nuttx`` is the ELF file generated by the build process. Please note that ``CONFIG_DEBUG_SYMBOLS`` must be enabled in the ``menuconfig``.

Please refer to :doc:`/quickstart/debugging` for more information about debugging techniques.

Stack Dump and Backtrace Dump
-----------------------------

NuttX has a feature to dump the stack of a task and to dump the backtrace of it (and of all
the other tasks). This feature is useful to debug the system when it is not behaving as expected,
especially when it is crashing.

In order to enable this feature, the following options must be enabled in the NuttX configuration:
``CONFIG_SCHED_BACKTRACE``, ``CONFIG_DEBUG_SYMBOLS`` and, optionally, ``CONFIG_ALLSYMS``.

.. note::
   The first two options enable the backtrace dump. The third option enables the backtrace dump
   with the associated symbols, but increases the size of the generated NuttX binary.

Espressif also provides a tool to translate the backtrace dump into a human-readable format.
This tool is called ``btdecode.sh`` and is available at ``tools/espressif/btdecode.sh`` of NuttX
repository.

.. note::
   This tool is not necessary if ``CONFIG_ALLSYMS`` is enabled. In this case, the backtrace dump
   contains the function names.

Example - Crash Dump
^^^^^^^^^^^^^^^^^^^^

A typical crash dump, caused by an illegal load with ``CONFIG_SCHED_BACKTRACE`` and
``CONFIG_DEBUG_SYMBOLS`` enabled, is shown below::

  xtensa_user_panic: User Exception: EXCCAUSE=001d task: backtrace
  _assert: Current Version: NuttX  10.4.0 2ae3246e40-dirty Sep 19 2024 12:59:10 xtensa
  _assert: Assertion failed user panic: at file: :0 task: backtrace process: backtrace 0x400f0724
  up_dump_register:    PC: 400f0754    PS: 00060530
  up_dump_register:    A0: 800e2fcc    A1: 3ffe1400    A2: 00000000    A3: 3ffe0470
  up_dump_register:    A4: 3ffe0486    A5: 3ffaf4b0    A6: 00000000    A7: 00000000
  up_dump_register:    A8: 800f0751    A9: 3ffe13d0   A10: 0000005a   A11: 3ffafcb0
  up_dump_register:   A12: 00000059   A13: 3ffaf600   A14: 00000002   A15: 3ffafaa4
  up_dump_register:   SAR: 00000018 CAUSE: 0000001d VADDR: 00000000
  up_dump_register:  LBEG: 4000c28c  LEND: 4000c296  LCNT: 00000000
  dump_stack: User Stack:
  dump_stack:   base: 0x3ffe0490
  dump_stack:   size: 00004048
  dump_stack:     sp: 0x3ffe1400
  stack_dump: 0x3ffe13e0: 00000059 3ffaf600 00000002 3ffafaa4 800e1eb4 3ffe1420 400f0724 00000002
  stack_dump: 0x3ffe1400: 3ffe0486 3ffaf4b0 00000000 00000000 00000000 3ffe1440 00000000 400f0724
  stack_dump: 0x3ffe1420: 3ffe0470 3ffafae8 00000000 3ffb0d2c 00000000 3ffe1460 00000000 00000000
  stack_dump: 0x3ffe1440: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
  stack_dump: 0x3ffe1460: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
  sched_dumpstack: backtrace| 2: 0x400ef738 0x40085152 0x40084d05 0x40084c7d 0x40080c84 0x400f0754 0x400e2fcc 0x400e1eb4
  sched_dumpstack: backtrace| 2: 0x40000000 0x400e2fcc 0x400e1eb4 0x40000000
  dump_tasks:    PID GROUP PRI POLICY   TYPE    NPX STATE   EVENT      SIGMASK          STACKBASE  STACKSIZE   COMMAND
  dump_task:       0     0   0 FIFO     Kthread - Ready              0000000000000000 0x3ffb0010      3056   Idle_Task
  dump_task:       1     1 100 RR       Task    - Waiting Semaphore  0000000000000000 0x3ffaec10      3024   nsh_main
  dump_task:       2     2 255 RR       Task    - Running            0000000000000000 0x3ffe0490      4048   backtrace task
  sched_dumpstack: backtrace| 0: 0x400e12bb 0x400826eb
  sched_dumpstack: backtrace| 1: 0x400edc59 0x400edb5b 0x400edb94 0x400e6c36 0x400e643c 0x400e6714 0x400e5830 0x400e56b8
  sched_dumpstack: backtrace| 1: 0x400e5689 0x400e2fcc 0x400e1eb4 0x40000000
  sched_dumpstack: backtrace| 2: 0x400ef738 0x40084ed4 0x400ed9ea 0x40085184 0x40084d05 0x40084c7d 0x40080c84 0x400f0754
  sched_dumpstack: backtrace| 2: 0x400e2fcc 0x400e1eb4 0x40000000 0x400e2fcc 0x400e1eb4 0x40000000

The lines starting with ``sched_dumpstack`` show the backtrace of the tasks. By checking it, it is
possible to track the root cause of the crash. Saving this output to a file and using the ``btdecode.sh``::

  ./tools/btdecode.sh esp32 /tmp/backtrace.txt
  Backtrace for task 2:
  0x400ef738: sched_dumpstack at sched_dumpstack.c:69
  0x40085152: _assert at assert.c:691
  0x40084d05: xtensa_user_panic at xtensa_assert.c:188 (discriminator 1)
  0x40084c7d: xtensa_user at ??:?
  0x40080c84: _xtensa_user_handler at xtensa_user_handler.S:194
  0x400f0754: assert_on_task at backtrace_main.c:158
   (inlined by) backtrace_main at backtrace_main.c:194
  0x400e2fcc: nxtask_startup at task_startup.c:70
  0x400e1eb4: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0
  0x400e2fcc: nxtask_startup at task_startup.c:70
  0x400e1eb4: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0

  Backtrace dump for all tasks:

  Backtrace for task 2:
  0x400ef738: sched_dumpstack at sched_dumpstack.c:69
  0x40084ed4: dump_backtrace at assert.c:418
  0x400ed9ea: nxsched_foreach at sched_foreach.c:69 (discriminator 2)
  0x40085184: _assert at assert.c:726
  0x40084d05: xtensa_user_panic at xtensa_assert.c:188 (discriminator 1)
  0x40084c7d: xtensa_user at ??:?
  0x40080c84: _xtensa_user_handler at xtensa_user_handler.S:194
  0x400f0754: assert_on_task at backtrace_main.c:158
   (inlined by) backtrace_main at backtrace_main.c:194
  0x400e2fcc: nxtask_startup at task_startup.c:70
  0x400e1eb4: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0
  0x400e2fcc: nxtask_startup at task_startup.c:70
  0x400e1eb4: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0

  Backtrace for task 1:
  0x400edc59: nxsem_wait at sem_wait.c:217
  0x400edb5b: nxsched_waitpid at sched_waitpid.c:165
  0x400edb94: waitpid at sched_waitpid.c:618
  0x400e6c36: nsh_builtin at nsh_builtin.c:163
  0x400e643c: nsh_execute at nsh_parse.c:652
   (inlined by) nsh_parse_command at nsh_parse.c:2840
  0x400e6714: nsh_parse at nsh_parse.c:2930
  0x400e5830: nsh_session at nsh_session.c:246
  0x400e56b8: nsh_consolemain at nsh_consolemain.c:79
  0x400e5689: nsh_main at nsh_main.c:80
  0x400e2fcc: nxtask_startup at task_startup.c:70
  0x400e1eb4: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0

  Backtrace for task 0:
  0x400e12bb: nx_start at nx_start.c:772 (discriminator 1)
  0x400826eb: __esp32_start at esp32_start.c:294
   (inlined by) __start at esp32_start.c:358

The above output shows the backtrace of the tasks. By checking it, it is possible to track the
functions that were being executed when the crash occurred.

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX:

========== ======= =====
Peripheral Support NOTES
========== ======= =====
ADC          Yes    Oneshot
AES          Yes
Bluetooth    Yes
Camera       No
CAN/TWAI     Yes
DMA          Yes
DAC          Yes    One-shot
eFuse        Yes
Ethernet     Yes
GPIO         Yes
I2C          Yes    Master and Slave mode supported
I2S          Yes
LCD          No     There is support for SPI displays
LED/PWM      Yes
MCPWM        Yes
Pulse_CNT    Yes
RMT          Yes
RNG          Yes
RSA          No
RTC          Yes
SD/MMC       Yes    SPI based SD card driver
SDIO         No
SHA          Yes    Also supports HMAC-SHA(1/256) 
SPI          Yes
SPIFLASH     Yes
SPIRAM       Yes
Timers       Yes
Touch        Yes
UART         Yes
Watchdog     Yes
Wi-Fi        Yes
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

Boundary Address (Embedded)
---------------------------

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

Boundary Address (External)
---------------------------

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
can be found :doc:`here </components/drivers/character/timers/timer>`.

Watchdog Timers
===============

ESP32 has 3 WDTs. 2 MWDTS from the Timers Module and 1 RWDT from the RTC Module
(Currently not supported yet). They're accessible as character drivers,
The configuration along with a guidance on how to run the example and the description
of the application level interface can be found
:doc:`here </components/drivers/character/timers/watchdog>`.

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

.. _esp32_wi-fi_sta:

Wi-Fi
=====

A standard network interface will be configured and can be initialized such as::

    nsh> ifup wlan0
    nsh> wapi psk wlan0 mypasswd 3
    nsh> wapi essid wlan0 myssid 1
    nsh> renew wlan0

In this case a connection to AP with SSID ``myssid`` is done, using ``mypasswd`` as
password. IP address is obtained via DHCP using ``renew`` command. You can check
the result by running ``ifconfig`` afterwards.

.. tip:: Boards usually expose a ``wifi`` defconfig which enables Wi-Fi

.. tip:: Please check :doc:`wapi </applications/wireless/wapi/index>` documentation for more
   information about its commands and arguments.

.. note:: The ``wapi psk`` command on Station mode sets a security threshold. That
   is, it enables connecting only to an equally or more secure network than the set
   threshold. ``wapi psk wlan0 mypasswd 3`` sets a WPA2-PSK-secured network and
   enables the device to connect to networks that are equally or more secure than
   that (WPA3-SAE, for instance, would be eligible for connecting to).

.. _esp32_wi-fi_softap:

Wi-Fi SoftAP
============

It is possible to use ESP32 as an Access Point (SoftAP). Actually there are some
boards config examples called sta_softap which enables this support

If you are using this board config profile you can run these commands to be able
to connect your smartphone or laptop to your board::

    nsh> ifup wlan1
    nsh> dhcpd_start wlan1
    nsh> wapi psk wlan1 mypasswd 3
    nsh> wapi essid wlan1 nuttxap 1

In this case, you are creating the access point ``nuttxapp`` in your board and to
connect to it on your smartphone you will be required to type the password ``mypasswd``
using WPA2.

.. tip:: Please check :doc:`wapi </applications/wireless/wapi/index>` documentation for more
   information about its commands and arguments.

The ``dhcpd_start`` is necessary to let your board to associate an IP to your smartphone.

Bluetooth
=========

These are the steps to test Bluetooth Low Energy (BLE) scan on ESP32 (i.e. Devkit board).
First configure to use the BLE board profile::

    $ make distclean
    $ ./tools/configure.sh esp32-devkitc:ble
    $ make flash ESPTOOL_PORT=/dev/ttyUSB0

Enter in the NSH shell using your preferred serial console tool and run the scan command::

    NuttShell (NSH) NuttX-10.2.0
    nsh> ifconfig
    bnep0   Link encap:UNSPEC at DOWN
            inet addr:0.0.0.0 DRaddr:0.0.0.0 Mask:0.0.0.0

    wlan0   Link encap:Ethernet HWaddr ac:67:b2:53:8b:ec at UP
            inet addr:10.0.0.2 DRaddr:10.0.0.1 Mask:255.255.255.0

    nsh> bt bnep0 scan start
    nsh> bt bnep0 scan stop
    nsh> bt bnep0 scan get
    Scan result:
    1.     addr:           63:14:2f:b9:9f:83 type: 1
           rssi:            -90
           response type:   3
           advertiser data: 1e ff 06 00 01 09 20 02 7c 33 a3 a7 cd c9 44 5b
    2.     addr:           52:ca:05:b5:ad:77 type: 1
           rssi:            -82
           response type:   3
           advertiser data: 1e ff 06 00 01 09 20 02 03 d1 21 57 bf 19 b3 7a
    3.     addr:           46:8e:b2:cd:94:27 type: 1
           rssi:            -92
           response type:   2
           advertiser data: 02 01 1a 09 ff c4 00 10 33 14 12 16 80 02 0a d4
    4.     addr:           46:8e:b2:cd:94:27 type: 1
           rssi:            -92
           response type:   4
           advertiser data: 18 09 5b 4c 47 5d 20 77 65 62 4f 53 20 54 56 20
    5.     addr:           63:14:2f:b9:9f:83 type: 1
           rssi:            -80
           response type:   3
        advertiser data: 1e ff 06 00 01 09 20 02 7c 33 a3 a7 cd c9 44 5b
    nsh>

I2S
===

The I2S peripheral is accessible using either the generic I2S audio driver or a specific
audio codec driver. Also, it's possible to use the I2S character driver to bypass the
audio subsystem and develop specific usages of the I2S peripheral.

.. note:: Note that the bit-width and sample rate can be modified "on-the-go" when using
   audio-related drivers. That is not the case for the I2S character device driver and
   such parameters are set on compile time through `make menuconfig`.

.. warning:: Some upper driver implementations might not handle both transmission and
   reception configured at the same time on the same peripheral.

Please check for usage examples using the :doc:`ESP32 DevKitC </platforms/xtensa/esp32/boards/esp32-devkitc/index>`.

Analog-to-digital converter (ADC)
=================================

Two ADC units are available for the ESP32:

* ADC1 with 8 channels
* ADC2 with 10 channels

Those units are independent and can be used simultaneously. During bringup, GPIOs for selected channels are
configured automatically to be used as ADC inputs.
If available, ADC calibration is automatically applied (see
`this page <https://docs.espressif.com/projects/esp-idf/en/v5.1/esp32/api-reference/peripherals/adc_calibration.html>`__ for more details).
Otherwise, a simple conversion is applied based on the attenuation and resolution.

Each ADC unit is accessible using the ADC character driver, which returns data for the enabled channels.

The ADC unit can be enabled in the menu :menuselection:`System Type --> ESP32 Peripheral Selection --> Analog-to-digital converter (ADC)`.

Then, it can be customized in the menu :menuselection:`System Type --> ADC Configuration`, which includes operating mode, gain and channels.

========== =========== ===========
 Channel    ADC1 GPIO   ADC2 GPIO
========== =========== ===========
0           36          4
1           37          0
2           38          2
3           39          15
4           32          13
5           33          12
6           34          14
7           35          27
8                       25
9                       26
========== =========== ===========

.. warning:: ADC2 channels 1, 2 and 3 are used as strapping pins and can present undefined behavior.

.. _using_qemu_esp32:

Using QEMU
==========

Get or build QEMU from `here <https://github.com/espressif/qemu/wiki>`__.

Enable the ``ESP32_QEMU_IMAGE`` config found in :menuselection:`Board Selection --> ESP32 binary image for QEMU`.

Build and generate the QEMU image::

 $ make bootloader
 $ make ESPTOOL_BINDIR=.

A QEMU-compatible ``nuttx.merged.bin`` binary image will be created. It can be run as::

 $ qemu-system-xtensa -nographic -machine esp32 -drive file=nuttx.merged.bin,if=mtd,format=raw

QEMU for ESP32 does not correctly define the chip revision as v3.0 so you have two options:

- Enable the ``CONFIG_ESP32_IGNORE_CHIP_REVISION_CHECK`` or
- Emulate the efuse as described `here <https://github.com/espressif/esp-toolchain-docs/blob/main/qemu/esp32/README.md#emulating-esp32-eco3>`__.

QEMU Networking
---------------

Networking is possible using the openeth MAC driver. Enable ``ESP32_OPENETH`` option and set the nic in QEMU::

 $ qemu-system-xtensa -nographic -machine esp32 -drive file=nuttx.merged.bin,if=mtd,format=raw -nic user,model=open_eth

.. _MCUBoot and OTA Update ESP32:

MCUBoot and OTA Update
======================

The ESP32 supports over-the-air (OTA) updates using MCUBoot.

Read more about the MCUBoot for Espressif devices `here <https://docs.mcuboot.com/readme-espressif.html>`__.

Executing OTA Update
--------------------

This section describes how to execute OTA update using MCUBoot.

1. First build the default ``mcuboot_update_agent`` config. This image defaults to the primary slot and already comes with Wi-Fi settings enabled::

    ./tools/configure.sh esp32-devkitc:mcuboot_update_agent

2. Build the MCUBoot bootloader::

    make bootloader

3. Finally, build the application image::

    make

Flash the image to the board and verify it boots ok.
It should show the message "This is MCUBoot Update Agent image" before NuttShell is ready.

At this point, the board should be able to connect to Wi-Fi so we can download a new binary from our network::

  NuttShell (NSH) NuttX-12.4.0
  This is MCUBoot Update Agent image
  nsh>
  nsh> wapi psk wlan0 <wifi_ssid> 3
  nsh> wapi essid wlan0 <wifi_password> 1
  nsh> renew wlan0

Now, keep the board as is and execute the following commands to **change the MCUBoot target slot to the 2nd slot**
and modify the message of the day (MOTD) as a mean to verify the new image is being used.

1. Change the MCUBoot target slot to the 2nd slot::

    kconfig-tweak -d CONFIG_ESPRESSIF_ESPTOOL_TARGET_PRIMARY
    kconfig-tweak -e CONFIG_ESPRESSIF_ESPTOOL_TARGET_SECONDARY
    kconfig-tweak --set-str CONFIG_NSH_MOTD_STRING "This is MCUBoot UPDATED image!"
    make olddefconfig

  .. note::
    The same changes can be accomplished through ``menuconfig`` in :menuselection:`System Type --> Bootloader and Image Configuration --> Target slot for image flashing`
    for MCUBoot target slot and in :menuselection:`System Type --> Bootloader and Image Configuration --> Search (motd) --> NSH Library --> Message of the Day` for the MOTD.

2. Rebuild the application image::

    make

At this point the board is already connected to Wi-Fi and has the primary image flashed.
The new image configured for the 2nd slot is ready to be downloaded.

To execute OTA, create a simple HTTP server on the NuttX directory so we can access the binary remotely::

  cd nuttxspace/nuttx
  python3 -m http.server
   Serving HTTP on 0.0.0.0 port 8000 (http://0.0.0.0:8000/) ...

On the board, execute the update agent, setting the IP address to the one on the host machine. Wait until image is transferred and the board should reboot automatically::

  nsh> mcuboot_agent http://10.42.0.1:8000/nuttx.bin
  MCUboot Update Agent example
  Downloading from http://10.42.0.1:8000/nuttx.bin
  Firmware Update size: 1048576 bytes
  Received: 512      of 1048576 bytes [0%]
  Received: 1024     of 1048576 bytes [0%]
  Received: 1536     of 1048576 bytes [0%]
  [.....]
  Received: 1048576  of 1048576 bytes [100%]
  Application Image successfully downloaded!
  Requested update for next boot. Restarting...

NuttShell should now show the new MOTD, meaning the new image is being used::

  NuttShell (NSH) NuttX-12.4.0
  This is MCUBoot UPDATED image!
  nsh>

Finally, the image is loaded but not confirmed.
To make sure it won't rollback to the previous image, you must confirm with ``mcuboot_confirm`` and reboot the board.
The OTA is now complete.

Secure Boot and Flash Encryption
--------------------------------

Secure Boot
^^^^^^^^^^^

Secure Boot protects a device from running any unauthorized (i.e., unsigned) code by checking that
each piece of software that is being booted is signed. On an ESP32, these pieces of software include
the second stage bootloader and each application binary. Note that the first stage bootloader does not
require signing as it is ROM code thus cannot be changed. This is achieved using specific hardware in
conjunction with MCUboot (read more about MCUboot `here <https://docs.mcuboot.com/>`__).

The Secure Boot process on the ESP32 involves the following steps performed:

1. The first stage bootloader verifies the second stage bootloader's RSA-PSS signature. If the verification is successful,
   the first stage bootloader loads and executes the second stage bootloader.

2. When the second stage bootloader loads a particular application image, the application's signature (RSA, ECDSA or ED25519) is verified
   by MCUboot.
   If the verification is successful, the application image is executed.

.. warning:: Once enabled, Secure Boot will not boot a modified bootloader. The bootloader will only boot an
   application firmware image if it has a verified digital signature. There are implications for reflashing
   updated images once Secure Boot is enabled. You can find more information about the ESP32's Secure boot
   `here <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/security/secure-boot-v2.html>`__.

Requirements
^^^^^^^^^^^^

First of all, we need to install ``imgtool`` (a MCUboot utility application to manipulate binary
images) and ``esptool`` on our compilation environment.
See the :ref:`Managing esptool on virtual environment <Managing esptool on virtual environment>` section
for recommendations on how to setup the Python environment to install these tools::

    $ pip install imgtool esptool

Now, we will create a folder to store the generated keys (such as ``~/signing_keys``)::

    $ mkdir ~/signing_keys && cd ~/signing_keys

With all set up, we can now generate keys to sign the bootloader and application binary images,
respectively, of the compiled project::

    $ espsecure.py generate_signing_key --version 2 bootloader_signing_key.pem
    $ imgtool keygen --key app_signing_key.pem --type rsa-3072

.. important:: The contents of the key files must be stored securely and kept secret.

Enabling Secure Boot and Flash Encryption
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To enable Secure Boot for the current project, go to the project's NuttX directory, execute ``make menuconfig`` and the following steps:

   1. Enable experimental features in :menuselection:`Build Setup --> Show experimental options`;

   2. Enable MCUboot in :menuselection:`Application Configuration --> Bootloader Utilities --> MCUboot`;

   3. Change image type to ``MCUboot-bootable format`` in :menuselection:`System Type --> Application Image Configuration --> Application Image Format`;

   4. Enable building MCUboot from the source code by selecting ``Build binaries from source``;
      in :menuselection:`System Type --> Application Image Configuration --> Source for bootloader binaries`;

   5. Enable Secure Boot in :menuselection:`System Type --> Application Image Configuration --> Enable hardware Secure Boot in bootloader`;

   6. If you want to protect the SPI Bus against data sniffing, you can enable Flash Encryption in
      :menuselection:`System Type --> Application Image Configuration --> Enable Flash Encryption on boot`.

Now you can design an update and confirm agent to your application. Check the `MCUboot design guide <https://docs.mcuboot.com/design.html>`_ and the
`MCUboot Espressif port documentation <https://docs.mcuboot.com/readme-espressif.html>`_ for
more information on how to apply MCUboot. Also check some `notes about the NuttX MCUboot port <https://github.com/mcu-tools/mcuboot/blob/main/docs/readme-nuttx.md>`_,
the `MCUboot porting guide <https://github.com/mcu-tools/mcuboot/blob/main/docs/PORTING.md>`_ and some
`examples of MCUboot applied in NuttX applications <https://github.com/apache/nuttx-apps/tree/master/examples/mcuboot>`_.

After you developed an application which implements all desired functions, you need to flash it into the primary image slot
of the device (it will automatically be in the confirmed state, you can learn more about image
confirmation `here <https://docs.mcuboot.com/design.html#image-swapping>`_).
To flash to the primary image slot, select ``Application image primary slot`` in
:menuselection:`System Type --> Application Image Configuration --> Target slot for image flashing`
and compile it using ``make -j ESPSEC_KEYDIR=~/signing_keys``.

When creating update images, make sure to change :menuselection:`System Type --> Application Image Configuration --> Target slot for image flashing`
to ``Application image secondary slot``.

.. important:: When deploying your application, make sure to disable UART Download Mode by selecting ``Permanently disabled`` in
   :menuselection:`System Type --> Application Image Configuration --> UART ROM download mode`
   and change usage mode to ``Release`` in `System Type --> Application Image Configuration --> Enable usage mode`.
   **After disabling UART Download Mode you will not be able to flash other images through UART.**

Flash Encryption
----------------

Flash encryption is intended for encrypting the contents of the ESP32's off-chip flash memory. Once this feature is enabled,
firmware is flashed as plaintext, and then the data is encrypted in place on the first boot. As a result, physical readout
of flash will not be sufficient to recover most flash contents.

The current state of flash encryption for ESP32 allows the use of Virtual E-Fuses and development mode, which permit users to evaluate and test the firmware before making definitive changes such as burning E-Fuses.

Flash encryption supports the following features:

  .. list-table::
    :header-rows: 1

    * - Feature
      - Description
    * - **Flash Encryption with Virtual E-Fuses**
      - Use flash encryption without burning E-Fuses. Default selection when flash encryption is enabled.
    * - **Flash Encryption in Development mode**
      - Allows reflashing an encrypted device by appending the ``--encrypt`` argument to the ``esptool.py write_flash`` command. This is done automatically if ``ESP32_SECURE_FLASH_ENC_FLASH_DEVICE_ENCRYPTED`` is set.
    * - **Flash Encryption in Release mode**
      - Does not allow reflashing the device. This is a permanent setting.
    * - **Flash Encryption key**
      - A user-generated key is required by default. Alternatively, a device-generated key is possible, but it will not be recoverable by the user (not recommended). See ``ESP32_SECURE_FLASH_ENC_USE_HOST_KEY``.
    * - **Encrypted MTD Partition**
      - If SPI Flash is enabled, an empty user MTD partition will be automatically encrypted on first flash.

.. note::

   It is **strongly suggested** to read the following before working on flash encryption:

   - `MCUBoot Flash Encryption <https://docs.mcuboot.com/readme-espressif.html#flash-encryption>`_
   - `General E-Fuse documentation <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/efuse.html>`_
   - `Flash Encryption Relevant E-Fuses <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/security/flash-encryption.html#relevant-efuses>`_

Flash Encryption Requirements
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Flash encryption requires burning E-Fuses to enable it on chip. This is not a reversible operation and should be done with caution.
There is, however, a way to test the flash encryption by simulating them on flash. Both paths are described below.

Build System Features
'''''''''''''''''''''

The build system contains some safeguards to avoid accidentally burning E-Fuses and automations for convenience. Those are summarized below:

  1. A yellow warning will show up during build alerting that flash encryption is enabled (same for Virtual E-Fuses).
  2. If ``ESP32_SECURE_FLASH_ENC_USE_HOST_KEY`` is set, build will fail if the flash encryption key is not found.
  3. If SPI Flash is enabled, the user MTD partition is automatically encrypted with the provided encryption key.
  4. ``make flash`` command will prompt the user for confirmation before burning the E-Fuse, if Virtual E-Fuses are disabled.


Simulating Flash Encryption with Virtual E-Fuses
'''''''''''''''''''''''''''''''''''''''''''''''''

It is highly recommended to use this method for testing the flash encryption before actually burning the E-Fuses.
The E-Fuses are stored in flash and persist between reboots. No real E-Fuses are changed.

To enable virtual E-Fuses for flash encryption testing, open ``menuconfig`` and:
  1. Enable flash encryption on boot on: :menuselection:`System Type --> Bootloader and Image Configuration`
  2. Verify Virtual E-Fuses are enabled (this is done by default): :menuselection:`System Type --> ESP32 Peripheral Support --> E-Fuse support`

.. note:: On ESP32, testing is possible with QEMU. If that is the case, on step 2 disable Virtual E-Fuse and use normal E-Fuse support.
  See `ESP32 QEMU <https://github.com/espressif/esp-toolchain-docs/tree/main/qemu/esp32>`_ and :ref:`using_qemu_esp32` for instructions on setting up
  QEMU with E-Fuse support

Now build the bootloader and the firmware. Flashing the device (or opening on QEMU) will trigger the following:
  1. On the first boot, the bootloader will encrypt the flash::

      ...
      [esp32] [WRN] eFuse virtual mode is enabled. If Secure boot or Flash encryption is enabled then it does not provide any security. FOR TESTING ONLY!
      [esp32] [WRN] [efuse] [Virtual] try loading efuses from flash: 0x10000 (offset)
      ...
      [esp32] [INF] [flash_encrypt] Encrypting bootloader...
      [esp32] [INF] [flash_encrypt] Bootloader encrypted successfully
      [esp32] [INF] [flash_encrypt] Encrypting primary slot...
      [esp32] [INF] [flash_encrypt] Encrypting remaining flash...
      [esp32] [INF] [flash_encrypt] Flash encryption completed
      ...
      [esp32] [INF] Resetting with flash encryption enabled...

  2. Device will reset and it should be now operating similar to an actual encrypted device::

      ...
      [esp32] [INF] Checking flash encryption...
      [esp32] [INF] [flash_encrypt] flash encryption is enabled (1 plaintext flashes left)
      [esp32] [INF] Disabling RNG early entropy source...
      [esp32] [INF] br_image_off = 0x20000
      [esp32] [INF] ih_hdr_size = 0x20
      [esp32] [INF] Loading image 0 - slot 0 from flash, area id: 1
      ...
      NuttShell (NSH) NuttX-12.8.0
      nsh>

Actual encryption and burning E-Fuses
'''''''''''''''''''''''''''''''''''''

E-Fuses are burned by esptool and the bootloader on the first boot after flashing with encryption enabled.
This process is automated on NuttX build system.

.. warning::  Burning E-Fuses is NOT a reversible operation and should be done with caution.

To build a firmware with E-Fuse support and flash encryption enabled, open ``menuconfig`` and:
  1. Enable flash encryption on boot on: :menuselection:`System Type --> Bootloader and Image Configuration`
  2. Disable Virtual E-Fuses :menuselection:`System Type --> ESP32 Peripheral Selection --> E-Fuse support`
  3. Check usage mode is Development (this allows reflashing, while Release mode does not).

.. note::  If using development mode of flash encryption (see menuconfig and documentation above), it is still possible to re-flash the device with esptool by
  setting ``ESP32_SECURE_FLASH_ENC_FLASH_DEVICE_ENCRYPTED`` which adds ``--encrypt`` argument to the ``esptool.py write_flash`` command.
  This will apply the burned encryption key to the image while flashing.

Flash Allocation for MCUBoot
----------------------------

When MCUBoot is enabled on ESP32, the flash memory is organized as follows
based on the default KConfig values:

**Flash Layout (MCUBoot Enabled)**

.. list-table::
   :header-rows: 1
   :widths: 40 20 20
   :align: left

   * - Region
     - Offset
     - Size
   * - Bootloader
     - 0x001000
     - 64KB
   * - E-Fuse Virtual (see Note)
     - 0x010000
     - 64KB
   * - Primary Application Slot (/dev/ota0)
     - 0x020000
     - 1MB
   * - Secondary Application Slot (/dev/ota1)
     - 0x120000
     - 1MB
   * - Scratch Partition (/dev/otascratch)
     - 0x220000
     - 256KB
   * - Storage MTD (optional)
     - 0x260000
     - 1MB
   * - Available Flash
     - 0x360000+
     - Remaining

.. raw:: html

   <div style="clear: both"></div>


**Note**: The E-Fuse Virtual region is optional and only used when
``ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH`` is enabled. However, this 64KB
location is always allocated in the memory layout to prevent accidental
erasure during board flashing operations, ensuring data preservation if
virtual E-Fuses are later enabled.

.. code-block:: text

    Memory Map (Addresses in hex):

    0x001000  ┌─────────────────────────────┐
              │                             │
              │      MCUBoot Bootloader     │
              │           (64KB)            │
              │                             │
    0x010000  ├─────────────────────────────┤
              │       E-Fuse Virtual        │
              │           (64KB)            │
    0x020000  ├─────────────────────────────┤
              │                             │
              │      Primary App Slot       │
              │            (1MB)            │
              │          /dev/ota0          │
              │                             │
    0x120000  ├─────────────────────────────┤
              │                             │
              │     Secondary App Slot      │
              │            (1MB)            │
              │          /dev/ota1          │
              │                             │
    0x220000  ├─────────────────────────────┤
              │                             │
              │      Scratch Partition      │
              │           (256KB)           │
              │       /dev/otascratch       │
              │                             │
    0x260000  ├─────────────────────────────┤
              │                             │
              │   Storage MTD (optional)    │
              │            (1MB)            │
              │                             │
    0x360000  ├─────────────────────────────┤
              │                             │
              │       Available Flash       │
              │         (Remaining)         │
              │                             │
              └─────────────────────────────┘

The key KConfig options that control this layout:

- ``ESPRESSIF_OTA_PRIMARY_SLOT_OFFSET`` (default: 0x20000)
- ``ESPRESSIF_OTA_SECONDARY_SLOT_OFFSET`` (default: 0x120000)
- ``ESPRESSIF_OTA_SLOT_SIZE`` (default: 0x100000)
- ``ESPRESSIF_OTA_SCRATCH_OFFSET`` (default: 0x220000)
- ``ESPRESSIF_OTA_SCRATCH_SIZE`` (default: 0x40000)
- ``ESPRESSIF_STORAGE_MTD_OFFSET`` (default: 0x260000 when MCUBoot enabled)
- ``ESPRESSIF_STORAGE_MTD_SIZE`` (default: 0x100000)
- ``ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH_OFFSET`` (default 0x10000 when MCUBoot enabled)

For MCUBoot operation:

- The **Primary Slot** contains the currently running application
- The **Secondary Slot** receives OTA updates
- The **Scratch Partition** is used by MCUBoot for image swapping during updates
- MCUBoot manages image validation, confirmation, and rollback functionality

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

_`Managing esptool on virtual environment`
==========================================

This section describes how to install ``esptool``, ``imgtool`` or any other Python packages in a
proper environment.

Normally, a Linux-based OS would already have Python 3 installed by default. Up to a few years ago,
you could simply call ``pip install`` to install packages globally. However, this is no longer recommended
as it can lead to conflicts between packages and versions. The recommended way to install Python packages
is to use a virtual environment.

A virtual environment is a self-contained directory that contains a Python installation for a particular
version of Python, plus a number of additional packages. You can create a virtual environment for each
project you are working on, and install the required packages in that environment.

Two alternatives are explained below, you can select any one of those.

Using pipx (recommended)
------------------------

``pipx`` is a tool that makes it easy to install Python packages in a virtual environment. To install
``pipx``, you can run the following command (using apt as example)::

    $ apt install pipx

Once you have installed ``pipx``, you can use it to install Python packages in a virtual environment. For
example, to install the ``esptool`` package, you can run the following command::

    $ pipx install esptool

This will create a new virtual environment in the ``~/.local/pipx/venvs`` directory, which contains the
``esptool`` package. You can now use the ``esptool`` command as normal, and so will the build system.

Make sure to run ``pipx ensurepath`` to add the ``~/.local/bin`` directory to your ``PATH``. This will
allow you to run the ``esptool`` command from any directory.

Using venv (alternative)
------------------------
To create a virtual environment, you can use the ``venv`` module, which is included in the Python standard
library. To create a virtual environment, you can run the following command::

    $ python3 -m venv myenv

This will create a new directory called ``myenv`` in the current directory, which contains a Python
installation and a copy of the Python standard library. To activate the virtual environment, you can run
the following command::

    $ source myenv/bin/activate

This will change your shell prompt to indicate that you are now working in the virtual environment. You can
now install packages using ``pip``. For example, to install the ``esptool`` package, you can run the following
command::

    $ pip install esptool

This will install the ``esptool`` package in the virtual environment. You can now use the ``esptool`` command as
normal. When you are finished working in the virtual environment, you can deactivate it by running the following
command::

    $ deactivate

This will return your shell prompt to its normal state. You can reactivate the virtual environment at any time by
running the ``source myenv/bin/activate`` command again. You can also delete the virtual environment by deleting
the directory that contains it.


Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
