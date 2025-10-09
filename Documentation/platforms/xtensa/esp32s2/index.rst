==================
Espressif ESP32-S2
==================

The ESP32-S2 is a series of single-core SoCs from Espressif based on Harvard
architecture Xtensa LX7 CPU and with on-chip support for Wi-Fi.

All embedded memory, external memory and peripherals are located on the
data bus and/or the instruction bus of the CPU. Multiple peripherals in
the system can access embedded memory via DMA.

ESP32-S2 Toolchain
==================

The toolchain used to build ESP32-S2 firmware can be either downloaded or built from the sources.

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

For ESP32-S2, the toolchain version is based on GGC 14.2.0 (``xtensa-esp-elf-14.2.0_20241119``)

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

  $ ./ct-ng xtensa-esp32s2-elf
  $ ./ct-ng build

  $ chmod -R u+w builds/xtensa-esp32s2-elf

  $ export PATH="crosstool-NG/builds/xtensa-esp32-elf/bin:$PATH"

These steps are given in the setup guide in
`ESP-IDF documentation <https://docs.espressif.com/projects/esp-idf/en/latest/get-started/linux-setup-scratch.html>`_.

Building and flashing NuttX
===========================

Installing esptool
------------------

First, make sure that ``esptool.py`` is installed and up-to-date.
This tool is used to convert the ELF to a compatible ESP32-S2 image and to flash the image into the board.

It can be installed with: ``pip install esptool>=4.8.1``.

.. warning::
    Installing ``esptool.py`` may required a Python virtual environment on newer systems.
    This will be the case if the ``pip install`` command throws an error such as:
    ``error: externally-managed-environment``.

    If you are not familiar with virtual environments, refer to `Managing esptool on virtual environment`_ for instructions on how to install ``esptool.py``.


Bootloader and partitions
-------------------------

NuttX can boot the ESP32-S2 directly using the so-called "Simple Boot". An externally-built
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

This is a two-step process where the first step converts the ELF file into an ESP32-S2 compatible binary
and the second step flashes it to the board. These steps are included in the build system and it is
possible to build and flash the NuttX firmware simply by running::

    $ make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

where:

* ``ESPTOOL_PORT`` is typically ``/dev/ttyUSB0`` or similar.
* ``ESPTOOL_BINDIR=./`` is the path of the externally-built 2nd stage bootloader and the partition table (if applicable): when built using the ``make bootloader``, these files are placed into ``nuttx`` folder.
* ``ESPTOOL_BAUD`` is able to change the flash baud rate if desired.

Flashing NSH Example
--------------------

This example shows how to build and flash the ``nsh`` defconfig for the ESP32-S2-Saola-1 board::

    $ cd nuttx
    $ make distclean
    $ ./tools/configure.sh esp32s2-saola-1:nsh
    $ make -j$(nproc)

When the build is complete, the firmware can be flashed to the board using the command::

    $ make -j$(nproc) flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

where ``<port>`` is the serial port where the board is connected::

  $ make flash ESPTOOL_PORT=/dev/ttyUSB0 ESPTOOL_BINDIR=./
  CP: nuttx.hex
  MKIMAGE: ESP32-S2 binary
  esptool.py -c esp32s2 elf2image --ram-only-header -fs 4MB -fm dio -ff 40m -o nuttx.bin nuttx
  esptool.py v4.8.1
  Creating esp32s2 image...
  Image has only RAM segments visible. ROM segments are hidden and SHA256 digest is not appended.
  Merged 1 ELF section
  Successfully created esp32s2 image.
  Generated: nuttx.bin
  esptool.py -c esp32s2 -p /dev/ttyUSB0 -b 921600   write_flash -fs detect -fm dio -ff 40m 0x1000 nuttx.bin
  esptool.py v4.8.1
  Serial port /dev/ttyUSB0
  Connecting....
  Chip is ESP32-S2 (revision v0.0)
  [...]
  Flash will be erased from 0x00001000 to 0x00032fff...
  Compressed 202280 bytes to 71796...
  Wrote 202280 bytes (71796 compressed) at 0x00001000 in 2.3 seconds (effective 698.5 kbit/s)...
  Hash of data verified.

  Leaving...
  Hard resetting via RTS pin...

Now opening the serial port with a terminal emulator should show the NuttX console::

  $ picocom -b 115200 /dev/ttyUSB0
  NuttShell (NSH) NuttX-12.8.0
  nsh> uname -a
  NuttX 12.8.0 759d37b97c-dirty Mar  5 2025 20:26:00 xtensa esp32s2-saola-1

Debugging
=========

This section describes debugging techniques for the ESP32-S2.

Debugging with ``openocd`` and ``gdb``
--------------------------------------

Espressif uses a specific version of OpenOCD to support ESP32-S2: `openocd-esp32 <https://github.com/espressif/>`_.

Please check `Building OpenOCD from Sources <https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32s2/api-guides/jtag-debugging/index.html#jtag-debugging-building-openocd>`_
for more information on how to build OpenOCD for ESP32-S2.

ESP32-S2 has dedicated pins for JTAG debugging. The following pins are used for JTAG debugging:

============= ===========
ESP32-S2 Pin  JTAG Signal
============= ===========
MTDO / GPIO40 TDO
MTDI / GPIO41 TDI
MTCK / GPIO39 TCK
MTMS / GPIO42 TMS
============= ===========

Some boards, like :ref:`ESP32-S2-Kaluga-1 Kit v1.3 <platforms/xtensa/esp32s2/boards/esp32s2-kaluga-1/index:ESP32-S2-Kaluga-1 Kit v1.3>` have a built-in JTAG debugger.

Other boards that don't have any built-in JTAG debugger can be debugged using an external JTAG debugger being connected
directly to the ESP32-S2 JTAG pins.

.. note:: One must configure the USB drivers to enable JTAG communication. Please check
  `Configure USB Drivers <https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32s2/api-guides/jtag-debugging/configure-ft2232h-jtag.html?highlight=udev#configure-usb-drivers>`_
  for configuring the JTAG adapter of the :ref:`ESP32-S2-Kaluga-1 <platforms/xtensa/esp32s2/boards/esp32s2-kaluga-1/index:ESP32-S2-Kaluga-1 Kit v1.3>` board
  and other FT2232-based JTAG adapters.

OpenOCD can then be used::

  openocd -s <tcl_scripts_path> -c 'set ESP_RTOS hwthread' -f board/esp32s2-kaluga-1.cfg -c 'init; reset halt; esp appimage_offset 0x1000'

.. note::
  - ``appimage_offset`` should be set to ``0x1000`` when ``Simple Boot`` is used. For MCUboot, this value should be set to
    ``CONFIG_ESPRESSIF_OTA_PRIMARY_SLOT_OFFSET`` value (``0x10000`` by default).
  - ``-s <tcl_scripts_path>`` defines the path to the OpenOCD scripts. Usually set to `tcl` if running openocd from its source directory.
    It can be omitted if `openocd-esp32` were installed in the system with `sudo make install`.

Once OpenOCD is running, you can use GDB to connect to it and debug your application::

  xtensa-esp32s2-elf-gdb -x gdbinit nuttx

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
  _assert: Current Version: NuttX  10.4.0 2ae3246e40-dirty Sep 19 2024 14:15:50 xtensa
  _assert: Assertion failed user panic: at file: :0 task: backtrace process: backtrace 0x400a0aa8
  up_dump_register:    PC: 400a0ad8    PS: 00060730
  up_dump_register:    A0: 8009312c    A1: 3ffbaf90    A2: 00000000    A3: 3ffba008
  up_dump_register:    A4: 3ffba01e    A5: 3ffb95c0    A6: 00000000    A7: 00000000
  up_dump_register:    A8: 800a0ad5    A9: 3ffbaf60   A10: 0000005a   A11: 3ffb9dc0
  up_dump_register:   A12: 00000059   A13: 3ffb9710   A14: 00000002   A15: 3ffb9bb4
  up_dump_register:   SAR: 00000018 CAUSE: 0000001d VADDR: 00000000
  dump_stack: User Stack:
  dump_stack:   base: 0x3ffba028
  dump_stack:   size: 00004040
  dump_stack:     sp: 0x3ffbaf90
  stack_dump: 0x3ffbaf70: 00000059 3ffb9710 00000002 3ffb9bb4 80091fbc 3ffbafb0 400a0aa8 00000002
  stack_dump: 0x3ffbaf90: 3ffba01e 3ffb95c0 00000000 00000000 00000000 3ffbafd0 00000000 400a0aa8
  stack_dump: 0x3ffbafb0: 3ffba008 3ffb9bf8 00000000 3ffb637c 00000000 3ffbaff0 00000000 00000000
  stack_dump: 0x3ffbafd0: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
  stack_dump: 0x3ffbaff0: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
  sched_dumpstack: backtrace| 2: 0x4009f930 0x4009d94a 0x40094f7d 0x400945cd 0x400228e1 0x400a0ad8 0x4009312c 0x40091fbc
  sched_dumpstack: backtrace| 2: 0x40000000 0x4009312c 0x40091fbc 0x40000000
  dump_tasks:    PID GROUP PRI POLICY   TYPE    NPX STATE   EVENT      SIGMASK          STACKBASE  STACKSIZE   COMMAND
  dump_task:       0     0   0 FIFO     Kthread - Ready              0000000000000000 0x3ffb7720      3056   Idle_Task
  dump_task:       1     1 100 RR       Task    - Waiting Semaphore  0000000000000000 0x3ffb8d20      3024   nsh_main
  dump_task:       2     2 255 RR       Task    - Running            0000000000000000 0x3ffba028      4040   backtrace task
  sched_dumpstack: backtrace| 0: 0x40091327
  sched_dumpstack: backtrace| 1: 0x4009dde1 0x4009dce3 0x4009dd1c 0x400969fa 0x40096200 0x400964d8 0x400955f4 0x4009547c
  sched_dumpstack: backtrace| 1: 0x4009544d 0x4009312c 0x40091fbc 0x40000000
  sched_dumpstack: backtrace| 2: 0x4009f930 0x4009d6cc 0x4009db72 0x4009d97c 0x40094f7d 0x400945cd 0x400228e1 0x400a0ad8
  sched_dumpstack: backtrace| 2: 0x4009312c 0x40091fbc 0x40000000 0x4009312c 0x40091fbc 0x40000000

The lines starting with ``sched_dumpstack`` show the backtrace of the tasks. By checking it, it is
possible to track the root cause of the crash. Saving this output to a file and using the ``btdecode.sh``::

  ./tools/btdecode.sh esp32s2 /tmp/backtrace.txt
  Backtrace for task 2:
  0x4009f930: sched_dumpstack at sched_dumpstack.c:69
  0x4009d94a: _assert at assert.c:691
  0x40094f7d: xtensa_user_panic at xtensa_assert.c:188 (discriminator 1)
  0x400945cd: xtensa_user at ??:?
  0x400228e1: _xtensa_user_handler at xtensa_user_handler.S:194
  0x400a0ad8: assert_on_task at backtrace_main.c:158
   (inlined by) backtrace_main at backtrace_main.c:194
  0x4009312c: nxtask_startup at task_startup.c:70
  0x40091fbc: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0
  0x4009312c: nxtask_startup at task_startup.c:70
  0x40091fbc: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0

  Backtrace dump for all tasks:

  Backtrace for task 2:
  0x4009f930: sched_dumpstack at sched_dumpstack.c:69
  0x4009d6cc: dump_backtrace at assert.c:418
  0x4009db72: nxsched_foreach at sched_foreach.c:69 (discriminator 2)
  0x4009d97c: _assert at assert.c:726
  0x40094f7d: xtensa_user_panic at xtensa_assert.c:188 (discriminator 1)
  0x400945cd: xtensa_user at ??:?
  0x400228e1: _xtensa_user_handler at xtensa_user_handler.S:194
  0x400a0ad8: assert_on_task at backtrace_main.c:158
   (inlined by) backtrace_main at backtrace_main.c:194
  0x4009312c: nxtask_startup at task_startup.c:70
  0x40091fbc: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0
  0x4009312c: nxtask_startup at task_startup.c:70
  0x40091fbc: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0

  Backtrace for task 1:
  0x4009dde1: nxsem_wait at sem_wait.c:217
  0x4009dce3: nxsched_waitpid at sched_waitpid.c:165
  0x4009dd1c: waitpid at sched_waitpid.c:618
  0x400969fa: nsh_builtin at nsh_builtin.c:163
  0x40096200: nsh_execute at nsh_parse.c:652
   (inlined by) nsh_parse_command at nsh_parse.c:2840
  0x400964d8: nsh_parse at nsh_parse.c:2930
  0x400955f4: nsh_session at nsh_session.c:246
  0x4009547c: nsh_consolemain at nsh_consolemain.c:79
  0x4009544d: nsh_main at nsh_main.c:80
  0x4009312c: nxtask_startup at task_startup.c:70
  0x40091fbc: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0

  Backtrace for task 0:
  0x40091327: nx_start at nx_start.c:772 (discriminator 1)

The above output shows the backtrace of the tasks. By checking it, it is possible to track the
functions that were being executed when the crash occurred.

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX:

========== ======= =====
Peripheral Support NOTES
========== ======= =====
ADC          Yes   Oneshot
AES          No
CAN/TWAI     Yes
DAC          No
DMA          Yes
eFuse        Yes
GPIO         Yes    Dedicated GPIO supported
I2C          Yes    Master and Slave mode supported
I2S          Yes
LED/PWM      Yes
Pulse_CNT    Yes
RMT          Yes
RNG          Yes
RSA          No
RTC          Yes
SD/MMC       Yes    SPI based SD card driver
SHA          Yes
SPI          Yes
SPIFLASH     Yes
SPIRAM       Yes
Timers       Yes
Touch        Yes
UART         Yes
USB OTG      No
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
.                  0x00000000 0x3EFFFFFF                 Reserved
Data               0x3F000000 0x3F3FFFFF External Memory
Data               0x3F400000 0x3F4FFFFF Peripheral
Data               0x3F500000 0x3FF7FFFF External Memory
.                  0x3FF80000 0x3FF9DFFF                 Reserved
Data               0x3FF9E000 0x3FFFFFFF Embedded Memory
Instruction        0x40000000 0x40071FFF Embedded Memory
.                  0x40072000 0x4007FFFF                 Reserved
Instruction        0x40080000 0x407FFFFF External Memory
.                  0x40800000 0x4FFFFFFF                 Reserved
Data / Instruction 0x50000000 0x50001FFF Embedded Memory
.                  0x50002000 0x5FFFFFFF                 Reserved
Data / Instruction 0x60000000 0x600BFFFF Peripheral
.                  0x600C0000 0x617FFFFF                 Reserved
Data / Instruction 0x61800000 0x61803FFF Peripheral
.                  0x61804000 0xFFFFFFFF                 Reserved
================== ========== ========== =============== ===============

Embedded Memory
---------------

=========== ========== ========== =============== ================== =====
BUS TYPE    START      LAST       DESCRIPTION     PERMISSION CONTROL NOTES
=========== ========== ========== =============== ================== =====
Data        0x3FF9E000 0x3FF9FFFF RTC FAST Memory YES
Data        0x3FFA0000 0x3FFAFFFF Internal ROM 1  NO
Data        0x3FFB0000 0x3FFB7FFF Internal SRAM 0 YES                DMA
Data        0x3FFB8000 0x3FFFFFFF Internal SRAM 1 YES                DMA
=========== ========== ========== =============== ================== =====

Boundary Address (Embedded)
---------------------------

====================== ========== ========== =============== ================== ===============
BUS TYPE               START      LAST       DESCRIPTION     PERMISSION CONTROL NOTES
====================== ========== ========== =============== ================== ===============
Instruction            0x40000000 0x4000FFFF Internal ROM 0  NO
Instruction            0x40010000 0x4001FFFF Internal ROM 1  NO
Instruction            0x40020000 0x40027FFF Internal SRAM 0 YES
Instruction            0x40028000 0x4006FFFF Internal SRAM 1 YES
Instruction            0x40070000 0x40071FFF RTC FAST Memory YES
Data / Instruction     0x50000000 0x50001FFF RTC SLOW Memory YES
====================== ========== ========== =============== ================== ===============

External Memory
---------------

=========== ========== ========== =============== ================== ===============
BUS TYPE    START      LAST       DESCRIPTION     PERMISSION CONTROL NOTES
=========== ========== ========== =============== ================== ===============
Data        0x3F000000 0x3F3FFFFF ICache          YES                Read
Data        0x3F500000 0x3FF7FFFF DCache          YES                Read and Write
=========== ========== ========== =============== ================== ===============

Boundary Address (External)
---------------------------

=========== ========== ========== =============== ================== ===============
BUS TYPE    START      LAST       DESCRIPTION     PERMISSION CONTROL NOTES
=========== ========== ========== =============== ================== ===============
Instruction 0x40080000 0x407FFFFF ICache          YES                Read
=========== ========== ========== =============== ================== ===============

Linker Segments
---------------

+---------------------+------------+-------------------+------+------------------------------+
| DESCRIPTION         | START      | END               | ATTR | LINKER SEGMENT NAME          |
+=====================+============+===================+======+==============================+
| FLASH mapped data:  | 0X3F000020 | 0X3F000020 +      | R    | drom0_0_seg (NOTE 1)         |
|     - .rodata       |            | FLASH_SIZE - 0x20 |      |                              |
|     - Constructors  |            |                   |      |                              |
|       /destructors  |            |                   |      |                              |
+---------------------+------------+-------------------+------+------------------------------+
| COMMON data RAM:    | 0X3FFB0000 | 0x3FFDE000        | RW   | dram0_0_seg (NOTE 2)         |
|  - .bss/.data       |            |                   |      |                              |
+---------------------+------------+-------------------+------+------------------------------+
| IRAM for PRO cpu:   | 0x40022000 | 0x40050000        | RX   | iram0_0_seg                  |
|  - Interrupt Vectors|            |                   |      |                              |
|  - Low level        |            |                   |      |                              |
|    handlers         |            |                   |      |                              |
|  - Xtensa/Espressif |            |                   |      |                              |
|    libraries        |            |                   |      |                              |
+---------------------+------------+-------------------+------+------------------------------+
| RTC fast memory:    | 0x40070000 | 0x40072000        | RWX  | rtc_iram_seg                 |
|  - .rtc.text        |            |                   |      |                              |
|    (unused?)        |            |                   |      |                              |
+---------------------+------------+-------------------+------+------------------------------+
| FLASH:              | 0x40080020 | 0x40080020 +      | RX   | irom0_0_seg (actually FLASH) |
|  - .text            |            | FLASH_SIZE        |      |                              |
|                     |            | (NOTE 3)          |      |                              |
+---------------------+------------+-------------------+------+------------------------------+
| RTC slow memory:    | 0x50000000 | 0x50002000        | RW   | rtc_slow_seg (NOTE 4)        |
|  - .rtc.data/rodata |            |                   |      |                              |
|    (unused?)        |            |                   |      |                              |
+---------------------+------------+-------------------+------+------------------------------+

.. note::
  (1) The linker script will reserve space at the beginning of the segment
      for MCUboot header if ESP32S2_APP_FORMAT_MCUBOOT flag is active.
  (2) Heap starts at the end of dram_0_seg.
  (3) Subtract 0x20 if ESP32S2_APP_FORMAT_MCUBOOT is not active.
  (4) Linker script will reserve space at the beginning and at the end
      of the segment for ULP coprocessor reserve memory.

64-bit Timers
=============

ESP32-S2 has 4 generic timers of 64 bits (2 from Group 0 and 2 from Group 1).
They're accessible as character drivers, the configuration along with a
guidance on how to run the example and the description of the application level
interface can be found in the :doc:`timer documentation </components/drivers/character/timers/timer>`.

Watchdog Timers
===============

ESP32-S2 has 3 WDTs. 2 MWDTs from the Timers Module and 1 RWDT from the RTC Module
(Currently not supported yet). They're accessible as character drivers,
The configuration along with a guidance on how to run the example and the description
of the application level interface can be found in the
:doc:`watchdog documentation </components/drivers/character/timers/watchdog>`.

I2S
===

The I2S peripheral is accessible using either the generic I2S audio driver or a specific
audio codec driver. Also, it's possible to use the I2S character driver to bypass the
audio subsystem and develop specific usages of the I2S peripheral.

.. note:: Note that the bit-width and sample rate can be modified "on-the-go" when using
   audio-related drivers. That is not the case for the I2S character device driver and
   such parameters are set on compile time through `make menuconfig`.

Please check for usage examples using the :doc:`ESP32-S2-Saola-1 </platforms/xtensa/esp32s2/boards/esp32s2-saola-1/index>`.

Analog-to-digital converter (ADC)
=================================

Two ADC units are available for the ESP32-S2, each with 10 channels.

Those units are independent and can be used simultaneously. During bringup, GPIOs for selected channels are
configured automatically to be used as ADC inputs.
If available, ADC calibration is automatically applied (see
`this page <https://docs.espressif.com/projects/esp-idf/en/v5.1/esp32s2/api-reference/peripherals/adc_calibration.html>`__ for more details).
Otherwise, a simple conversion is applied based on the attenuation and resolution.

Each ADC unit is accessible using the ADC character driver, which returns data for the enabled channels.

The ADC unit can be enabled in the menu :menuselection:`System Type --> ESP32-S2 Peripheral Selection --> Analog-to-digital converter (ADC)`.

Then, it can be customized in the menu :menuselection:`System Type --> ADC Configuration`, which includes operating mode, gain and channels.

========== =========== ===========
 Channel    ADC1 GPIO   ADC2 GPIO
========== =========== ===========
0           1           11
1           2           12
2           3           13
3           4           14
4           5           15
5           6           16
6           7           17
7           8           18
8           9           19
9           10          20
========== =========== ===========

.. warning:: Minimum and maximum measurable voltages may saturate around 100 mV and 3000 mV, respectively.

Wi-Fi
======

.. tip:: Boards usually expose a ``wifi`` defconfig which enables Wi-Fi.

A standard network interface will be configured and can be initialized such as::

    nsh> ifup wlan0
    nsh> wapi psk wlan0 mypasswd 3
    nsh> wapi essid wlan0 myssid 1
    nsh> renew wlan0

In this case a connection to AP with SSID ``myssid`` is done, using ``mypasswd`` as
password. IP address is obtained via DHCP using ``renew`` command. You can check
the result by running ``ifconfig`` afterwards.

.. tip:: Please refer to :ref:`ESP32 Wi-Fi Station Mode <esp32_wi-fi_sta>`
  for more information.

Wi-Fi SoftAP
============

It is possible to use ESP32-S2 as an Access Point (SoftAP).

.. tip:: Boards usually expose a ``sta_softap`` defconfig which enables Wi-Fi
   (STA + SoftAP).

If you are using this board config profile you can run these commands to be able
to connect your smartphone or laptop to your board::

    nsh> ifup wlan1
    nsh> dhcpd_start wlan1
    nsh> wapi psk wlan1 mypasswd 3
    nsh> wapi essid wlan1 nuttxap 1

In this case, you are creating the access point ``nuttxapp`` in your board and to
connect to it on your smartphone you will be required to type the password ``mypasswd``
using WPA2.

.. tip:: Please refer to :ref:`ESP32 Wi-Fi SoftAP Mode <esp32_wi-fi_softap>`
  for more information.

The ``dhcpd_start`` is necessary to let your board to associate an IP to your smartphone.

.. _MCUBoot and OTA Update S2:

MCUBoot and OTA Update
======================

The ESP32-S2 supports over-the-air (OTA) updates using MCUBoot.

Read more about the MCUBoot for Espressif devices `here <https://docs.mcuboot.com/readme-espressif.html>`__.

Executing OTA Update
--------------------

This section describes how to execute OTA update using MCUBoot.

1. First build the default ``mcuboot_update_agent`` config. This image defaults to the primary slot and already comes with Wi-Fi settings enabled::

    ./tools/configure.sh esp32s2-saola-1:mcuboot_update_agent

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

Flash Encryption
----------------

Flash encryption is intended for encrypting the contents of the ESP32-S2's off-chip flash memory. Once this feature is enabled,
firmware is flashed as plaintext, and then the data is encrypted in place on the first boot. As a result, physical readout
of flash will not be sufficient to recover most flash contents.

The current state of flash encryption for ESP32-S2 allows the use of Virtual E-Fuses and development mode, which permit users to evaluate and test the firmware before making definitive changes such as burning E-Fuses.

Flash encryption supports the following features:

  .. list-table::
    :header-rows: 1

    * - Feature
      - Description
    * - **Flash Encryption with Virtual E-Fuses**
      - Use flash encryption without burning E-Fuses. Default selection when flash encryption is enabled.
    * - **Flash Encryption in Development mode**
      - Allows reflashing an encrypted device by appending the ``--encrypt`` argument to the ``esptool.py write_flash`` command. This is done automatically if ``ESP32S2_SECURE_FLASH_ENC_FLASH_DEVICE_ENCRYPTED`` is set.
    * - **Flash Encryption in Release mode**
      - Does not allow reflashing the device. This is a permanent setting.
    * - **Flash Encryption key**
      - A user-generated key is required by default. Alternatively, a device-generated key is possible, but it will not be recoverable by the user (not recommended). See ``ESP32S2_SECURE_FLASH_ENC_USE_HOST_KEY``.
    * - **Encrypted MTD Partition**
      - If SPI Flash is enabled, an empty user MTD partition will be automatically encrypted on first flash.

.. note::

   It is **strongly suggested** to read the following before working on flash encryption:

   - `MCUBoot Flash Encryption <https://docs.mcuboot.com/readme-espressif.html#flash-encryption>`_
   - `General E-Fuse documentation <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/system/efuse.html>`_
   - `Flash Encryption Relevant E-Fuses <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/security/flash-encryption.html#relevant-efuses>`_

Flash Encryption Requirements
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Flash encryption requires burning E-Fuses to enable it on chip. This is not a reversible operation and should be done with caution.
There is, however, a way to test the flash encryption by simulating them on flash. Both paths are described below.

Build System Features
'''''''''''''''''''''

The build system contains some safeguards to avoid accidentally burning E-Fuses and automations for convenience. Those are summarized below:

  1. A yellow warning will show up during build alerting that flash encryption is enabled (same for Virtual E-Fuses).
  2. If ``ESP32S2_SECURE_FLASH_ENC_USE_HOST_KEY`` is set, build will fail if the flash encryption key is not found.
  3. If SPI Flash is enabled, the user MTD partition is automatically encrypted with the provided encryption key.
  4. ``make flash`` command will prompt the user for confirmation before burning the E-Fuse, if Virtual E-Fuses are disabled.


Simulating Flash Encryption with Virtual E-Fuses
'''''''''''''''''''''''''''''''''''''''''''''''''

It is highly recommended to use this method for testing the flash encryption before actually burning the E-Fuses.
The E-Fuses are stored in flash and persist between reboots. No real E-Fuses are changed.

To enable virtual E-Fuses for flash encryption testing, open ``menuconfig`` and:
  1. Enable flash encryption on boot on: :menuselection:`System Type --> Bootloader and Image Configuration`
  2. Verify Virtual E-Fuses are enabled (this is done by default): :menuselection:`System Type --> ESP32-S2 Peripheral Support --> E-Fuse support`

Now build the bootloader and the firmware. Flashing the device will trigger the following:
  1. On the first boot, the bootloader will encrypt the flash::

      ...
      [esp32s2] [WRN] eFuse virtual mode is enabled. If Secure boot or Flash encryption is enabled then it does not provide any security. FOR TESTING ONLY!
      [esp32s2] [WRN] [efuse] [Virtual] try loading efuses from flash: 0x10000 (offset)
      ...
      [esp32s2] [INF] [flash_encrypt] Encrypting bootloader...
      [esp32s2] [INF] [flash_encrypt] Bootloader encrypted successfully
      [esp32s2] [INF] [flash_encrypt] Encrypting primary slot...
      [esp32s2] [INF] [flash_encrypt] Encrypting remaining flash...
      [esp32s2] [INF] [flash_encrypt] Flash encryption completed
      ...
      [esp32s2] [INF] Resetting with flash encryption enabled...

  2. Device will reset and it should be now operating similar to an actual encrypted device::

      ...
      [esp32s2] [INF] Checking flash encryption...
      [esp32s2] [INF] [flash_encrypt] flash encryption is enabled (1 plaintext flashes left)
      [esp32s2] [INF] Disabling RNG early entropy source...
      [esp32s2] [INF] br_image_off = 0x20000
      [esp32s2] [INF] ih_hdr_size = 0x20
      [esp32s2] [INF] Loading image 0 - slot 0 from flash, area id: 1
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
  2. Disable Virtual E-Fuses :menuselection:`System Type --> ESP32-S2 Peripheral Selection --> E-Fuse support`
  3. Check usage mode is Development (this allows reflashing, while Release mode does not).

.. note::  If using development mode of flash encryption (see menuconfig and documentation above), it is still possible to re-flash the device with esptool by
  setting ``ESP32S2_SECURE_FLASH_ENC_FLASH_DEVICE_ENCRYPTED`` which adds ``--encrypt`` argument to the ``esptool.py write_flash`` command.
  This will apply the burned encryption key to the image while flashing.

Flash Allocation for MCUBoot
----------------------------

When MCUBoot is enabled on ESP32-S2, the flash memory is organized as follows
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

.. _esp32s2_ulp:

ULP RISC-V Coprocessor
======================

The ULP RISC-V core is a 32-bit coprocessor integrated into the ESP32-S2 SoC.
It is designed to run independently of the main high-performance (HP) core and is capable of executing lightweight tasks
such as GPIO polling, simple peripheral control and I/O interactions.

This coprocessor benefits to offload simple tasks from HP core (e.g., GPIO polling , I2C operations, basic control logic) and
frees the main CPU for higher-level processing

For more information about ULP RISC-V Coprocessor `check here <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s2/api-reference/system/ulp-risc-v.html>`__.

Features of the ULP RISC-V Coprocessor
--------------------------------------

* Processor Architecture
   - RV32IMC RISC-V core — Integer (I), Multiplication/Division (M), and Compressed (C) instructions
   - Runs at 17.5 MHz
* Memory
   - Access to 8 KB of RTC slow memory (RTC_SLOW_MEM) memory region, and registers in RTC_CNTL, RTC_IO, and SARADC peripherals
* Debugging
   - Logging via bit-banged UART
   - Shared memory for state inspection
   - Panic or exception handlers can trigger wake-up or signal to main CPU if main CPU is in sleep
* Peripheral support
   - RTC domain peripherals (RTC GPIO, RTC I2C, ADC)

Loading Binary into ULP RISC-V Coprocessor
------------------------------------------

There are two ways to load a binary into LP-Core:
  - Using a prebuilt binary
  - Using NuttX internal build system to build your own (bare-metal) application

When using a prebuilt binary, the already compiled output for the ULP system whether built from NuttX
or the ESP-IDF environment can be leveraged. However, whenever the ULP code needs to be modified, it must be rebuilt separately,
and the resulting .bin file has to be integrated into NuttX. This workflow, while compatible, can become tedious.

With NuttX internal build system, the ULP binary code can be built and flashed from a single location. It is more convenient but
using build system has some dependencies on example side.

Both methods requires ``CONFIG_ESP32S2_ULP_COPROC_RESERVE_MEM`` variable to enable ULP RISC-V core.
These variables can be set using ``make menuconfig`` or ``kconfig-tweak`` commands.

Additionally, a Makefile needs to be provided to specify the ULP application name,
source path of the ULP application, and either the binary (for prebuilt) or the source files (for internal build).
This Makefile must include the ULP makefile after the variable set process on ``arch/xtensa/src/common/espressif/esp_ulp.mk`` integration script.
For more information please refer to :ref:`ulp example Makefile. <ulp_makefile>`

Makefile Variables for ULP RISC-V Core Build:
---------------------------------------------

- ``ULP_APP_NAME``: Sets name for the ULP RISC-V application. This variable also be used as prefix (e.g. ULP RISC-V application bin variable name)
- ``ULP_APP_FOLDER``: Specifies the directory containing the ULP RISC-V application's source codes.
- ``ULP_APP_BIN``: Defines the path of the prebuilt ULP RISC-V binary.
- ``ULP_APP_C_SRCS``: Lists all C source files (.c) that need to be compiled for the ULP RISC-V application.
- ``ULP_APP_ASM_SRCS``: Lists all assembly source files (.S or .s) to be assembled.
- ``ULP_APP_INCLUDES``: Specifies additional include directories for the compiler and assembler.

Here is an Makefile example when using prebuilt binary for ULP RISC-V core:

.. code-block:: console

   ULP_APP_NAME = esp_ulp
   ULP_APP_FOLDER = $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)$(CHIP_SERIES)
   ULP_APP_BIN = $(TOPDIR)$(DELIM)Documentation$(DELIM)platforms$(DELIM)$(CONFIG_ARCH)$(DELIM)$(CONFIG_ARCH_CHIP)$(DELIM)boards$(DELIM)$(CONFIG_ARCH_BOARD)$(DELIM)ulp_riscv_blink.bin

   include $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)common$(DELIM)espressif$(DELIM)esp_ulp.mk

Here is an example for enabling ULP and using the prebuilt test binary for ULP RISC-V core::

    make distclean
    ./tools/configure.sh esp32s2-saola-1:nsh
    kconfig-tweak --set-val CONFIG_ESP32S2_ULP_COPROC_RESERVE_MEM 8176
    kconfig-tweak -e CONFIG_ESPRESSIF_ULP_USE_TEST_BIN
    make olddefconfig
    make -j

Creating an ULP RISC-V Coprocessor Application
----------------------------------------------

To use NuttX's internal build system to compile the bare-metal ULP RISC-V Coprocessor binary, check the following instructions.

First, create a folder for the ULP source and header files into your NuttX example.
This folder is just for ULP project and it is an independent project. Therefore, the NuttX example guide should not be followed
for ULP example (folder location is irrelevant. It can be the same of the `nuttx-apps` repository, for instance).
To include the ULP folder in the build system, don't forget to include the ULP Makefile in the NuttX example Makefile. Lastly, configuration variables
needed to enable ULP core instructions can be found above.

NuttX's internal functions or POSIX calls are not supported.

Here is an example:

- ULP UART Snippet:

.. code-block:: C

  #include "ulp_riscv.h"
  #include "ulp_riscv_utils.h"
  #include "ulp_riscv_print.h"
  #include "ulp_riscv_uart_ulp_core.h"
  #include "sdkconfig.h"

  static ulp_riscv_uart_t s_print_uart;

  int main (void)
  {
    ulp_riscv_uart_cfg_t cfg = {
        .tx_pin = 0,
    };
    ulp_riscv_uart_init(&s_print_uart, &cfg);
    ulp_riscv_print_install((putc_fn_t)ulp_riscv_uart_putc, &s_print_uart);

    while(1)
    {
      ulp_riscv_print_str("Hello from the LP core!!\r\n");
      ulp_riscv_delay_cycles(1000 * ULP_RISCV_CYCLES_PER_MS);
    }

    return 0;
  }

For more information about ULP RISC-V Coprocessor examples `check here <https://github.com/espressif/esp-idf/tree/master/examples/system/ulp/lp_core>`__.
After these settings follow the same steps as for any other configuration to build NuttX. Build system checks ULP project path,
adds every source and header file into project and builds it.

To sum up, here is an example. ``ulp_example/ulp (../ulp_example/ulp)`` folder selected as example
to create a subfolder for ULP but folder that includes ULP source code can be anywhere. For more information about
custom apps, please follow NuttX `Custom Apps How-to <https://nuttx.apache.org/docs/latest/guides/customapps.html#custom-apps-how-to>`__ guide,
this example will demonstrate how to add ULP code into a custom application:

- Tree view:

.. code-block:: text

   nuttxspace/
   ├── nuttx/
   └── apps/
   └── ulp_example/
       └── Makefile
       └── Kconfig
       └── ulp_example.c
       └── ulp/
           └── Makefile
           └── ulp_main.c

- Contents in Makefile:

.. code-block:: console

   include $(APPDIR)/Make.defs

   PROGNAME  = $(CONFIG_EXAMPLES_ULP_EXAMPLE_PROGNAME)
   PRIORITY  = $(CONFIG_EXAMPLES_ULP_EXAMPLE_PRIORITY)
   STACKSIZE = $(CONFIG_EXAMPLES_ULP_EXAMPLE_STACKSIZE)
   MODULE    = $(CONFIG_EXAMPLES_ULP_EXAMPLE)

   MAINSRC = ulp_example.c

   include $(APPDIR)/Application.mk

   include ulp/Makefile

- Contents in Kconfig:

.. code-block:: console

   config EXAMPLES_ULP_EXAMPLE
     bool "ULP Example"
     default n

- Contents in ulp_example.c:

.. code-block:: C

   #include <nuttx/config.h>
   #include <stdio.h>
   #include <fcntl.h>
   #include <unistd.h>
   #include <sys/ioctl.h>
   #include <inttypes.h>
   #include <stdint.h>
   #include <stdbool.h>

   #include "ulp/ulp/ulp_main.h"
   /* Files that holds ULP binary header */

   #include "ulp/ulp/ulp_code.h"

   int main (void)
    {
      int fd;
      fd = open("/dev/ulp", O_WRONLY);
      if (fd < 0)
        {
          printf("Failed to open ULP: %d\n", errno);
          return -1;
        }
      /* ulp_example is the prefix which can be changed with ULP_APP_NAME makefile
       * variable to access ULP binary code variable */
      write(fd, ulp_example_bin, ulp_example_bin_len);
      return 0;
    }

.. _ulp_makefile:

- Contents in ulp/Makefile:

.. code-block:: console

  ULP_APP_NAME = ulp_example
  ULP_APP_FOLDER = $(APPDIR)$(DELIM)ulp_example$(DELIM)ulp
  ULP_APP_C_SRCS = ulp_main.c

  include $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)common$(DELIM)espressif$(DELIM)esp_ulp.mk

- Contents in ulp_main.c:

.. code-block:: C

   #include <stdio.h>
   #include <stdint.h>
   #include <stdbool.h>
   #include "ulp_riscv.h"
   #include "ulp_riscv_utils.h"
   #include "ulp_riscv_gpio.h"

   #define GPIO_PIN 0

   #define nop() __asm__ __volatile__ ("nop")

   bool gpio_level_previous = true;

   int main (void)
    {
       while (1)
           {
           ulp_riscv_gpio_output_level(GPIO_PIN, gpio_level_previous);
           gpio_level_previous = !gpio_level_previous;
           for (int i = 0; i < 10000; i++)
             {
               nop();
             }
           }

       return 0;
    }

- Command to build::

    make distclean
    ./tools/configure.sh esp32s2-saola-1:nsh
    kconfig-tweak --set-val CONFIG_ESP32S2_ULP_COPROC_RESERVE_MEM 8176
    kconfig-tweak -e CONFIG_DEV_GPIO
    kconfig-tweak -e CONFIG_EXAMPLES_ULP_EXAMPLE
    make olddefconfig
    make -j

Here is an example of a single ULP application. However, support is not limited to just
one application. Multiple ULP applications are also supported.
By following the same guideline, multiple ULP applications can be created and loaded using ``write`` POSIX call.
Each NuttX application can build one ULP application. Therefore, to build multiple ULP applications, multiple NuttX
applications are needed to create each ULP binary. This limitation only applies when using the NuttX build system to
build multiple ULP applications; it does not affect the ability to load multiple ULP applications built by other means.

ULP binary can be included in NuttX application by adding
``#include "ulp/ulp/ulp_code.h"`` line. Then, the ULP binary is accessible by using the ULP application
prefix (defined by the ``ULP_APP_NAME`` variable in the ULP application Makefile) with the ``bin`` keyword to
access the binary data (e.g., if ``ULP_APP_NAME`` is ``ulp_test``, the binary variable will be ``ulp_test_bin``)
and ``bin_len`` keyword to access its length (e.g., ``ulp_test_bin_len`` for ``ULP_APP_NAME`` is ``ulp_test``).

Accessing the ULP RISC-V Coprocessor Program Variables
------------------------------------------------------

Global symbols defined in the ULP application are available to the HP core through a shared memory region. To read or write ULP variables,
direct reading/writing to such memory positions are not allowed. POSIX calls are needed instead. To access the ULP variable through the HP core,
consider that its name is defined by the ULP application prefix (defined by the ``ULP_APP_NAME`` variable in the ULP application Makefile) + the ULP application variable.
For example if HP core tries to access a ULP application variable named ``result`` and ``ULP_APP_NAME`` in the ULP application Makefile set as ``ulp_app``, required name for
that variable will be ``ulp_app_result``.
``FIONREAD`` or ``FIONWRITE`` ioctl calls are, then, performed with the address of a ``struct symtab_s`` previously defined with the name of the variable to be read or written.

.. warning::
  Ensure that the related ULP application is running. Otherwise, another ULP application may interfere by using the same memory space for a different variables.

Here is a snippet for reading and writing to a ULP variable named ``var_test`` (assuming the ``ULP_APP_NAME`` is set to ``ulp``) through the HP core:

.. code-block:: C

   #include <nuttx/config.h>
   #include <stdio.h>
   #include <fcntl.h>
   #include <unistd.h>
   #include <sys/ioctl.h>
   #include "nuttx/symtab.h"

   int main (void)
    {
      uint32_t ulp_var;
      int fd;
      struct symtab_s sym =
      {
        .sym_name = "ulp_var_test",
        .sym_value = &ulp_var,
      };
      fd = open("/dev/ulp", O_RDWR);
      ioctl(fd, FIONREAD, &sym);
      if (ulp_var != 0)
        {
          ulp_var = 0;
          ioctl(fd, FIONWRITE, &sym);
        }

      return OK;
    }

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
