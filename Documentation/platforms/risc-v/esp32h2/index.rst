==================
Espressif ESP32-H2
==================

The ESP32-H2 is an ultra-low-power and highly integrated SoC with a RISC-V
core and supports 2.4 GHz transceiver, Bluetooth 5 (LE) and the 802.15.4 protocol.

* Address Space
  - 452 KB of internal memory address space accessed from the instruction bus
  - 452 KB of internal memory address space accessed from the data bus
  - 832 KB of peripheral address space
  - 16 MB of external memory virtual address space accessed from the instruction bus
  - 16 MB of external memory virtual address space accessed from the data bus
  - 260 KB of internal DMA address space
* Internal Memory
  - 128 KB ROM
  - 320 KB SRAM (16 KB can be configured as Cache)
  - 4 KB of SRAM in RTC
* External Memory
  - Up to 16 MB of external flash
* Peripherals
  - Multiple peripherals
* GDMA
  - 7 modules are capable of DMA operations.

ESP32-H2 Toolchain
==================

A generic RISC-V toolchain can be used to build ESP32-H2 projects. It's recommended to use the same
toolchain used by NuttX CI. Please refer to the Docker
`container <https://github.com/apache/nuttx/tree/master/tools/ci/docker/linux/Dockerfile>`_ and
check for the current compiler version being used. For instance:

.. code-block::

  ###############################################################################
  # Build image for tool required by RISCV builds
  ###############################################################################
  FROM nuttx-toolchain-base AS nuttx-toolchain-riscv
  # Download the latest RISCV GCC toolchain prebuilt by xPack
  RUN mkdir riscv-none-elf-gcc && \
  curl -s -L "https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v13.2.0-2/xpack-riscv-none-elf-gcc-13.2.0-2-linux-x64.tar.gz" \
  | tar -C riscv-none-elf-gcc --strip-components 1 -xz

It uses the xPack's prebuilt toolchain based on GCC 13.2.0-2.

Installing
----------

First, create a directory to hold the toolchain:

.. code-block:: console

  $ mkdir -p /path/to/your/toolchain/riscv-none-elf-gcc

Download and extract toolchain:

.. code-block:: console

  $ curl -s -L "https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v13.2.0-2/xpack-riscv-none-elf-gcc-13.2.0-2-linux-x64.tar.gz" \
  | tar -C /path/to/your/toolchain/riscv-none-elf-gcc --strip-components 1 -xz

Add the toolchain to your `PATH`:

.. code-block:: console

  $ echo "export PATH=/path/to/your/toolchain/riscv-none-elf-gcc/bin:$PATH" >> ~/.bashrc

You can edit your shell's rc files if you don't use bash.

Building and flashing NuttX
===========================

Bootloader and partitions
-------------------------

NuttX can boot the ESP32-H2 directly using the so-called "Simple Boot".
An externally-built 2nd stage bootloader is not required in this case as all
functions required to boot the device are built within NuttX. Simple boot does not
require any specific configuration (it is selectable by default if no other
2nd stage bootloader is used). For compatibility among other SoCs and future options
of 2nd stage bootloaders, the commands ``make bootloader`` and the ``ESPTOOL_BINDIR``
option (for the ``make flash``) are kept (and ignored if Simple Boot is used).

If other features are required, an externally-built 2nd stage bootloader is needed.
The bootloader is built using the ``make bootloader`` command. This command generates
the firmware in the ``nuttx`` folder. The ``ESPTOOL_BINDIR`` is used in the
``make flash`` command to specify the path to the bootloader. For compatibility
among other SoCs and future options of 2nd stage bootloaders, the commands
``make bootloader`` and the ``ESPTOOL_BINDIR`` option (for the ``make flash``)
can be used even if no externally-built 2nd stage bootloader
is being built (they will be ignored if Simple Boot is used, for instance)::

  $ make bootloader

.. note:: It is recommended that if this is the first time you are using the board with NuttX to
   perform a complete SPI FLASH erase.

    .. code-block:: console

      $ esptool.py erase_flash

Building and flashing
---------------------

First, make sure that ``esptool.py`` is installed.  This tool is used to convert
the ELF to a compatible ESP32-H2 image and to flash the image into the board.
It can be installed with: ``pip install esptool==4.8.dev4``.

Configure the NuttX project: ``./tools/configure.sh esp32h2-devkit:nsh``
Run ``make`` to build the project.  Note that the conversion mentioned above is
included in the build process.
The ``esptool.py`` is used to flash all the binaries. However, this is also
included in the build process and we can build and flash with::

  make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

Where ``<port>`` is typically ``/dev/ttyUSB0`` or similar and ``./`` is
the path to the folder containing the externally-built 2nd stage bootloader for
the ESP32-H2 as explained above.

Debugging
=========

This section describes debugging techniques for the ESP32-H2.

Debugging with ``openocd`` and ``gdb``
--------------------------------------

Espressif uses a specific version of OpenOCD to support ESP32-H2: `openocd-esp32 <https://github.com/espressif/>`_.

Please check `Building OpenOCD from Sources <https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32h2/api-guides/jtag-debugging/index.html#jtag-debugging-building-openocd>`_
for more information on how to build OpenOCD for ESP32-H2.

You do not need an external JTAG to debug, the ESP32-H2 integrates a
USB-to-JTAG adapter.

.. note:: One must configure the USB drivers to enable JTAG communication. Please check
  `Configure USB Drivers <https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32h2/api-guides/jtag-debugging/configure-builtin-jtag.html#configure-usb-drivers>`_
  for more information.

OpenOCD can then be used::

  openocd -c 'set ESP_RTOS hwthread; set ESP_FLASH_SIZE 0' -f board/esp32h2-builtin.cfg

If you want to debug with an external JTAG adapter it can
be connected as follows:

============ ===========
ESP32-H2 Pin JTAG Signal
============ ===========
GPIO2        TMS
GPIO5        TDI
GPIO4        TCK
GPIO3        TDO
============ ===========

Furthermore, an efuse needs to be burnt to be able to debug::

  espefuse.py -p <port> burn_efuse DIS_USB_JTAG

.. warning:: Burning eFuses is an irreversible operation, so please
  consider the above option before starting the process.

OpenOCD can then be used::

  openocd  -c 'set ESP_RTOS hwthread; set ESP_FLASH_SIZE 0' -f board/esp32h2-ftdi.cfg

Once OpenOCD is running, you can use GDB to connect to it and debug your application::

  riscv-none-elf-gdb -x gdbinit nuttx

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

  riscv_exception: EXCEPTION: Store/AMO access fault. MCAUSE: 00000007, EPC: 42012df0, MT0
  riscv_exception: PANIC!!! Exception = 00000007
  _assert: Current Version: NuttX  10.4.0 2ae3246e40-dirty Sep 19 2024 14:53:33 risc-v
  _assert: Assertion failed panic: at file: :0 task: backtrace process: backtrace 0x42012daa
  up_dump_register: EPC: 42012df0
  up_dump_register: A0: 0000005a A1: 408095e4 A2: 00000001 A3: 00000088
  up_dump_register: A4: 00007fff A5: 00000001 A6: 00000000 A7: 00000000
  up_dump_register: T0: 00000000 T1: 00000000 T2: ffffffff T3: 00000000
  up_dump_register: T4: 00000000 T5: 00000000 T6: 00000000
  up_dump_register: S0: 408086ae S1: 40808698 S2: 00000000 S3: 00000000
  up_dump_register: S4: 00000000 S5: 00000000 S6: 00000000 S7: 00000000
  up_dump_register: S8: 00000000 S9: 00000000 S10: 00000000 S11: 00000000
  up_dump_register: SP: 40809640 FP: 408086ae TP: 00000000 RA: 42012df0
  dump_stack: User Stack:
  dump_stack:   base: 0x408086b8
  dump_stack:   size: 00004040
  dump_stack:     sp: 0x40809640
  stack_dump: 0x40809620: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00001880
  stack_dump: 0x40809640: 00000000 408082b0 42012daa 42006e1e 00000000 00000000 40808698 00000002
  stack_dump: 0x40809660: 00000000 00000000 00000000 42004d8a 00000000 00000000 00000000 00000000
  stack_dump: 0x40809680: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
  sched_dumpstack: backtrace| 2: 0x42012df0
  dump_tasks:    PID GROUP PRI POLICY   TYPE    NPX STATE   EVENT      SIGMASK          STACKBASE  STACKSIZE   COMMAND
  dump_tasks:   ----   --- --- -------- ------- --- ------- ---------- ---------------- 0x40805120      2048   irq
  dump_task:       0     0   0 FIFO     Kthread - Ready              0000000000000000 0x408068b0      2032   Idle_Task
  dump_task:       1     1 100 RR       Task    - Waiting Semaphore  0000000000000000 0x408077c8      1992   nsh_main
  dump_task:       2     2 255 RR       Task    - Running            0000000000000000 0x408086b8      4040   backtrace task
  sched_dumpstack: backtrace| 0: 0x42008420
  sched_dumpstack: backtrace| 1: 0x420089a2
  sched_dumpstack: backtrace| 2: 0x42012df0

The lines starting with ``sched_dumpstack`` show the backtrace of the tasks. By checking it, it is
possible to track the root cause of the crash. Saving this output to a file and using the ``btdecode.sh``::

  ./tools/btdecode.sh esp32h2 /tmp/backtrace.txt
  Backtrace for task 2:
  0x42012df0: assert_on_task at backtrace_main.c:158
   (inlined by) backtrace_main at backtrace_main.c:194

  Backtrace dump for all tasks:

  Backtrace for task 2:
  0x42012df0: assert_on_task at backtrace_main.c:158
   (inlined by) backtrace_main at backtrace_main.c:194

  Backtrace for task 1:
  0x420089a2: sys_call2 at syscall.h:227
   (inlined by) up_switch_context at riscv_switchcontext.c:95

  Backtrace for task 0:
  0x42008420: up_idle at esp_idle.c:74

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX:

==============  =======
Peripheral      Support
==============  =======
ADC              No
AES              No
Bluetooth        No
CAN/TWAI         Yes
DMA              Yes
ECC              No
eFuse            Yes
GPIO             Yes
HMAC             No
I2C              Yes
I2S              No
Int. Temp.       No
LED              No
LED_PWM          Yes
MCPWM            No
Pulse Counter    Yes
RMT              No
RNG              No
RSA              No
RTC              Yes
SD/MMC           No
SDIO             No
SHA              No
SPI              Yes
SPIFLASH         Yes
Timers           Yes
UART             Yes
Watchdog         Yes
Wifi             No
XTS              No
==============  =======

Supported Boards
================

.. toctree::
  :glob:
  :maxdepth: 1

  boards/*/*
