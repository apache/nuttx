==================
Espressif ESP32-P4
==================

.. tags:: chip:esp32p4, arch:risc-v, vendor:espressif

The ESP32-P4 is a high-performance, highly integrated SoC featuring RISC-V
processors, rich multimedia capabilities, and abundant peripherals. It targets
applications that demand efficient power usage, compact design, security, high
performance, and reliability.

Key highlights include:

* RISC-V 32-bit processors
  * Dual-core High Performance (HP) running at 360 MHz
  * Low Power core (LP) at 40 MHz
* Powerful image and voice processing capability
* In-package PSRAM (16 MB or 32 MB depending on the chip variant)
* 55 GPIOs and rich peripheral set
* Additional multimedia and I/O features commonly used on ESP32-P4 based designs, such as USB 2.0, MIPI-CSI/DSI, and H.264 encoder

Typical application scenarios include smart home, industrial automation, audio devices,
IoT sensor hubs, data loggers, video streaming cameras, USB devices, and speech/image
recognition systems.

References: see the ESP Hardware Design Guidelines for ESP32-P4 and the Technical
Reference Manual for detailed SoC architecture, memory map, and peripheral
descriptions.

* Product overview: `ESP32-P4 Product Overview <https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32p4/product-overview.html>`_
* Datasheet: `ESP32-P4 Datasheet (v1.3) <https://documentation.espressif.com/esp32-p4-chip-revision-v1.3_datasheet_en.pdf>`_
* Technical Reference Manual: `ESP32-P4 TRM (v1.3) <https://documentation.espressif.com/esp32-p4-chip-revision-v1.3_technical_reference_manual_en.pdf>`_

ESP32-P4 Toolchain
==================

A generic RISC-V toolchain can be used to build ESP32-P4 projects. It's recommended
to use the same toolchain version used by NuttX CI for RISC-V.
Please refer to the Docker
`container <https://github.com/apache/nuttx/tree/master/tools/ci/docker/linux/Dockerfile>`_
and check for the current compiler version being used. For instance:

.. code-block::

  ###############################################################################
  # Build image for tool required by RISCV builds
  ###############################################################################
  FROM nuttx-toolchain-base AS nuttx-toolchain-riscv
  # Download the latest RISCV GCC toolchain prebuilt by xPack
  RUN mkdir -p riscv-none-elf-gcc && \
    curl -s -L "https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v14.2.0-3/xpack-riscv-none-elf-gcc-14.2.0-3-linux-x64.tar.gz" \
    | tar -C riscv-none-elf-gcc --strip-components 1 -xz

Installing
----------

First, create a directory to hold the toolchain:

.. code-block:: console

  $ mkdir -p /path/to/your/toolchain/riscv-none-elf-gcc

Download and extract the recommended toolchain (example uses xPack GCC):

.. code-block:: console

  $ curl -s -L "https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v14.2.0-3/xpack-riscv-none-elf-gcc-14.2.0-3-linux-x64.tar.gz" \
  | tar -C /path/to/your/toolchain/riscv-none-elf-gcc --strip-components 1 -xz

Add the toolchain to your ``PATH``:

.. code-block:: console

  $ echo "export PATH=/path/to/your/toolchain/riscv-none-elf-gcc/bin:$PATH" >> ~/.bashrc

You can edit your shell's rc files if you don't use bash.

Building and flashing NuttX
===========================

Installing esptool
------------------

First, make sure that ``esptool.py`` is installed and up-to-date.
This tool is used to convert the ELF to an ESP image and to flash the image into the board.

It can be installed with: ``pip install esptool``.

.. warning::
    Installing ``esptool.py`` may require a Python virtual environment on newer systems.
    If the ``pip install`` command is blocked by an externally-managed environment policy,
    consider using a virtual environment (``python3 -m venv``) or a tool like ``pipx``.

Bootloader and partitions
-------------------------

NuttX supports booting ESP SoCs directly using "Simple Boot". Depending on future
requirements, an externally-built 2nd stage bootloader (e.g., MCUboot) may also be
used. Refer to board-specific documentation for the selected boot method and
partitioning.

Building and Flashing
---------------------

Building and flashing are board-specific. Once an ESP32-P4 board is selected via
``./tools/configure.sh``, in general you can build and flash with:

.. code-block:: console

    $ make -j$(nproc)
    $ make -j$(nproc) flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

Where ``<port>`` is the serial/USB port connected to your board.

Please check `Supported Boards`_ for the actual commands.

Debugging
=========

Debugging with ``openocd`` and ``gdb``
--------------------------------------

Espressif uses a specific version of OpenOCD to support ESP chips: ``openocd-esp32``.
Please check `Building OpenOCD from Sources <https://docs.espressif.com/projects/esp-idf/en/stable/api-guides/jtag-debugging/index.html#jtag-debugging-building-openocd>`_
for more information on how to build OpenOCD.

ESP32-P4 integrates a USB-to-JTAG adapter, so no external JTAG adapter is necessary.

OpenOCD can then be used::

  openocd -s <tcl_scripts_path> -c 'set ESP_RTOS hwthread; set ESP_ONLYCPU 1' -f board/esp32p4-builtin.cfg -c 'init; reset halt; esp appimage_offset 0x2000'

.. note::
  - ``appimage_offset`` should be set to ``0x2000`` when ``Simple Boot`` is used.
  - ``-s <tcl_scripts_path>`` defines the path to the OpenOCD scripts. Usually set to `tcl` if running openocd from its source directory.
    It can be omitted if `openocd-esp32` was installed in the system with `sudo make install`.

Once OpenOCD is running, GDB can be used to connect to the device and debug the running application::

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

NuttX can dump the stack of a task and produce backtraces to aid debugging,
especially when diagnosing exceptions and crashes.

Enable the following options to use this feature: ``CONFIG_SCHED_BACKTRACE``
and ``CONFIG_DEBUG_SYMBOLS``; optionally enable ``CONFIG_ALLSYMS`` to have
symbolized backtraces at runtime (at the expense of larger binary size).

When ``CONFIG_ALLSYMS`` is not enabled, use NuttX's ``./tools/btdecode.sh`` tool to
translate addresses into symbols by providing the captured console log and the
ELF file.

Example - Crash Dump
^^^^^^^^^^^^^^^^^^^^

A typical crash dump, caused by an illegal load with ``CONFIG_SCHED_BACKTRACE`` and
``CONFIG_DEBUG_SYMBOLS`` enabled, is shown below::

  riscv_exception: EXCEPTION: Store/AMO access fault. MCAUSE: 00000007, EPC: 40017174, MTVAL: 00000000
  riscv_exception: PANIC!!! Exception = 00000007
  dump_assert_info: Current Version: NuttX  10.4.0 baf7d9dd3b-dirty Oct 24 2025 15:54:40 risc-v
  dump_assert_info: Assertion failed panic: at file: common/riscv_exception.c:134 task: backtrace process: backtrace 0x4001712e
  up_dump_register: EPC: 40017174
  up_dump_register: A0: 0000005a A1: 00000000 A2: 00000002 A3: 00000004
  up_dump_register: A4: 7ffffffe A5: 00000000 A6: 7fffffff A7: 00000000
  up_dump_register: T0: 4fc1a058 T1: 0000000f T2: ffffffff T3: 00000000
  up_dump_register: T4: 00000000 T5: 00000000 T6: 00000000
  up_dump_register: S0: 4ff0a776 S1: 4ff0a760 S2: 00000000 S3: 00000000
  up_dump_register: S4: 00000000 S5: 00000000 S6: 00000000 S7: 00000000
  up_dump_register: S8: 00000000 S9: 00000000 S10: 00000000 S11: 00000000
  up_dump_register: SP: 4ff0b710 FP: 4ff0a776 TP: 00000000 RA: 40017174
  dump_stackinfo: User Stack:
  dump_stackinfo:   base: 0x4ff0a780
  dump_stackinfo:   size: 00004048
  dump_stackinfo:     sp: 0x4ff0b710
  stack_dump: 0x4ff0b6f0: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00001880
  stack_dump: 0x4ff0b710: 00000000 4ff0a3b0 4001712e 400098f8 00000000 00000000 00000002 4ff0a760
  stack_dump: 0x4ff0b730: 00000000 00000000 00000000 400070c4 00000000 00000000 00000000 00000000
  stack_dump: 0x4ff0b750: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
  sched_dumpstack: backtrace| 2: 0x40017174
  dump_tasks:    PID GROUP PRI POLICY   TYPE    NPX STATE   EVENT      SIGMASK          STACKBASE  STACKSIZE   COMMAND
  dump_tasks:   ----   --- --- -------- ------- --- ------- ---------- ---------------- 0x4ff066c0      2048   irq
  dump_task:       0     0   0 FIFO     Kthread -   Ready              0000000000000000 0x4ff08984      2032   Idle_Task
  dump_task:       1     1 100 RR       Task    -   Waiting Semaphore  0000000000000000 0x4ff09898      1992   nsh_main
  dump_task:       2     2 255 RR       Task    -   Running            0000000000000000 0x4ff0a780      4048   backtrace task
  sched_dumpstack: backtrace| 0: 0x4000bb20
  sched_dumpstack: backtrace| 1: 0x4000c088
  sched_dumpstack: backtrace| 2: 0x40017174

The lines starting with ``sched_dumpstack`` show the backtrace of the tasks. By checking it, it is
possible to track the root cause of the crash. Saving this output to a file and using the ``btdecode.sh``::

  ./tools/btdecode.sh esp32p4 backtrace.log
  Backtrace for task 2:
  0x40017174: assert_on_task at backtrace_main.c:168
  (inlined by) backtrace_main at backtrace_main.c:204

  Backtrace dump for all tasks:

  Backtrace for task 2:
  0x40017174: assert_on_task at backtrace_main.c:168
  (inlined by) backtrace_main at backtrace_main.c:204

  Backtrace for task 1:
  0x4000c088: sys_call0 at syscall.h:161
  (inlined by) up_switch_context at riscv_switchcontext.c:83

  Backtrace for task 0:
  0x4000bb20: up_idle at esp_idle.c:76

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX on
ESP32-P4 based on current Kconfig options and upstream support status. Refer to
board documentation for what is enabled by default.

HP Peripherals
--------------

================= ======= ==================================
Peripheral        Support NOTES
================= ======= ==================================
SPI                Yes     SPI2 master/slave; bitbang
I2C                Yes
I2S                Yes
ADC                Yes
ISP                No
PPA                No
GPIO               Yes     Dedicated GPIO supported
UART               Yes
Bit Scrambler      No
SD/MMC Host        No
H264 Encoder       No
2D-DMA             No
TWAI (CAN)         Yes     TWAI0/1
Pulse Counter      Yes     Implemented as Quadrature Encoder
RMT                Yes
USB Serial/JTAG    Yes
JPEG Codec         No
I3C                No
GDMA               Yes
SOC ETM            No
USB 2.0 OTG        No
Camera Interface   No
MIPI CSI           No
LED PWM            Yes
MCPWM              Yes     Motor control and capture
Parallel IO        No
LCD Interface      No
MIPI DSI           No
Timers             Yes
Watchdog           Yes     MWDT0/1 and RWDT
Ethernet           No
Brownout           No
Debug Probe        No
================= ======= ==================================

LP Peripherals
--------------

================= ======= ==================================
Peripheral        Support NOTES
================= ======= ==================================
LP SPI             No
LP I2C             No
LP I2S             No
LP UART            No
LP GPIO            No
LP Timers          No
LP ADC             No
Temperature        Yes     Internal temperature sensor
Touch Sensor       No
eFuse              Yes     Virtual eFuses supported
================= ======= ==================================


Security
--------

================= ======= ==================================
Peripheral        Support NOTES
================= ======= ==================================
SHA                Yes
RSA                No
ECC                No
HMAC               No
TRNG               No
ECDSA              No
TEE                No
APM                No
AES                No
Digital Signature  No
Secure Boot        No
XTS_AES            No
4096-bit OTP       No
PMP/PMA            Mo
================= ======= ==================================

.. note::
   The exact feature availability and default pin mapping depend on the board
   and configuration. Consult the board documentation and the
   :file:`arch/risc-v/src/common/espressif/Kconfig` for feature flags and
   pin selections.

Supported Boards
================

.. toctree::
  :glob:
  :maxdepth: 1

  boards/*/*
