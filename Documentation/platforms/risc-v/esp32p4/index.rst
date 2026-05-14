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

.. note:: NuttX by default supports ESP32-P4 chip revisions v3.0 and above.
          If your build and flash works fine but firmware does not run as
          expected (i.e. stuck in a boot loop) then additional tuning is
          required, see `ESP32-P4 Chip Revisions`_ section for details.


ESP32-P4 Toolchain
==================

A generic RISC-V toolchain can be used to build ESP32-P4 projects.
You can use standard ``riscv32-esp-elf-gcc`` provided by ESP IDF Tools.
However, it is recommended to use the same toolchain version as used by
the NuttX CI for RISC-V. Please refer to our CI Docker configuration
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

Building with CMake
-------------------

General CMake usage (out-of-tree build, ``menuconfig`` target, and so on) is described in
:doc:`/quickstart/compiling_cmake`. The ESP32-P4 arch uses the shared Espressif RISC-V
integration, which enables post-build steps that produce ``nuttx.bin`` (and related images)
under the **CMake binary directory**; the build log also prints suggested ``esptool.py``
command lines for your layout (including the ESP32-P4 image offset when applicable).

Example (NuttX shell defconfig, Ninja generator)::

  $ cd nuttx
  $ cmake -B build -DBOARD_CONFIG=esp32p4-function-ev-board:nsh -GNinja
  $ cmake --build build

To reconfigure the tree after changing options (same as other NuttX CMake boards)::

  $ cmake --build build -t menuconfig
  $ cmake --build build

Persistent HAL cache (``NXTMPDIR``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Pass ``-DNXTMPDIR=ON`` at **configure** time to reuse a persistent clone of the
``esp-hal-3rdparty`` repository under ``nuttx/../nxtmpdir/esp-hal-3rdparty``. CMake checks
the expected revision; if it does not match, the cache directory is refreshed. This cuts
repeat configure/build time when the HAL checkout would otherwise be re-fetched into the
binary directory.

Example::

  $ cmake -B build -DBOARD_CONFIG=esp32p4-function-ev-board:nsh -DNXTMPDIR=ON -GNinja
  $ cmake --build build

MCUBoot: building the 2nd-stage bootloader (``-t bootloader``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For configurations that use MCUboot, build the bootloader the same way as
with Make, but via the CMake target::

  $ cmake --build build -t bootloader

The image is installed as ``mcuboot-esp32p4.bin`` in the NuttX **source** directory (not
inside ``build/``).

.. note::

   Flashing paths differ from the pure-Make flow: the application image is under your CMake
   build directory (for example ``build/nuttx.bin``), while MCUboot binaries live next to
   ``nuttx`` sources. Use the ``esptool.py`` hints printed at the end of the build, or the
   same offsets as documented for ``make flash`` with ``ESPTOOL_BINDIR``.

Target flashing (``-t flash``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After a successful CMake build, you can flash the chip with the ``flash`` custom target.
This is the CMake-side equivalent of the Make ``FLASH`` logic in
``tools/espressif/Config.mk``.

**Serial port:** you must set ``ESPTOOL_PORT`` to a non-empty value (for example
``/dev/ttyUSB0``). If it is unset or empty, the flash step fails.

Example::

  $ export ESPTOOL_PORT=/dev/ttyUSB0
  $ cmake --build build -t flash

Or for a single invocation::

  $ ESPTOOL_PORT=/dev/ttyUSB0 cmake --build build -t flash

CMake limitations
^^^^^^^^^^^^^^^^^^

The following is **not** supported when building with CMake yet; use the Make-based build instead:

* **ULP / LP core** — Low-power coprocessor support (``CONFIG_ESPRESSIF_USE_LP_CORE``) is
  not wired up for CMake (ULP/LP integration is TODO).


.. _esp32p4_debug:

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
SPIRAM / PSRAM     Yes
Watchdog           Yes     MWDT0/1 and RWDT
Ethernet           Yes
Brownout           No
Debug Probe        No
================= ======= ==================================

LP Peripherals
--------------

================= ======= ==================================
Peripheral        Support NOTES
================= ======= ==================================
LP SPI             No
LP I2C             Yes
LP I2S             No
LP UART            Yes
LP GPIO            No
LP Timers          No
LP ADC             Yes
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
AES                Yes
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


.. _esp32p4_ulp:

ULP LP Core Coprocessor
=======================

The ULP LP core (Low-power core) is a 32-bit RISC-V coprocessor integrated into the ESP32-P4 SoC.
It is designed to run independently of the main high-performance (HP) core and is capable of executing lightweight tasks
such as GPIO polling, simple peripheral control and I/O interactions.

This coprocessor benefits to offload simple tasks from HP core (e.g., GPIO polling , I2C operations, basic control logic) and
frees the main CPU for higher-level processing

For more information about ULP LP Core Coprocessor `check here <https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-reference/system/ulp-lp-core.html>`__.

Features of the ULP LP-Core
---------------------------

* Processor Architecture
   - RV32I RISC-V core with IMAC extensions—Integer (I), Multiplication/Division (M), Atomic (A), and Compressed (C) instructions
   - Runs at 40 MHz
* Memory
   - Access to 16 KB of low-power ROM (LP-ROM) any time
   - Access to 32 KB of low-power memory (LP-RAM) and LP-domain peripherals any time
   - Access to 768 KB of L2 SRAM any time with latency
   - Full access to all of the chip's memory and peripherals when the HP core is active
* Debugging
   - Built-in JTAG debug module for external debugging
   - Supports LP UART for logging from the ULP itself
   - Includes a panic handler capable of dumping register state via LP UART on exceptions
* Peripheral support
   - LP domain peripherals (LP GPIO, LP I2C, LP UART, LP SPI, LP Mailbox, LP Timer and more)
   - Full access HP domain peripherals when when the HP core is active

Loading Binary into ULP LP-Core
-------------------------------

There are two ways to load a binary into LP-Core:
  - Using a prebuilt binary
  - Using NuttX internal build system to build your own (bare-metal) application

When using a prebuilt binary, the already compiled output for the ULP system whether built from NuttX
or the ESP-IDF environment can be leveraged. However, whenever the ULP code needs to be modified, it must be rebuilt separately,
and the resulting .bin file has to be integrated into NuttX. This workflow, while compatible, can become tedious.

With NuttX internal build system, the ULP binary code can be built and flashed from a single location. It is more convenient but
using build system has some dependencies on example side.

Both methods requires ``CONFIG_ESPRESSIF_USE_LP_CORE`` variable to enable ULP core
and it can be set using ``make menuconfig`` or ``kconfig-tweak`` commands.

Additionally, a Makefile needs to be provided to specify the ULP application name,
source path of the ULP application, and either the binary (for prebuilt) or the source files (for internal build).
This Makefile must include the ULP makefile after the variable set process on ``arch/risc-v/src/common/espressif/esp_ulp.mk`` integration script.
For more information please refer to :ref:`ulp example Makefile. <ulp_makefile>`

Makefile Variables for ULP Core Build:
--------------------------------------

- ``ULP_APP_NAME``: Sets name for the ULP application. This variable also be used as prefix (e.g. ULP application bin variable name)
- ``ULP_APP_FOLDER``: Specifies the directory containing the ULP application's source codes.
- ``ULP_APP_BIN``: Defines the path of the prebuilt ULP binary.
- ``ULP_APP_C_SRCS``: Lists all C source files (.c) that need to be compiled for the ULP application.
- ``ULP_APP_ASM_SRCS``: Lists all assembly source files (.S or .s) to be assembled.
- ``ULP_APP_INCLUDES``: Specifies additional include directories for the compiler and assembler.

Here is an Makefile example when using prebuilt binary for ULP core:

.. code-block:: console

   ULP_APP_NAME = esp_ulp
   ULP_APP_FOLDER = $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)$(CHIP_SERIES)
   ULP_APP_BIN = $(TOPDIR)$(DELIM)Documentation$(DELIM)platforms$(DELIM)$(CONFIG_ARCH)$(DELIM)$(CONFIG_ARCH_CHIP)$(DELIM)boards$(DELIM)$(CONFIG_ARCH_BOARD)$(DELIM)ulp_riscv_blink.bin

   include $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)common$(DELIM)espressif$(DELIM)esp_ulp.mk


Here is an example for enabling ULP and using the prebuilt test binary for ULP core::

    make distclean
    ./tools/configure.sh esp32p4-function-ev-board:nsh
    kconfig-tweak -e CONFIG_ESPRESSIF_USE_LP_CORE
    kconfig-tweak -e CONFIG_ESPRESSIF_ULP_USE_TEST_BIN
    make olddefconfig
    make -j

Creating an ULP LP-Core Application
-----------------------------------

To use NuttX's internal build system to compile the bare-metal LP binary, check the following instructions.

First, create a folder for the ULP source and header files into your NuttX example.
This folder is just for ULP project and it is an independent project. Therefore, the NuttX example guide should not be followed
for ULP example (folder location is irrelevant. It can be the same of the `nuttx-apps` repository, for instance).
To include the ULP folder in the build system, don't forget to include the ULP Makefile in the NuttX example Makefile. Lastly, configuration variables
needed to enable ULP core instructions can be found above.

NuttX's internal functions or POSIX calls are not supported.

Here is an example:

- ULP UART Snippet:

.. code-block:: C

  #include <stdint.h>
  #include "ulp_lp_core_print.h"
  #include "ulp_lp_core_utils.h"
  #include "ulp_lp_core_uart.h"
  #include "ulp_lp_core_gpio.h"

  #define nop() __asm__ __volatile__ ("nop")

  int main (void)
  {
    while(1)
    {

      lp_core_printf("Hello from the LP core!!\r\n");
      for (int i = 0; i < 10000; i++)
        {
          nop();
        }
    }

    return 0;
  }

For more information about ULP Core Coprocessor examples `check here <https://github.com/espressif/esp-idf/tree/master/examples/system/ulp/lp_core>`__.
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

   #include <stdint.h>
   #include <stdbool.h>
   #include "ulp_lp_core_gpio.h"

   #define GPIO_PIN 0

   #define nop() __asm__ __volatile__ ("nop")

   bool gpio_level_previous = true;

   int main (void)
    {
       while (1)
           {
           ulp_lp_core_gpio_set_level(GPIO_PIN, gpio_level_previous);
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
    ./tools/configure.sh esp32p4-function-ev-board:nsh
    kconfig-tweak -e CONFIG_ESPRESSIF_GPIO_IRQ
    kconfig-tweak -e CONFIG_DEV_GPIO
    kconfig-tweak -e CONFIG_ESPRESSIF_USE_LP_CORE
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

ULP LP-Core Wakeup Configuration
--------------------------------

By default, ULP LP-Core is woken up by HP core but other wakeup sources can be selected.

The available wakeup sources are:

* ``CONFIG_ESPRESSIF_ULP_WAKEUP_HP_CPU``: Wakeup by HP core
* ``CONFIG_ESPRESSIF_ULP_WAKEUP_LP_TIMER``: Wakeup by LP timer
* ``CONFIG_ESPRESSIF_ULP_WAKEUP_LP_UART``: Wakeup by LP UART activity
* ``CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO``: Wakeup by LP IO

Accessing the ULP LP-Core Program Variables
-------------------------------------------

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

Debugging ULP LP-Core
---------------------

To debug ULP LP-Core please first refer to :ref:`Debugging section. <esp32p4_debug>`
Debugging ULP core consist same steps with some small differences. First of all, configuration file
needs to be changed from ``board/esp32p4-builtin.cfg`` or ``board/esp32p4-ftdi.cfg`` to
``board/esp32p4-lpcore-builtin.cfg`` or ``board/esp32p4-lpcore-ftdi.cfg`` depending on preferred debug adapter.

LP core supports limited set of HW exceptions, so, for example, writing at address
0x0 will not cause a panic as it would be for the code running on HP core.
This can be overcome to some extent by enabling undefined behavior sanitizer for LP core application,
so ubsan can help to catch some errors. But note that it will increase code size significantly and
it can happen that application won't fit into RTC RAM.
To enable ubsan for ULP please add ``CONFIG_ESPRESSIF_ULP_ENABLE_UBSAN`` in menuconfig.


ESP32-P4 Chip Revisions
=======================

.. attention:: NuttX by default supports ESP32-P4 chip revisions starting
               from v3.0. 

Different ESP32-P4 chip revisions contain internal hardware breaking
changes. Revisions 3.0 and higher are not compatible with 0.x and 1.x.
Older chip revisions may work but require dedicated firmware builds.
Compatibility is verified by the 2nd stage bootloader against following
firmware build parameters: ``CONFIG_ESP32P4_SELECTS_REV_LESS_V3``,
``CONFIG_ESP32P4_REV_*``, ``CONFIG_ESP_REV_*``,
and ``CONFIG_ESP_EFUSE_BLOCK_REV_*``.
These parameters are set with ``make menuconfig`` in the ``System Type`` menu:

* ``Select ESP32-P4 revisions <3.0`` - enable if your chip is older than v3.0.
* ``Minimum Supported ESP32-P4 Revision`` - select minimum chip revision
  on which the firmware can boot, refuse to boot otherwise (boot loop).
  Note that older chip revisions may contain bugs or have missing features.
  Selecting wide range of supported revisions will increase firmware size.
* ``Minimum Supported ESP32-P4 eFuse Block Revision`` - select minimum eFuse
  block revision on which the firmware can boot, refuse to boot otherwise
  (boot loop).

You can check your chip revision with ``esptool`` utility that is used
by ``make flash`` command, for instance::

    esptool v5.2.0
    Connected to ESP32-P4 on /dev/cuaU0:
    Chip type:          ESP32-P4 (revision v1.3)
    Features:           Dual Core + LP Core, 400MHz
    Crystal frequency:  40MHz
    MAC:                XXX


Supported Boards
================

.. toctree::
  :glob:
  :maxdepth: 1

  boards/*/*

Other Boards
------------

* `WaveShare ESP32-P4-Nano <https://www.waveshare.com/wiki/ESP32-P4-NANO>`_
  is similar in design to
  :ref:`ESP32-P4-Function-EV-Board <esp32p4-function-ev-board>`.
  Aside from a few onboard component differences you can play around using
  existing configurations for start. Note that it may contain ESP32-P4 v1.3 so
  additional configuration tuning is required (see `ESP32-P4 Chip Revisions`_).
