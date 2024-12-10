==================
Espressif ESP32-S3
==================

The ESP32-S3 is a series of single and dual-core SoCs from Espressif
based on Harvard architecture Xtensa LX7 CPUs and with on-chip support
for Bluetooth and Wi-Fi.

All embedded memory, external memory and peripherals are located on the
data bus and/or the instruction bus of these CPUs. With some minor
exceptions, the address mapping of two CPUs is symmetric, meaning they
use the same addresses to access the same memory. Multiple peripherals in
the system can access embedded memory via DMA.

On dual-core SoCs, the two CPUs are typically named "PRO_CPU" and "APP_CPU"
(for "protocol" and "application"), however for most purposes the
two CPUs are interchangeable.

ESP32-S3 Toolchain
==================

The toolchain used to build ESP32-S3 firmware can be either downloaded or built from the sources.
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
  # Download the latest ESP32 GCC toolchain prebuilt by Espressif
  RUN mkdir -p xtensa-esp32-elf-gcc && \
    curl -s -L "https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/xtensa-esp32-elf-12.2.0_20230208-x86_64-linux-gnu.tar.xz" \
    | tar -C xtensa-esp32-elf-gcc --strip-components 1 -xJ

  RUN mkdir -p xtensa-esp32s2-elf-gcc && \
    curl -s -L "https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/xtensa-esp32s2-elf-12.2.0_20230208-x86_64-linux-gnu.tar.xz" \
    | tar -C xtensa-esp32s2-elf-gcc --strip-components 1 -xJ

  RUN mkdir -p xtensa-esp32s3-elf-gcc && \
    curl -s -L "https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/xtensa-esp32s3-elf-12.2.0_20230208-x86_64-linux-gnu.tar.xz" \
    | tar -C xtensa-esp32s3-elf-gcc --strip-components 1 -xJ

For ESP32-S3, the toolchain version is based on GGC 12.2.0 (``xtensa-esp32s3-elf-12.2.0_20230208``)

The prebuilt Toolchain (Recommended)
------------------------------------

First, create a directory to hold the toolchain:

.. code-block:: console

  $ mkdir -p /path/to/your/toolchain/xtensa-esp32s3-elf-gcc

Download and extract toolchain:

.. code-block:: console

  $ curl -s -L "https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/xtensa-esp32s3-elf-12.2.0_20230208-x86_64-linux-gnu.tar.xz" \
  | tar -C xtensa-esp32s3-elf-gcc --strip-components 1 -xJ

Add the toolchain to your `PATH`:

.. code-block:: console

  $ echo "export PATH=/path/to/your/toolchain/xtensa-esp32s3-elf-gcc/bin:$PATH" >> ~/.bashrc

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

  $ ./ct-ng xtensa-esp32s3-elf
  $ ./ct-ng build

  $ chmod -R u+w builds/xtensa-esp32s3-elf

  $ export PATH="crosstool-NG/builds/xtensa-esp32-elf/bin:$PATH"

These steps are given in the setup guide in
`ESP-IDF documentation <https://docs.espressif.com/projects/esp-idf/en/latest/get-started/linux-setup-scratch.html>`_.

Building and flashing NuttX
===========================

Bootloader and partitions
-------------------------

NuttX can boot the ESP32-S3 directly using the so-called "Simple Boot". An externally-built
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

First, make sure that ``esptool.py`` is installed.  This tool is used to convert the ELF to a
compatible ESP32-S3 image and to flash the image into the board.
It can be installed with: ``pip install esptool==4.8.dev4``.

It's a two-step process where the first converts the ELF file into an ESP32-S3 compatible binary
and the second flashes it to the board. These steps are included in the build system and it is
possible to build and flash the NuttX firmware simply by running::

    $ make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

where ``<port>`` is typically ``/dev/ttyUSB0`` or similar. ``ESPTOOL_BINDIR=./`` is the path of the
externally-built 2nd stage bootloader and the partition table (if applicable): when built using the
``make bootloader``, these files are placed into ``nuttx`` folder. ``ESPTOOL_BAUD`` is able to
change the flash baud rate if desired.

Debugging
=========

This section describes debugging techniques for the ESP32-S3.

Debugging with ``openocd`` and ``gdb``
--------------------------------------

Espressif uses a specific version of OpenOCD to support ESP32-S3: `openocd-esp32 <https://github.com/espressif/>`_.

Please check `Building OpenOCD from Sources <https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32s3/api-guides/jtag-debugging/index.html#jtag-debugging-building-openocd>`_
for more information on how to build OpenOCD for ESP32-S3.

The quickest and most convenient way to start with JTAG debugging is through a USB cable
connected to the D+/D- USB pins of ESP32-S3. No need for an external JTAG adapter and
extra wiring/cable to connect JTAG to ESP32-S3. Most of the ESP32-S3 boards have a
USB connector that can be used for JTAG debugging.
This is the case for the :ref:`ESP32-S3-DevKit <platforms/xtensa/esp32s3/boards/esp32s3-devkit/index:ESP32S3-DevKit>` board.

.. note:: One must configure the USB drivers to enable JTAG communication. Please check
  `Configure USB Drivers <https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32s3/api-guides/jtag-debugging/configure-builtin-jtag.html?highlight=udev#configure-usb-drivers>`_
  for more information.

OpenOCD can then be used::

  openocd -c 'set ESP_RTOS hwthread; set ESP_FLASH_SIZE 0' -f board/esp32s3-builtin.cfg

Once OpenOCD is running, you can use GDB to connect to it and debug your application::

  xtensa-esp32s3-elf-gdb -x gdbinit nuttx

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
  _assert: Current Version: NuttX  10.4.0 2ae3246e40-dirty Sep 19 2024 14:19:27 xtensa
  _assert: Assertion failed user panic: at file: :0 task: backtrace process: backtrace 0x42020c90
  up_dump_register:    PC: 42020cc0    PS: 00060930
  up_dump_register:    A0: 82012d10    A1: 3fc8e2e0    A2: 00000000    A3: 3fc8d350
  up_dump_register:    A4: 3fc8d366    A5: 3fc8c900    A6: 00000000    A7: 00000000
  up_dump_register:    A8: 82020cbd    A9: 3fc8e2b0   A10: 0000005a   A11: 3fc8d108
  up_dump_register:   A12: 00000059   A13: 3fc8ca50   A14: 00000002   A15: 3fc8cefc
  up_dump_register:   SAR: 00000018 CAUSE: 0000001d VADDR: 00000000
  up_dump_register:  LBEG: 40056f08  LEND: 40056f12  LCNT: 00000000
  dump_stack: User Stack:
  dump_stack:   base: 0x3fc8d370
  dump_stack:   size: 00004048
  dump_stack:     sp: 0x3fc8e2e0
  stack_dump: 0x3fc8e2c0: 00000059 3fc8ca50 00000002 3fc8cefc 82011ba0 3fc8e300 42020c90 00000002
  stack_dump: 0x3fc8e2e0: 3fc8d366 3fc8c900 00000000 00000000 00000000 3fc8e320 00000000 42020c90
  stack_dump: 0x3fc8e300: 3fc8d350 3fc8cf40 00000000 3fc8912c 00000000 3fc8e340 00000000 00000000
  stack_dump: 0x3fc8e320: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
  stack_dump: 0x3fc8e340: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
  sched_dumpstack: backtrace| 2: 0x4201fc6c 0x403773a0 0x40376f69 0x40376ee1 0x40374ca9 0x42020cc0 0x42012d10 0x42011ba0
  sched_dumpstack: backtrace| 2: 0x40000000 0x40000000 0x42012d10 0x42011ba0 0x40000000 0x40000000
  dump_tasks:    PID GROUP PRI POLICY   TYPE    NPX STATE   EVENT      SIGMASK          STACKBASE  STACKSIZE   COMMAND
  dump_tasks:   ----   --- --- -------- ------- --- ------- ---------- ---------------- 0x3fc8b220      2048   irq
  dump_task:       0     0   0 FIFO     Kthread - Ready              0000000000000000 0x3fc8a630      3056   Idle_Task
  dump_task:       1     1 100 RR       Task    - Waiting Semaphore  0000000000000000 0x3fc8c468      1992   nsh_main
  dump_task:       2     2 255 RR       Task    - Running            0000000000000000 0x3fc8d370      4048   backtrace task
  sched_dumpstack: backtrace| 0: 0x42010f37 0x40374dda 0x40374e9a 0x40045c04 0x40043ab9 0x40034c48 0x40000000
  sched_dumpstack: backtrace| 1: 0x4201e131 0x4201e033 0x4201e06c 0x42017056 0x4201685c 0x42016b34 0x42015c50 0x42015ad8
  sched_dumpstack: backtrace| 1: 0x42015aa9 0x42012d10 0x42011ba0 0x40000000 0x40000000
  sched_dumpstack: backtrace| 2: 0x4201fc6c 0x40377098 0x4201df02 0x403773ed 0x40376f69 0x40376ee1 0x40374ca9 0x42020cc0
  sched_dumpstack: backtrace| 2: 0x42012d10 0x42011ba0 0x40000000 0x40000000 0x42012d10 0x42011ba0 0x40000000 0x40000000

The lines starting with ``sched_dumpstack`` show the backtrace of the tasks. By checking it, it is
possible to track the root cause of the crash. Saving this output to a file and using the ``btdecode.sh``::

  ./tools/btdecode.sh esp32s3 /tmp/backtrace.txt
  Backtrace for task 2:
  0x4201fc6c: sched_dumpstack at sched_dumpstack.c:69
  0x403773a0: _assert at assert.c:691
  0x40376f69: xtensa_user_panic at xtensa_assert.c:188 (discriminator 1)
  0x40376ee1: xtensa_user at ??:?
  0x40374ca9: _xtensa_user_handler at xtensa_user_handler.S:194
  0x42020cc0: assert_on_task at backtrace_main.c:158
   (inlined by) backtrace_main at backtrace_main.c:194
  0x42012d10: nxtask_startup at task_startup.c:70
  0x42011ba0: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0
  0x40000000: ?? ??:0
  0x42012d10: nxtask_startup at task_startup.c:70
  0x42011ba0: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0
  0x40000000: ?? ??:0

  Backtrace dump for all tasks:

  Backtrace for task 2:
  0x4201fc6c: sched_dumpstack at sched_dumpstack.c:69
  0x40377098: dump_backtrace at assert.c:418
  0x4201df02: nxsched_foreach at sched_foreach.c:69 (discriminator 2)
  0x403773ed: _assert at assert.c:726
  0x40376f69: xtensa_user_panic at xtensa_assert.c:188 (discriminator 1)
  0x40376ee1: xtensa_user at ??:?
  0x40374ca9: _xtensa_user_handler at xtensa_user_handler.S:194
  0x42020cc0: assert_on_task at backtrace_main.c:158
   (inlined by) backtrace_main at backtrace_main.c:194
  0x42012d10: nxtask_startup at task_startup.c:70
  0x42011ba0: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0
  0x40000000: ?? ??:0
  0x42012d10: nxtask_startup at task_startup.c:70
  0x42011ba0: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0
  0x40000000: ?? ??:0

  Backtrace for task 1:
  0x4201e131: nxsem_wait at sem_wait.c:217
  0x4201e033: nxsched_waitpid at sched_waitpid.c:165
  0x4201e06c: waitpid at sched_waitpid.c:618
  0x42017056: nsh_builtin at nsh_builtin.c:163
  0x4201685c: nsh_execute at nsh_parse.c:652
   (inlined by) nsh_parse_command at nsh_parse.c:2840
  0x42016b34: nsh_parse at nsh_parse.c:2930
  0x42015c50: nsh_session at nsh_session.c:246
  0x42015ad8: nsh_consolemain at nsh_consolemain.c:79
  0x42015aa9: nsh_main at nsh_main.c:80
  0x42012d10: nxtask_startup at task_startup.c:70
  0x42011ba0: nxtask_start at task_start.c:75
  0x40000000: ?? ??:0
  0x40000000: ?? ??:0

  Backtrace for task 0:
  0x42010f37: nx_start at nx_start.c:772 (discriminator 1)
  0x40374dda: __esp32s3_start at esp32s3_start.c:439 (discriminator 1)
  0x40374e9a: __start at ??:?
  0x40045c04: ?? ??:0
  0x40043ab9: ?? ??:0
  0x40034c48: ?? ??:0
  0x40000000: ?? ??:0

The above output shows the backtrace of the tasks. By checking it, it is possible to track the
functions that were being executed when the crash occurred.

Using QEMU
==========

Get or build QEMU from `here <https://github.com/espressif/qemu/wiki>`__. The minimum supported version is 9.0.0.

Enable the ``ESP32S3_QEMU_IMAGE`` config found in :menuselection:`Board Selection --> ESP32S3 binary image for QEMU`.

Enable ``ESP32S3_APP_FORMAT_LEGACY``.

Build and generate the QEMU image::

 $ make bootloader
 $ make ESPTOOL_BINDIR=.

A QEMU-compatible ``nuttx.merged.bin`` binary image will be created. It can be run as::

 $ qemu-system-xtensa -nographic -machine esp32s3 -drive file=nuttx.merged.bin,if=mtd,format=raw

QEMU Networking
---------------

Networking is possible using the openeth MAC driver. Enable ``ESP32S3_OPENETH`` option and set the nic in QEMU:

 $ qemu-system-xtensa -nographic -machine esp32s3 -drive file=nuttx.merged.bin,if=mtd,format=raw -nic user,model=open_eth

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX:

========== ======= =====
Peripheral Support NOTES
========== ======= =====
ADC          YES
AES          YES
Bluetooth    No
CAMERA       No
CAN/TWAI     Yes
DMA          Yes
eFuse        No
GPIO         Yes
I2C          No
I2S          Yes
LCD          No
LED_PWM      No
MCPWM        Yes
Pulse_CNT    Yes
RMT          No
RNG          No
RSA          No
RTC          Yes
SD/MMC       Yes
SDIO         No
SHA          No
SPI          Yes
SPIFLASH     Yes
SPIRAM       Yes
Timers       Yes
Touch        Yes
UART         Yes
USB OTG      No
USB SERIAL   Yes
Watchdog     Yes
Wi-Fi        Yes   WPA3-SAE supported
========== ======= =====

.. _esp32s3_peripheral_support:

Wi-Fi
-----

.. tip:: Boards usually expose a ``wifi`` defconfig which enables Wi-Fi. On ESP32-S3,
   SMP is enabled to enhance Wi-Fi performance.

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
------------

It is possible to use ESP32-S3 as an Access Point (SoftAP).

.. tip:: Boards usually expose a ``sta_softap`` defconfig which enables Wi-Fi
   (STA + SoftAP). On ESP32-S3, SMP is enabled to enhance Wi-Fi performance.

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

Supported Boards
================

.. toctree::
  :glob:
  :maxdepth: 1

  boards/*/*
