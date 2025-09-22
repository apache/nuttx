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
  # Download the latest ESP32, ESP32-S2 and ESP32-S3 GCC toolchain prebuilt by Espressif
  RUN mkdir -p xtensa-esp-elf-gcc && \
    curl -s -L "https://github.com/espressif/crosstool-NG/releases/download/esp-14.2.0_20241119/xtensa-esp-elf-14.2.0_20241119-x86_64-linux-gnu.tar.xz" \
    | tar -C xtensa-esp-elf-gcc --strip-components 1 -xJ

For ESP32-S3, the toolchain version is based on GGC 14.2.0 (``xtensa-esp-elf-14.2.0_20241119``)

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

  $ ./ct-ng xtensa-esp32s3-elf
  $ ./ct-ng build

  $ chmod -R u+w builds/xtensa-esp32s3-elf

  $ export PATH="crosstool-NG/builds/xtensa-esp32-elf/bin:$PATH"

These steps are given in the setup guide in
`ESP-IDF documentation <https://docs.espressif.com/projects/esp-idf/en/latest/get-started/linux-setup-scratch.html>`_.

Building and flashing NuttX
===========================

Installing esptool
------------------

First, make sure that ``esptool.py`` is installed and up-to-date.
This tool is used to convert the ELF to a compatible ESP32-S3 image and to flash the image into the board.

It can be installed with: ``pip install esptool>=4.8.1``.

.. warning::
    Installing ``esptool.py`` may required a Python virtual environment on newer systems.
    This will be the case if the ``pip install`` command throws an error such as:
    ``error: externally-managed-environment``.

    If you are not familiar with virtual environments, refer to `Managing esptool on virtual environment`_ for instructions on how to install ``esptool.py``.

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

This is a two-step process where the first step converts the ELF file into an ESP32-S3 compatible binary
and the second step flashes it to the board. These steps are included in the build system and it is
possible to build and flash the NuttX firmware simply by running::

    $ make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

where:

* ``ESPTOOL_PORT`` is typically ``/dev/ttyUSB0`` or similar.
* ``ESPTOOL_BINDIR=./`` is the path of the externally-built 2nd stage bootloader and the partition table (if applicable): when built using the ``make bootloader``, these files are placed into ``nuttx`` folder.
* ``ESPTOOL_BAUD`` is able to change the flash baud rate if desired.

Flashing NSH Example
--------------------

This example shows how to build and flash the ``nsh`` defconfig for the ESP32-S3-DevKitC-1 board::

    $ cd nuttx
    $ make distclean
    $ ./tools/configure.sh esp32s3-devkit:nsh
    $ make -j$(nproc)

When the build is complete, the firmware can be flashed to the board using the command::

    $ make -j$(nproc) flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

where ``<port>`` is the serial port where the board is connected::

  $ make flash ESPTOOL_PORT=/dev/ttyUSB0 ESPTOOL_BINDIR=./
  CP: nuttx.hex
  MKIMAGE: ESP32-S3 binary
  esptool.py -c esp32s3 elf2image --ram-only-header -fs 4MB -fm dio -ff 40m -o nuttx.bin nuttx
  esptool.py v4.8.1
  Creating esp32s3 image...
  Image has only RAM segments visible. ROM segments are hidden and SHA256 digest is not appended.
  Merged 1 ELF section
  Successfully created esp32s3 image.
  Generated: nuttx.bin
  esptool.py -c esp32s3 -p /dev/ttyUSB0 -b 921600  write_flash -fs detect -fm dio -ff 40m 0x0000 nuttx.bin
  esptool.py v4.8.1
  Serial port /dev/ttyUSB0
  Connecting....
  Chip is ESP32-S3 (QFN56) (revision v0.1)
  [...]
  Flash will be erased from 0x00000000 to 0x00032fff...
  Flash params set to 0x0230
  Compressed 206776 bytes to 74469...
  Wrote 206776 bytes (74469 compressed) at 0x00000000 in 2.7 seconds (effective 620.3 kbit/s)...
  Hash of data verified.

  Leaving...
  Hard resetting via RTS pin...

Now opening the serial port with a terminal emulator should show the NuttX console::

  $ picocom -b 115200 /dev/ttyUSB0
  NuttShell (NSH) NuttX-12.8.0
  nsh> uname -a
  NuttX 12.8.0 759d37b97c-dirty Mar  5 2025 20:23:46 xtensa esp32s3-devkit

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

  openocd -s <tcl_scripts_path> -c 'set ESP_RTOS hwthread' -f board/esp32s3-builtin.cfg -c 'init; reset halt; esp appimage_offset 0x0'

.. note::
  - ``appimage_offset`` should be set to ``0x0`` when ``Simple Boot`` is used. For MCUboot, this value should be set to
    ``CONFIG_ESP32S3_OTA_PRIMARY_SLOT_OFFSET`` value (``0x10000`` by default).
  - ``-s <tcl_scripts_path>`` defines the path to the OpenOCD scripts. Usually set to `tcl` if running openocd from its source directory.
    It can be omitted if `openocd-esp32` were installed in the system with `sudo make install`.

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
ADC          Yes   Oneshot
AES          Yes
Bluetooth    Yes
Camera       No
CAN/TWAI     Yes
DMA          Yes
eFuse        Yes
GPIO         Yes   Dedicated GPIO supported
I2C          Yes   Master and Slave mode supported
I2S          Yes
LCD          No
LED/PWM      Yes
MCPWM        Yes
Pulse_CNT    Yes
RMT          Yes
RNG          Yes
RSA          No
RTC          Yes
SDIO         No
SD/MMC       Yes
SHA          Yes
SPI          Yes
SPIFLASH     Yes
SPIRAM       Yes
Timers       Yes
Touch        Yes
UART         Yes
USB OTG      Yes   CDC/ACM console supported
USB SERIAL   Yes
Watchdog     Yes
Wi-Fi        Yes   WPA3-SAE supported
========== ======= =====

.. _esp32s3_peripheral_support:

Analog-to-digital converter (ADC)
---------------------------------

Two ADC units are available for the ESP32-S3, each with 10 channels.

Those units are independent and can be used simultaneously. During bringup, GPIOs for selected channels are
configured automatically to be used as ADC inputs.
If available, ADC calibration is automatically applied (see
`this page <https://docs.espressif.com/projects/esp-idf/en/v5.1/esp32s3/api-reference/peripherals/adc_calibration.html>`__ for more details).
Otherwise, a simple conversion is applied based on the attenuation and resolution.

Each ADC unit is accessible using the ADC character driver, which returns data for the enabled channels.

The ADC unit can be enabled in the menu :menuselection:`System Type --> ESP32-S3 Peripheral Selection --> Analog-to-digital converter (ADC)`.

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

.. _MCUBoot and OTA Update S3:

MCUBoot and OTA Update
======================

The ESP32-S3 supports over-the-air (OTA) updates using MCUBoot.

Read more about the MCUBoot for Espressif devices `here <https://docs.mcuboot.com/readme-espressif.html>`__.

Executing OTA Update
--------------------

This section describes how to execute OTA update using MCUBoot.

1. First build the default ``mcuboot_update_agent`` config. This image defaults to the primary slot and already comes with Wi-Fi settings enabled::

    ./tools/configure.sh esp32s3-devkit:mcuboot_update_agent

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

Flash Allocation for MCUBoot
----------------------------

When MCUBoot is enabled on ESP32-S3, the flash memory is organized as follows
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
     - 0x000000
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

    0x000000  ┌─────────────────────────────┐
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

Wi-Fi
=====

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

Power Management
================

.. tip:: Boards usually expose a pm defconfig which enables power management
  features. On ESP32-S3, different low power modes can be used to reduce power
  consumption depending on the application.

When using this board configuration profile, two wakeup sources are available:

- Timer (mandatory) : Every time the board enters sleep mode, a timer is started. Once the defined time is reached, the board wakes up.
- EXT1 (optional): The board wakes up whenever the selected EXT1 GPIO is asserted to the configured level.

PSRAM
-----

The external PSRAM is supported in ESP32-S3. The PSRAM is mapped to the data bus during
the boot process. The PSRAM is used as a heap memory and is available for the application.

Please check the following examples for more information:

* :ref:`esp32s3-devkit:psram_octal <platforms/xtensa/esp32s3/boards/esp32s3-devkit/index:psram_octal>`
* :ref:`esp32s3-devkit:psram_quad <platforms/xtensa/esp32s3/boards/esp32s3-devkit/index:psram_quad>`

Moving not initialized data to the external PSRAM
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Static or global not-initialized data can be moved to the external PSRAM. Usually allocated at the
``.bss`` memory segment, this data can be set to another section in the external PSRAM.
Set the attribute ``__attribute__ ((section (".ext_ram.bss")))`` to the variable. For example::

  __attribute__ ((section (".ext_ram.bss"))) static uint8_t my_data[1024];

``my_data`` will be allocated in the external PSRAM and can be explicitly initialized on runtime.

This is particularly useful when the internal RAM is not enough to hold all the data.

.. _esp32s3_ulp:

ULP RISC-V Coprocessor
======================

The ULP RISC-V core is a 32-bit coprocessor integrated into the ESP32-S3 SoC.
It is designed to run independently of the main high-performance (HP) core and is capable of executing lightweight tasks
such as GPIO polling, simple peripheral control and I/O interactions.

This coprocessor benefits to offload simple tasks from HP core (e.g., GPIO polling , I2C operations, basic control logic) and
frees the main CPU for higher-level processing

For more information about ULP RISC-V Coprocessor `check here <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/ulp-risc-v.html>`__.

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

Both methods requires `CONFIG_ESP32S3_ULP_COPROC_ENABLED` and `CONFIG_ESP32S3_ULP_COPROC_RESERVE_MEM` variables to set ULP RISC-V core and
`CONFIG_ESPRESSIF_ULP_RISCV_PROJECT_PATH` variable to set the path to the ULP project or prebuilt binary file
relative to NuttX root folder.
These variables can be set using `make menuconfig` or `kconfig-tweak` commands.

Here is an example for enabling ULP and using a prebuilt binary for ULP RISC-V core::

    make distclean
    ./tools/configure.sh esp32s3-devkit:nsh
    kconfig-tweak -e CONFIG_ESP32S3_ULP_COPROC_ENABLED
    kconfig-tweak --set-val CONFIG_ESP32S3_ULP_COPROC_RESERVE_MEM 8176
    kconfig-tweak --set-str CONFIG_ESPRESSIF_ULP_RISCV_PROJECT_PATH "Documentation/platforms/xtensa/esp32s3/boards/esp32s3-devkit/ulp_riscv_blink.bin"
    make olddefconfig
    make -j

Creating an ULP RISC-V Coprocessor Application
----------------------------------------------

To use NuttX's internal build system to compile the bare-metal ULP RISC-V Coprocessor binary, check the following instructions.

First, create a folder for the ULP source and header files. This folder is just for ULP project and it is
an independent project. Therefore, the NuttX example guide should not be followed, and no Makefile or similar
build files should be added. Also folder location could be anywhere. To include ULP folder into build
system don't forget to set `CONFIG_ESPRESSIF_ULP_RISCV_PROJECT_PATH` variable with path of the ULP project folder relative to
NuttX root folder. Instructions for setting up can be found above.

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

To sum up, here is an complete example. `ulp_example/ulp (../ulp_example/ulp)` folder selected as example
to create a subfolder for ULP but folder that includes ULP source code can be anywhere:

- Tree view:

.. code-block:: text

   nuttxspace/
   ├── nuttx/
   └── apps/
   └── ulp_example/
       └── ulp/
           └── ulp_main.c

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
    ./tools/configure.sh esp32s3-devkitc:nsh
    kconfig-tweak -e CONFIG_ESP32S3_ULP_COPROC_ENABLED
    kconfig-tweak --set-val CONFIG_ESP32S3_ULP_COPROC_RESERVE_MEM 8176
    kconfig-tweak -e CONFIG_DEV_GPIO
    kconfig-tweak --set-str CONFIG_ESPRESSIF_ULP_RISCV_PROJECT_PATH "../ulp_example/ulp"
    make olddefconfig
    make -j

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
