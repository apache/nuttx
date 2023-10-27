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
(or any time you which to modify either of these). An easy way is to use prebuilt binaries for NuttX `from here <https://github.com/espressif/esp-nuttx-bootloader>`__. In there you will find instructions to rebuild these if necessary.
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
ADC          No
AES          Yes
Bluetooth    Yes
CAN/TWAI     Yes
DMA          Yes
eFuse        Yes
Ethernet     Yes
GPIO         Yes
I2C          Yes
I2S          Yes
LED_PWM      Yes
MCPWM        No
Pulse_CNT    No
RMT          Yes
RNG          Yes
RSA          No
RTC          Yes
SD/MMC       No
SDIO         No
SHA          No
SPI          Yes
SPIFLASH     Yes
SPIRAM       Yes
Timers       Yes
Touch        Yes
UART         Yes
Watchdog     Yes
Wifi         Yes
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

Using QEMU
==========

First follow the instructions `here <https://github.com/espressif/qemu/wiki>`__ to build QEMU.

Enable the ``ESP32_QEMU_IMAGE`` config found in :menuselection:`Board Selection --> ESP32 binary image for QEMU`.

Download the bootloader and the partition table from https://github.com/espressif/esp-nuttx-bootloader/releases
and place them in a directory, say ``../esp-bins``.

Build and generate the QEMU image::

 $ make ESPTOOL_BINDIR=../esp-bins

A QEMU-compatible ``nuttx.merged.bin`` binary image will be created. It can be run as::

 $ qemu-system-xtensa -nographic -machine esp32 -drive file=nuttx.merged.bin,if=mtd,format=raw

Secure Boot and Flash Encryption
================================

Secure Boot
-----------

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

.. note:: As the bootloader image is built on top of the Hardware Abstraction Layer component
   of `ESP-IDF <https://github.com/espressif/esp-idf>`_, the
   `API port by Espressif <https://docs.mcuboot.com/readme-espressif.html>`_ will be used
   by MCUboot rather than the original NuttX port.

Flash Encryption
----------------

Flash encryption is intended for encrypting the contents of the ESP32's off-chip flash memory. Once this feature is enabled,
firmware is flashed as plaintext, and then the data is encrypted in place on the first boot. As a result, physical readout
of flash will not be sufficient to recover most flash contents.

.. warning::  After enabling Flash Encryption, an encryption key is generated internally by the device and
   cannot be accessed by the user for re-encrypting data and re-flashing the system, hence it will be permanently encrypted.
   Re-flashing an encrypted system is complicated and not always possible. You can find more information about the ESP32's Flash Encryption
   `here <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/security/flash-encryption.html>`__.

Prerequisites
-------------

First of all, we need to install ``imgtool`` (a MCUboot utility application to manipulate binary
images) and ``esptool`` (the ESP32 toolkit)::

    $ pip install imgtool esptool

We also need to make sure that the python modules are added to ``PATH``::

    $ echo "PATH=$PATH:/home/$USER/.local/bin" >> ~/.bashrc

Now, we will create a folder to store the generated keys (such as ``~/signing_keys``)::

    $ mkdir ~/signing_keys && cd ~/signing_keys

With all set up, we can now generate keys to sign the bootloader and application binary images,
respectively, of the compiled project::

    $ espsecure.py generate_signing_key --version 2 bootloader_signing_key.pem
    $ imgtool keygen --key app_signing_key.pem --type rsa-3072

.. important:: The contents of the key files must be stored securely and kept secret.

Enabling Secure Boot and Flash Encryption
-----------------------------------------

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
`examples of MCUboot applied in Nuttx applications <https://github.com/apache/nuttx-apps/tree/master/examples/mcuboot>`_.

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
