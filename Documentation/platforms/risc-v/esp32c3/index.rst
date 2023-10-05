==================
Espressif ESP32-C3
==================

The ESP32-C3 is an ultra-low-power and highly integrated SoC with a RISC-V
core and supports 2.4 GHz Wi-Fi and Bluetooth Low Energy.

* Address Space
  - 800 KB of internal memory address space accessed from the instruction bus
  - 560 KB of internal memory address space accessed from the data bus
  - 1016 KB of peripheral address space
  - 8 MB of external memory virtual address space accessed from the instruction bus
  - 8 MB of external memory virtual address space accessed from the data bus
  - 480 KB of internal DMA address space
* Internal Memory
  - 384 KB ROM
  - 400 KB SRAM (16 KB can be configured as Cache)
  - 8 KB of SRAM in RTC
* External Memory
  - Up to 16 MB of external flash
* Peripherals
  - 35 peripherals
* GDMA
  - 7 modules are capable of DMA operations.

ESP32-C3 Toolchain
==================

A generic RISC-V toolchain can be used to build ESP32-C3 projects. It's recommended to use the same
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
   curl -s -L "https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v12.3.0-1/xpack-riscv-none-elf-gcc-12.3.0-1-linux-x64.tar.gz" \
   | tar -C riscv-none-elf-gcc --strip-components 1 -xz

It uses the xPack's prebuilt toolchain based on GCC 12.3.0.

Installing
----------

First, create a directory to hold the toolchain:

.. code-block:: console

   $ mkdir -p /path/to/your/toolchain/riscv-none-elf-gcc

Download and extract toolchain:

.. code-block:: console

   $ curl -s -L "https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v12.3.0-1/xpack-riscv-none-elf-gcc-12.3.0-1-linux-x64.tar.gz" \
   | tar -C /path/to/your/toolchain/riscv-none-elf-gcc --strip-components 1 -xz

Add the toolchain to your `PATH`:

.. code-block:: console

   $ echo "export PATH=/path/to/your/toolchain/riscv-none-elf-gcc/bin:$PATH" >> ~/.bashrc

You can edit your shell's rc files if you don't use bash.

Second stage bootloader and partition table
===========================================

The NuttX port for now relies on IDF's second stage bootloader to carry on some hardware
initializations.  The binaries for the bootloader and the partition table can be found in
this repository: https://github.com/espressif/esp-nuttx-bootloader
That repository contains a dummy IDF project that's used to build the bootloader and
partition table, these are then presented as Github assets and can be downloaded
from: https://github.com/espressif/esp-nuttx-bootloader/releases
Download ``bootloader-esp32c3.bin`` and ``partition-table-esp32c3.bin`` and place them
in a folder, the path to this folder will be used later to program them. This
can be: ``../esp-bins``

Building and flashing
=====================

First make sure that ``esptool.py`` is installed.  This tool is used to convert
the ELF to a compatible ESP32 image and to flash the image into the board.
It can be installed with: ``pip install esptool``.

Configure the NuttX project: ``./tools/configure.sh esp32c3-devkit:nsh``
Run ``make`` to build the project.  Note that the conversion mentioned above is
included in the build process.
The ``esptool.py`` command to flash all the binaries is::

     esptool.py --chip esp32c3 --port /dev/ttyUSBXX --baud 921600 write_flash 0x0 bootloader.bin 0x8000 partition-table.bin 0x10000 nuttx.bin

However, this is also included in the build process and we can build and flash with::

   make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=../esp-bins

Where ``<port>`` is typically ``/dev/ttyUSB0`` or similar and ``../esp-bins`` is
the path to the folder containing the bootloader and the partition table
for the ESP32-C3 as explained above.
Note that this step is required only one time.  Once the bootloader and partition
table are flashed, we don't need to flash them again.  So subsequent builds
would just require: ``make flash ESPTOOL_PORT=/dev/ttyUSBXX``

Debugging with OpenOCD
======================

Download and build OpenOCD from Espressif, that can be found in
https://github.com/espressif/openocd-esp32

If you have an ESP32-C3 ECO3, no external JTAG is required to debug, the ESP32-C3
integrates a USB-to-JTAG adapter.

OpenOCD can then be used::

   openocd -c 'set ESP_RTOS none' -f board/esp32c3-builtin.cfg

For versions prior to ESP32-C3 ECO3, an external JTAG adapter is needed.
It can be connected as follows::

  TMS -> GPIO4
  TDI -> GPIO5
  TCK -> GPIO6
  TDO -> GPIO7

Furthermore, an efuse needs to be burnt to be able to debug::

  espefuse.py -p <port> burn_efuse DIS_USB_JTAG

OpenOCD can then be used::

  openocd  -c 'set ESP_RTOS none' -f board/esp32c3-ftdi.cfg

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX:

=========== ======= =====
Peripheral  Support NOTES
=========== ======= =====
ADC          Yes
AES          Yes
Bluetooth    Yes
CDC Console  Yes    Rev.3
DMA          Yes
eFuse        Yes
GPIO         Yes
I2C          Yes
LED_PWM      Yes
RNG          Yes
RSA          Yes
RTC          Yes
SHA          Yes
SPI          Yes
SPIFLASH     Yes
Timers       Yes
Touch        Yes
UART         Yes
Watchdog     Yes
Wifi         Yes
=========== ======= =====

Secure Boot and Flash Encryption
================================

Secure Boot
-----------

Secure Boot protects a device from running any unauthorized (i.e., unsigned) code by checking that
each piece of software that is being booted is signed. On an ESP32-C3, these pieces of software include
the second stage bootloader and each application binary. Note that the first stage bootloader does not
require signing as it is ROM code thus cannot be changed. This is achieved using specific hardware in
conjunction with MCUboot (read more about MCUboot `here <https://docs.mcuboot.com/>`__).

The Secure Boot process on the ESP32-C3 involves the following steps performed:

1. The first stage bootloader verifies the second stage bootloader's RSA-PSS signature. If the verification is successful,
   the first stage bootloader loads and executes the second stage bootloader.

2. When the second stage bootloader loads a particular application image, the application's signature (RSA, ECDSA or ED25519) is verified
   by MCUboot.
   If the verification is successful, the application image is executed.

.. warning:: Once enabled, Secure Boot will not boot a modified bootloader. The bootloader will only boot an
   application firmware image if it has a verified digital signature. There are implications for reflashing
   updated images once Secure Boot is enabled. You can find more information about the ESP32-C3's Secure boot
   `here <https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/security/secure-boot-v2.html>`__.

.. note:: As the bootloader image is built on top of the Hardware Abstraction Layer component
   of `ESP-IDF <https://github.com/espressif/esp-idf>`_, the
   `API port by Espressif <https://docs.mcuboot.com/readme-espressif.html>`_ will be used
   by MCUboot rather than the original NuttX port.

Flash Encryption
----------------

Flash encryption is intended for encrypting the contents of the ESP32-C3's off-chip flash memory. Once this feature is enabled,
firmware is flashed as plaintext, and then the data is encrypted in place on the first boot. As a result, physical readout
of flash will not be sufficient to recover most flash contents.

.. warning::  After enabling Flash Encryption, an encryption key is generated internally by the device and
   cannot be accessed by the user for re-encrypting data and re-flashing the system, hence it will be permanently encrypted.
   Re-flashing an encrypted system is complicated and not always possible. You can find more information about the ESP32-C3's Flash Encryption
   `here <https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/security/flash-encryption.html>`__.

Prerequisites
-------------

First of all, we need to install ``imgtool`` (a MCUboot utility application to manipulate binary
images) and ``esptool`` (the ESP32-C3 toolkit)::

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


Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
