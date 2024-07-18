===========
PINE64 Ox64
===========

`Ox64 <https://wiki.pine64.org/wiki/Ox64>`_ is a RISC-V Single-Board Computer
based on the Bouffalo Lab BL808 RISC-V SoC with C906 64-bit CPU Core and
E907 / E902 32-bit CPU Cores supported by 64 MB of embedded PSRAM memory,
with built-in WiFi, Bluetooth and Zigbee radio interfaces.

Ox64 comes in a breadboard-friendly form factor. It has a microSD Card slot,
USB 2.0 Type-C port and other peripheral interfaces.

Features
========

- **System on Chip:** Bouffalo Lab BL808
    - **CPU:** 
        - 64-bit T-Head C906 "D0" (RV64IMAFCV)
        - 32-bit T-Head E907 "M0" (RV32IMAFCP)
        - 32-bit T-Head E902 "LP" (RV32E[M]C)
    - **RAM:** Embedded 64 MB PSRAM
- **Wireless:** 2.4 GHz 1T1R WiFi 802.11 b/g/n, Bluetooth 5.2, Zigbee
- **Ethernet:** 10 / 100 Mbps (optional)
- **Storage:** On-board 128 Mbit (16 MB) XSPI NOR Flash Memory, microSD (SDHC and SDXC)
- **USB:** USB 2.0 OTG
- **Audio:** Microphone and Speaker (optional)
- **Video Input:** Dual-lane MIPI CSI port for Camera Module (USB-C)
- **Expansion Ports:** 26 GPIO pins (including SPI, I2C and UART)

Serial Console
==============

A **USB Serial Adapter** that supports 2 Mbps (like `CH340G Serial Adapter <https://lupyuen.github.io/articles/ox64#test-the-usb-serial-adapter>`_)
is required to run NuttX on Ox64.

Connect the USB Serial Adapter to Ox64 Serial Console at:

========== ========
USB Serial Ox64 Pin
========== ========
TX         Pin 31 (GPIO 17 / UART3 RX)
RX         Pin 32 (GPIO 16 / UART3 TX)
GND        Pin 33 (GND)
========== ========

On the USB Serial Adapter, set the **Voltage Level** to 3V3.

Connect Ox64 to our computer with the USB Serial Adapter.
On our computer, start a Serial Terminal and connect to the USB Serial Port
at **2 Mbps**.

NuttX will appear in the Serial Console when it boots on Ox64.

RISC-V Toolchain
================

Before building NuttX for Ox64, download the **RISC-V Toolchain riscv64-unknown-elf**
from `SiFive RISC-V Tools <https://github.com/sifive/freedom-tools/releases/tag/v2020.12.0>`_.

Add the downloaded toolchain ``riscv64-unknown-elf-toolchain-.../bin``
to the ``PATH`` Environment Variable.

Check the RISC-V Toolchain:

.. code:: console

   $ riscv64-unknown-elf-gcc -v

Building
========

To build NuttX for Ox64, :doc:`install the prerequisites </quickstart/install>` and
:doc:`clone the git repositories </quickstart/install>` for ``nuttx`` and ``apps``.

Configure the NuttX project and build the project:

.. code:: console

   $ cd nuttx
   $ tools/configure.sh ox64:nsh
   $ make
   $ riscv64-unknown-elf-objcopy -O binary nuttx nuttx.bin

This produces the NuttX Kernel ``nuttx.bin``.  Next, build the NuttX Apps Filesystem:

.. code:: console

   $ make export
   $ pushd ../apps
   $ tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.tar.gz
   $ make import
   $ popd
   $ genromfs -f initrd -d ../apps/bin -V "NuttXBootVol"

This generates the Initial RAM Disk ``initrd``.

Package the NuttX Kernel and Initial RAM Disk into a NuttX Image:

.. code:: console

   $ head -c 65536 /dev/zero >/tmp/nuttx.pad
   $ cat nuttx.bin /tmp/nuttx.pad initrd >Image

The NuttX Image ``Image`` will be copied to a microSD Card in the next step.

Booting
=======

To boot NuttX on Ox64, flash
`OpenSBI and U-Boot Bootloader <https://lupyuen.github.io/articles/ox64>`_ to Ox64.

NuttX boots on Star64 via a microSD Card. Prepare a
`Linux microSD Card <https://lupyuen.github.io/articles/ox64>`_ for Ox64.

Copy the file ``Image`` from the previous section
and overwrite the file on the microSD Card.

Check that Ox64 is connected to our computer via a USB Serial Adapter at 2 Mbps.

Insert the microSD Card into Ox64 and power up Ox64 via the Micro USB Port.
NuttX boots on Ox64 and NuttShell (nsh) appears in the Serial Console.

To see the available commands in NuttShell:

.. code:: console

   $ help

Configurations
==============

nsh
---

Basic configuration that runs NuttShell (nsh).
This configuration is focused on low level, command-line driver testing.
Built-in applications are supported, but none are enabled.
Serial Console is enabled on UART3 at 2 Mbps.

adc
---

This configuration enables support for the general purpose ADC and the adc example app.
By default, the ADC will scan external channels 3, 4, 6, 7 and 9 (GPIO pins 11, 6, 12,
13 and 18). Serial Console is enabled on UART3 at 2 Mbps.

spi
---

This configuration enables support for SPI0 and spitool.
By default, GPIO14 is MISO, 13 is MOSI, 15 is SCLK and 12 is SS.
Serial Console is enabled on UART3 at 2 Mbps.

timer
-----

This configuration enables support for hardware timers and the timer example app.
Serial Console is enabled on UART3 at 2 Mbps.
