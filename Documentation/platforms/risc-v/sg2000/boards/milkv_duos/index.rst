============
Milk-V Duo S
============

`Milk-V Duo S <https://milkv.io/duo-s>`_ is a RISC-V Single-Board Computer
based on the SOPHGO SG2000 RISC-V SoC with T-Head C906 64-bit Main Processor,
512 MB of SIP DRAM memory and 100 Mbps Ethernet.

Features
========

- **System on Chip:** SOPHGO SG2000
    - **CPU:** 
        - 64-bit T-Head C906 1.0 GHz (Main Processor)
        - 64-bit T-Head C906 700 MHz (Co-Processor)
        - 64-bit Arm Cortex-A53 1.0 GHz (Alternate Main Processor)

- **MCU:** 8051 with 6 KB SRAM
- **Memory:** SIP DRAM 512 MB
- **TPU:** 0.5 TOPS INT8
- **Storage:** 1 x microSD Connector, 1 x eMMC Pad
- **USB:** 1 x Type-C for Power and Data or 1 x USB 2.0 A Port Host
- **MIPI CSI:** 1 x 16P FPC Connector (MIPI CSI 2-Lane), 1 x 15P FPC Connector (MIPI CSI 2-Lane)	
- **Sensor Support:** 5M @ 30 FPS
- **MIPI DSI:** Via GPIO Header (MIPI DSI 4-Lane)	
- **Ethernet:** 100 Mbps Ethernet Port (RJ45)
- **Wireless:** Optional Wi-Fi 6 / Bluetooth 5
- **Audio:** Via GPIO Pin	
- **GPIO:** Up to 39 x GPIO Pin (via 2 x 26-Pin GPIO Header)
- **Others:** 1 x Boot Switch, 1 x Recovery Key, 1 x Reset Key

Serial Console
==============

A **USB Serial Adapter** is required to run NuttX on Milk-V Duo S,
**CP2102** is recommended. CH340 might not work correctly with Duo S.

Connect the USB Serial Adapter to Duo S Serial Console at:

========== ================
USB Serial Milk-V Duo S Pin
========== ================
GND        Pin 6 (GND)
RX         Pin 8 (XGPIOA 16 / UART0 TX)
TX         Pin 10 (XGPIOA 17 / UART0 RX)
========== ================

On the USB Serial Adapter, set the **Voltage Level** to 3V3.

Connect Duo S to our computer with the USB Serial Adapter.
On our computer, start a Serial Terminal and connect to the USB Serial Port
at **115.2 kbps**:

.. code:: console

   $ screen /dev/ttyUSB0 115200

NuttX will appear in the Serial Console when it boots on Duo S.

RISC-V Toolchain
================

Before building NuttX for Milk-V Duo S, download the toolchain for
`xPack GNU RISC-V Embedded GCC (riscv-none-elf) <https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases>`_.

Add the downloaded toolchain ``xpack-riscv-none-elf-gcc-.../bin``
to the ``PATH`` Environment Variable.

Check the RISC-V Toolchain:

.. code:: console

   $ riscv-none-elf-gcc -v

Building
========

To build NuttX for Milk-V Duo S, :doc:`install the prerequisites </quickstart/install>` and
:doc:`clone the git repositories </quickstart/install>` for ``nuttx`` and ``apps``.

Configure the NuttX project and build the project:

.. code:: console

   $ cd nuttx
   $ tools/configure.sh milkv_duos:nsh
   $ make

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
   $ cat nuttx.bin /tmp/nuttx.pad initrd >Image-sg2000

The NuttX Image ``Image-sg2000`` will be copied to the TFTP Server in the next step.

Booting
=======

NuttX requires a microSD Card with U-Boot Bootloader. Prepare a
`Linux microSD Card <https://lupyuen.github.io/articles/sg2000#download-the-linux-microsd>`_
for Duo S.

To boot NuttX on Milk-V Duo S, flip the `Main Processor Switch <https://lupyuen.github.io/articles/sg2000#boot-without-microsd>`_
to **RV** (RISC-V).
On our computer, `install the TFTP Server <https://lupyuen.github.io/articles/sg2000#boot-nuttx-over-tftp>`_.

Copy the file ``Image-sg2000`` from the previous section to the TFTP Server,
together with the Device Tree:

.. code:: console

   $ wget https://github.com/lupyuen2/wip-nuttx/releases/download/sg2000-1/cv181x_milkv_duos_sd.dtb
   $ scp Image-sg2000 \
      tftpserver:/tftpfolder/Image-sg2000
   $ scp cv181x_milkv_duos_sd.dtb \
      tftpserver:/tftpfolder/cv181x_milkv_duos_sd.dtb

Check that Duo S is connected to our computer via a USB Serial Adapter at 115.2 kbps:

.. code:: console

   $ screen /dev/ttyUSB0 115200

Insert the microSD Card into Duo S, connect the Ethernet Port and power up via the USB-C Port.

When Duo S boots, press Enter to see the U-Boot Prompt.
Run these commands to `boot NuttX over TFTP <https://lupyuen.github.io/articles/sg2000#boot-nuttx-over-tftp>`_:

.. code:: console

   # Change to your TFTP Server
   $ setenv tftp_server 192.168.x.x
   $ saveenv
   $ dhcp ${kernel_addr_r} ${tftp_server}:Image-sg2000
   $ tftpboot ${fdt_addr_r} ${tftp_server}:cv181x_milkv_duos_sd.dtb
   $ fdt addr ${fdt_addr_r}
   $ booti ${kernel_addr_r} - ${fdt_addr_r}

Or configure U-Boot to `boot NuttX automatically <https://lupyuen.github.io/articles/sg2000#boot-nuttx-over-tftp>`_.

NuttX boots on Duo S and NuttShell (nsh) appears in the Serial Console.
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
Serial Console is enabled on UART0 at 115.2 kbps.

Peripheral Support
==================

NuttX for Milk-V Duo S supports these peripherals:

======================== ======= =====
Peripheral               Support NOTES
======================== ======= =====
UART                     Yes
======================== ======= =====
