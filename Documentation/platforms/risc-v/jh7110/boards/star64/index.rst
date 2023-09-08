=============
PINE64 Star64
=============

`Star64 <https://wiki.pine64.org/wiki/STAR64>`_ is a 64-bit RISC-V based
Single Board Computer powered by StarFive JH7110 Quad-Core SiFive U74 64-Bit CPU,
Imagination Technology BX-4-32 GPU and supports up to 8GB 1866MHz LPDDR4 memory.

It provides an eMMC module socket, MicroSD Card slot, PCI-e, Pi-2 Bus, USB 3.0
and many other peripheral interfaces for makers to integrate with sensors
and other devices.

Features
========

- **System on Chip:** StarFive JH7110
    - **CPU:** SiFive RISC-V U74 Application Cores (4 cores, RV64GCB) and SiFive RISC-V S7 Monitor Core (single core, RV64IMACB)
    - **GPU:** Imagination Technology BXE-4-32
    - **RAM:** LPDDR4 2GB / 4GB / 8GB
- **Video:** Digital Video Output up to 4K @ 30 Hz, 4K HDR @ 60 fps
- **Audio:** 3.5mm Audio Jack
- **Ethernet:** Single or Dual 10 / 100 / 1000Mbps
- **Wireless:** 2.4 GHz / 5 Ghz MIMO WiFi 802.11 b/g/n/ac with Bluetooth 5.2 (Realtek RTL8852BU)
- **Storage:** 128 Mbit (16 MByte) XSPI NOR flash Memory, Bootable microSD (SDHC and SDXC up to 256 GB), Bootable eMMC
- **USB:** 1 x USB 3.0 Dedicated Host Port, 3 x USB 2.0 Host Ports
- **Expansion Ports:** PCIe 2.0 x 1 lane, 2 x 20 pins "Pi2" GPIO Header
- **MIPI DSI Port:** 4-lane MIPI DSI port for LCD Panel
- **MIPI CSI Port:** 4-lane MIPI CSI port for Camera Module

Serial Console
==============

A **USB Serial Adapter** (like `CH340G Serial Adapter <https://pine64.com/product/serial-console-woodpecker-edition/>`_)
is required to run NuttX on Star64.

Connect the USB Serial Adapter to Star64's **GPIO Header** at:

========== ===========
USB Serial GPIO Header
========== ===========
GND        Pin 6 (GND)
RX         Pin 8 (UART0 TX)
TX         Pin 10 (UART0 RX)
========== ===========

On the USB Serial Adapter, set the **Voltage Level** to 3V3.

Connect Star64 to our computer with the USB Serial Adapter.
On our computer, start a Serial Terminal and connect to the USB Serial Port
at **115.2 kbps**.

NuttX will appear in the Serial Console when it boots on Star64.

RISC-V Toolchain
================

Before building NuttX for Star64, download the **RISC-V Toolchain riscv64-unknown-elf**
from `SiFive RISC-V Tools <https://github.com/sifive/freedom-tools/releases/tag/v2020.12.0>`_.

Add the downloaded toolchain ``riscv64-unknown-elf-toolchain-.../bin``
to the ``PATH`` Environment Variable.

Check the RISC-V Toolchain:

.. code:: console

   $ riscv64-unknown-elf-gcc -v

Building
========

To build NuttX for Star64, :doc:`install the prerequisites </quickstart/install>` and
:doc:`clone the git repositories </quickstart/install>` for ``nuttx`` and ``apps``.

Configure the NuttX project and build the project:

.. code:: console

   $ cd nuttx
   $ tools/configure.sh star64:nsh
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

Download the `Device Tree jh7110-visionfive-v2.dtb <https://github.com/starfive-tech/VisionFive2/releases/download/VF2_v3.1.5/jh7110-visionfive-v2.dtb>`_
from `StarFive VisionFive2 Software Releases <https://github.com/starfive-tech/VisionFive2/releases>`_
into the ``nuttx`` folder.

Inside the ``nuttx`` folder, create a Text File named ``nuttx.its``
with the following content:

::

   /dts-v1/;

   / {
     description = "NuttX FIT image";
     #address-cells = <2>;

     images {
       vmlinux {
         description = "vmlinux";
         data = /incbin/("./nuttx.bin");
         type = "kernel";
         arch = "riscv";
         os = "linux";
         load = <0x0 0x40200000>;
         entry = <0x0 0x40200000>;
         compression = "none";
       };

       ramdisk {
         description = "buildroot initramfs";
         data = /incbin/("./initrd");
         type = "ramdisk";
         arch = "riscv";
         os = "linux";
         load = <0x0 0x46100000>;
         compression = "none";
         hash-1 {
           algo = "sha256";
         };
       };

       fdt {
         data = /incbin/("./jh7110-visionfive-v2.dtb");
         type = "flat_dt";
         arch = "riscv";
         load = <0x0 0x46000000>;
         compression = "none";
         hash-1 {
           algo = "sha256";
         };
       };
     };

     configurations {
       default = "nuttx";

       nuttx {
         description = "NuttX";
         kernel = "vmlinux";
         fdt = "fdt";
         loadables = "ramdisk";
       };
     };
   };

Package the NuttX Kernel, Initial RAM Disk and Device Tree into a
Flat Image Tree:

.. code:: console

   $ sudo apt install u-boot-tools
   $ mkimage -f nuttx.its -A riscv -O linux -T flat_dt starfiveu.fit

The Flat Image Tree ``starfiveu.fit`` will be copied to a microSD Card
in the next step.

Booting
=======

NuttX boots on Star64 via a microSD Card. To prepare the microSD Card, download the
`microSD Image sdcard.img <https://github.com/starfive-tech/VisionFive2/releases/download/VF2_v3.1.5/sdcard.img>`_
from `StarFive VisionFive2 Software Releases <https://github.com/starfive-tech/VisionFive2/releases>`_.

Write the downloaded image to a microSD Card with
`Balena Etcher <https://www.balena.io/etcher/>`_ or 
`GNOME Disks <https://wiki.gnome.org/Apps/Disks>`_.

Copy the file ``starfiveu.fit`` from the previous section
and overwrite the file on the microSD Card.

Check that Star64 is connected to our computer via a USB Serial Adapter.

Insert the microSD Card into Star64 and power up Star64.
NuttX boots on Star64 and NuttShell (nsh) appears in the Serial Console.

To see the available commands in NuttShell:

.. code:: console

   $ help

`Booting NuttX over TFTP <https://lupyuen.github.io/articles/tftp>`_
is also supported on Star64.

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

NuttX for PinePhone supports these peripherals:

======================== ======= =====
Peripheral               Support NOTES
======================== ======= =====
UART                     Yes
======================== ======= =====
