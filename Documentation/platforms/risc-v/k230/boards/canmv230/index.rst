=============
CanMV K230
=============

The `Kendryte K230 <https://www.canaan.io/product/k230>`_ SoC contains two indepedent T-Head C908 based RV64GC CPU cores. The CPU1 even has RVV1.0 vector extension and operates at higher speed. The SoC contains accelerators for depth image processing, audio processing and neural network inferencing etc.

The `CanMV K230 <https://developer.canaan-creative.com/k230/dev/zh/CanMV_K230_%E6%95%99%E7%A8%8B.html>`_ is a raspberry-pi sized single board computer with 512MB DRAM and a microSD card slot for booting. It comes with serial console, Ethernet, HDMI and USB/OTG ports. Unfortuunately it doesn't support JTAG alike debugging interfaces.

The `K230 SDK <https://github.com/kendryte/k230_sdk>`_ contains source code, libraries and user guides for booting up an AMP enviroment with Linux on CPU0 and RT-Thread on CPU1. 

K230 boots from CPU0 and loads U-Boot into DRAM first, then U-Boot kicks off OpenSBI wrapped Linux/RTT OS images on respective CPU cores accordingly.

The K230 U-Boot operates in machine mode, thus provides an ideal environment for NuttX. allowing one to run flat or kernel builds in theory.


Preparations
============

Please follow the K230 SDK to prepare a booting SD card for the board, or use prebuilt boot image from `here <https://kendryte-download.canaan-creative.com/developer/k230/k230_canmv_sdcard_v1.2_nncase_v2.5.1.img.gz>`_. 

Make sure that before trying NuttX:

- The board can boot default SDK image normally.
- U-Boot console can be accessed from host(e.g. `minicom -D /dev/ttyACM0`).
- U-Boot has access to a TFTP service is available.
- You can drop files to the TFTP service folder.

Note for below NuttX tests, the SD image is only used to enter U-Boot console.

RISC-V Toolchain
================

Before building NuttX for Star64, download the **RISC-V Toolchain riscv64-unknown-elf** from `XPack <https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack>`_ or use "gcc-riscv64-unknown-elf" on Ubuntu.


Building
========

To build NuttX for CanMV, :doc:`install the prerequisites </quickstart/install>` and :doc:`clone the git repositories </quickstart/install>` for ``nuttx`` and ``apps``.

Configure the NuttX project and build the project:

.. code:: console

   $ cd nuttx
   $ tools/configure.sh canmv230:nsh
   $ make -j4

There should have `nuttx.bin` generated.

Booting
=======

Copy the `nuttx.bin` to the TFTP service folder and work with the U-Boot console:

.. code:: console

   k230# usb start
   k230# ping $serverip
   k230# tftp 8000000 nuttx.bin
   k230# boot_barememtal 0 8000000 $filesize

Then the `nsh> `console should appear, type `help` to see available commands.
