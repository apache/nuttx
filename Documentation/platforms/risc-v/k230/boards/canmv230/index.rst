=============
CanMV K230
=============

The `Kendryte K230 <https://www.canaan.io/product/k230>`_ SoC contains two indepedent T-Head C908 based RV64GC CPU cores. The CPU1 even has RVV1.0 vector extension and operates at higher speed. The SoC contains accelerators for depth image processing, audio processing and neural network inferencing etc.

The `CanMV K230 <https://developer.canaan-creative.com/k230/dev/zh/CanMV_K230_%E6%95%99%E7%A8%8B.html>`_ is a raspberry-pi sized single board computer with 512MB DRAM and a microSD card slot for booting. It comes with serial console, Ethernet, HDMI and USB/OTG ports. Unfortuunately it doesn't support JTAG alike debugging interfaces.

The `K230 SDK <https://github.com/kendryte/k230_sdk>`_ contains source code, libraries and user guides for booting up an AMP enviroment with Linux on CPU0 and RT-Thread on CPU1. 

K230 boots from CPU0 and loads U-Boot into DRAM first, then U-Boot kicks off OpenSBI wrapped Linux/RTT OS images on respective CPU cores accordingly.

The K230 U-Boot runs in machine mode, thus it can run flat or kernel NuttX builds. The kernel build shall run with or without SBI layer.

Preparations
============

Take the prebuilt CanMV-k230 boot image from `here <https://gitee.com/yf1972/filexfers/tree/canmv230-tools-for-nuttx-v1.2>`_ as the default K230 SDK doesn't support RiscV standard PTE format at least till v1.2. This package also contains an extract of the OpenSBI from K230 SDK v1.2 release, which is needed to wrap Nuttx kernel build binary.

Make sure that before trying NuttX:

- The board can boot with prebuilt CanMV-k230 image.
- Device console access available (e.g. `minicom -D /dev/ttyACM0`).
- U-Boot connectivity to TFTP service available.

For below NuttX tests, the microSD card is only used to enter the U-Boot console environment, as NuttX isn't using any storage yet.

Toolchains
==========

Before building NuttX, download the **RISC-V Toolchain riscv64-unknown-elf** from `XPack <https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack>`_ or get the stock "gcc-riscv64-unknown-elf" via `apt` on Ubuntu.


Building
========

To build NuttX for CanMV, :doc:`install the prerequisites </quickstart/install>` and :doc:`clone the git repositories </quickstart/install>` for ``nuttx`` and ``apps``.

Configure and build FLAT mode NuttX:

.. code:: console

   $ cd nuttx
   $ make distclean && tools/configure.sh canmv230:nsh
   $ make -j4

This should have `nuttx.bin` generated, it can be loaded by U-Boot on the board.

The NuttX KERNEL build requires two build passes: first pass to build kernel w/ dummy ROMFS and apps, second pass to build the kernel with real ROMFS image containing apps built in first pass.

.. code:: console

   $ cd nuttx
   $ make distclean && tools/configure.sh canmv230:knsh
   $ make -j4    # first pass to build kernel
   $ (make export; cd ../apps; tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.gz; make import)  # build the apps
   $ (cd ../apps/; tools/mkromfsimg.sh ../nuttx/boards/risc-v/k230/canmv230/src/romfs_boot.c) # update the ROMFS image
   $ make -j4    # build again to pick up the ROMFS

The built `nuttx.bin` can be then wrapped with K230 OpenSBI like below:

.. code:: console

   $ cd $HOME
   $ tar xvf canmv230-opensbi-dtb.tar.xz
   $ export OSBI=$HOME/opensbi
   $ cd /tmp/aaa    # use a temporary work folder
   $ make -C $OSBI O=$(pwd) PLATFORM=generic\
          CROSS_COMPILE=riscv64-unknown-elf- FW_PIC=n K230_LIITLE_CORE=1\
          FW_FDT_PATH=$OSBI/k230.dtb FW_PAYLOAD_PATH=nuttx.bin -j4
   $ cp platform/generic/firmware/fw_payload.bin tftp-server-path/nuttx.bin

Please use actual paths on your host for `nuttx.bin` and TFTP folder when running above commands.

Booting
=======

Within U-boot console, load `nuttx.bin` from TFTP service and run it:

.. code:: console

   k230# usb start
   k230# ping $serverip
   k230# tftp 8000000 nuttx.bin
   k230# go 8000000

Then the `nsh> ` console should appear, type `help` to see available commands.

Issues
======

 - The `ostest` app has non-zero exit code in Kernel build.

