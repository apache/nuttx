=============
CanMV K230
=============

The `CanMV K230 <https://developer.canaan-creative.com/k230/dev/zh/CanMV_K230_%E6%95%99%E7%A8%8B.html>`_ is a raspberry-pi sized single board computer with 512MB DRAM and a microSD card slot for booting. It comes with serial console, Ethernet, HDMI and USB/OTG ports. Unfortuunately it doesn't support JTAG alike debugging interfaces.

The `K230 SDK <https://github.com/kendryte/k230_sdk>`_ contains source code, libraries and user guides for booting up an AMP enviroment with Linux on CPU0 and RT-Thread on CPU1. 

K230 boots from CPU0 and loads U-Boot SPL into DRAM first, then U-Boot kicks off OpenSBI wrapped Linux/RTT OS images on respective CPU cores accordingly.

The K230 U-Boot kicks off firmwares in machine mode, thus it allows flat, protected or kernel
NuttX `build modes <https://nuttx.apache.org/docs/latest/implementation/processes_vs_tasks.html>`_. The kernel build mode further works with OpenSBI or a builtin minimal SBI layer.

Preparations
============

Take the prebuilt CanMV-k230 boot image from `here <https://gitee.com/yf1972/filexfers/tree/canmv230-tools-for-nuttx-v1.2>`_ as the default K230 SDK doesn't support RiscV standard PTE format at least till v1.2. The package also contains an extract of the OpenSBI from K230 SDK v1.2 release, which is needed to wrap the `canmv230/knsh` kernel build. The K230 SBI extract is
also available at `this Github repository <https://github.com/yf13/k230osbi>`_, it will updated
over the time to match updates at NuttX repository.

Make sure that before trying NuttX:

- The board can boot with prebuilt CanMV-k230 image.
- Device console access available (e.g. ``minicom -D /dev/ttyACM0``).
- U-Boot connectivity to TFTP service available.

For below NuttX tests, the microSD card is only used to enter the U-Boot console environment, as NuttX isn't using any storage yet.

Toolchains
==========

To build NuttX, we can use the stock  **gcc-riscv64-unknown-elf** toolchain on Ubuntu, or download the RISC-V Toolchain riscv64-unknown-elf from `XPack <https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack>`_.


Building
========


To build NuttX for CanMV-k230, :doc:`install the prerequisites </quickstart/install>` and :doc:`clone the git repositories </quickstart/install>` for ``nuttx`` and ``apps``.

FLAT Build
----------

FLAT build is straightforward:

.. code:: console

   $ cd nuttx
   $ make distclean && tools/configure.sh canmv230:nsh
   $ make -j4

The generated `nuttx.bin` can then be tried on the target.

PROTECTED Build
---------------

PROTECTED build can be done like below:

.. code:: console

   $ cd nuttx
   $ make distclean && tools/configure.sh canmv230:pnsh
   $ make -j4

There will be `nuttx.bin` and `nuttx_user.bin` generated. We need pad `nuttx.bin` to so that to fill memory gap till user space flash start then combine it with `nuttx_user.bin` to form the final binary for run on the target. Say the gap between uflash and kflash is 256KB in `scripts/ld-protected.script`, we can pad-combine them like below:

.. code:: console

   $ dd if=/dev/zero of=/tmp/padded bs=1024 count=256
   $ dd if=nuttx.bin of=/tmp/padded conv=notrunc
   $ cat /tmp/padded nuttx_user.bin > /tftp-folder/nuttx.bin

The combined `nuttx.bin` in TFTP service folder can then be tried on target.

KERNEL Build
------------

KERNEL build requires two build passes:

- First pass to build kernel and export package so that to build apps as ROMFS.
- Second pass to build the kernel with real ROMFS image containing the apps.

There are two configurations for KERNEL build mode:

- The ``canmv230/knsh`` is for use within standard SBI environment.
- The ``canmv230/nsbi`` uses a built-in minimal SBI environment.

The ``canmv230/nsbi`` has smaller footprint and is simpler to use, the ``canmv230/knsh`` is more tedious to build and is for situatinos with standard SBI environment.

Take the following steps to build the kernel export package:

.. code:: console

   $ # first pass to build kernel exports
   $ cd nuttx
   $ make distclean && tools/configure.sh canmv230:knsh
   $ make -j4
   $ make export # build nuttx-export-*.gz package

With export package, we can then build the apps and ROMFS:

.. code:: console

   $ cd apps
   $ # import the nuttx-export-*.gz package from kernel
   $ tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.gz
   $ make import)  # build the apps
   $ # generate ROMFS image for contents in apps/bin folder
   $ tools/mkromfsimg.sh ../nuttx/arch/risc-v/src/board/romfs_boot.c

Once ROMFS for apps is ready, build the kernel again:

.. code:: console

   $ cd nuttx
   $ make -j4    # build kernel again with real ROMFS

The ``nuttx.bin`` is the artifact of kernel build. For ``canmv230/nsbi`` case, simply copy it to the TFTP folder then run on the target.

For ``canmv230/knsh`` case, take additional steps to wrap the artifact with the  OpenSBI extract from the K230 SDK downloaded above:

.. code:: console

   $ cd $HOME
   $ # unpack the K230 OpenSBI extract
   $ tar xvf canmv230-opensbi-dtb.tar.xz
   $ export OSBI=$HOME/opensbi
   $ cd /tmp/aaa    # use a temporary folder
   $ make -C $OSBI O=$(pwd) PLATFORM=generic \
          CROSS_COMPILE=riscv64-unknown-elf- FW_PIC=n K230_LIITLE_CORE=1 \
          FW_FDT_PATH=$OSBI/k230.dtb FW_PAYLOAD_PATH=nuttx.bin -j4
   $ cp platform/generic/firmware/fw_payload.bin tftp-server-path/nuttx.bin

Please use actual paths on your host for ``nuttx.bin`` and TFTP folder when running above commands.

This Github `repository <https://github.com/yf13/k230osbi>`_ contains latest version of the K230 OpenSBI extract.


Running
=======

Within U-boot console, load ``nuttx.bin`` from TFTP and run it as shown below:

.. code:: console

   k230# usb start
   k230# ping $serverip
   k230# tftp 8000000 nuttx.bin
   k230# go 8000000

Then the ``nsh`` console should appear, type ``help`` to see available commands.


Asymmetric Multi Processing
===========================

We can do Asymmetric Multi Processing on K230 using the little core as master and the big core as remote.

Take the ``canmv230/master`` and ``canmv230/remote`` configurations to build the master and remote NuttX images respectively. They are both kernel builds derived from ``canmv230/nsbi`` mentioned above, so we can follow above kernel mode build steps to build them.

Let's put the NuttX image files as ``master.bin`` and ``remote.bin`` respectively on the TFTP folder. To run them on K230 device, do the following from U-Boot console:


.. code:: console

   k230# usb start
   k230# ping $serverip
   k230# tftp 7000000 remote.bin
   k230# tftp 8000000 master.bin
   k230# go 8000000

Then we should see the "master> " prompt, this is the master console. where we can further run the ``cu`` command and press Return key to see the remote console, within remote console type ``~.`` to get back to the master console. 

There is a `session log <https://github.com/apache/nuttx/pull/11673>`_ showing how to enter remote node and check file system status then get back and check master file system status.


Issues
======

- The ``ostest`` app has non-zero exit code in Kernel mode.

