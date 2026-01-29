===========
qemu-armv7r
===========

This board configuration will use QEMU to emulate generic ARM v7-R series
hardware platform and provides support for these devices:

* ARM Generic Timer
* PL011 UART controller
* VirtIO Device

.. note::
   The upstream QEMU does not support ARM Cortex-R virtual machine. This board
   uses the OpenVela QEMU fork which adds ARM Cortex-R support. You can find
   the source code and prebuilt binaries at:
   https://github.com/open-vela/platform_external_qemu/commits/trunk/

Prerequisites
=============

Download and extract the prebuilt QEMU from OpenVela. For Linux x86_64, download from:
https://github.com/open-vela/prebuilts_qemu_linux-x86_64

The QEMU binary path will be referred to as ``<QEMU_PATH>`` in the following sections::

     # Example path structure after extraction:
     # <QEMU_PATH>/bin/qemu-system-arm
     # <QEMU_PATH>/share/qemu/

.. note::
   The ``-L`` option is required because the QEMU shared resources are not
   installed to system directories. It specifies the directory for the QEMU
   BIOS and device tree files.

Getting Started
===============

NSH (Flat Build)
----------------

Configuring NuttX and compile::

     $ ./tools/configure.sh -l qemu-armv7r:nsh
     $ make -j`nproc`

Running with qemu::

     $ <QEMU_PATH>/bin/qemu-system-arm \
       -L <QEMU_PATH>/share/qemu \
       -M virt -semihosting -nographic -cpu cortex-r5f \
       -kernel ./nuttx

PNSH (Protected Build)
----------------------

This is a configuration for testing the BUILD_PROTECTED configuration
with MPU support::

  $ cd nuttx
  $ ./tools/configure.sh qemu-armv7r:pnsh
  $ make -j`nproc`

Running with qemu (note: both nuttx and nuttx_user must be loaded)::

  $ <QEMU_PATH>/bin/qemu-system-arm \
    -L <QEMU_PATH>/share/qemu \
    -M virt -semihosting -nographic -cpu cortex-r5f \
    -device loader,file=./nuttx_user \
    -device loader,file=./nuttx

Debugging with QEMU
===================

The nuttx ELF image can be debugged with QEMU.

1. To debug the nuttx (ELF) with symbols, make sure the following change have
   been applied to defconfig::

     +CONFIG_DEBUG_SYMBOLS=y

2. Run QEMU at shell terminal 1 (for NSH flat build)::

     $ <QEMU_PATH>/bin/qemu-system-arm \
       -L <QEMU_PATH>/share/qemu \
       -M virt -semihosting -nographic -cpu cortex-r5f \
       -kernel ./nuttx -S -s

   Or for PNSH protected build::

     $ <QEMU_PATH>/bin/qemu-system-arm \
       -L <QEMU_PATH>/share/qemu \
       -M virt -semihosting -nographic -cpu cortex-r5f \
       -device loader,file=./nuttx_user \
       -device loader,file=./nuttx -S -s

3. Run gdb with TUI, connect to QEMU, load nuttx and continue (at shell terminal 2)::

     $ arm-none-eabi-gdb -tui --eval-command='target remote localhost:1234' nuttx
     (gdb) c
     Continuing.
     ^C
     Program received signal SIGINT, Interrupt.
     (gdb)

   For PNSH protected build, you need to load the user space symbols as well::

     $ arm-none-eabi-gdb -tui --eval-command='target remote localhost:1234' nuttx
     (gdb) add-symbol-file nuttx_user
     (gdb) c