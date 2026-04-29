=========
qemu-i486
=========

Port of NuttX to QEMU in i486 mode.  This port will also run on real i486
hardware (Google the Bifferboard).

QEMU
====

QEMU is a generic and open source machine emulator and virtualizer.

QEMU Installation
-----------------

You can install QEMU for x86 on Linux Ubuntu this way::

   $ sudo apt install qemu-system-x86

Executing QEMU
--------------

Running QEMU with the generated ``nuttx.elf`` file::

  $ qemu-system-i386 -cpu 486 -m 2 -kernel nuttx.elf -nographic

Configuration
=============

Use the default configuration process::

   $ ./tools/configure.sh qemu-i486:<configname>

These are the available configurations:

nsh
---

Enables the NuttShell for i486 emulated on QEMU.
After running the QEMU command above with nuttx.elf you will see::

   SeaBIOS (version 1.16.3-debian-1.16.3-2)

   iPXE (https://ipxe.org) 00:03.0 CA00 PCI2.10 PnP PMM+0018AED0+00000000 CA00

   Booting from ROM..
   NuttShell (NSH) NuttX-12.13.0
   nsh>

