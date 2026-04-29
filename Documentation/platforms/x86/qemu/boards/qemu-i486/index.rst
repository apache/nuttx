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

vga_fb
------

This board profile will initialize the QEMU i486 with VGA framebuffer support::

   $ qemu-system-i386 -cpu 486 -m 1024 -kernel nuttx.elf -vga std -serial stdio

   NuttShell (NSH) NuttX-12.13.0
   nsh> fb
   fb_ioctl: cmd: 10241 arg: 1589592
   VideoInfo:
         fmt: 6
        xres: 320
        yres: 240
     nplanes: 1
    fb_ioctl: cmd: 10242 arg: 1589600
   PlaneInfo (plane 0):
       fbmem: 0x170650
       fblen: 76800
      stride: 320
     display: 0
         bpp: 8
   Mapped FB: 0x170650
    0: (  0,  0) (320,240)
    1: ( 29, 21) (262,198)
    2: ( 58, 42) (204,156)
    3: ( 87, 63) (146,114)
    4: (116, 84) ( 88, 72)
    5: (145,105) ( 30, 30)
   Test finished
   nsh>
