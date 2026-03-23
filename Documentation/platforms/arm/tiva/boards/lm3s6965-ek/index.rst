===========
lm3s6965-ek
===========

The `Stellaris LM3S6965 Evaluation Kit
<https://www.ti.com/tool/EK-LM3S6965>`_ is an ARM Cortex-M3 based
development board featuring the LM3S6965 microcontroller with an
integrated 10/100 Ethernet controller. NuttX can run on this board
both on real hardware and under QEMU emulation
(``qemu-system-arm -M lm3s6965evb``).

Features of the LM3S6965 Evaluation Kit:

* LM3S6965 microcontroller (ARM Cortex-M3, 50 MHz)
* 256 KB flash, 64 KB SRAM
* Integrated 10/100 Ethernet MAC and PHY
* OLED graphics display (128 x 96 pixel)
* MicroSD card slot
* USB interface for debugging and power supply
* Standard ARM 20-pin JTAG debug connector

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected
as follows::

    $ ./tools/configure.sh lm3s6965-ek:<subdir>

Where ``<subdir>`` is one of the following:

discover
--------

A network discovery configuration::

    $ ./tools/configure.sh -l lm3s6965-ek:discover
    $ make

nsh
---

Configures the NuttShell (NSH) with serial and telnet interfaces::

    $ ./tools/configure.sh -l lm3s6965-ek:nsh
    $ make

nx
--

An NX graphics configuration::

    $ ./tools/configure.sh -l lm3s6965-ek:nx
    $ make

qemu-flat
---------

A FLAT memory model configuration for running under QEMU::

    $ ./tools/configure.sh -l lm3s6965-ek:qemu-flat
    $ make

qemu-protected
--------------

A PROTECTED memory model configuration for running under QEMU::

    $ ./tools/configure.sh -l lm3s6965-ek:qemu-protected
    $ make

qemu-kostest
-------------

A PROTECTED memory model configuration with kernel ostest for running
under QEMU::

    $ ./tools/configure.sh -l lm3s6965-ek:qemu-kostest
    $ make

qemu-nxflat
------------

A FLAT memory model configuration with NXFLAT binary support for running
under QEMU::

    $ ./tools/configure.sh -l lm3s6965-ek:qemu-nxflat
    $ make

tcpecho
-------

A TCP echo server configuration::

    $ ./tools/configure.sh -l lm3s6965-ek:tcpecho
    $ make

Running with QEMU
=================

FLAT build (qemu-flat)
----------------------

::

    $ qemu-system-arm -M lm3s6965evb -nographic \
        -kernel nuttx.bin

PROTECTED build (qemu-protected, qemu-kostest)
-----------------------------------------------

The PROTECTED build produces separate kernel and user binaries.
Use ``-device loader`` to load the user binary at its link address::

    $ qemu-system-arm -M lm3s6965evb -nographic \
        -kernel nuttx.bin \
        -device loader,file=nuttx_user.bin,addr=0x20000

With networking (qemu-flat)
---------------------------

The ``lm3s6965evb`` machine includes a built-in Stellaris Ethernet NIC
(``stellaris_enet``). To enable user-mode networking with the host,
add the ``-nic user`` option::

    $ qemu-system-arm -M lm3s6965evb -nographic \
        -kernel nuttx.bin -nic user

With networking (qemu-protected)
---------------------------------

For the PROTECTED build, also load the user binary::

    $ qemu-system-arm -M lm3s6965evb -nographic \
        -kernel nuttx.bin \
        -device loader,file=nuttx_user.bin,addr=0x20000 \
        -nic user

To exit QEMU, press ``Ctrl-A`` then ``X``.

Debugging with QEMU
====================

1. To debug the NuttX ELF with symbols, ensure that the following
   configuration option is enabled in your defconfig (it is already
   set in the ``lm3s6965-ek:qemu-protected`` defconfig)::

     CONFIG_DEBUG_SYMBOLS=y

2. Run QEMU at shell terminal 1 with GDB server enabled::

     $ qemu-system-arm -M lm3s6965evb -nographic \
         -kernel nuttx.bin -S -s

   For a PROTECTED build, also load the userspace binary::

     $ qemu-system-arm -M lm3s6965evb -nographic \
         -kernel nuttx.bin \
         -device loader,file=nuttx_user.bin,addr=0x20000 -S -s

3. Run GDB with TUI, connect to QEMU, load NuttX and continue
   (at shell terminal 2)::

     $ arm-none-eabi-gdb -tui --eval-command='target remote localhost:1234' nuttx
     (gdb) c
     Continuing.
     ^C
     Program received signal SIGINT, Interrupt.
     (gdb)
