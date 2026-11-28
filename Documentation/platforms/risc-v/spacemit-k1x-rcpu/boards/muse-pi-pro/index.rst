========================
SpacemiT MUSE Pi Pro
========================

.. tags:: arch:risc-v, chip:spacemit-k1x-rcpu, vendor:spacemit

The MUSE Pi Pro is a SpaceMiT K1 board: eight X60 application cores running
Linux next to a Nuclei N308 real-time core.  This documentation covers running
NuttX on the real-time core.

The real-time core has no boot media of its own.  Its firmware is delivered by
the application processor at power on, or loaded by a debugger during
development.  The console comes out of the core's own UART.

Features
========

- SpaceMiT K1 SoC; the core is described in the chip documentation
- Console UART on GPIO47 (TX) and GPIO48 (RX), 3.3 V
- JTAG for the real-time core, through the SoC debug transport

Serial Console
==============

The console is the real-time core's UART at ``0xc0881000``, brought out on
GPIO47 and GPIO48, at 115200 8N1 with no flow control.  It appears on the host
as a USB serial adapter.

This is not the application processor's console: that is a different UART on a
different adapter, so two serial adapters are expected on this board.

Building
========

::

   ./tools/configure.sh muse-pi-pro:nsh
   make

Loading the firmware
====================

The image is linked into the ``rcpu_mem_0`` region at ``0x30300000``.

With a debugger
---------------

This is the development path.  The application processor has to have booted at
least once, because it is what programs the DDR window translation the image is
linked against.

Start OpenOCD on the real-time core::

   openocd -f interface/jlink.cfg -c "transport select jtag" \
           -f target/riscv.cfg \
           -c "set _TARGETNAME k1.rcpu" \
           -c "riscv create k1.rcpu 0" \
           -c "k1.rcpu configure -event examine-end { riscv set_ir idle }" \
           -c "init" -c "targets k1.rcpu"

and load the image with GDB::

   riscv64-unknown-elf-gdb nuttx
   (gdb) set architecture riscv:rv32
   (gdb) target remote 127.0.0.1:1024
   (gdb) monitor halt
   (gdb) load
   (gdb) set $pc = 0x30300000
   (gdb) monitor resume
   (gdb) detach
   (gdb) quit

Do not use OpenOCD's ``reset`` on this board: it resets the whole SoC without
bringing the application processor back up, and recovering needs a power cycle.

Through remoteproc
------------------

This is the delivery path: the application processor loads the firmware at power
on, with no debugger and no manual step.

Two changes to the SpaceMiT Buildroot SDK are needed and are kept in the
``readme`` directory of this board port: a device tree change that names this
firmware and points the entry at the address it is linked at, and a change so
the firmware is copied into the initramfs the kernel reads it from.  The
firmware itself is copied into
``buildroot-ext/board/spacemit/k1/target_overlay/lib/firmware/`` before the SDK
is built.

The shell then comes up on the real-time core's UART::

   NuttShell (NSH) NuttX-13.1.1-RC0
   nsh>

Supported features
==================

- interrupt driven console and NuttX shell
- system tick from the core private timer
- ECLIC interrupts
- heap in DDR
- procfs

Not supported yet: rpmsg to the application processor, and suspend.
