README.txt
==========

This board configuration will use QEMU to emulate generic ARM v7-A series
hardware platform and provides support for these devices:

 - GICv2 interrupt controllers
 - ARM Generic Timer
 - PL011 UART controller

Contents
========
  - Getting Started
  - Status
  - Platform Features
  - Debugging with QEMU
  - FPU Support and Performance
  - SMP Support
  - References

Getting Started
===============

1. Configuring and running
  1.1 Single Core
   Configuring NuttX and compile:
   $ ./tools/configure.sh -l qemu-armv7a:nsh
   $ make
   Running with qemu
   $ qemu-system-arm -cpu cortex-a7 -nographic \
     -machine virt,virtualization=off,gic-version=2 \
     -net none -chardev stdio,id=con,mux=on -serial chardev:con \
     -mon chardev=con,mode=readline -kernel ./nuttx

Debugging with QEMU
===================

The nuttx ELF image can be debugged with QEMU.

1. To debug the nuttx (ELF) with symbols, make sure the following change have
   applied to defconfig.

+CONFIG_DEBUG_SYMBOLS=y

2. Run QEMU(at shell terminal 1)

   Single Core
   $ qemu-system-arm -cpu cortex-a7 -nographic \
     -machine virt,virtualization=off,gic-version=2 \
     -net none -chardev stdio,id=con,mux=on -serial chardev:con \
     -mon chardev=con,mode=readline -kernel ./nuttx -S -s

3. Run gdb with TUI, connect to QEMU, load nuttx and continue (at shell terminal 2)

   $ arm-none-eabi-gdb -tui --eval-command='target remote localhost:1234' nuttx
   (gdb) c
   Continuing.
   ^C
   Program received signal SIGINT, Interrupt.
   nx_start () at armv7-a/arm_head.S:209
   (gdb)

