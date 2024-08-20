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

2. knsh
------
  This is a configuration of testing the BUILD_KERNEL configuration
  $ cd nuttx
  $ ./tools/configure.sh qemu-armv7a:knsh
  $ make V=1 -j7
  $ make export V=1
  $ cd ../apps
  $ ./tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.tar.gz
  $ make import V=1
  $ cd ../nuttx
  $ qemu-system-arm -semihosting -M virt -m 1024 -nographic -kernel ./nuttx

  NuttShell (NSH) NuttX-12.3.0-RC0
  nsh> uname -a
  NuttX 12.3.0-RC0 28dee592a3-dirty Oct 12 2023 03:03:07 arm qemu-armv7a
  nsh> ps
    PID GROUP PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK           STACK   USED  FILLED COMMAND
      0     0   0 FIFO     Kthread N-- Ready              0000000000000000 004088 000896  21.9%  Idle_Task
      1     1 100 RR       Kthread --- Waiting  Semaphore 0000000000000000 004040 000304   7.5%  lpwork 0x40119398 0x401193ac
      2     2 100 RR       Task    --- Running            0000000000000000 003032 001032  34.0%  /system/bin/init
  nsh> free
                     total       used       free    largest  nused  nfree
          Kmem:  133058556      16644  133041912  133041152     41      3
          Page:  134217728    1105920  133111808  133111808
  nsh> /system/bin/hello
  Hello, World!!
  nsh>

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

