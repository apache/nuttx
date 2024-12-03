================
MPS3 AN547 Board
================

The MPS3 AN547 board configuration uses QEMU to emulate a generic ARM v8-M
series hardware platform and provides support for the following devices:

 - ARM Generic Timer
 - CMSDK UART controller

Getting Started
===============

Configuring and Running (Single Core)
-------------------------------------

1. Configuring NuttX and Compiling::

     ./tools/configure.sh -l mps3-an547:nsh
     make

2. Running with QEMU::

     $ qemu-system-arm -M mps3-an547 -nographic -kernel nuttx.bin

3. Pic ostest::

     ./tools/configure.sh mps3-an547:picostest
     make -j20
     genromfs -f romfs.img -d ../apps/bin/
     qemu-system-arm -M mps3-an547 -m 2G -nographic \
     -kernel nuttx.bin -gdb tcp::1127 \
     -device loader,file=romfs.img,addr=0x60000000
     nsh> /pic/hello
     nsh> /pic/ostest

4. Pic bootloader boot to ap, and run ostest::

     ./tools/configure.sh mps3-an547:ap
     make -j20
     mkdir -p pic
     arm-none-eabi-strip --remove-section=.rel.text --remove-section=.comment --strip-unneeded nuttx -o pic/boot
     genromfs -a -f 128 ../romfs.img -d pic
     make distclean -j20
     ./tools/configure.sh mps3-an547:bl
     make -j20
     qemu-system-arm -M mps3-an547 -m 2G -nographic \
     -kernel nuttx.bin -gdb tcp::1127 \
     -device loader,file=../romfs.img,addr=0x60000000
     bl> boot /pic/boot
     ap> ostest

Precautions
===========

In the new version of QEMU (9.20), the UART RX interrupt and TX interrupt have been swapped.
Adjustments need to be made using menuconfig::

    CONFIG_CMSDK_UART0_TX_IRQ=50
    CONFIG_CMSDK_UART0_RX_IRQ=49

For details, see `fix RX/TX interrupts order <https://github.com/qemu/qemu/commit/5a558be93ad628e5bed6e0ee062870f49251725c>`_

Debugging with QEMU
===================

The NuttX ELF image can be debugged using QEMU.

1. Enable Debug Symbols.
   Ensure the following change is applied to ``defconfig``::

     CONFIG_DEBUG_SYMBOLS=y

2. Run QEMU::

     qemu-system-arm -M mps3-an547 -nographic -kernel nuttx.bin -S -s

3. Run GDB with TUI::

     arm-none-eabi-gdb -tui --eval-command='target remote localhost:1234' nuttx
