=================
MPS3 AN547 Board
=================

The MPS3 AN547 board configuration uses QEMU to emulate a generic ARM v8-M series hardware platform and provides support for the following devices:

 - ARM Generic Timer
 - CMSDK UART controller

Getting Started
===============

Configuring and Running
-----------------------

### Single Core

1. **Configuring NuttX and Compiling:**

   ```bash
   $ ./tools/configure.sh -l mps3-an547:nsh
   $ make
   ```

2. **Running with QEMU:**

   ```bash
   $ qemu-system-arm -M mps3-an547 -nographic -kernel nuttx.bin
   ```

Debugging with QEMU
===================

The NuttX ELF image can be debugged using QEMU.

1. **Enable Debug Symbols:**

   Ensure the following change is applied to `defconfig`:

   ```bash
   +CONFIG_DEBUG_SYMBOLS=y
   ```

2. **Run QEMU:**

   ```bash
   $ qemu-system-arm -M mps3-an547 -nographic -kernel nuttx.bin -S -s
   ```

3. **Run GDB with TUI:**

   ```bash
   $ arm-none-eabi-gdb -tui --eval-command='target remote localhost:1234' nuttx
   ```
