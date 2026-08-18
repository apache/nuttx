==================
Lichee RV 86 Panel
==================

The Sipeed Lichee RV 86 Panel is based on the Allwinner D1 SoC.  The board
contains a single T-Head C906 RV64 core and 512 MiB of external DDR3 memory.
This NuttX port runs in supervisor mode under OpenSBI and uses a FLAT address
space.

The initial port supports:

* UART0 as the serial console;
* the D1 PLIC;
* the native D1 Timer1 as the scheduler tick source; and
* an NSH configuration.

Serial Console
==============

UART0 is available on PB8 (TX) and PB9 (RX).  Connect a 3.3 V USB-to-serial
adapter with crossed TX and RX signals and a common ground.  Configure the
terminal for 115200 baud, 8 data bits, no parity, one stop bit and no flow
control.

Toolchain
=========

Install a ``riscv-none-elf`` bare-metal toolchain and add its ``bin``
directory to ``PATH``.  The configuration builds for RV64IMAFDC with the
LP64D ABI.

Building
========

Configure and build NuttX from the NuttX source directory:

.. code:: console

   $ tools/configure.sh lichee-rv-86-panel:nsh
   $ make

The configuration links NuttX at ``0x40200000`` and produces ``nuttx`` and
``nuttx.bin``.  It uses the RAM region from ``0x40200000`` to ``0x48000000``.

The tested U-Boot flow uses a legacy image wrapper.  Install ``mkimage`` and
create the image as follows:

.. code:: console

   $ mkimage -A riscv -O linux -T kernel -C none -a 0x40200000 \
       -e 0x40200000 -n "Apache NuttX Allwinner D1" \
       -d nuttx.bin nuttx-d1.img

Booting NuttX
=============

The board must already have a working D1 BootROM, U-Boot SPL, OpenSBI and
U-Boot boot chain.  NuttX is the supervisor-mode payload; it does not replace
SPL, OpenSBI or U-Boot.

Copy ``nuttx-d1.img`` to ``/boot`` on the existing root filesystem.  At the
U-Boot prompt, verify the MMC device and partition, then load the image to a
temporary address and boot it:

.. code:: console

   => ext4load mmc 0:1 0x48000000 /boot/nuttx-d1.img
   => bootm 0x48000000 - ${fdtcontroladdr}

U-Boot relocates the payload to its ``0x40200000`` load address.  The initial
NuttX port receives the device-tree pointer from the firmware but does not
consume the device tree.

Watchdog
========

Some U-Boot configurations leave the D1 RISC-V watchdog running with an
approximately 16-second timeout.  Early NuttX startup writes the D1 watchdog
disable key to the watchdog mode register before normal initialization, so a
manual U-Boot watchdog-disable command is not required.

Scheduler Tick
==============

The scheduler uses native D1 Timer1 as an external interrupt through the
PLIC.  Timer1 uses OSC24M with a divide-by-one prescaler in periodic mode.
With ``CONFIG_USEC_PER_TICK=1000``, the interval register is programmed with
23999.  The inclusive down-counter therefore consumes exactly 24000 input
clocks per interrupt and provides a 1000 Hz scheduler tick.

Configurations
==============

nsh
---

The ``nsh`` configuration provides an interactive NuttShell on UART0.  It is
a FLAT supervisor-mode build intended for initial command-line bring-up.
