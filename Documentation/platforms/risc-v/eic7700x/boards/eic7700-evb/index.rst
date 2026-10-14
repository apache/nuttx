=================
ESWIN EIC7700 EVB
=================

.. tags:: arch:risc-v, chip:eic7700x, vendor:eswin

.. figure:: eic7700-evb.jpg
   :align: center
   :alt: The ESWIN EIC7700 EVB, a development board carrying the EIC7700X SoC

   ESWIN EIC7700 EVB

The `EIC7700 EVB <https://www.eswincomputing.com/en/products/index/36.html>`_
is ESWIN's own evaluation board for the
:doc:`EIC7700X <../../index>` SoC.  Where the
:doc:`PINE64 StarPro64 <../starpro64/index>` is a single board computer built
around the chip, the EVB brings out most of the SoC's interfaces, so it is the
board to develop drivers on.

Features
========

* ESWIN EIC7700X, 4 x RV64GC 1.4 GHz RISC-V cores
* 16 GB LPDDR5
* eMMC, microSD and SPI NOR flash, the last holding the boot firmware
* 2 x Ethernet (GMAC, RGMII)
* 2 x USB 3.0, host and device capable
* HDMI output, with CEC
* PCIe 3.0 slot, and two M.2 slots, for SATA and for a WiFi and Bluetooth
  module
* USB serial console, RS232 on a DB9, and further UARTs on the headers
* Expansion headers carrying GPIO, I2C, SPI and PWM
* PWM fan header

.. warning::

   This port is under development and drives a subset of the board.
   `Peripheral Support`_ below records what it drives.

Serial Console
==============

The console is UART0 at **115200 8N1**.  It is wired to the on-board FT4232
USB bridge, so a single USB cable carries it and no separate USB serial
adapter is needed.  The bridge presents four ports, of which UART0 is the
third; on Linux that is usually ``/dev/ttyUSB2``, the first of the four
being the JTAG interface:

.. code:: console

   $ screen /dev/ttyUSB2 115200

Buttons and LEDs
================

The board has four LEDs on GPIO lines 107 to 110 and one push button, ``OK``,
on GPIO line 6.  NuttX does not drive any of them yet.

Power Supply
============

The board is powered through its barrel jack.  The core rails, including the
NPU rail, are set by regulators on I2C bus 1, which NuttX leaves alone: the
firmware has already configured them by the time NuttX starts, and writing to
them changes a core voltage.

RISC-V Toolchain
================

Install `xPack GNU RISC-V Embedded GCC (riscv-none-elf)
<https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases>`_ and
add its ``bin`` directory to ``PATH``, as described for the
:doc:`PINE64 StarPro64 <../starpro64/index>`.

Building NuttX
==============

Configure and build:

.. code:: console

   $ cd nuttx
   $ tools/configure.sh eic7700-evb:nsh
   $ make

Then build the applications filesystem and package it with the kernel:

.. code:: console

   $ make export
   $ pushd ../apps
   $ tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.tar.gz
   $ make import
   $ popd
   $ boards/risc-v/eic7700x/common/tools/mkimage.sh

The image is the kernel, then padding, then a RAM disk holding the
applications.  Use the script rather than padding by hand: the RAM disk is
found at run time by searching memory for its header, and that search runs
after BSS has been cleared, so the disk has to start above ``_ebss`` or it is
zeroed before anything looks for it.  The script reads ``_ebss`` from the
kernel and pads to suit.

The result is ``Image-eic7700-evb``.

Booting NuttX
=============

The board boots over TFTP from U-Boot, as the
:doc:`PINE64 StarPro64 <../starpro64/index>` does.  Copy
``Image-eic7700-evb`` and the device tree to the TFTP server:

.. code:: console

   $ wget https://github.com/lupyuen/nuttx-starpro64/raw/refs/heads/main/eic7700-evb.dtb
   $ scp Image-eic7700-evb eic7700-evb.dtb tftpserver:/tftpfolder/

Interrupt U-Boot with Ctrl-C at power on and boot the image:

.. code:: console

   # Change to your TFTP Server
   $ setenv tftp_server 192.168.x.x
   $ saveenv
   $ dhcp ${kernel_addr_r} ${tftp_server}:Image-eic7700-evb
   $ tftpboot ${fdt_addr_r} ${tftp_server}:eic7700-evb.dtb
   $ fdt addr ${fdt_addr_r}
   $ booti ${kernel_addr_r} - ${fdt_addr_r}

NuttShell appears on the console.

Configurations
==============

.. code:: console

   $ tools/configure.sh eic7700-evb:<config-name>

nsh
---

NuttShell on UART0 at 115200 8N1, with the RAM disk mounted and ``/proc``
available.  Built-in applications are supported; none are enabled.

Peripheral Support
==================

NuttX for the EIC7700 EVB supports these peripherals:

======================== ======= =====
Peripheral               Support NOTES
======================== ======= =====
UART                     Yes
CPU clock control        Yes     400 MHz to 1.4 GHz, measured not assumed
Watchdog                 Yes     Four, timeout resets the chip
======================== ======= =====

Watchdog
========

Four Synopsys watchdogs whose timeout genuinely resets the chip, registered
as ``/dev/watchdog0`` to ``3``.  With the auto-monitor configured, both
boards ship it on, the kernel arms every one at boot and feeds them from
a kernel timer, so a hang anywhere becomes a reboot in about eleven
seconds.  An application may take one over at any time with ``WDIOC_START``
(the kernel stops feeding that one permanently), poke one with
``WDIOC_KEEPALIVE``, or disarm one by writing ``V`` before closing.

Timeouts are powers of two of the 200 MHz peripheral clock: a third of a
millisecond up to 10.74 seconds, always rounded up, and requests beyond
the ceiling are refused with an error rather than quietly shortened.
The auto-monitor plans its feeding schedule around the number it asked
for, and a shorter dog under a longer schedule dies on time, every time.
The configured 8 second timeout is therefore granted as 10.74 seconds,
fed every 4.

Why the board last reset is printed at every boot and served through
``BOARDIOC_RESET_CAUSE``::

   wdt: last reset: WATCHDOG (08)

**Panic policy is a configuration choice.**  Without
``CONFIG_WATCHDOG_PANIC_NOTIFIER``, the shipped default, a kernel
panic starves the dogs and the board reboots itself, cause recorded.
With it, the kernel stops all four at the moment of panic and the crash
scene keeps forever for a debugger; the stop path is deliberately free
of locks and allocation so it works from a dying kernel.

**For JTAG sessions** two rescues exist, both one line.  Freeze every
dog from the debugger (a halted hart cannot feed them, and a halt longer
than the timeout otherwise reboots the board under the session)::

   mww 0x51828444 0x0

and release them again by writing ``0xf``.  The same register is how the
driver itself stops a dog, since the enable bit is unclearable by
design: held in block reset is the off state.

Two hardware notes learned on silicon rather than from the manual.  The
timeout-range register must be written and read back: the manual's own
advice to clear the protection-level register first makes the block
silently drop timeout writes while still accepting the enable, arming a
third-of-a-millisecond watchdog nothing can outrun.  And all four
instances' reset outputs genuinely reach the chip reset: starving any
one of them reboots the machine.
