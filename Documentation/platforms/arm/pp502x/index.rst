===================
PortalPlayer PP502x
===================

.. tags:: arch:arm, chip:pp502x

The PortalPlayer PP502x family has two ARM7TDMI cores, a 128 KiB internal SRAM,
an SDRAM interface and the peripherals of a portable media player. The port
runs the main core (CPU) from SDRAM and keeps the second core (COP) asleep:
the ARM7TDMI support of NuttX is for one core.
The PP5022C of the iPod Video 5.5G is the part that the port was developed on.

Supported Features
==================

* Both banks of the interrupt controller, with the high bank cascaded through
  bit 30 of the normal bank
* System timer on Timer 1, and ``up_udelay()`` on the free-running microsecond
  counter
* The 16550 compatible UART 0 through the generic 16550 driver, as console
* Core clock: the 24 MHz crystal, or the 80 MHz PLL when
  ``CONFIG_PP502X_PLL_BOOST`` is set (the default)
* Idle by stopping the core until an interrupt
* The CPU cache, turned on during start-up
* The size of the SDRAM (32 MiB or 64 MiB), probed during start-up

Not Supported
=============

GPIO, I2C, DMA, I2S, the ATA controller, USB, the click wheel interface, the
power management unit, the piezo speaker and the display have no driver. The
register maps of the blocks that the port does not use are not defined.

The common ARM7TDMI code selects ``CONFIG_ARCH_DCACHE`` and
``CONFIG_ARCH_ICACHE``. Their operations in ``arm_cache.S`` use CP15
instructions, which an ARM7TDMI does not have. The ``nsh`` and ``ostest``
configurations do not link them. A configuration that reaches them, such as
the ELF loader, ``mmap`` or reset on assert, is not supported.

The port builds with Make and with CMake.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
