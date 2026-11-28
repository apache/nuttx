================
SpacemiT K1 RCPU
================

.. tags:: arch:risc-v, chip:spacemit-k1x-rcpu, vendor:spacemit

The SpaceMiT K1 pairs eight X60 application cores with a Nuclei N308 real-time
core.  This port targets the real-time core.

The core does not boot on its own: the application processor releases it and
loads its firmware, the way a host starts a remote processor.  The vendor
firmware is ESOS, an RT-Thread based system, and it is delivered the same way.
NuttX can be delivered that way too, or loaded by a debugger during development.

Address space
=============

The K1 user manual, section 6.3 *Real-Time CPU Domain Address Mapping*, gives
the core its own address space.  Two entries matter:

=================== ======================= ==========
Region              Address range           Size
=================== ======================= ==========
SRAM                ``0x0000_0000``         256 KB
DDR                 ``0x3000_0000``         256 MB
=================== ======================= ==========

The SRAM is private to the core; the DDR entry is a window whose contents are
described by the ``DDR_REMAP_BASE`` register (manual section 14.3.2.6)::

   core address = 0x30000000 + (ddr address - ddr_remap_base)

The application processor programs ``ddr_remap_base``; the K1 device tree sets
it to ``0x100000``.

What the firmware may use
=========================

The window is the hardware view.  What the firmware may use is decided by
software: the K1 device tree lists the memory reserved for this core, and the
kernel registers those regions so that the loader can translate into them.
Five megabytes are reserved in total:

===================== ========= ==========================
Region                Size      Used by the vendor for
===================== ========= ==========================
``rcpu_mem_0``        2 MB      firmware image
``rcpu_mem_heap``     2 MB      heap
``vdev0buffer``       984 KB    rpmsg buffers
``vdev0vring0/1``     12 KB ea  rpmsg
``rsc_table``         16 KB     resource table
===================== ========= ==========================

The split is a convention rather than a hardware rule.  These are ordinary
memory regions, and the loader only translates addresses that fall inside one of
them, so the rest of the window cannot be reached.

The SRAM cannot be used for the firmware image because the application processor
cannot reach it.  It remains available to a firmware that relocates code or data
into it once running.

Resource table
==============

Linux refuses a firmware whose ELF has no ``.resource_table`` section, which the
loader looks up by name.  Its address comes from the linker script and only has
to fall inside one of the reserved regions; this port uses ``0x302fc000``, the
same place the vendor firmware keeps it.

The table shipped here is the bare header with no entries, which is enough to
boot.

Interrupts
==========

The core reports the base of its ECLIC and machine timer through the Nuclei
IRegion rather than at fixed addresses, so the driver reads them back at run
time from ``CSR_MCFG_INFO`` and ``CSR_MIRGB_INFO``, falling back to the Nuclei
defaults only if no IRegion is reported.  On K1 the ECLIC is at ``0xe0020000``
and the timer at ``0xe0030000``.

NuttX numbers interrupts as ``RISCV_IRQ_ASYNC`` plus the ECLIC source id; the
ids come from the map the vendor RTOS ships in
``libcpu/risc-v/spacemit/n308/riscv-clic.h``.  The console UART is source 63, so
it is NuttX interrupt 79.

Console
=======

The console is the core's own UART at ``0xc0881000``, which the manual
(section 16.3) describes as a 16550A compatible port with PXA extensions.  The
generic 16550 driver is used with ``CONFIG_16550_REGWIDTH=32``, the default
``CONFIG_16550_REGINCR=1`` and ``CONFIG_16550_SET_MCR_OUT2``; the functional
clock has to be brought out of reset before anything is transmitted.

Toolchain
=========

Any RISC-V GCC with an ``rv32imafc`` / ``ilp32f`` multilib works;
``riscv-none-elf-gcc`` from xPack is what the NuttX CI uses.

Supported boards
================

.. toctree::
   :glob:

   boards/*/index.rst
