=====================
SpacemiT K1 RCPU
=====================

.. tags:: arch:risc-v, chip:spacemit-k1x-rcpu, vendor:spacemit

The SpacemiT K1 pairs eight X60 application cores with a Nuclei N308
real-time core.  This port targets the real-time core; the application cores
run Linux.

The core is not a general purpose CPU that boots on its own.  The application
processor releases it and loads its firmware, the way a host starts a remote
processor, and the vendor firmware (ESOS, an RT-Thread based system) is
delivered the same way.  NuttX can be delivered the same way, or loaded by a
debugger during development.

- Nuclei N308 RISC-V core (RV32IMAFDCP), 245 MHz
- ECLIC interrupt controller with hardware vectoring, NMI
- 64-bit Core-Private Timer
- 16 KB instruction and 16 KB data cache
- 256 KB private SRAM at address zero
- 8 PMP entries with 4 KB granularity

Memory map
==========

The private SRAM is at address zero and is fast, but the application processor
cannot reach it: it is private to the real-time core and no carveout covers
it.  Firmware therefore runs from shared DDR, which the application processor
loads through remoteproc.

============= ============ ============ =============================
RCPU address  DDR address  Size         Contents
============= ============ ============ =============================
``0x302fc000`` ``0x003fc000`` 16 KB      resource table
``0x30300000`` ``0x00400000`` 2 MB       firmware, heap
============= ============ ============ =============================

The core reaches these through a 256 MB window that starts at ``0x30000000``;
the application processor translates it to DDR with ``ddr-remap-base``, which
the K1 device tree sets to ``0x100000``.  Both regions above are carveouts
named ``rsc_table`` and ``rcpu_mem_0``.

The private SRAM is left unused, but it is a valid target for hot code and
data if a future change relocates them at start up.

Interrupts
==========

The core reports the base of its ECLIC and of the machine timer through the
Nuclei IRegion rather than at fixed addresses, so those are read back at run
time from ``CSR_MCFG_INFO`` and ``CSR_MIRGB_INFO``.  On K1 the IRegion is at
``0xe0000000``, which puts the ECLIC at ``0xe0020000`` and the timer at
``0xe0030000``.

NuttX interrupt numbers are ``RISCV_IRQ_ASYNC`` plus the ECLIC source id, so
source 63 is interrupt 79.

Console
=======

The console is the core's own UART at ``0xc0881000``, which the K1 user manual
section 16.3 describes as a 16550A compatible port with PXA extensions.  The
generic 16550 driver is used with ``CONFIG_16550_REGWIDTH=32`` and the default
``CONFIG_16550_REGINCR=1``, which together give the four byte register stride
this part uses, and with ``CONFIG_16550_SET_MCR_OUT2`` because the interrupt
output is gated by ``MCR.OUT2``.

The UART functional clock has to be enabled before anything is transmitted;
the board brings it up out of reset.

Toolchain
=========

Any RISC-V GCC with an ``rv32imafc`` / ``ilp32f`` multilib works.  The xPack
``riscv-none-elf-gcc`` used by the NuttX CI (14.2.0) provides that multilib
and is the recommended toolchain.

Loading the firmware
====================

During development the image can be loaded by a debugger, since it is linked
for the address it runs at.  A product loads it through the application
processor's remoteproc interface; the board documentation describes the vendor
side changes that requires.

Supported boards
================

.. toctree::
   :glob:

   boards/*/index.rst
