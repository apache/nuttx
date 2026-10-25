================
Bluetrum AB32VG1
================

.. tags:: arch:risc-v, chip:ab32vg1, vendor:bluetrum

The Bluetrum AB32VG1 (also sold as AB5301A, code name "CRWN") is a Bluetooth
audio SoC with a 32-bit RISC-V core:

* RV32IMC core, up to 120 MHz (48 MHz by default in NuttX)
* 192 KB of RAM in three blocks:

  ============  ===========  =========  ==============================
  Name          Address      Size       Use in NuttX
  ============  ===========  =========  ==============================
  SRAM          0x00010000   128 KB     image (from 0x12000), heap
  AECRAM        0x00050000   30.5 KB    libc/mm/driver code, heap
  ICRAM         0x00060000   32 KB      .bss, idle stack, heap
  ============  ===========  =========  ==============================

* 1 MB SPI NOR flash in the package (SPI0, port G)
* 3 UARTs, 6 timers, GPIO, RTC, watchdog, SD host, USB, audio codec, FM
  radio, Bluetooth

There is no public reference manual for the CPU core and interrupt
controller.  The `register manual
<https://github.com/BLUETRUM/AB32VG1_DOC>`_ covers most peripherals; the
rest of the information used by this port comes from the RT-Thread BSP
(``bsp/bluetrum``) and from experiments on the hardware, documented below.

Core and interrupts
===================

The core implements the RISC-V unprivileged ISA (RV32IMC) but **not** the
standard machine-mode trap architecture:

* There are no CSRs.  Any CSR instruction (including ``rdcycle``), ``ecall``
  or ``ebreak`` raises an exception, and after an exception the core traps
  again on every following instruction.  Exceptions are therefore fatal and
  the port never executes these instructions (images can be checked with
  ``riscv-none-elf-objdump -d nuttx | grep csr``).
* Trap state and interrupt control are memory-mapped registers: ``PICCON``
  (bit 0: interrupt enable, bit 1: controller enable), ``PICEN`` (one enable
  bit per source), ``PICPR`` (priority), ``PICADR`` (vector table base),
  ``PICPND`` (pending), ``EPC`` and ``EPICCON`` (``PICCON.IE`` saved on
  trap).
* The vector table has fixed offsets from ``PICADR``: +0x10 exceptions and
  high priority interrupts, +0x40 low priority interrupts, +0x80 XIP cache
  misses.  NuttX uses low priority interrupts only.
* ``mret`` returns to ``EPC`` and restores ``PICCON.IE`` from ``EPICCON``;
  this also works outside of a trap.  Low priority interrupts do not nest.

The chip selects ``CONFIG_ARCH_RV_CUSTOM_TRAP``, which lets a chip replace
the CSR-based parts of the common RISC-V code: ``up_irq_save()`` and
``up_irq_restore()`` operate on ``PICCON``, and system calls (used for
context switches) use a *software trap*: ``sys_callN()`` calls
``ab32vg1_syscall_trap()``, which builds the same register frame as a
hardware trap with interrupts masked, dispatches it like an ``ecall`` and
returns through the common trap exit (``EPC``/``EPICCON`` + ``mret``).

Memory and boot
===============

Code normally runs from RAM.  The mask ROM can execute from flash through a
software-managed instruction cache (that is how RT-Thread runs), but every
interrupt handler and everything it calls must then live in RAM, which does
not suit NuttX.  NuttX is therefore loaded into RAM entirely:

* by the mask ROM UART loader, for development (``ab32vg1_dl run``), or
* from flash by a small stage-1 loader started by the mask ROM
  (``ab32vg1_dl flash``).

The mask ROM copies the boot code named by the boot header of the flash
(scrambled, CRC-checked) to RAM and calls it.  The boot code must clear CPU
register 0x508 early: otherwise the mask ROM takes control back, into its
UART loader, some tens of milliseconds later.  That is also the only way
the UART loader is reached after a reset once a boot header is present, so
boot code must listen for the host before clearing 0x508 (see the board
documentation).

The ROM loader can only write SRAM up to 0x2e800 (it keeps its own data
above), so the image is split: code of the C library, memory manager and
drivers goes to AECRAM, ``.bss`` and the idle stack to ICRAM.  The heap uses
all remaining RAM (about 72 KB with the ``nsh`` configuration).

Supported Peripherals
=====================

==========  =======  =====================================================
Peripheral  Support  Notes
==========  =======  =====================================================
Clock       Yes      PLL0 (240 MHz) / n, ``CONFIG_AB32VG1_SYSCLK_MHZ``
Timer 0     Yes      System tick
UART0       Yes      Single-wire on PA7, console
GPIO        Yes      No pin interrupts yet
Watchdog    Reset    Used for ``up_systemreset()``
Flash       Tools    Programmed by ``ab32vg1_dl``, no MTD driver yet
UART1/2     No
SPI, I2C    No
RTC         No
SD          No
USB         No
Audio, BT   No
==========  =======  =====================================================

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
