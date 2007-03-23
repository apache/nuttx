Architecture-Specific Code
^^^^^^^^^^^^^^^^^^^^^^^^^^
Table of Contents
^^^^^^^^^^^^^^^^^

  o Architecture-Specific Code
  o Summary of Files
  o Supported Architectures
  o Configuring NuttX

Architecture-Specific Code
^^^^^^^^^^^^^^^^^^^^^^^^^^

The file include/nuttx/arch.h identifies all of the APIs that must
be provided by the architecture specific logic.  (It also includes
arch/<arch-name>/arch.h as described below).

Directory Structure
^^^^^^^^^^^^^^^^^^^

The arch directory contains architecture specific logic.  The complete
board port in is defined by the architecture-specific code in this
directory (plus the board-specific configurations in the config/
subdirectory).  Each architecture must provide a subdirectory <arch-name>
under arch/ with the following characteristics:


	<arch-name>
	|-- include
	|   |-- arch.h
	|   |-- irq.h
	|   `-- types.h
	`-- src
	    |-- Makefile
	    `-- (architecture-specific source files)

Summary of Files
^^^^^^^^^^^^^^^^

include/arch.h
  This is a hook for any architecture specific definitions that may
  be needed by the system.  It is included by include/nuttx/arch.h

include/types.h
  This provides architecture/toolchain-specific definitions for
  standard types.  This file should typedef:

    sbyte, ubyte, uint8, boolean, sint16, uint16, sint32, uint32

  and

    sint64, uint64

  if the architecture supports 64-bit integers.

    irqstate_t

  Must be defined to the be the size required to hold the interrupt
  enable/disable state.

  This file will be included by include/sys/types.h and be made
  available to all files.

include/irq.h
  This file needs to define some architecture specific functions (usually
  inline if the compiler supports inlining) and structure.  These include:

  - struct xcptcontext.  This structures represents the saved context
    of a thread.

  - irqstate_t irqsave(void) -- Used to disable all interrupts.

  - void irqrestore(irqstate_t flags) -- Used to restore interrupt
    enables to the same state as before irqsave was called.

  This file must also define NR_IRQS, the total number of IRQs supported
  by the board.

src/Makefile
  This makefile will be executed to build the targets src/libup.a and
  src/up_head.o.  The up_head.o file holds the entry point into the system
  (power-on reset entry point, for example).  It will be used in
  the final link with libup.a and other system archives to generate the
  final executable.

Supported Architectures
^^^^^^^^^^^^^^^^^^^^^^^

arch/sim
    A user-mode port of NuttX to the x86 Linux platform is available.
    The purpose of this port is primarily to support OS feature developement.
    This port does not support interrupts or a real timer (and hence no
    round robin scheduler)  Otherwise, it is complete.

arch/c5471
    TI TMS320C5471 (also called TMS320DM180 or just C5471).
    NuttX operates on the ARM7 of this dual core processor. This port
    complete, verified, and included in the NuttX release 0.1.1.

arch/dm320
    TI TMS320DM320 (also called just DM320).
    NuttX operates on the ARM9EJS of this dual core processor.  This port
    complete, verified, and included in the NuttX release 0.2.1.

arch/m68322
    A work in progress.

arch/pjrc-8051
    8051 Microcontroller.  This port is not quite ready for prime time.

Other ports for the for the TI TMS320DM270 and for MIPS are in various states
of progress


