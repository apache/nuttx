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

The NuttX configuration consists of:

o Processor architecture specific files.  These are the files contained
  in the arch/<arch-name>/ directory discussed in this README.

o Chip/SoC specific files.  Each processor processor architecture
  is embedded in chip or System-on-a-Chip (SoC) architecture.  The
  full chip architecture includes the processor architecture plus
  chip-specific interrupt logic, general purpose I/O (GIO) logic, and
  specialized, internal peripherals (such as UARTs, USB, etc.).

  These chip-specific files are contained within chip-specific
  sub-directories in the arch/<arch-name>/ directory and are selected
  via the CONFIG_ARCH_name selection

o Board specific files.  In order to be usable, the chip must be
  contained in a board environment.  The board configuration defines
  additional properties of the board including such things as
  peripheral LEDs, external peripherals (such as network, USB, etc.).

  These board-specific configuration files can be found in the
  configs/<board-name>/ sub-directories.

This README will address the processor architecture specific files
that are contained in the arch/<arch-name>/ directory. The file
include/nuttx/arch.h identifies all of the APIs that must
be provided by this architecture specific logic.  (It also includes
arch/<arch-name>/arch.h as described below).

Directory Structure
^^^^^^^^^^^^^^^^^^^

The arch directory contains architecture specific logic.  The complete
board port in is defined by the architecture-specific code in this
directory (plus the board-specific configurations in the config/
subdirectory).  Each architecture must provide a subdirectory <arch-name>
under arch/ with the following characteristics:


	<arch-name>/
	|-- include/
	|   |--<chip-name>/
	|   |  `-- (chip-specific header files)
	|   |--<other-chips>/
	|   |-- arch.h
	|   |-- irq.h
	|   `-- types.h
	`-- src/
	    |--<chip-name>/
	    |  `-- (chip-specific source files)
	    |--<other-chips>/
	    |-- Makefile
	    `-- (architecture-specific source files)

Summary of Files
^^^^^^^^^^^^^^^^

include/<chip-name>/
  This sub-directory contains chip-specific header files.

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

src/<chip-name>/
  This sub-directory contains chip-specific source files.

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

arch/arm
    This directory holds common ARM architectures.  At present, this includes
    the following subdirectories:

    arch/arm/include and arch/arm/common
        Common ARM logic.

    arch/arm/include/c5471 and arch/arm/src/c5471
        TI TMS320C5471 (also called TMS320DM180 or just C5471).
        NuttX operates on the ARM7 of this dual core processor. This port
        complete, verified, and included in the NuttX release 0.1.1.

    arch/arm/include/dm320 and arch/arm/src/dm320
        TI TMS320DM320 (also called just DM320).
        NuttX operates on the ARM9EJS of this dual core processor.  This port
        complete, verified, and included in the NuttX release 0.2.1.

    arch/arm/include/lpc214x and arch/arm/src/lpc214x
        These directories provide support for NXP LPC214x family of
        processors.
        STATUS: This port is in progress and should be available in the
        nuttx-0.2.5 release.

arch/m68322
    A work in progress.
    STATUS:  Stalled for the moment.

arch/pjrc-8051
    8051 Microcontroller.  This port is not quite ready for prime time.

The following architecture directories are deprecated.  They have been
replaced by the logic in arm/arm and will deleted when arch/arm is fully
verified.

arch/c5471
    Replaced with arch/arm/include/c5471 and arch/arm/src/c5471

arch/dm320
    Replaced with arch/arm/include/dm320 and arch/arm/src/dm320

Other ports for the for the TI TMS320DM270 and for MIPS are in various states
of progress


