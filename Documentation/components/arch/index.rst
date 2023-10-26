==========================
Architecture-Specific Code
==========================

The NuttX configuration consists of:

* Processor architecture specific files.  These are the files contained
  in the ``arch/<arch-name>/`` directory discussed in this file.

* Chip/SoC specific files.  Each processor architecture is embedded in
  chip or System-on-a-Chip (SoC) architecture.  The full chip
  architecture includes the processor architecture plus chip-specific
  interrupt logic, general purpose I/O (GPIO) logic, and specialized,
  internal peripherals (such as UARTs, USB, etc.).

  These chip-specific files are contained within chip-specific
  sub-directories in the ``arch/<arch-name>/`` directory and are selected
  via the CONFIG_ARCH_name selection

* Board specific files.  In order to be usable, the chip must be
  contained in a board environment.  The board configuration defines
  additional properties of the board including such things as peripheral
  LEDs, external peripherals (such as network, USB, etc.).

  These board-specific configuration files can be found in the
  ``boards/<arch>/<chip>/<board>`` sub-directories.

This file will address the processor architecture specific files that
are contained in the ``arch/<arch-name>/`` directory.  The file
include/nuttx/arch.h identifies all of the APIs that must be provided by
this architecture specific logic.  (It also includes
``arch/<arch-name>/arch.h`` as described below).

Directory Structure in ``arch/``
================================

The ``arch/`` directory contains architecture-specific logic.  The complete
board port is defined by the architecture-specific code in this
directory plus the board-specific configurations in the ``boards/``
directory.  Each architecture must provide a subdirectory <arch-name>
under ``arch/`` with the following characteristics::

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
================

``include/<chip-name>/``

  This sub-directory contains chip-specific header files.

``include/arch.h``

  This is a hook for any architecture specific definitions that may be
  needed by the system.  It is included by ``include/nuttx/arch.h``

``include/types.h``

  This provides architecture/toolchain-specific definitions for standard
  types.  This file should typedef: ``_int8_t``, ``_uint8_t``, ``_int16_t``,
  ``_uint16_t``, ``_int32_t``, ``_uint32_t``

  and if the architecture supports 64-bit integers: ``_int24_t``, ``_uint24_t``,
  ``_int64_t``, ``_uint64_t``

  NOTE that these type names have a leading underscore character.  This
  file will be included (indirectly) by ``include/stdint.h`` and typedef'ed
  to the final name without the underscore character.  This roundabout
  way of doings things allows the stdint.h to be removed from the
  ``include/`` directory in the event that the user prefers to use the
  definitions provided by their toolchain header files.

``irqstate_t``

  Must be defined to the size required to hold the interrupt
  enable/disable state.

  This file will be included by ``include/sys/types.h`` and be made
  available to all files.

``include/irq.h``

  This file needs to define some architecture-specific functions
  (usually inline if the compiler supports inlining) and structures.
  These include:

``struct xcptcontext``

  This structure represents the saved context ofa thread.

``irqstate_t up_irq_save(void)``

  Used to disable all interrupts.

``void up_irq_restore(irqstate_t flags)``

  Used to restore interrupt enables to the same state as before ``up_irq_save``
  was called.

  NOTE: These interfaces are not available to application code but can
  only be used within the operating system code.  And, in general, these
  functions should **never** be called directly, not unless you know
  absolutely well what you are doing.  Rather you should typically use
  the wrapper functions ``enter_critical_section()`` and
  ``leave_critical_section()`` as prototyped in ``include/nuttx/irq.h``.

  This file must also define NR_IRQS, the total number of IRQs supported
  by the board.

``src/<chip-name>/``

  This sub-directory contains chip-specific source files.

``src/Makefile``

  This makefile will be executed to build the targets src/libup.a and
  src/up_head.o.  The up_head.o file holds the entry point into the
  system (power-on reset entry point, for example).  It will be used in
  the final link with libup.a and other system archives to generate the
  final executable.

Supported Architectures
=======================

The list of supported architectures can be found in :ref:`Supported Platforms <platforms>`.
