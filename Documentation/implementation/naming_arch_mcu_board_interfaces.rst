=================================================
Naming of Architecture, MCU, and Board Interfaces
=================================================
 
What's the meaning of the prefix ``up_`` in NuttX?
Which functions should be prefixed with ``up_``?

``up_`` is supposed to stand for **microprocessor**;
the `u` is like the Greek letter micron: µ. So it would be **µP** which
is a common shortening of the word microprocessor.
I don't like that name very much. I wish I would have used ``arch_`` instead.
But now I think I am stuck with ``up_``.

Common Microcontroller Interfaces
=================================

Any interface that is common to all microcontroller and is used throughout
the system should be prefixed with ``up_`` and prototyped in
``include/nuttx/arch.h``. The definitions in that header file provide the
common interface between NuttX and the architecture-specific implementation
in ``arch/``.

Microcontroller-Specific Interfaces
===================================

An interface which is unique to a certain microcontroller should be prefixed
with the name of the microcontroller, for example ``stm32_``,
and be prototyped in some header file in the ``arch/`` directories.
These are interfaces used only by logic within the ``arch/`` and ``boards/``
directories.

Architecture-Specific Interfaces
================================

An interface which is unique to a CPU architecture, but shared among
microcontrollers that implement that CPU architecture should be prefixed
with the name of architecture. For example, the architecture-specific
interfaces used by STM32 would begin with ``arm_``.
These are interfaces used only by logic within the ``arch/`` and ``boards/``
directories.

There is also a ``arch/<architecture>/include/<chip>/chip.h`` header file
that can be used to communicate other microprocessor-specific information
between the board logic and even application logic.

Application logic may, for example, need to know specific capabilities
of the chip. Prototypes in that ``chip.h`` header file should follow
the microprocessor-specific naming convention.

Common Board Interfaces
=======================

Any interface that is common to all boards should be prefixed with ``board_``
and should also be prototyped in ``include/nuttx/arch.h``.

These ``board_`` definitions provide the interface between the board-level
logic and the architecture-specific logic.

There is also a ``boards/<arch>/<chip>/<board>/include/board.h`` header file
that can be used to communicate other board-specific information between
the architecture logic and even application logic.

Any definitions which are common between a single architecture and several
boards should go in this ``board.h`` header file;
``include/nuttx/arch.h`` is reserved for board-related definitions common
to all architectures.

Specific Interfaces
===================

Any interface which is unique to a board should be prefixed with the board
name, for example ``stm32f4discovery_``. Sometimes the board name is too long
so ``stm32_`` would be okay too. These should be prototyped in
``boards/<arch>/<chip>/<board>/src/<board>.h`` and should not be used outside
of that directory since board-specific definitions have no meaning outside
of the board directory.
