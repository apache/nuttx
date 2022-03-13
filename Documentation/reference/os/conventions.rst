==================================
Naming and Header File Conventions
==================================

-  **Common Microprocessor Interfaces**. Any interface that is
   common to all microprocessors should be prefixed with ``up_``
   and prototyped in ``include/nuttx/arch.h``. The definitions in
   that header file provide the common interface between NuttX and
   the architecture-specific implementation in ``arch/``.

      ``up_`` is supposed to stand for microprocessor; the ``u``
      is like the Greek letter micron: ľ. So it would be ``ľP``
      which is a common shortening of the word microprocessor. I
      don't like that name very much. I wish I would have used a
      more obvious prefix like ``arch_`` instead -- then I would
      not have to answer this question so often.

-  **Microprocessor-Specific Interfaces**. An interface which is
   unique to a certain microprocessor should be prefixed with the
   name of the microprocessor, for example ``stm32_``, and be
   prototyped in some header file in the ``arch/`` directories.

   There is also a ``arch/<architecture>/include/<chip>/chip.h``
   header file that can be used to communicate other
   microprocessor-specific information between the board logic and
   even application logic. Application logic may, for example,
   need to know specific capabilities of the chip. Prototypes in
   that ``chip.h`` header file should follow the
   microprocessor-specific naming convention.

-  **Common Board Interfaces**. Any interface that is common to
   all boards should be prefixed with ``board_`` and should also
   be prototyped in ``include/nuttx/board.h``. These ``board_``
   definitions provide the interface between the board-level logic
   and the commaon and architecture-specific logic.

-  **Board-Specific Interfaces**. Any interface which is unique to
   a board should be prefixed with the board name, for example
   ``stm32f4discovery_``. Sometimes the board name is too long so
   ``stm32_`` would be okay too. These should be prototyped in
   ``boards/<arch>/<chip>/<board>/src/<board>.h`` and should not
   be used outside of that directory since board-specific
   definitions have no meaning outside of the board directory.

-  **Scope of Inclusions**. Header files are made accessible to
   internal OS logic and to applications through symbolic links
   and through *include paths* that are provided to the C/C++
   compiler. Through these include paths, the NuttX build system
   also enforces modularity in the design. For example, one
   important design principle is architectural *layering*. In this
   case I am referring to the OS as layered into application
   interface, common internal OS logic, and lower level
   platform-specific layers. The platform-specific layers all
   reside in the either ``arch/`` sub-directories on the
   ``config/`` subdirectories: The former sub-directories are
   reserved for microcontroller-specific logic and the latter for
   board-specific logic.

   In the strict, layered NuttX architecture, the upper level OS
   services are always available to platform-specific logic.
   However, the opposite is *not* true: Common OS logic must never
   have any dependency on the lower level platform-specific code.
   The OS logic must be totally agnostic about its hardware
   environment. Similarly, microcontroller-specific logic was be
   completely ignorant of board-specific logic.

   This strict layering is enforced in the NuttX build system by
   controlling the compiler include paths: Higher level code can
   never include header files from either; of the
   platform-specific source directories; microcontroller-specific
   code can never include header files from the board-specific
   source directories. The board-specific directories are, then,
   at the bottom of the layered hierarchy.

   An exception to these inclusion restrictions is the
   platform-specific *include/*. These are made available to
   higher level OS logic. The microcontroller-specific include
   directory will be linked at ``include/arch/chip`` and, hence,
   can be included like ``#include <arch/hardware/chip.h``.
   Similarly, the board-specific include directory will be linked
   at ``include/arch/board`` and, hence, can be included like
   ``#include <arch/board/board.h``.

   Keeping in the spirit of the layered architecture, these
   publicly visible header files must *not* export
   platform-specific definitions; Only platform-specific
   realizations of standardized declarations should be visible.
   Those *standardized declarations* should appear in common
   header files such as those provided by ``include/nuttx/arch.h``
   and ``include/nuttx/board.h``. Similarly, these publicly
   visible header file must *not* include files that reside in the
   inaccessible platform-specific source directories. For example,
   the board-specific
   ``boards/<arch>/<chip>/<board>/include/board.h`` header file
   must never include microcontroller-specific header files that
   reside in ``arch/<arch>/src/<mcu>``. That practice will cause
   inclusion failures when the publicly visible file is included
   in common logic outside of the platform-specific source
   directories.

