==========================
Including Files in board.h
==========================

.. warning:: 
    Migrated from: https://cwiki.apache.org/confluence/display/NUTTX/Including+Files+in+board.h

Global Scope of board.h
=======================

Each board under the ``boards/`` directory must provide a header file call 
``board.h`` in the board's ``include/`` sub-directory.

When the board is configured symbolic links will be created to the board's 
``board.h`` header file at ``include/arch/board/board.h``. Header files at that 
location have `Global Scope` meaning they can be included by C/C++ logic 
anywhere in the system. For example, the ``board.h`` file can be included by
any file with:

.. code-block:: C

    #include <arch/board/board.h>

Restricted Scope of Architecture-Specific Header Files
======================================================

Each architecture also provides internal architecture-specific header files 
that can be included only by (1) the architecture logic itself and (2) by 
board logic based on that architecture. This is made possible by special 
include path options in the CFLAGS provided by the NuttX build system to 
the architecture and board logic `only`.

For this reason, the scope of the architecture-specific header files is 
`restricted`. Any attempt to include the architecture-specific header files 
by logic other than architecture- or board-specific logic will result in a 
compilation error. This is the intended behavior; it is implemented this way 
to prevent the use of such `restricted`, non-portable files throughout the OS. 
Doing so would degrade the modularity of the OS and would take many giant 
steps down the path toward `spaghetti code`.


Errors due to Mixed Scope of Header Files
=========================================

The ``board.h`` header file is included in many contexts including both by the 
architecture- and board-specific logic but also by other common OS logic. 
As examples:

* Architecture-specific code may include the ``board.h`` header file to get GPIO 
  pin definitions or to get configuration information to configure processor 
  clocking.
* Common LED and button drivers include board's ``board.h`` header file to get 
  LED and button definitions, respectively.
* Applications may include the ``board.h`` header file to get, for example, 
  ``boardctl()`` board-specific IOCTL commands.

In the first example, the ``board.h`` header file often uses pre-processor symbols 
that require definition from an architecture-specific header file. For example, 
GPIO pin definitions or clock configuration register settings. However, `you 
must avoid the temptation to include any of those architecture-specific header 
files in the board.h header file. This is prohibited!` That would be a 
"time bomb" waiting to cause a compilation failure in the future.

So How Do I Avoid This?
=======================

The only way to avoid this header file inclusion trap is:

* Never include architecture-specific header files in the ``board.h`` header file. 
  Instead,
* Include the header files needed by the the ``board.h`` header file in the C file 
  that includes ``board.h``
* Make sure that ``board.h`` is the last header file included.