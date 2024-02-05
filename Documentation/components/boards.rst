==============
Boards Support
==============

This page discusses the board support logic for NuttX.

The ``nuttx/boards`` directory is a part of the internal OS.  It should contain
only OS bring-up logic and driver initialization logic.

**THERE SHOULD BE NO APPLICATION CALLABLE LOGIC IN THIS DIRECTORY.**

If you have board-specific, application callable logic, that logic should not
go here.  Please consider using a sub-directory under ``apps/platform`` instead.

Board-Specific Configurations
=============================

The NuttX configuration consists of:

* Processor architecture specific files.  These are the files contained
  in the ``arch/<arch>/`` directory.

* Chip/SoC specific files.  Each processor architecture is embedded
  in a chip or System-on-a-Chip (SoC) architecture.  The full chip
  architecture includes the processor architecture plus chip-specific
  interrupt logic, general purpose I/O (GIO) logic, and specialized,
  internal peripherals (such as UARTs, USB, etc.).

  These chip-specific files are contained within chip-specific
  sub-directories in the ``arch/<arch>/`` directory and are selected
  via the ``CONFIG_ARCH_name`` selection

* Board specific files.  In order to be usable, the chip must be
  contained in a board environment.  The board configuration defines
  additional properties of the board including such things as
  peripheral LEDs, external peripherals (such as network, USB, etc.).

  These board-specific configuration files can be found in the
  ``boards/<arch>/<chip>/<board>/`` sub-directories.
  Additional configuration information may be available in board-specific documentation pages
  at ``Documentation/platforms/<arch>/<chip>/<board>``.

The ``boards/`` subdirectory contains configuration data for each board.  These
board-specific configurations plus the architecture-specific configurations in
the ``arch/`` subdirectory completely define a customized port of NuttX.

``boards/`` Directory Structure
===============================

The ``boards/`` directory contains board specific configuration logic.  Each
board must provide a subdirectory ``<board>`` under ``boards/`` with the
following characteristics::

  <board>
  |-- include/
  |   `-- (board-specific header files)
  |-- src/
  |   |-- Makefile
  |   `-- (board-specific source files)
  |-- <config1-dir>
  |   |-- Make.defs
  |   `-- defconfig
  |-- <config2-dir>
  |   |-- Make.defs
  |   `-- defconfig
  ...

Summary of Files
================

* ``include/`` -- This directory contains board specific header files.  This
  directory will be linked as include/arch/board at configuration time and
  can be included via #include <arch/board/header.h>``.  These header file
  can only be included by files in ``arch/<arch>include/`` and
  ``arch/<arch>/src``

* ``src/`` -- This directory contains board specific drivers.  This
  directory will be linked as ``arch/<arch>/src/board`` at configuration
  time and will be integrated into the build system.

* ``src/Makefile`` -- This makefile will be invoked to build the board specific
  drivers.  It must support the following targets:  ``libext$(LIBEXT)``, ``clean``,
  and ``distclean``.

A board may have various different configurations using these common source
files.  Each board configuration is described by two files:  Make.defs and
defconfig.  Typically, each set of configuration files is retained in a
separate configuration sub-directory (``<config1-dir>``, ``<config2-dir>``, ..
in the above diagram).

* ``Make.defs`` -- This makefile fragment provides architecture and
  tool-specific build options.  It will be included by all other
  makefiles in the build (once it is installed).  This make fragment
  should define::

    Tools: CC, LD, AR, NM, OBJCOPY, OBJDUMP
    Tool options: CFLAGS, LDFLAGS

  When this makefile fragment runs, it will be passed TOPDIR which
  is the path to the root directory of the build.  This makefile
  fragment should include::

    $(TOPDIR)/.config          : NuttX configuration
    $(TOPDIR)/tools/Config.mk  : Common definitions

  Definitions in the ``Make.defs`` file probably depend on some of the
  settings in the ``.config`` file.  For example, the ``CFLAGS`` will most likely be
  different if ``CONFIG_DEBUG_FEATURES=y``.

  The included ``tools/Config.mk`` file contains additional definitions that may
  be overridden in the architecture-specific ``Make.defs`` file as necessary::

    COMPILE, ASSEMBLE, ARCHIVE, CLEAN, and MKDEP macros

* ``defconfig`` -- This is a configuration file similar to the Linux
  configuration file.  In contains variable/value pairs like::

    CONFIG_VARIABLE=value

  This configuration file will be used at build time:

    (1) as a makefile fragment included in other makefiles, and
    (2) to generate include/nuttx/config.h which is included by
        most C files in the system.

Configuration Variables
=======================

At one time, this section provided a list of all NuttX configuration
variables. However, NuttX has since converted to use the kconfig-frontends
tools (See https://bitbucket.org/nuttx/tools/src/master/kconfig-frontends/.)
Now, the NuttX configuration is determined by a self-documenting set of
Kconfig files.

The current NuttX configuration variables are also documented in separate,
auto-generated configuration variable document.  That configuration variable
document is generated using the kconfig2html tool that can be found in the
nuttx/tools directory. That tool analyzes the NuttX Kconfig files and
generates an excruciatingly boring HTML document.

The latest boring configuration variable documentation can be regenerated at
any time using that tool or, more appropriately, the wrapper script at
nuttx/tools/mkconfigvars.sh.  That script will generate the file
nuttx/Documentation/NuttXConfigVariables.html.

Supported Boards
================

The list of supported boards can be found in :ref:`Supported Platforms <platforms>`.

Configuring NuttX
=================

Configuring NuttX requires only copying::

  boards/<arch>/<chip>/<board>/<config-dir>/Make.def to ${TOPDIR}/Make.defs
  boards/<arch>/<chip>/<board>/<config-dir>/defconfig to ${TOPDIR}/.config

- ``tools/configure.sh``

  There is a script that automates these steps.  The following steps will
  accomplish the same configuration::

   tools/configure.sh <board>:<config-dir>

  There is an alternative Windows batch file that can be used in the
  windows native environment like::

    tools\configure.bat <board>:<config-dir>

  See :doc:`tools/index` for more information about these scripts.

  And if your application directory is not in the standard location (``../apps``
  or ``../apps-<version>``), then you should also specify the location of the
  application directory on the command line like::

    cd tools
    ./configure.sh -a <app-dir> <board>:<config-dir>


Adding a New Board Configuration
================================

Okay, so you have created a new board configuration directory.
Now, how do you hook this board into the configuration system so
that you can select with ``make menuconfig``?

You will need modify the file ``boards/Kconfig``. Let's look at
the STM32F4-Discovery configuration in the ``Kconfig`` file and
see how we would add a new board directory to the configuration.
For this configuration let's say that you new board resides in the
directory ``boards/myarch/mychip/myboard``; It uses an MCU
selected with ``CONFIG_ARCH_CHIP_MYMCU``; and you want the board
to be selected with ``CONFIG_ARCH_BOARD_MYBOARD``. Then here is
how you can clone the STM32F4-Discovery configuration in
``boards/Kconfig`` to support your new board configuration.

In ``boards/Kconfig`` for the stm32f4-discovery, you will see a
configuration definition like this:

The above selects the STM32F4-Discovery board. The ``select``
lines say that the board has both LEDs and buttons and that the
board can generate interrupts from the button presses. You can
just copy the above configuration definition to a new location
(notice that they the configurations are in alphabetical order).
Then you should edit the configuration to support your board. The
final configuration definition might look something like:

Later in the ``boards/Kconfig`` file, you will see a long, long
string configuration with lots of defaults like this:

This logic will assign string value to a configuration variable
called ``CONFIG_ARCH_BOARD`` that will name the directory where
the board-specific files reside. In our case, these files reside
in ``boards/myarch/mychip/myboard`` and we add the following to
the long list of defaults (again in alphabetical order):

Now the build system knows where to find your board configuration!

And finally, add something like this near the bottom of
``boards/myarch/mychip/myboard``:

This includes additional, board-specific configuration variable
definitions in ``boards/myarch/mychip/myboard/Kconfig``.

Building Symbol Tables
======================

Symbol tables are needed at several of the binfmt interfaces in order to bind
a module to the base code.  These symbol tables can be tricky to create and
will probably have to be tailored for any specific application, balancing
the number of symbols and the size of the symbol table against the symbols
required by the applications.

The top-level System.map file is one good source of symbol information
(which, or course, was just generated from the top-level nuttx file
using the GNU 'nm' tool).

There are also common-separated value (CSV) values in the source try that
provide information about symbols.  In particular::

  nuttx/syscall/syscall.csv - Describes the NuttX RTOS interface, and
  nuttx/lib/libc.csv        - Describes the NuttX C library interface.

There is a tool at nuttx/tools/mksymtab that will use these CSV files as
input to generate a generic symbol table.  See ``nuttx/tools/README.txt`` for
more information about using the mksymtab tool.
