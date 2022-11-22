.. include:: /substitutions.rst

.. todo::
  This is mostly untouched from the original documentation. It does
  not really belong to "quickstart". Also, this needs cleanup.

.. _organization:

===================
Directory Structure
===================

This is included for reference, and it's not necessary to know
all the details at first.

The general directory layout for NuttX is
very similar to the directory structure of the Linux kernel -- at
least at the most superficial layers. At the top level is the main
makefile and a series of sub-directories identified below and
discussed in the following paragraphs:

**Configuration Files**. The NuttX configuration consists of logic
in processor architecture directories, *chip/SoC* directories, and
board configuration directories. The complete configuration is
specified by several settings in the NuttX configuration file.

-  *Processor architecture specific files*. These are the files
   contained in the ``arch/``\ *<arch-name>*\ ``/`` directory and
   are discussed in a paragraph
   `below <#archdirectorystructure>`__. As an example, all ARM
   processor architectures are provided under the ``arch/arm/``
   directory which is selected with the ``CONFIG_ARCH="arm"``
   configuration option.

   Variants of the processor architecture may be provided in
   sub-directories of the Extending this example, the ARMv7-M ARM
   family is supported by logic in ``arch/arm/include/armv7-m``
   and ``arch/arm/src/armv7-m`` directories which are selected by
   the ``CONFIG_ARCH_CORTEXM3=y``, ``CONFIG_ARCH_CORTEXM4=y``, or
   ``CONFIG_ARCH_CORTEXM7=y`` configuration options

-  *Chip/SoC specific files*. Each processor architecture is
   embedded in a *System-on-a-Chip* (SoC) architecture. The full
   SoC architecture includes the processor architecture plus
   chip-specific interrupt logic, clocking logic, general purpose
   I/O (GPIO) logic, and specialized, internal peripherals (such
   as UARTs, USB, etc.).

   These chip-specific files are contained within chip-specific
   sub-directories also under the ``arch/``\ *<arch-name>*\ ``/``
   directory and are selected via the ``CONFIG_ARCH_CHIP``
   selection.

   As an example, the STMicro STM32 SoC architecture is based on
   the ARMv7-M processor and is supported by logic in the
   ``arch/arm/include/stm32`` and ``arch/arm/src/stm32``
   directories which are selected with the
   ``CONFIG_ARCH_CHIP="stm32"`` configuration setting.

-  *Board specific configurations*. In order to be usable, the
   chip must be contained in a board environment. The board
   configuration defines additional properties of the board
   including such things as peripheral LEDs, external peripherals
   (such as networks, USB, etc.).

   These board-specific configuration files can be found in the
   ``boards/``\ *<arch-name>*\ ``/``\ *<chip-name>*\ ``/``\ *<board-name>*\ ``/``
   sub-directories and are discussed in a paragraph
   `below <#boardsdirectorystructure>`__.

   The directory ``boards/arm/stm32/stm32f4disovery/``, as an
   example, holds board-specific logic for the STM32F4 Discovery
   board and is selected via the
   ``CONFIG_ARCH_BOARD="stm32f4discovery"`` configuration setting.

``nuttx/Documentation``
=======================

This directory holds the NuttX documentation. It's made with
the `Sphinx documentation system <https://www.sphinx-doc.org>`_. See the
README.md file for information on how to build it.

``nuttx/arch``
==============

Arch Subdirectory Structure
---------------------------

This directory contains several sub-directories, each containing
architecture-specific logic. The task of porting NuttX to a new
processor consists of add a new subdirectory under ``arch/``
containing logic specific to the new architecture. The complete
board port in is defined by the architecture-specific code in this
directory (plus the board-specific configurations in the
``config/`` subdirectory). Each architecture must provide a
subdirectory, *<arch-name>* under ``arch/`` with the following
characteristics:

Arch Summary of Files
---------------------

-  ``include/``\ *<chip-name>*\ ``/`` This sub-directory contains
   chip-specific header files.
-  ``include/arch.h``: This is a hook for any architecture
   specific definitions that may be needed by the system. It is
   included by ``include/nuttx/arch.h``.
-  ``include/types.h``: This provides
   architecture/toolchain-specific definitions for standard types.
   This file should ``typedef``:

   and if the architecture supports 24- or 64-bit integers

   NOTE that these type names have a leading underscore character.
   This file will be included(indirectly) by include/stdint.h and
   typedef'ed to the final name without the underscore character.
   This roundabout way of doings things allows the stdint.h to be
   removed from the include/ directory in the event that the user
   prefers to use the definitions provided by their toolchain
   header files

   And finally

   Must be defined to the be the size required to hold the
   interrupt enable/disable state.

   This file will be included by include/sys/types.h and be made
   available to all files.

-  ``include/irq.h``: This file needs to define some architecture
   specific functions (usually inline if the compiler supports
   inlining) and some structures. These include:

   -  ``struct xcptcontext``: This structures represents the saved
      context of a thread.
   -  ``irqstate_t up_irq_save(void)``: Used to disable all
      interrupts. In the case of multi-CPU platforms, this
      function disables interrupts on CPUs.
   -  ``void up_irq_restore(irqstate_t flags)``: Used to restore
      interrupt enables to the same state as before
      ``up_irq_save()`` was called.

   This file must also define ``NR_IRQS``, the total number of
   IRQs supported by the board.

-  ``include/syscall.h``: This file needs to define some
   architecture specific functions (usually inline if the compiler
   supports inlining) to support software interrupts or
   *syscall*\ s that can be used all from user-mode applications
   into kernel-mode NuttX functions. This directory must always be
   provided to prevent compilation errors. However, it need only
   contain valid function declarations if the architecture
   supports the ``CONFIG_BUILD_PROTECTED`` or
   ``CONFIG_BUILD_KERNEL``\ configurations.

   -  ``uintptr_t sys_call0(unsigned int nbr)``: ``nbr`` is one of
      the system call numbers that can be found in
      ``include/sys/syscall.h``. This function will perform a
      system call with no (additional) parameters.
   -  ``uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1)``:
      ``nbr`` is one of the system call numbers that can be found
      in ``include/sys/syscall.h``. This function will perform a
      system call with one (additional) parameter.
   -  ``uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1, uintptr_t parm2)``:
      ``nbr`` is one of the system call numbers that can be found
      in ``include/sys/syscall.h``. This function will perform a
      system call with two (additional) parameters.
   -  ``uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1, uintptr_t parm2, uintptr_t parm3)``:
      ``nbr`` is one of the system call numbers that can be found
      in ``include/sys/syscall.h``. This function will perform a
      system call with three (additional) parameters.
   -  ``uintptr_t sys_call4(unsigned int nbr, uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4)``:
      ``nbr`` is one of the system call numbers that can be found
      in ``include/sys/syscall.h``. This function will perform a
      system call with four (additional) parameters.
   -  ``uintptr_t sys_call5(unsigned int nbr, uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5)``:
      ``nbr`` is one of the system call numbers that can be found
      in ``include/sys/syscall.h``. This function will perform a
      system call with five (additional) parameters.
   -  ``uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5, uintptr_t parm6)``:
      ``nbr`` is one of the system call numbers that can be found
      in ``include/sys/syscall.h``. This function will perform a
      system call with six (additional) parameters.

   This file must also define ``NR_IRQS``, the total number of
   IRQs supported by the board.

-  ``src/``\ *<chip-name>*\ ``/`` This sub-directory contains
   chip-specific source files.
-  ``src/Makefile``: This makefile will be executed to build the
   targets ``src/libup.a`` and ``src/up_head.o``. The
   ``up_head.o`` file holds the entry point into the system
   (power-on reset entry point, for example). It will be used in
   the final link with ``libup.a`` and other system archives to
   generate the final executable.
-  *(architecture-specific source files)*. The file
   ``include/nuttx/arch.h`` identifies all of the APIs that must
   be provided by the architecture specific logic. (It also
   includes ``arch/``\ *<arch-name>*\ ``/arch.h`` as described
   above).

Supported Architectures
-----------------------

**Architecture- and Chip-Specific Directories**. All processor
architecture-specific directories are maintained in
sub-directories of the ``arch/`` directory. Different chips or
SoC's may implement the same processor core. Chip-specific logic
can be found in sub-directories under the architecture directory.
Current architecture/chip directories are summarized below:

-  ``arch/sim``: A user-mode port of NuttX to the x86 Linux or
   Cygwin platform is available. The purpose of this port is
   primarily to support OS feature development. This port does not
   support interrupts or a real timer (and hence no round robin
   scheduler) Otherwise, it is complete.
-  ``arch/arm``: This directory holds common ARM architectures.
-  ``arch/avr``: This directory holds common AVR and AVR32
   architectures.
-  ``arch/mips``: This directory holds common MIPS architectures.
   This include PIC32MX and PIC32MZ.
-  ``arch/misoc``: This directory supports the Misoc LM3
   architecture.
-  ``arch/or1K``: This directory supports the OpenRISC mor1kx
   architecture.
-  ``arch/renesas``: This directory is the home for various
   Renesas architectures, currently only the M16C and vererable
   SuperH-1 architectures.
-  ``arch/xtensa``: This directory supports the Xtensa LX6
   architecture as used by the ESP32.
-  ``arch/z16f``: Zilog z16f Microcontroller.
-  ``arch/z80``: This directory holds 8-bit ZiLOG architectures.
   At present, this includes the Zilog z80, ez80Acclaim! and
   z8Encore! Microcontrollers.

``nuttx/binfmt``
================

The ``binfmt/`` subdirectory contains logic for loading binaries
in the file system into memory in a form that can be used to
execute them.

``nuttx/audio``
===============

The ``audio/`` subdirectory contains the NuttX audio sub-system.

.. _nuttx_boards:

``nuttx/boards``
================

The ``boards/`` subdirectory contains custom logic and board
configuration data for each board. These board-specific
configurations plus the architecture-specific configurations in
the ``arch/`` subdirectory complete define a customized port of
NuttX.

Boards Subdirectory Structure
-----------------------------

The ``boards/`` directory contains board specific configuration
files. Each board must provide a sub-directory <board-name> under
``boards/``\ *<arch-name>*\ ``/``>\ *<chip-name>*\ ``/`` with the
following characteristics:

Boards Summary of Files
-----------------------

**Board Specific Logic**

-  ``include/``: This directory contains board specific header
   files. This directory will be linked as ``include/arch/board``
   at configuration time and can be included via
   ``#include <arch/board/header.h>``. These header file can only
   be included by files in ``arch/``\ *<arch-name>*\ ``/include/``
   and ``arch/``\ *<arch-name>*\ ``/src/``.
-  ``src/``: This directory contains board specific drivers. This
   directory will be linked as
   *<config>*\ ``/arch/``\ *<arch-name>*\ ``/src/board`` at
   configuration time and will be integrated into the build
   system.
-  ``src/Makefile``: This makefile will be invoked to build the
   board specific drivers. It must support the following targets:
   ``libext$(LIBEXT)``, ``clean``, and ``distclean``.

**Board Specific Configuration Sub-Directories**

The
``boards/``\ *<arch-name>*\ ``/``\ *<chip-name>*\ ``/``\ *<board-name>*\ ``/configs``
sub-directory holds all of the files that are necessary to
configure NuttX for the particular board. A board may have various
different configurations using the common source files. Each board
configuration is described by two files: ``Make.defs`` and
``defconfig``. Typically, each set of configuration files is
retained in a separate configuration sub-directory
(*<config1-dir>*, *<config2-dir>*, .. in the above diagram).

NOTE: That the ``Make.defs`` file may reside in one of two
locations: There may be a unique Make.defs file for each
configuration in the configuration directory *OR* if that file is
absent, there may be a common board ``Make.defs`` file in the
``/scripts`` directory. The ``Make.defs`` file in the
configuration takes precedence if it is present.

The procedure for configuring NuttX is described
`below <#configuringnuttx>`__, This paragraph will describe the
contents of these configuration files.

-  ``Make.defs``: This makefile fragment provides architecture and
   tool-specific build options. It will be included by all other
   makefiles in the build (once it is installed). This make
   fragment should define:

   -  Tools: ``CC``, ``LD``, ``AR``, ``NM``, ``OBJCOPY``,
      ``OBJDUMP``
   -  Tool options: ``CFLAGS``, ``LDFLAGS``

   When this makefile fragment runs, it will be passed ``TOPDIR``
   which is the path to the root directory of the build. This
   makefile fragment should include:

   -  ``$(TOPDIR)/.config`` : NuttX configuration
   -  ``$(TOPDIR)/tools/Config.mk`` : Common definitions

   Definitions in the ``Make.defs`` file probably depend on some
   of the settings in the .\ ``config`` file. For example, the
   ``CFLAGS`` will most likely be different if
   ``CONFIG_DEBUG_FEATURES=y``.

   The included ``tools/Config.mk`` file contains additional
   definitions that may be overridden in the architecture-specific
   Make.defs file as necessary:

   -  ``COMPILE``, ``ASSEMBLE``, ``ARCHIVE``, ``CLEAN``, and
      ``MKDEP`` macros

-  ``defconfig``: This is a configuration file similar to the
   Linux configuration file. In contains variable/value pairs
   like:

   -  ``CONFIG_VARIABLE``\ =value

   This configuration file will be used at build time:

   #. As a makefile fragment included in other makefiles, and
   #. to generate ``include/nuttx/config.h`` which is included by
      most C files in the system.

Supported Boards
----------------

All of the specific boards supported by NuttX are identified in
the
`README.txt <https://github.com/apache/nuttx/blob/master/boards/README.txt>`__
file.

Adding a New Board Configuration
--------------------------------

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

``nuttx/crypto``
================

This sub-directory holds the NuttX cryptographic sub-system.

``nuttx/drivers``
=================

This directory holds architecture-independent device drivers.

``nuttx/fs``
============

This directory contains the NuttX file system. This file system is
described `below <#NxFileSystem>`__.

``nuttx/graphics``
==================

This directory contains files for graphics/video support under
NuttX.

``nuttx/include``
=================

This directory holds NuttX header files. Standard header files
file retained in can be included in the *normal* fashion:

``nuttx``
=========

This is a (almost) empty directory that has a holding place for
generated static libraries. The NuttX build system generates a
collection of such static libraries in this directory during the
compile phase. These libraries are then in a known place for the
final link phase where they are accessed to generated the final
binaries.

``nuttx/libs/libc``
===================

This directory holds a collection of standard libc-like functions
with custom interfaces into NuttX.

Normally the logic in this file builds to a single library
(``libc.a``). However, if NuttX is built as a separately compiled
kernel (with ``CONFIG_BUILD_PROTECTED=y`` or
``CONFIG_BUILD_KERNEL=y``), then the contents of this directory
are built as two libraries: One for use by user programs
(``libc.a``) and one for use only within the <kernel> space
(``libkc.a``).

These user/kernel space libraries (along with the sycalls of
```nuttx/syscall`` <#DirStructSyscall>`__) are needed to support
the two differing protection domains.

Directory structure:

``nuttx/libs/libxx``
====================

This directory holds a tiny, minimal standard std C++ that can be
used to build some, simple C++ applications in NuttX.

``nuttx/mm``
============

This is the NuttX memory manager.

``nuttx/net``
=============

This directory contains the implementation of the NuttX networking
layer including internal socket APIs.

``nuttx/sched``
===============

The files forming core of the NuttX RTOS reside here.

``nuttx/syscall``
=================

If NuttX is built as a separately compiled kernel (with
``CONFIG_BUILD_PROTECTED=y`` or ``CONFIG_BUILD_KERNEL=y``), then
the contents of this directory are built. This directory holds a
syscall interface that can be used for communication between
user-mode applications and the kernel-mode RTOS.

``nuttx/tools``
===============

This directory holds a collection of tools and scripts to simplify
configuring, building and maintaining NuttX.

Refer to the README file in the ``tools`` directory for more
information about the individual files. Some of these tools are
discussed below as well in the discussion of `configuring and
building <#configandbuild>`__ NuttX.

``nuttx/wireless``
==================

This directory holds support for hardware-independent wireless
support.

``nuttx/Makefile``
==================

The top-level ``Makefile`` in the ``$(TOPDIR)`` directory contains
all of the top-level control logic to build NuttX.

