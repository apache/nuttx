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
   contained in the ``arch/<arch-name>/`` directory and
   are discussed in a paragraph
   `below <#arch-subdirectory-structure>`__. As an example, all ARM
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
   sub-directories also under the ``arch/<arch-name>/``
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
   ``boards/<arch-name>/<chip-name>/<board-name>/``
   sub-directories and are discussed in a paragraph
   `below <#boards-subdirectory-structure>`__.

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

This sub-directory holds the NuttX supported architectures.

For details see :doc:`/components/arch/index`.

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
``boards/<arch-name>/<chip-name>/``.

See :doc:`/components/boards` for details.

``nuttx/cmake``
===============

This sub-directory holds the NuttX CMake functions.

For details see :doc:`/components/cmake`.

``nuttx/crypto``
================

This sub-directory holds the NuttX cryptographic sub-system.

For details see :doc:`/components/crypto`.

``nuttx/drivers``
=================

This directory holds architecture-independent device drivers.

For details see :doc:`/components/drivers/index`.

``nuttx/fs``
============

This directory contains the NuttX file system. This file system is
described `below <#NxFileSystem>`__.

``nuttx/graphics``
==================

This directory contains files for graphics/video support under
NuttX.

For details see :doc:`/components/nxgraphics/index`.

``nuttx/include``
=================

This directory holds NuttX header files. Standard header files
file retained in can be included in the *normal* fashion:

``nuttx/libs/libc``
===================

This directory holds a collection of standard libc-like functions
with custom interfaces into NuttX.

For details see :doc:`/components/libs/index`.

``nuttx/mm``
============

This is the NuttX memory manager.

For details see :doc:`/components/mm/index`.

``nuttx/net``
=============

This directory contains the implementation of the NuttX networking
layer including internal socket APIs.

For details see :doc:`/components/net/index`.

``nuttx/openamp``
=================

This directory contains OpenAMP support for NuttX.

For details see :doc:`/components/openamp`.

``nuttx/pass1``
===============

TODO

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

For details see :doc:`/components/syscall`.

``nuttx/tools``
===============

This directory holds a collection of tools and scripts to simplify
configuring, building and maintaining NuttX.

Refer to the :doc:`/components/tools/index` page for more
information about the individual files. Some of these tools are
discussed below as well in the discussion of `configuring and
building <#configandbuild>`__ NuttX.

``nuttx/video``
===============

This directory holds support for video sub-system.

For details see :doc:`/components/video`.

``nuttx/wireless``
==================

This directory holds support for hardware-independent wireless
support.

For details see :doc:`/components/wireless`.

``nuttx/CMakeList.txt``
=======================

The top-level ``CMakeList.txt`` file.

``nuttx/Makefile``
==================

The top-level ``Makefile`` in the ``$(TOPDIR)`` directory contains
all of the top-level control logic to build NuttX.
