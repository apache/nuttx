=====================================
fdpic: FDPIC module build tooling
=====================================

``tools/fdpic`` is everything needed to build an FDPIC module out of tree.  A
module is an ELF shared object whose read-only segment the target maps straight
out of flash and executes in place, while its writable segment is copied to RAM
once per running instance.  It links against nothing: libc and everything else
are imported from the firmware's exported symbol table at load time.

The loader that consumes these modules is the ELF loader, enabled by
``CONFIG_FDPIC``.  The format, the toolchain and the load-time contract are
described in :doc:`/components/fdpic`.

Contents
========

============================ ==================================================
File                         Purpose
============================ ==================================================
``nuttx-fdpic.mk``           The module build itself; include it from a
                             two-line makefile
``fdpic-verify.sh``          Checks a built module's imports resolve against
                             the firmware
``nuttx-exports.sh``         Turns ``libs/libc/exec_symtab.c`` into a symbol
                             list
``fdpic-embed.py``           Turns a built module into a C header, for carrying
                             one inside an image
``build-binutils.sh``        Builds the ``arm-uclinuxfdpiceabi`` binutils, the
                             one from-source dependency
============================ ==================================================

Building a module
=================

A whole module is three lines of makefile beside the source.  Taking
``apps/examples/fdpicxip/modules/qsorter.c``, which is a module in its own
right, as the source:

.. code:: makefile

   MODULE = qsorter
   SRCS   = qsorter.c

   include /path/to/nuttx/tools/fdpic/nuttx-fdpic.mk

Then:

.. code:: console

   $ make NUTTX_DIR=/path/to/nuttx
     CC   qsorter.c
     LD   qsorter.fdpic
   OK    qsorter.fdpic: FDPIC, entry 0x2a1, 4 imports resolved

``NUTTX_DIR`` has to be a configured, built tree: the compile needs its headers
and the verify step needs the export table generated into
``libs/libc/exec_symtab.c``.

Toolchain
=========

Two toolchains are involved.  The stock ``arm-none-eabi`` compiler does the
compiling -- it emits perfectly good FDPIC objects for both C and C++ -- and
``arm-uclinuxfdpiceabi`` **binutils** does the linking, because
``arm-none-eabi-ld`` cannot produce an FDPIC object at all.  So the
from-source dependency is binutils alone, which ``build-binutils.sh`` builds in
about a minute.

Verification runs as part of the default target on purpose: a module that
imports a symbol the firmware does not export links perfectly happily and fails
only once it is on the target, as a bare ``-ENOENT`` that names nothing.

Modules carried inside an image
===============================

``fdpic-embed.py`` exists for applications that have to load a module before
there is any way to put files on the target, so they embed one and write it out
at run time.  ``apps/examples/fdpicxip/modules/`` uses it that way, and is the
worked example of driving this tooling for several modules at once.
