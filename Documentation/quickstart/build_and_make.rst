.. include:: /substitutions.rst
.. _build_and_make:

Build and Make Details
======================

This is included for reference, and it's not necessary to know all these details.

As described in :ref:`compiling`, you use ``make`` at the root ``nuttx/`` directory to build NuttX. This is also
referenced as ``$(TOPDIR)`` in the ``Makefile``.

Root Directory
--------------

The ``$(TOPDIR)`` directory holds:

-  The top level ```Makefile`` <#topmakefile>`__ that controls the
   NuttX build.

That directory also holds:

-  The makefile fragment :ref:`.config <nuttx_boards>`
   that describes the current configuration, and
-  The makefile fragment :ref:`Make.defs <nuttx_boards>`
   that provides customized build targets.

Environment Variables
---------------------

The specific environmental definitions
are unique for each board but should include, as a minimum,
updates to the ``PATH`` variable to include the full path to the
architecture-specific toolchain identified in
:ref:`Make.defs <nuttx_boards>`.

First Time Make
---------------

Additional configuration actions will be taken the first time that system is built. These additional steps
include:

-  Auto-generating the file ``include/nuttx/config.h`` using the
   ``$(TOPDIR)/.config`` file.
-  Auto-generating the file ``$(TOPDIR)/.version`` with version
   0.0 if one does not exist.
-  Auto-generating the file ``include/nuttx/version.h`` using the
   ``$(TOPDIR)/.version`` file.
-  Creating a link to
   ``$(TOPDIR)/arch/``\ *<arch-name>*\ ``/include`` at
   ``$(TOPDIR)/include/arch``.
-  Creating a link to
   ``$(TOPDIR)/boards/``\ *<arch-name>*\ ``/``\ *<chip-name>*\ ``/``\ *<board-name>*\ ``/include``
   at ``$(TOPDIR)/include/arch/board``.
-  Creating a link to
   ``$(TOPDIR)/boards/``\ *<arch-name>*\ ``/``\ *<chip-name>*\ ``/``\ *<board-name>*\ ``/src``
   at ``$(TOPDIR)/arch/``\ *<arch-name>*\ ``/src/board``
-  Creating a link to ``${APPDIR}/include`` at
   ``$(TOPDIR)/include/apps``
-  Creating make dependencies.
