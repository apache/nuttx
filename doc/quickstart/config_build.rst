========================
Configuring and Building
========================

Configuring NuttX
=================

**Manual Configuration**. Configuring NuttX requires only copying
the `board-specific configuration files <#boardconfigsubdirs>`__
into the top level directory which appears in the make files as
the make variable, ``$(TOPDIR)``. This could be done manually as
follows:

-  Copy
   ``boards/``\ *<arch-name>*\ ``/``\ *<chip-name>*\ ``/``\ *<board-name>*\ ``/configs/[``\ *<config-dir>*\ ``/]Make.defs``
   to ``$(TOPDIR)/Make.defs`` (see NOTE below),
-  Copy
   ``boards/``\ *<arch-name>*\ ``/``\ *<chip-name>*\ ``/``\ *<board-name>*\ ``/configs/[``\ *<config-dir>*\ ``/]defconfig``
   to ``$(TOPDIR)/.config``

Where *<board-name>* is the name of one of the sub-directories of
the NuttX ```boards/`` <#DirStructConfigs>`__ directory. This
sub-directory name corresponds to one of the supported boards
identified `above <#supportedboards>`__. <config-dir> is the
optional, specific configuration directory for the board. And
<app-dir> is the location of the optional application directory.

NOTE: Recall that the ``Make.defs`` file may reside in either the
``boards/``\ *<arch-name>*\ ``/``\ *<chip-name>*\ ``/``\ *<board-name>*/``configs/[``\ *<config-dir>*
directory or in the
``boards/``\ *<arch-name>*\ ``/``\ *<chip-name>*\ ``/``\ *<board-name>*/``scripts``.

**Automated Configuration**. There is a script that automates
these steps. The following steps will accomplish the same
configuration:

There is an alternative Windows batch file, ``configure.bat``,
that can be used instead of ``configure.sh`` in the windows native
environment like:

And, to make sure that other platforms are supported, there is
also a C program at ``tools/configure.c`` that can be compiled to
establish the board configuration on all platforms.

   NOTE (2019-08-6): As of this writing, changes to the boards/
   directly have made ``configure.bat`` unusable. For the native
   Windows environment, ``configure.c`` is recommended until that
   batch file can be repaired.

See ``tools/README.txt`` for more information about these scripts.
Or use the -h option with ``configure.sh>``

If your application directory is not in the standard location
(``../apps`` or ``../apps-<version>``), then you should also
specify the location of the application directory on the command
line like:

**Version Files**. The NuttX build expects to find a version file
located in the top-level NuttX build directory. That version file
is called ``.version``. The correct version file is installed in
each versioned NuttX released. However, if you are working from an
GIT snapshot, then there will be no version file. If there is no
version file, the top-level ``Makefile`` will create a dummy
``.version`` file on the first make. This dummy version file will
contain all zeroes for version information. If that is not what
you want, they you should run the ``version.sh`` script to create
a better ``.version`` file.

You can get help information from the ``version.sh`` script using
the ``-h`` option. For example:

As an example, the following command will generate a version file
for version 6.1 using the current GIT revision number:

The ``.version`` file is also used during the build process to
create a C header file at ``include/nuttx/version.h`` that
contains the same version information. That version file may be
used by your C applications for, as an example, reporting version
information.

**Additional Configuration Steps**. The remainder of configuration
steps will be performed by
```$(TOPDIR)/Makefile`` <#topmakefile>`__ the first time the
system is built as described below.

Building NuttX
==============

**Building NuttX**. Once NuttX has been configured as described
`above <#configuringnuttx>`__, it may be built as follows:

The ``$(TOPDIR)`` directory holds:

-  The top level ```Makefile`` <#topmakefile>`__ that controls the
   NuttX build.

That directory also holds:

-  The makefile fragment ```.config`` <#boardconfigsubdirs>`__
   that describes the current configuration, and
-  The makefile fragment ```Make.defs`` <#boardconfigsubdirs>`__
   that provides customized build targets.

**Environment Variables**. The specific environmental definitions
are unique for each board but should include, as a minimum,
updates to the ``PATH`` variable to include the full path to the
architecture-specific toolchain identified in
```Make.defs`` <#boardconfigsubdirs>`__.

**First Time Make.** Additional configuration actions will be
taken the first time that system is built. These additional steps
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
