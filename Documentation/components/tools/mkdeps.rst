.. _mkdeps:

===================================================================
``mkdeps.c``, ``cnvwindeps.c``, ``mkwindeps.sh``, ``mknulldeps.sh``
===================================================================

NuttX uses the GCC compiler's capabilities to create Makefile dependencies.
The program mkdeps is used to run GCC in order to create the dependencies.
If a NuttX configuration uses the GCC toolchain, its Make.defs file (see
:doc:`/components/boards`) will include a line like::

    MKDEP = $(TOPDIR)/tools/mkdeps[.exe] (See NOTE below)

If the NuttX configuration does not use a GCC compatible toolchain, then
it cannot use the dependencies and instead it uses mknulldeps.sh::

    MKDEP = $(TOPDIR)/tools/mknulldeps.sh

The mknulldeps.sh is a stub script that does essentially nothing.

mkwindeps.sh is a version that creates dependencies using the Windows
native toolchain.  That generates Windows native paths in the dependency
file.  But the mkwindeps.sh uses cnvwindeps.c to convert the Windows
paths to POSIX paths.  This adds some time to the Windows dependency
generation but is generally the best option available for that mixed
environment of Cygwin with a native Windows GCC toolchain.

mkdeps.c generates mkdeps (on Linux) or mkdeps.exe (on Windows).
However, this version is still under-development.  It works well in
the all POSIX environment or in the all Windows environment but also
does not work well in mixed POSIX environment with a Windows toolchain.
In that case, there are still issues with the conversion of things like
'c:\Program Files' to 'c:program files' by bash.  Those issues may,
eventually be solvable but for now continue to use mkwindeps.sh in
that mixed environment.
