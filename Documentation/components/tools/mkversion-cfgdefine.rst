=================================================
``mkversion.c``, ``cfgdefine.c``, ``cfgdefine.h``
=================================================

This is C file that is used to build mkversion program.  The mkversion
program is used during the initial NuttX build.

When you build NuttX there should be a version file called .version in
the top level NuttX directory (See Documentation/NuttXPortingGuide.html).
The first time you make NuttX, the top-level makefile will build the
mkversion executable from mkversion.c (using Makefile.host).  The top-level
Makefile will then execute the mkversion program to convert the
.version file in the top level directory into include/nuttx/version.h.
version.h provides version information that can be included by C files.

