================================================
``mkconfig.c``, ``cfgdefine.c``, ``cfgdefine.h``
================================================

These are C files that are used to build mkconfig program.  The mkconfig
program is used during the initial NuttX build.

When you configure NuttX, you will copy a configuration file called .config
in the top level NuttX directory (See :doc:`/components/boards` or
Documentation/NuttXPortingGuide.html).  The first time you make NuttX,
the top-level makefile will build the mkconfig executable from mkconfig.c
(using Makefile.host).  The top-level Makefile will then execute the mkconfig
program to convert the .config file in the top level directory into
include/nuttx/config.h.  config.h is a another version of the NuttX
configuration that can be included by C files.
