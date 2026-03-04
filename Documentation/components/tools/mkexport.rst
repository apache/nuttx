==============================
``mkexport.sh``, ``Export.mk``
==============================

These implement part of the top-level Makefile's 'export' target. That
target will bundle up all of the NuttX libraries, header files, and the
startup object into an export-able, binary NuttX distribution. The
Export.mk is used only by the mkexport.sh script to parse out options
from the top-level Make.defs file.

USAGE: tools/mkexport.sh [-d] [-z] [-u] -t <top-dir> [-x <lib-ext>] -l "lib1 [lib2 [lib3 ...]]"

This script also depends on the environment variable MAKE which is set
in the top-level Makefile before starting mkexport.sh.  If MAKE is not
defined, the script will set it to `which make`.
