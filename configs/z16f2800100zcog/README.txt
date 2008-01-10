README.txt
^^^^^^^^^^

ZDS-II Compiler Versions
^^^^^^^^^^^^^^^^^^^^^^^^

The ZDS-II version 4.10.2 will not compiler NuttX.  It reports "internal
errors" on some of the files.  Upgreads to ZDS-II are available for download
from the Zilog website: http://www.zilog.com/software/zds2.asp

Thusfar, I have encountered no insolvable problems with the newer 4.11.0
version of the toolchain.

If you use any version of ZDS-II other than 4.11.0, you will have to modify
two files:  (1) configs/z16f2800100zcog/*/setenv.sh and (2)
configs/z16f2800100zcog/*/Make.defs.

Configuration Subdirectories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- source/ and include/
    These directories contain common logic for all z16f2800100zcog
    configurations.

Variations on the basic z8f162800100zcog configuration are maintained
in subdirectories.  To configure any specific configuration, do the
following steps:

   cd <nuttx-top-directory>/tools
   ./configure.sh z16f2800100zcog/<sub-directory>
   cd <nuttx-top-directgory>
   make

Where <sub-directory> is the specific board configuration that you
wish to build.  The following board-specific configurations are
available:

- ostest
    This builds the examples/ostest application for execution from RAM.

 
