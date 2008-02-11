README.txt
^^^^^^^^^^

ZDS-II Compiler Versions
^^^^^^^^^^^^^^^^^^^^^^^^

If you use any version of ZDS-II other than 4.10.1 or if you install ZDS-II
at any location other than the default location, you will have to modify
two files:  (1) configs/z8encore000zco/*/setenv.sh and (2)
configs/z8encore000zco/*/Make.defs.

Configuration Subdirectories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- src/ and include/
    These directories contain common logic for all z8encore000zco
    configurations.

Variations on the basic z8encore000zco configuration are maintained
in subdirectories.  To configure any specific configuration, do the
following steps:

   cd <nuttx-top-directory>/tools
   ./configure.sh z8encore000zco/<sub-directory>
   cd <nuttx-top-directgory>
   make

Where <sub-directory> is the specific board configuration that you
wish to build.  The following board-specific configurations are
available:

- ostest
    This builds the examples/ostest application for execution from FLASH.
    See examples/README.txt for information about ostest.

Check out any README.txt files in these <sub-directory>s.
