README.txt
^^^^^^^^^^

ZDS-II Compiler Versions
^^^^^^^^^^^^^^^^^^^^^^^^

4.10.2
  The ZDS-II version 4.10.2 will not compile NuttX.  It reports "internal
  errors" on some of the files.  Upgrades to ZDS-II are available for
  download from the Zilog website: http://www.zilog.com/software/zds2.asp

4.11.0
  NuttX compiles correctly with the newer 4.11.0 version of the ZDS-II
  toolchain.  However, I have found a few issues:

  - The code will not run without the -reduceopt option.  There is,
    apparently, some optimization related issue.  This issue has not
    been analyzed as of this writing.

  - Not all NuttX logic will not run with the -regvars option.  There is
    at least one failure that has been reported to ZiLOG as incident 81400.

  - The Pascal add-on interpreter includes a large switch statement and
    exposes another compiler problem.  This is reported as incident 81459.

If you use any version of ZDS-II other than 4.11.0 or if you install ZDS-II
at any location other than the default location, you will have to modify
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
    This builds the examples/ostest application for execution from FLASH.
    See examples/README.txt for information about ostest.

- pashello

    Configures to use examples/pashello for execution from FLASH
    See examples/README.txt for information about pashello.

Check out any README.txt files in these <sub-directory>s.
