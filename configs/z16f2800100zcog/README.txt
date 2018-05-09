README.txt
==========

This is the README file for the NuttX port to the ZiLog ZNEO MCU.

- Console output is on UART0.

- NOTE: My board has a 20MHz crystal, but I have heard of other boards with
  18.432MHz crystals.  If you board has a diff crystal installed, then
  modify the system frequency in include/board.h.

Contents
========

  - ZDS-II Compiler Versions
  - Patches
  - Serial Console
  - Selecting Configurations
  - Configuration Sub-directories

ZDS-II Compiler Versions
========================

Version 4.10.2

  The ZDS-II version 4.10.2 will not compile NuttX.  It reports "internal
  errors" on some of the files.  Upgrades to ZDS-II are available for
  download from the Zilog website: http://www.zilog.com/software/zds2.asp

Version 4.11.0

  NuttX compiles correctly with the newer 4.11.0 version of the ZDS-II
  toolchain.  However, I have found a few issues:

  - The code will not run without the -reduceopt option.  There is,
    apparently, some optimization related issue.  This issue has not
    been analyzed as of this writing.

  - Not all NuttX logic will not run with the -regvars option.  There is
    at least one failure that has been reported to ZiLOG as incident 81400.

  - The Pascal add-on interpreter includes a large switch statement and
    exposes another compiler problem.  This is reported as incident 81459.

Version 4.11.1

  As of this writing (30 September 2010), the latest release of ZDS-II for the
  ZNEO is 4.11.1.  It is unknown if this release includes fixes for incidents
  81400 and 81459 or not.  It is unknown if the code will run without -reduceopt
  either. (Basically, it compiles with 4.11.1, but is untested with that version).

Version 4.12.0

  Never tested

Version 5.0.0

  Never tested

Version 5.0.1

  On November 29, 2012, all of the z16f configurations were converted to use 5.0.1,
  but have not been verified on a running target.

  Paths were also updated that are specific to a 32-bit toolchain running on
  a 64 bit windows platform.  Change to a different toolchain, you will need
  to modify the versioning in the Make.defs file; if you want to build
  on a different platform, you will need to change the path in the ZDS binaries
  in those that file and your PATH environment variable.

Other Versions

  If you use any version of ZDS-II other than 5.0.1 or if you install ZDS-II
  at any location other than the default location, you will have to modify
  configs/z16f2800100zcog/*/Make.defs.  Simply edit that file, changing
  5.0.1 to whatever.  Also make sure that your PATH environment variable
  includes th correct path to the toolchain.

Patches
=======

A bug has been found in the ZDS-II toolchain version 5.0.1.  a patch is
available to work around the bug.  A summary of the nature the bug and
instructions for applying the patch follow.

Parameters are passed different to variadic functions (i.e., functions
that accept a varying number of parameters) than to regular functions.  For
most functions, parameters are passed in registers, beginning with R1.  But
for variadic functions, all parameters must be passed on the stack.

The logic works correctly for global functions, local functions, and most
function pointers.  It does not work correctly for the case where a variadic
function point is included within a structure.  In that case, the caller
inappropriately passes the parameters in registers; the receiver will
attempt to recover the parameters from the stack and a failure then follows.

This bug prevents the use of NSH with the ZNEO.  However, a patch has been
developed that works around the problem.  That patch can be found at
configs/z16f2800100zcog/tools/zneo-zdsii-5_0_1-variadic-func-fix.patch.  In
that directory is also a bash script that will apply that patch for you.

The patch would be applied when NuttX is configured as follows:

  tools/configure.sh z16f2800100zcog/nsh
  dopatch.sh
  make

The patch can also be removed with:

  dopatch.sh -R

See the section "Selecting Configurations" below.

Serial Console
==============

By default, console output is on UART1 which corresponds to the DB9
connector labelled CONSOLE.

UART1 is also available on JP2:

  MCU PIN     GPIO  JP2
  Pin 86 TXD1 PD5   JP2 Pin 26
  Pin 87 RXD1 PD4   JP2 Pin 27
  Vcc               JP2 Pin 59
  GND               JP2 Pins 19, 39, 46, 48, 56

Selecting Configurations
========================

Variations on the basic z8f162800100zcog configuration are maintained
in subdirectories.  To configure any specific configuration, do the
following steps:

   tools/configure.sh z16f2800100zcog/<sub-directory>
   make

Where <sub-directory> is the specific board configuration that you
wish to build.  The following board-specific configurations are
available.  You may also need to apply a path to NuttX before making.
Please refer the section "Patches" above"

Configuration Sub-directories
=============================

source/ and include/
--------------------

  These directories contain common logic for all z16f2800100zcog
  configurations.

nsh
---
  nsh:
    This configuration directory will built the NuttShell (NSH).  See
    the NSH user manual in the documents directory (or online at nuttx.org).
    See also the README.txt file in the nsh sub-directory for information
    about using ZDS-II.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this configuration assumes that you are using the
       Cygwin environment on Windows.  An option is to use the native
       CMD.exe window build as described in the top-level README.txt
       file.  To set up that configuration:

       -CONFIG_WINDOWS_CYGWIN=y
       +CONFIG_WINDOWS_NATIVE=y

       And after configuring, make sure that CONFIG_APPS_DIR uses
       the back slash character.  For example:

        CONFIG_APPS_DIR="..\apps"

      NOTES:

      a. If you need to change the toolchain path used in Make.defs, you
         will need to use the short 8.3 filenames to avoid spaces.  On my
         PC, C:\PROGRA~1\ is is C:\Program Files\ and C:\PROGRA~2\ is
         C:\Program Files (x86)\
      b. I have not tried to use this configuration with the native
         Windows build, but I would expect the same issues as is listed
         for the ostest configuration..

   STATUS:

     1. Note that you must apply the ZNEO patch if you are using ZDS-II 5.0.1.
        See the README.txt file in the parent directory for more information.
        The configuration will run correctly with the patch applied.

ostest
------

    This builds the examples/ostest application for execution from FLASH.
    See the README.txt file in the ostest sub-directory for information
    about using ZDS-II.  See also apps/examples/README.txt for information
    about ostest.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this configuration assumes that you are using the
       Cygwin environment on Windows.  An option is to use the native
       CMD.exe window build as described in the top-level README.txt
       file.  To set up that configuration:

       -CONFIG_WINDOWS_CYGWIN=y
       +CONFIG_WINDOWS_NATIVE=y

       And after configuring, make sure that CONFIG_APPS_DIR uses
       the back slash character.  For example:

        CONFIG_APPS_DIR="..\apps"

      NOTES:

      a. If you need to change the toolchain path used in Make.defs, you
         will need to use the short 8.3 filenames to avoid spaces.  On my
         PC, C:\PROGRA~1\ is is C:\Program Files\ and C:\PROGRA~2\ is
         C:\Program Files (x86)\
      b. At present, the native Windows build fails at the final link stages.
         The failure is due to problems in arch/z16/src/nuttx.linkcmd that
         is autogenerated by arch/z16/src/Makefile.  The basic problem
         is the spurious spaces and and carriage returns are generated at
         the end of the lines after a line continuation (\ ^M).  If these
         trailing bad characters are manually eliminated, then the build
         will succeed on the next try.

pashello
--------

    Configures to use examples/pashello for execution from FLASH
    See examples/README.txt for information about pashello.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. The last time I tried building this configuration, there were
       a few undefined symbols from the PCODE logic.  It might require
       a little TLC to get this all working again.

    3. The native windows build has not been tried with this configuration
       but should, in principle, work (see notes for the ostest configuration
       above).

Check out any README.txt files in these <sub-directory>s.
