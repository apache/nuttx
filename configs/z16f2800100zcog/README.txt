README.txt
==========

This is the README file for the NuttX port to the ZiLog ZNEO MCU.

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

Version 5.0.1

  On November 29, 2012, all of the z16f configurations were converted to use 5.0.1,
  but have not been verified on a running target.

  Paths were also updated that are specific to a 32-bit toolchain running on
  a 64 bit windows platform.  Change to a different toolchain, you will need
  to modify the versioning in Make.defs and setenv.sh; if you want to build
  on a different platform, you will need to change the path in the ZDS binaries
  in those same files.

Other Versions

  If you use any version of ZDS-II other than 5.0.1 or if you install ZDS-II
  at any location other than the default location, you will have to modify
  two files:  (1) configs/z16f2800100zcog/*/setenv.sh and (2)
  configs/z16f2800100zcog/*/Make.defs.  Simply edit these two files, changing
  5.0.1 to whatever.

Selecting Configurations
========================

Variations on the basic z8f162800100zcog configuration are maintained
in subdirectories.  To configure any specific configuration, do the
following steps:

   cd <nuttx-top-directory>/tools
   ./configure.sh z16f2800100zcog/<sub-directory>
   cd <nuttx-top-directory>
   make

Where <sub-directory> is the specific board configuration that you
wish to build.  The following board-specific configurations are
available.

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
    See also the README.txt file in the ostest sub-directory for information
    about using ZDS-II.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

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
      Currently, NSH failes nsh_consoleoutput().  Here is an example.
      This echo command causes the system to hang:

        nsh> echo abc

      Below is some annotated output from the debugger.  Here is the 30,000 ft view:

        - cmd_echo loops for each argv[i], 1 >=i > argc.

        - It calls:

            vtbl->output(vtbl, "%s ", argv[i])

          where the prototype for output is:

            int (*output)(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...);

        - vtbl->output maps to nsh_consoleoutput() in this case.

        - cmd_echo passes all of the arguments to output in registers.

        - nsh_consoleoutput expects all of the parameters on the stack.

        - nsh_console calls vfprintf() using bad values from the stack.

        - vfprintf crashes and never returns.

      Looks like a compiler bug to me.

      # int cmd_echo(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
      #
      # All input parameters are in registers
      #
      #   R1=00802DA0                              # vtbl
      #   R2=00000002                              # argc
      #   R3=00802D15                              # argv
      #                                            # argv[0]=00802DD7
      #                                            # argv[1]=00802DDC
      #                                            # 00802DD7="echo\0abc\0"
      #   SP=00802CDD

      0001024C 05F0 PUSHMHI <R8-R11>               # SP=00802CCD
      0001024E 4418 LD   R8,R1                     # R8=00802DA0  vtbl
      00010250 442A LD   R10,R2                    # R10=00000002 argc
      00010252 443B LD   R11,R3                    # R11=00802D15 argv
      00010254 3901 LD   R9,#%1                    # R9=00000001  arg index
      00010256 C00C JP   %10270

      00010270 A5A9 CP   R9,R10                    # Bottom of loop
      00010272 E1F2 JP   lt,%10258

      00010258 48840010 LD   R4,%10(R8)            # R4=00011156 adddress of output() method
      0001025C 4490 LD   R0,R9                     # R0=00000001 Index of argv[1]
      0001025E BC20 SLL  R0,#%2                    # R0=00000004 Offset to argv[1]
      00010260 A0B0 ADD  R0,R11                    # R0=00802D19 Address of argv[1]
      00010262 4481 LD   R1,R8                     # R1=00802DA0 vtbl address
      00010264 452200008ADB LD   R2,#%8ADB         # R2=00008ADB = "%s "
      0001026A 1203 LD   R3,(R0)                   # R3=00802DDC Value of argv[1]
      0001026C F214 CALL (R4)                      # Call vtbl->output(vtbl, "%s ", argv[i]);
                                                   # vtbl->output is nsh_consoleoutput

      # static int nsh_consoleoutput(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...)
      #
      # All parameters are in registers:
      #
      #   R1=00802DA0 vtbl address
      #   R2=00008ADB "%s "
      #   R3=00802DDC Value of argv[1]

      # First is a check if the output file is open
      #
      #  if (nsh_openifnotopen(pstate) != 0)
      #   {
      #     return ERROR;
      #   }

      00011156 0800 LINK #%0                       # SP=00802CC9, R14=00802CC9
      00011158 5C81 LD   R1,%8(FP)                 # R1=0000017F Should be value file FILE * for output
      0001115A DF96 CALL %11088                    # Call nsh_openifnotopen(), returns R0=00000000
      0001115C 9000 CP   R0,#%0
      0001115E E602 JP   z,%11164                  # Skip over error return

      00011160 30FF LD   R0,#-%1
      00011162 C007 JP   %11172

      # Then the failing call to vfprintf:
      #
      #   va_start(ap, fmt);
      #   ret = vfprintf(pstate->cn_outstream, fmt, ap);
      #   va_end(ap);
      #
      #   return ret;

      00011164 4D03 LEA  R3,%10(FP)                # R3=00802CD5 ap=GARBAGE
      00011166 5C80 LD   R0,%8(FP)                 # R0=0000017F Should be value of pstate
      00011168 48010033 LD   R1,%33(R0)            # R1=01000000 pstate->cn_outstream.  Looks suspicious
      0001116C 5CC2 LD   R2,%C(FP)                 # R2=00802DA0
      0001116E F10003FB CALL %11968                # Call vfprintf(01000000, 00802DA0, 00802CD5)
                                                   # All arguments are bad
                                                   # Does not survive call to vfprintf

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
          and misc/tools/

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
      b. You can't use setenv.sh in the native Windows environment.  Try
         scripts/setenv.bat instead.
      c. At present, the native Windows build fails at the final link stages.
         The failure is due to problems in arch/z16/src/nuttx.linkcmd that
         is autogenerated by arch/z16/src/Makefile.  The basic problem
         is the spurious spaces and and carrirage returns are generated at
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
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. The last time I tried building this configuration, there were
       a few undefined symbols from the PCODE logic.  It might require
       a little TLC to get this all working again.

    3. The native windows build has not been tried with this configuration
       but should, in principle, work (see notes for the ostest configuration
       above).

Check out any README.txt files in these <sub-directory>s.
