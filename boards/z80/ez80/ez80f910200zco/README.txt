README.txt
^^^^^^^^^^

ZDS-II Compiler Versions
^^^^^^^^^^^^^^^^^^^^^^^^

Different configurations have been build for this board using ZDS-11
Versions 4.11.0, 4.11.1, and 5.1.1  You have to check the files */Make.defs
to see how the build is configured:  Check the definitions of
ZDSVERSION (if present) and ZDSINSTALLDIR.

NOTE:  Different versions of the ZDS-II compiler may also require
different versions of .linkcmd and .zdsproj files as well.

Version 5.3.3

  As of this writing, this is the latest version available.

Version 4.11.0

  The 5.3.3 version of the ZDS-II tools is the default configured for
  all ez80 boards.  However, it is the older version 4.11.0 that this code
  has been verified against.

Version 5.1.1

  On June 22, 2011 I verified that these configurations build successfully
  with the 5.1.1 ZDS-II version.  On November 12, 2012, all of the configurations
  were converted to use 5.1.1, but have not been verified on a running target.

  The above kludge for 4.11.0 is not required with 5.1.1.

  Paths were also updated that are specific to a 32-bit toolchain running on
  a 64 bit windows platform.  Change to a different toolchain, you will need
  to modify the versioning in Make.defs; if you want to build on a different
  platform, you will need to change the path the ZDS binaries in that
  files as well as in your PATH environment variable.

Version 5.2.1

  On January 31, 2016, I tried ZDS-II version 5.2.1.  I believe that I have
  all of the changes in place to support that version.  At compile time,
  however, I encountered compiler internal errors.  I did not pursue that
  build any further.

Version 5.3.0

  I verified compilation using 5.30 on June 2, 2019.  To use this version,
  I had to make spurious modification to the implementation of gmtimer() to
  work around an internal compiler error.  I have still not verified that
  are no errors in the compiled code.

Other Versions
  If you use any version of ZDS-II other than 5.1.1, 5.2.1, or 5.3.0/3 or
  if you install ZDS-II at any location other than the default location,
  you will have to modify three files:  (1) arch/arm/z80/src/ez80/Kconfig,
  (2) boards/z80/ez80/ez80f910200zco/scripts/Make.defs and, perhaps, (3)
  arch/z80/src/ez80/Toolchain.defs.

Configurations
^^^^^^^^^^^^^^

Common Configuration Notes
--------------------------

  1. src/ and include/

     These directories contain common logic for all ez80f910200zco
     configurations.

  2. Variations on the basic ez80f910200zco configuration are maintained
     in subdirectories.  To configure any specific configuration, do the
     following steps:

       tools/configure.sh ez80f910200zco:<sub-directory>
       make

     Where <sub-directory> is the specific board configuration that you
     wish to build.  The available board-specific configurations are
     summarized in the following paragraph.

     Use configure.bat instead of configure.sh if you are building in a
     native Windows environment.

  3. This configuration uses the mconf-based configuration tool.  To
     change this configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

Configuration Subdirectories
----------------------------

  dhcpd:
    This builds the DHCP server using the examples/dhcpd application
    (for execution from FLASH.) See apps/examples/README.txt for information
    about the dhcpd example.

  httpd:
    This builds the uIP web server example using the examples/webserver application
    (for execution from FLASH). See apps/examples/README.txt for information
    about httpd.

  nsh:
    This configuration builds the NuttShell (NSH).  That code can be
    found in examples/nsh.  For more information see:  examples/nsh/README.txt
    and Documentation/NuttShell.html.

  nettest:
    This configuration is used for testing the eZ80F91 EMAC driver.  It
    builds examples/nettest.  See examples/README.txt for more information
    about nettest.

  poll:
    This configuration is also used for testing the eZ80F91 EMAC driver.  It
    builds examples/poll.  See examples/README.txt for more information
    about the poll test.

Check out any README.txt files in these <sub-directory>s.
