libpcode README
===============

Configuration Dependencies
--------------------------
In order to use this module, you must first install the P-Code virtual
machine.  You can get this from the Pascal package or from misc/pascal in
the GIT repository. See the README.txt file at misc/pascal for installation
instructions.  The correct location to install the P-code virtual machine is
at apps/interpreters.

Other required configuration settings:

  CONFIG_NFILE_DESCRIPTORS > 3
  CONFIG_BINFMT_DISABLE=n
  CONFIG_PCODE=y

Directory Contents
------------------
This directory holds support files for the P-Code binary format.  For other
binary formats, the library directory contains critical logic for the binary
format.  But this is not the case with the P-code binary format;  since the
binary file is interpreted, little additional support is necessary.  As a
result, this directory includes only a few files needed by the binfmt build
logic and to support unit-level testing of the P-Code binary format.

Files include in this directory include:

1. This README.txt file

2. Build support file:

   Kconfig, Make.defs

3. Unit test support files:

   hello.pas -- Pascal "Hello, World!" source file
   hello.pex -- P-Code POFF format file created by compiling hello.pas
   romfs.img -- A ROMFS filsystem image created by:

     make image
     cp hello.pex image/.
     genromfs -f romfs.img -d image -V pofftest
     rm -rf image

   romfs.h  -- a C header file containing the ROMFS file system in an
     initialized C structure.  This file was created via:

     xxd -g 1 -i romfs.img >romfs.h

     then cleaned up with an editor to conform with NuttX coding standards.

Test Configuration
------------------
Here is a simple test configuration using the NuttX simulator:

1. Install the sim/nsh configuration:

   cd tools
   ./configure.sh sim/nsh
   cd ..

2. Install p-code virtual machine as described above.

3. Modify the configuration using 'make menuconfig'.  Change the following
   selections:

   This enables general BINFMT support:

     CONFIG_DEBUG_BINFMT=y
     CONFIG_BINFMT_EXEPATH=y

   This enables building of the P-Code virtual machine:

     CONFIG_INTERPRETERS_PCODE=y

   And the P-Code runtime support:

     CONFIG_SYSTEM_PRUN=y

   This enables building the PCODE binary format

     CONFIG_PCODE=y
     CONFIG_PCODE_PRIORITY=100
     CONFIG_PCODE_STACKSIZE=2048

   This enables building and mount a test filesystem:

     CONFIG_PCODE_TEST_FS=y
     CONFIG_PCODE_TEST_DEVMINOR=3
     CONFIG_PCODE_TEST_DEVPATH="/dev/ram3"
     CONFIG_PCODE_TEST_MOUNTPOINT="/bin"

   Debug options can also be enabled with:

    CONFIG_DEBUG=y
    CONFIG_DEBUG_BINFMT=y
    CONFIG_DEBUG_VERBOSE=y

4. In lieu of a a real test application, this Quick'n'Dirty patch can be used
   to initialize the P-Code binary format:

   @@ -115,6 +115,7 @@ const struct symtab_s CONFIG_EXECFUNCS_SYMTAB[1];
    /****************************************************************************
     * Name: nsh_main
     ****************************************************************************/
   +int pcode_initialize(void);

    int nsh_main(int argc, char *argv[])
    {
   @@ -143,6 +144,7 @@ int nsh_main(int argc, char *argv[])
         exitval = 1;
       }
    #endif
   +(void)pcode_initialize();

    /* Initialize the NSH library */

5. Then after building nuttx.exe you should be able to run the P-Code hello
   world example like:

   nsh> hello.pex
