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

Issues
------

1. As implemented now, there is a tight coupling between the nuttx/directory
   and the apps/ directory.  That should not be the case; the nuttx/ logic
   should be completely independent of apps/ logic (but not vice versa).

2. The current implementation will not work in the CONFIG_KERNEL_BUILD.
   This is because of the little proxy logic (function pcode_proxy() in the
   file pcode.c).  (a) That logic would attempt to link with P-code logic
   that resides in user space.  That will not work.  And (2) that proxy
   would be started in user mode but in the kernel address space which will
   certainly crash immediately.

The general idea to fix both of these problems is as follows:

1. Eliminate the pcode_proxy.  Instead start a P-Code execution program that
   resides in the file system.  That P-Code execution program already
   exists.  It is in apps/system/prun.  This program should be built as,
   say, an ELF binary and installed in a file system.

2. Add a configuration setting that gives the full path to where the pexec
   program is stored in the filesystem.

3. Modify the logic so that the P-Code execution program runs (instead of
   the requested program) an it received the full path the the P-Code file
   on the command line.  This might be accomplished by simply modifying the
   argv[] structure in the struct binary_s instance.

   The current start-up logic in binfmt_execmodule.c would have modified to
   handle this special start-up.  Perhaps the struct binfmt_s could be
   extended to include an exec() method that provides custom start up logic?

4. Add a task start hook to the program.  Here is where we can setup up the
   on_exit() function that will clean up after the P-Code program terminates.

There are many other smaller issues to be resolved, but those are the main
ones.

A more complex solution might include a user-space p-code daemon that
receives the P-Code path in a POSIX message and starts a P-Code interpreter
thread wholly in user space.
