README
======

This README file describes the contents of the build configurations available
for the NuttX QEMU i486 port.

Contents
========

  * QEMU
    - Building QEMU
    - Cygwin Build Problems
    - Running QEMU
  * Toolchains
    - Cygwin Buildroot Toolchain
    - Buildroot Instructions
  * FAQ
  * Configurations
    - ostest
    - nsh

QEMU
====

QEMU is a generic and open source machine emulator and virtualizer.  Here are
some links (which will probably be mostly outdated by the time your read this):

  Home Page:     http://wiki.qemu.org/Main_Page
  Downloads:     http://wiki.qemu.org/Download
  Documentation: http://wiki.qemu.org/Manual
  Usage:         qemu -nographic -kernel nuttx.elf

Building QEMU
-------------

  tar zxf qemu-0.14.0.tar.gz
  cd qemu-0.14.0
  ./configure --target-list=i386-softmmu
  make
  make install

Cygwin Build Problems
---------------------

  Error:

    "gcc: The -mno-cygwin flag has been removed; use a mingw-targeted cross-compiler."

  Workaround:

    None known.  It does not seem possible to build QEMU using the Cygwin gcc.
    I tried editing configure.  Removing the following line will allow QEMU to
    configure:

      QEMU_CFLAGS="-mno-cygwin $QEMU_CFLAGS"

    However, it then fails later during the compilation phase.

  Recommendation:

    1. Google for "qemu windows download" and download some pre-built QEMU
       binaries.  I found 0.14.0 here: http://dietpc.org/windows/qemu/, or
    2. Try building QEMU with MingGW (I understand that this is difficult).

  NOTE: As of this writing, I have not been successful getting ANY pre-built
  version of QEMU to work successfully with WinXP.  The same binaries work
  fine on Vista, however.

Running QEMU
------------

  In the top-level NuttX directory:

    qemu -cpu 486 -m 2 -kernel nuttx.elf -nographic

  The -nographic option redirects COM1 to your console.  However, the -nographic
  option does not work under Cygwin.  For simple testing under Cygwin, I use

    qemu -cpu 486 -m 2 -kernel nuttx.elf -serial file:test.txt

  which will send COM1 output to the file test.txt.

Toolchains
==========

  Two target environments are supported: (1) Linux and (2) Cygwin under Windows.
  Any GCC toolchain that can produce i486 ELF binaries should work.  On Linux,
  you can probably use the installed system gcc.  But that will not work with
  Cygwin.  Why?  Because the Cygwin gcc (and probably the MinGW gcc as well) do
  not produce ELF final binaries but, rather, DOS MZ executables (i.e., .exe
  files).  Those cannot be used with QEMU.

    NOTE: It has also been reported to me that with a certain Ubuntu virtual
    installation, the native x86 did not build correct i486 code.  Other
    installations of the same vintage do not have such issues.  However,
    there is always a possibility that any GCC release will be incompatible
    with i486.  That fallback used by this person in this particular case
    was to use the Buildroot i486 cross-development tool described below.  I
    suspect that this was not necessary, but it was a simple work-around
    that allowed that person to build a work-able system.

  In any event, the PATH environment variable should be modified to point to
  the correct path to the GCC toolchain.

Cygwin Buildroot Toolchain
--------------------------

  With Cygwin the solution is to build an i486 cross-development toolchain to
  generate the i486 ELF files needed by QEMU.  The NuttX buildroot package will
  create such a toolchain.

  NOTE: As of this writing, none of the released buildroot packages support the
  i486 build.  This is only available in GIT or in any any 1.10 or later buildroot
  release.

Buildroot Instructions
----------------------

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     tools/configure.sh qemu-i486/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/i486-defconfig-4.3.3 .config

  6. make oldconfig

  7. make

  8. Make sure that the PATH variable includes the path to the newly built
     binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you
  run into problems building the toolchain for Cygwin under Windows.

FAQ
===

Q: I get the following error message, "undefined reference to '__stack_chk_fail'"
A: Add -fno-stack-protector to ARCHCPUFLAGS in you Make.defs file.  Switch the
   commenting on the following two lines in the Make.defs file:

   ARCHCPUFLAGS = -march=i486 -mtune=i486 -fno-builtin
   #ARCHCPUFLAGS = -march=i486 -mtune=i486 -fno-builtin -fno-stack-protector

Configurations
==============

Common Configuration Notes
--------------------------

  1. Each Qemu-i486 Web Server configuration is maintained in a sub-directory
     and can be selected as follow:

       tools/configure.sh qemu-i486/<subdir>

     Where <subdir> is one of the configuration sub-directories described in
     the following paragraph.

  2. These configurations use the mconf-based configuration tool.  To
     change a configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

  3. By default, all configurations assume the Linux.  This is easily
     reconfigured:

        CONFIG_HOST_LINUX=y

Configuration Sub-Directories
-----------------------------

  ostest

    The "standard" NuttX examples/ostest configuration.

  nsh

    Configures the NuttShell (nsh) located at examples/nsh.
