README
^^^^^^

This README file describes the contents of the build configurations available
for the NuttX QEMU i486 port.

Contents
^^^^^^^^

  * QEMU
  * Configurations

QEMU
^^^^

QEMU is a generic and open source machine emulator and virtualizer.  Here are
some links (which are mostly outdated by the time your read this):

  Home Page:     http://wiki.qemu.org/Main_Page
  Downloads:     http://wiki.qemu.org/Download
  Documentation: http://wiki.qemu.org/Manual
  Usage:         qemu -nographic -kernel nuttx.elf

Building QEMU:

  tar zxf qemu-0.14.0.tar.gz 
  cd qemu-0.14.0
  ./configure
  make
  make install

Cygwin build problems:

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
	2. Try building QEMU with MingGW

Configurations
^^^^^^^^^^^^^^

ostest

  The "standard" NuttX examples/ostest configuration.  This
  configuration may be selected as follows:

    cd <nuttx-directory>/tools
    ./configure.sh qemu-i486/ostest
