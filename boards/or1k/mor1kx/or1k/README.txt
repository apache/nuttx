README
======

  Generic OpenRISC board, suitable for use with Qemu, for example.

Contents
========

  o STATUS
  o Pre-built or1k-elf Toolchain (newlib)
  o OpenRISC GNU tool chain from source
  o OpenOCD
  o Qemu

STATUS
======

  2018-04-23:  I have been trying to retrace all of Matt Thompson's steps to
    get or1k building running on Qemu.   I am stuck at the moment because it
    looks like there is some problem with my Kubuntu package manager.  The
    Qemu configuration gives:

      ERROR:  glib-2.22 gthread-2.0 is requred to compile QEMU.

    But it looks like to do have a later version of gthread-2.0 installed.

Pre-built or1k-elf Toolchain (newlib)
=====================================

Ref: https://openrisc.io/newlib/

Download and Install the Toolchain

You can install pre-compiled toolchains and install them on your Linux system.
We have prebuilt-toolchains for releases of the different components that are
preferably installed to /opt/toolchains/or1k-elf. You can find all releases
here. Those are current releases:

  GCC 4.9.2, Binutils 2.26, Newlib 2.3.0 (+or1k backports), GDB 7.11
  https://github.com/openrisc/newlib/releases/download/v2.3.0-1/or1k-elf_gcc4.9.3_binutils2.26_newlib2.3.0-1_gdb7.11.tgz

  GCC 5.2.0, Binutils 2.26, Newlib 2.3.0 (+or1k backports), GDB 7.11
  https://github.com/openrisc/newlib/releases/download/v2.3.0-1/or1k-elf_gcc5.2.0_binutils2.26_newlib2.3.0-1_gdb7.11.tgz

After downloading a release you can extract it anywhere in your filesystem, we
recommend to /opt/toolchains/or1k-elf/. You need to add the toolchain to your
path:

  export PATH=/opt/toolchains/or1k-elf/bin:${PATH}

OpenRISC GNU tool chain from source
===================================

ref: https://github.com/juliusbaxter/mor1kx-dev-env/wiki/OpenRISC-tool-chain-installation-guide

These instructions are as per the project's GNU tool chain page on [OpenCores]
(http://opencores.org/or1k).

What is required first is a copy of the tool chain source. There are two
repositories - one for GCC (called or1k-gcc) and one for the rest of the GNU
tools and libraries (binutils, GDB, newlib, called or1k-src). We must get the
entirety of both.

You will need to download the repositories as a zip file OR use git.
Download zip files (save into $HOME/or1k):

  or1k-src - https://github.com/openrisc/or1k-src/archive/or1k.zip
  or1k-gcc - https://github.com/openrisc/or1k-gcc/archive/or1k.zip

and unzip into the $HOME/or1k directory, making 2 directories or1k-src-or1k/
and or1k-gcc-or1k/

Rename those directories to be without the trailing -or1k so

  mv or1k-src-or1k or1k-src
  mv or1k-gcc-or1k or1k-gcc

or with git clone:

  mkdir $HOME/or1k && cd $HOME/or1k
  git clone git://github.com/openrisc/or1k-src.git
  git clone git://github.com/openrisc/or1k-gcc.git

Once the source trees are in place, we will build.

We will install the tool chain into /opt/or1k-toolchain. Make sure that
directory is writeable eg.:

  sudo mkdir /opt/or1k-toolchain
  sudo chown $USER /opt/or1k-toolchain

The following commands will build the tool chain (starting in the $HOME/or1k
directory):

  # Build the first set of tools, binutils etc.
  # NOTE: on 32-bit machines --disable-werror is needed due to an enum acting as bit mask is considered signed

  mkdir bld-or1k-src bld-or1k-gcc
  cd bld-or1k-src
  ../or1k-src/configure --target=or1k-elf --prefix=/opt/or1k-toolchain --enable-shared --disable-itcl --disable-tk --disable-tcl --disable-winsup --disable-libgui --disable-rda --disable-sid --disable-sim --disable-gdb --with-sysroot --disable-newlib --disable-libgloss --disable-werror
  make
  make install

  # Build gcc

  cd ../bld-or1k-gcc
  ../or1k-gcc/configure --target=or1k-elf --prefix=/opt/or1k-toolchain --enable-languages=c --disable-shared --disable-libssp
  make
  make install

  # build newlib and gdb (without or1ksim in this case)
  cd ../bld-or1k-src
  ../or1k-src/configure --target=or1k-elf --prefix=/opt/or1k-toolchain --enable-shared --disable-itcl --disable-tk --disable-tcl --disable-winsup --disable-libgui --disable-rda --disable-sid --enable-sim --disable-or1ksim --enable-gdb --with-sysroot --enable-newlib --enable-libgloss --disable-werror
  make
  make install

  # build gcc again, this time with newlib
  cd ../bld-or1k-gcc
  ../or1k-gcc/configure --target=or1k-elf --prefix=/opt/or1k-toolchain --enable-languages=c,c++ --disable-shared --disable-libssp --with-newlib
  make
  make install

Finally, we will want to run the following to put this path in our .bashrc file:

  echo "# OpenRISC tool chain path" >> ~/.bashrc
  echo "export PATH=$PATH:/opt/or1k-toolchain/bin"

OpenOCD
=======

ref: https://github.com/juliusbaxter/mor1kx-dev-env/wiki/OpenRISC-tool-chain-installation-guide

OpenOCD is the debug proxy we'll use to talk to the board over JTAG.

Download the source to $HOME/or1k with

  git clone https://github.com/openrisc/openOCD.git

Go into the OpenOCD directory and, the very first time, you must bootstrap it:

  ./bootstrap

Once that is finished, configure and compile it:

  ./configure --enable-usb_blaster_libftdi --enable-adv_debug_sys --enable-altera_vjtag --enable-maintainer-mode
  make

You can run make install if you like, too.

Qemu
====

The compiled ELF that works in or1ksim (https://github.com/openrisc/or1ksim).

Ref: https://github.com/openrisc/or1ksim

Or1ksim is a generic OpenRISC 1000 architecture simulator capable of emulating
OpenRISC based computer systems at the instruction level. It includes models of
a range of peripherals, allowing complete systems to be modeled.  For full
details see http://opencores.org/or1k/Or1ksim

This is a variant of the standard Or1ksim, which uses or1k as the architecture
name, rather than or32. At some stage in the future this will be merged in, so
that either architecture name is supported.

Or1k Build
----------

Or1ksim uses a standard GNU autoconf/automake installation and is designed to
be built in a separate build directory. So from the main directory, a minimal
install can be done with

  cd or1ksim
  mkdir bd
  cd bd
  ../configure
  make
  sudo make install

This will install the executables 'sim', 'profile', and 'mprofile' at
/user/local/bin and libraries at /usr/local/lib.

The UART must be enabled in sim.cfg BEFORE the build in order for the NSH
configuration to work:

   section uart
  -  enabled  = 0
  +  enabled  = 1

Qemu Build
----------

Download:

  https://www.qemu.org/download/#source

Configure and build

  Ref: https://wiki.qemu.org/Documentation/Platforms/OpenRISC

  ./configure --target-list=or1k-softmmu
  make

Then this command will get it running:

  qemu-system-or1k -kernel nuttx-or1k-sim.elf -serial stdio -nographic -monitor none
