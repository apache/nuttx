xtrs README
^^^^^^^^^^^^^

TRS80 Model 3.  This port uses a vintage computer based on the Z80.
An emulator for this computer is available to run TRS80 programs on a 
linux platform (http://www.tim-mann.org/xtrs.html).

The SDCC toolchain is available from http://sdcc.sourceforge.net/.  All
testing has been performed using version 2.6.0 of the SDCC toolchain.

Configuring NuttX
^^^^^^^^^^^^^^^^^

  ostest
    This configuration performs a simple, minimal OS test using
    examples/ostest.  This can be configurated as follows:

	cd tools
	./configure.sh xtrs/ostest
	cd -
	. ./setenv.sh


  nsh
    This configuration file builds NSH (examples/nsh).  This
    configuration is not functional due to issues with use of the
    simulated serial driver (see the TODO list).

    This configuration can be selected by:

	cd tools
	./configure.sh xtrs/nsh
	cd -
	. ./setenv.sh

 pashello
    Configures to use examples/pashello for execution from FLASH
    See examples/README.txt for information about pashello.

    This configuration is not usable because the resulting binary
    is too large for the z80 address space.

    This configuration can be selected by:

	cd tools
	./configure.sh xtrs/pashello
	cd -
	. ./setenv.sh

Building the SDCC toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The SDCC toolchain is built with the standard configure/make/make install
sequence.  However, some special actions are required to generate libraries
compatible with this build.  First start with the usual steps

  download
  unpack
  cd sdcc
  ./configure

But before making, we need to apply a patch to the SDCC 2.6.0 source
so that the z80 assembler can handle long symbol names

  Apply sdcc-2.6.0-asz80-symlen.patch
  cd sdcc/device/lib

Then make the SDCC binaries

  cd sdcc
  make

and install SDCC:

  sudo make install
