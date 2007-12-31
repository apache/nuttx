z80sim README
^^^^^^^^^^^^^

This port uses a primitive, emulated Z80 and the SDCC toolchain.
The instruction set emulator can be found in the NuttX CVS at
http://nuttx.cvs.sourceforge.net/nuttx/misc/sims/z80sim

The SDCC toolchain is available from http://sdcc.sourceforge.net/.  All
testing has been performed using verison 2.6.0 of the SDDC toolchain.

Configuring NuttX
^^^^^^^^^^^^^^^^^

  defconfig
    The default configuration file, defconfig, performs a simple,
    minimal OS test using examples/ostest.  This can be
    configurated as follows:

	cd tools
	./configure.sh z80sim
	cd -
	. ./setenv.sh


  nshconfig
    This configuration file builds NSH (examples/nsh).

    This alternative configurations can be selected by:

	(Seleted the default configuration as show above)
	cp config/z80sim/nshconfig .config


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
