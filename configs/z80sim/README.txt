pjrc-8051 README
^^^^^^^^^^^^^^^^

This port uses a primitive, simulated Z80 and the SDCC toolchain.

The SDCC toolchain is available from http://sdcc.sourceforge.net/.  All
testing has been performed using verison 2.6.0 of the SDDC toolchain.

Building the SDCC toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The SDCC toolchain is built with the standard configure/make/make install
sequence.  However, some special actions are required to generate libraries
compatible with this build.  First start with the usual steps

  download
  unpack
  cd sdcc
  ./configure
  make
