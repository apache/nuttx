=========
Zilog Z80
=========

**Z80 Instruction Set Simulator**. This port uses the
`SDCC <http://sdcc.sourceforge.net/>`__ toolchain under Linux or Cygwin
(verified using version 2.6.0). This port has been verified using only a
Z80 instruction simulator called z80sim.

**STATUS:** This port is complete and stable to the extent that it can
be tested using an instruction set simulator. Refer to the NuttX board
`README <https://github.com/apache/nuttx/blob/master/boards/z80/z80/z80sim/README.txt>`__
file for further information.

**XTRS: TRS-80 Model I/III/4/4P Emulator for Unix**. A very similar Z80
port is available for `XTRS <http://www.tim-mann.org/xtrs.html>`__, the
TRS-80 Model I/III/4/4P Emulator for Unix. That port also uses the
`SDCC <http://sdcc.sourceforge.net/>`__ toolchain under Linux or Cygwin
(verified using version 2.6.0).

**STATUS:** Basically the same as for the Z80 instruction set simulator.
This port was contributed by Jacques Pelletier. Refer to the NuttX board
`README <https://bitbucket.org/patacongo/obsoleted/src/master/configs/xtrs/README.txt>`__
file for further information.

**NOTE:** This port was removed from the NuttX source tree on
2017-11-24. It was removed because (1) it is unfinished, unverified, and
unsupported, and (2) the TRS-80 simulation is a sub-optimal platform.i
That platform includes a 16-bit ROM image and only a 48Kb RAM space for
NuttX. The removed board support is still available in the ``Obsoleted``
repository if anyone would ever like to resurrect it.

   \* A highly modified `buildroot <http://buildroot.uclibc.org/>`__ is
   available that may be used to build a NuttX-compatible ELF toolchain
   under Linux or Cygwin. Configurations are available in that buildroot
   to support ARM, Cortex-M3, avr, m68k, m68hc11, m68hc12, m9s12,
   blackfin, m32c, h8, and SuperH ports.

