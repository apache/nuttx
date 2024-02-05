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


The ``arch/z80`` directories contain files to support a variety of 8-bit architectures
from ZiLOG (and spin-architectures such as the Rabbit2000).  The arch/z80/src/z80
sub-directory contains logic unique to the classic Z80 chip.

Files in this directory include:

``z80_head.asm``
    This is the main entry point into the Z80 program.  This includes the
    handler for the RESET, power-up interrupt vector and address zero and all
    RST interrupts.

``z80_rom.asm``
    Some architectures may have ROM located at address zero.  In this case, a
    special version of the "head" logic must be used.  This special "head"
    file is probably board-specific and, hence, belongs in the board-specific
    boards/z80/z80/<board-name>/src directory.  This file may, however, be
    used as a model for such a board-specific file.

    z80_rom.S is enabled by specifying CONFIG_LINKER_ROM_AT_0000 in the
    configuration file.

    A board specific version in the boards/z80/z80/<board-name>/src directory
    can be used by:

    1. Define CONFIG_ARCH_HAVEHEAD
    2. Add the board-specific head file, say <filename>.asm, to
       boards/z80/z80/<board-name>/src
    3. Add a file called Make.defs in the boards/z80/z80/<board-name>/src
       directory containing the line:  HEAD_ASRC = <file-name>.asm

``Make.defs``
    This is the standard makefile fragment that must be provided in all
    chip directories.  This fragment identifies the chip-specific file to
    be used in building libarch.

``chip.h``
    This is the standard header file that must be provided in all chip
    directories.

``z80_initialstate.c``, ``z80_copystate.c``,  ``z80_restoreusercontext.asm``, and ``z80_saveusercontext.asm``
    These files implement the Z80 context switching logic

``z80_schedulesigaction.c`` and  ``z80_sigdeliver.c``
    These files implement Z80 signal handling.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
