==========
Zilog Z180
==========

**P112**. The P112 is a hobbyist single board computer based on a 16MHz
Z80182 with up to 1MB of memory, serial, parallel and diskette IO, and
realtime clock, in a 3.5-inch drive form factor. The P112 computer
originated as a commercial product of "D-X Designs Pty Ltd"[ of
Australia.

Dave Brooks was successfully funded through Kickstarter for and another
run of P112 boards in November of 2012. In addition Terry Gulczynski
makes additional P112 derivative hobbyist home brew computers.

**STATUS:** Most of the NuttX is in port for both the Z80182 and for the
P112 board. Boards from Kickstarter project will not be available,
however, until the third quarter of 2013. So it will be some time before
this port is verified on hardware. Refer to the NuttX board
`README <https://github.com/apache/nuttx/blob/master/boards/z80/z180/p112/README.txt>`__
file for further information.

The arch/z80 directories contain files to support a variety of 8-bit architectures
from ZiLOG (and spin-architectures such as the Rabbit2000).  The arch/z80/src/z180
sub-directory contains logic unique to the classic Z180 family of chips.

Files in this directory include:

``z180_head.asm``
	This is the main entry point into the Z180 program.  This includes the
	handler for the RESET, power-up interrupt vector and address zero and all
	RST interrupts.

``z180_rom.asm``
	Some architectures may have ROM located at address zero.  In this case, a
	special version of the "head" logic must be used.  This special "head"
	file is probably board-specific and, hence, belongs in the board-specific
	boards/z80/z180/<board-name>/src directory.  This file may, however, be
	used as a model for such a board-specific file.

	z180_rom.S is enabled by specifying CONFIG_LINKER_ROM_AT_0000 in the
	configuration file.

	A board specific version in the boards/z80/z180/<board-name>/src
	directory can be used by:

	1. Define CONFIG_ARCH_HAVEHEAD
	2. Add the board-specific head file, say <filename>.asm, to
	   boards/z80/z180/<board-name>/src
	3. Add a file called Make.defs in the boards/z80/z180/<board-name>/src
	   directory containing the line:  HEAD_ASRC = <file-name>.asm

``Make.defs``
	This is the standard makefile fragment that must be provided in all
	chip directories.  This fragment identifies the chip-specific file to
	be used in building libarch.

``chip.h``
	This is the standard header file that must be provided in all chip
	directories.

``z180_initialstate.c``, ``z180_copystate.c``,  ``z180_restoreusercontext.asm``, ``z180_saveusercontext.asm``
  These files implement the Z180 context switching logic

``z180_schedulesigaction.c`` and  ``z180_sigdeliver.c``
	These files implement Z180 signal handling.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
