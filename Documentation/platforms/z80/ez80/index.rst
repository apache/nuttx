===================
Zilog eZ80 Acclaim!
===================

**Zilog eZ80Acclaim! Microcontroller**. There are four eZ80Acclaim!
ports:

-  The ZiLOG ez80f0910200kitg development kit.
-  The ZiLOG ez80f0910200zcog-d development kit.
-  The MakerLisp CPU board.
-  The Z20x DIY computing system.

All three boards are based on the eZ80F091 part and all use the Zilog
ZDS-II Windows command line tools. The development environment is either
Windows native or Cygwin or MSYS2 under Windows.

It is also possible to compile using ``clang`` and the GNU ``binutils``
toolchain. You must have a variant of ``clang`` that supports the eZ80,
and an install of ``binutils`` built with Z80 support.

``clang`` with eZ80 support is available as part of the Texas Instruments
CE 85+ unofficial `toolchain <https://ce-programming.github.io/toolchain/>`
and requires a further `patch <https://github.com/codebje/ez80-toolchain/tree/master/clang>`
to support GNU assembler syntax.

GNU ``binutils`` supports the Z80 family. It will require compilation with
appropriate configuration to enable support.

C intrinsics are also required. Some may be found in the Zilog ZDS-II
distribution, requiring some modification to build with the GNU assembler.
Additional intrinsics for 64-bit support must be supplied.
