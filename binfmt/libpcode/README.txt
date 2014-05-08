libpcode README
===============

Configuration Dependencies
--------------------------
In order to use this module, you must first install the P-Code virtual
machine.  You can get this from the Pascal package or from misc/pascal in
the GIT repository. See the README.txt file at misc/pascal for installation
instructions.  The correct location to install the P-code virtual machine is
at apps/interpreters.

Other required configuration settings:

  CONFIG_NFILE_DESCRIPTORS > 3
  CONFIG_BINFMT_DISABLE=n
  CONFIG_PCODE=y

Directory Contents
------------------
This directory holds support files for the P-Code binary format.  For other
binary formats, the library directory contains critical logic for the binary
format.  But this is not the case with the P-code binary format;  since the
binary file is interpreted, little additional support is necessary.  As a
result, this directory includes only a few files needed by the binfmt build
logic and to support unit-level testing of the P-Code binary format.

Files include in this directory include:

1. This README.txt file

2. Build support file:

   Kconfig, Make.defs

3. Unit test support files:

   hello.pas -- Pascal "Hello, World!" source file
   hello.pex -- P-Code POFF format file created by compiling hello.pas
   romfs.img -- A ROMFS filsystem image created by:

     make image
     cp hello.pex image/.
     genromfs -f romfs.img -d image -V pofftest
     rm -rf image

   romfs.h  -- a C header file containing the ROMFS file system in an
     initialized C structure.  This file was created via:

     xxd -g 1 -i romfs.img >romfs.h

     then cleaned up with an editor to conform with NuttX coding standards.
