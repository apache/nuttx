==================
``elf`` ELF loader
==================

This example builds a small ELF loader test case. This includes several test
programs under ``examples/elf`` tests. These tests are built using the relocatable
ELF format and installed in a configurable file system. At run time, the file system
is mounted and each program is executed. Requires ``CONFIG_ELF``.
Two distinct types of file systems are supported: internal (included in the NuttX binary)
and external (not included in the binary). The external file systems need to be uploaded
manually. For the internal file systems the sample supports the ``romfs`` and ``cromfs``
filesystems. For the external file systems any of the provided file systems can be used
(``vfat``, ...), but also the ``romfs`` filesystem for which the image (``romfs.img``) is
automatically generated.

Configuration options:

- ``CONFIG_EXAMPLES_ELF_DEVMINOR`` – The minor device number of the ROMFS block
  driver. For example, the ``N`` in ``/dev/ramN``. Used for registering the RAM
  block driver that will hold the ROMFS file system containing the ELF
  executables to be tested. Default: ``0``.

- ``CONFIG_EXAMPLES_ELF_DEVPATH`` – The path to the ROMFS block driver device.
  This must match ``EXAMPLES_ELF_DEVMINOR``. Used for registering the RAM block
  driver that will hold the ROMFS file system containing the ELF executables to
  be tested. Default: ``/dev/ram0``.

**Notes**:

1. The arch ``Toolchain.defs`` sets ``CELFFLAGS``, ``CXXELFFLAGS`` and
   ``LDELFFLAGS``. For example, on ARM they contain ``-mlong-calls``, the entry
   point ``_start`` and the linker script ``libs/libc/elf/gnu-elf.ld``. A board
   ``scripts/Make.defs`` can add an option with ``+=``, for example
   ``LDELFFLAGS += -melf32lriscv``. Do not set these variables again with
   ``=``: the board then loses the options that the arch adds for the
   configuration, for example ``crt0.o`` and the PIC and FDPIC options.

2. ELF size: The ELF files in this example are, be default, quite large because
   they include a lot of build garbage. You can greatly reduce the size of the
   ELF binaries are using the ``objcopy --strip-unneeded`` command to remove
   un-necessary information from the ELF files.

3. Simulator. You cannot use this example with the NuttX simulator on Cygwin.
   That is because the Cygwin GCC does not generate ELF file but rather some
   Windows-native binary format.

   If you really want to do this, you can create a NuttX x86 buildroot toolchain
   and use that be build the ELF executables for the ROMFS file system.

4. When generating the ``romfs.img`` for external filesystem the image (``romfs.img``)
   needs to be manually copied to the start of the configured ``CONFIG_EXAMPLES_ELF_DEVPATH``
   using ``openocd`` or your preferred programmer.
