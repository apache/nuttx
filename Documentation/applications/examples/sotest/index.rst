=====================================
``sotest`` Shared Library Module Test
=====================================

This example builds a small shared library module test case. The test shared
library is built using the relocatable ELF format and installed in a ROMFS file
system. At run time, the shared library is installed and exercised. Requires
``CONFIG_LIBC_DLFCN``. Other configuration options:

- ``CONFIG_EXAMPLES_SOTEST_DEVMINOR`` – The minor device number of the ROMFS block
  driver. For example, the ``N`` in ``/dev/ramN``. Used for registering the RAM
  block driver that will hold the ROMFS file system containing the ELF
  executables to be tested. Default: ``0``.

- ``CONFIG_EXAMPLES_SOTEST_DEVPATH`` – The path to the ROMFS block driver device.
  This must match ``EXAMPLES_ELF_DEVMINOR``. Used for registering the RAM block
  driver that will hold the ROMFS file system containing the ELF executables to
  be tested. Default: ``/dev/ram0``.

**Notes**:

1. The arch ``Toolchain.defs`` sets ``CMODULEFLAGS`` and ``LDMODULEFLAGS``. For
   example, on ARM they contain ``-mlong-calls``, ``-r`` and the linker script
   ``libs/libc/elf/gnu-elf.ld``. A board ``scripts/Make.defs`` can add an option
   with ``+=``, for example ``LDMODULEFLAGS += -melf32lriscv``. Do not set these
   variables again with ``=``: the board then loses the options that the arch
   adds for the configuration, for example the FDPIC options.

2. This example also requires ``genromfs``. ``genromfs`` can be built as part of the
   nuttx toolchain. Or can built from the ``genromfs`` sources that can be found
   in the NuttX tools repository (``genromfs-0.5.2.tar.gz``). In any event, the
   ``PATH`` variable must include the path to the ``genromfs`` executable.

3. ELF size: The ELF files in this example are, be default, quite large because
   they include a lot of build garbage. You can greatly reduce the size of the
   ELF binaries are using the ``objcopy --strip-unneeded`` command to remove
   un-necessary information from the ELF files.

4. Simulator. You cannot use this example with the NuttX simulator on Cygwin.
   That is because the Cygwin GCC does not generate ELF file but rather some
   Windows-native binary format.

   If you really want to do this, you can create a NuttX x86 buildroot toolchain
   and use that to build the ELF executables for the ROMFS file system.
