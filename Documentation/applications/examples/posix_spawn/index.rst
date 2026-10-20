=====================================
``posix_spawn`` posix_spawn Unit Test
=====================================

This is a simple test of the ``posix_spawn()`` API. The example derives from
``examples/elf``. As a result, these tests are built using the relocatable ELF
format installed in a ROMFS file system. At run time, the test program in the
ROMFS file system is spawned using ``posix_spawn()``.

Requires:

- ``CONFIG_BINFMT_DISABLE=n`` – Don't disable the binary loader.
- ``CONFIG_ELF=y`` – Enable ELF binary loader.
- ``CONFIG_LIBC_EXECFUNCS=y`` – Enable support for posix_spawn.
- ``CONFIG_EXECFUNCS_SYMTAB_ARRAY="g_spawn_exports"`` – The name of the symbol
  table created by the test.
- ``CONFIG_EXECFUNCS_NSYMBOLS_VAR="g_spawn_nexports"`` – Name of variable holding
  the number of symbols.
- ``CONFIG_POSIX_SPAWN_STACKSIZE=768`` – This default setting.

Test-specific configuration options:

- ``CONFIG_EXAMPLES_POSIXSPAWN_DEVMINOR`` – The minor device number of the ROMFS
  block. driver. For example, the ``N`` in ``/dev/ramN``. Used for registering the
  RAM block driver that will hold the ROMFS file system containing the ELF
  executables to be tested. Default: ``0``.

- ``CONFIG_EXAMPLES_POSIXSPAWN_DEVPATH`` – The path to the ROMFS block driver
  device. This must match ``EXAMPLES_POSIXSPAWN_DEVMINOR``. Used for registering
  the RAM block driver that will hold the ROMFS file system containing the ELF
  executables to be tested. Default: ``/dev/ram0``.

**Notes**:

1. The arch ``Toolchain.defs`` sets ``CELFFLAGS``, ``CXXELFFLAGS`` and
   ``LDELFFLAGS``. For example, on ARM they contain ``-mlong-calls``, the entry
   point ``_start`` and the linker script ``libs/libc/elf/gnu-elf.ld``. A board
   ``scripts/Make.defs`` can add an option with ``+=``, for example
   ``LDELFFLAGS += -melf32lriscv``. Do not set these variables again with
   ``=``: the board then loses the options that the arch adds for the
   configuration, for example ``crt0.o`` and the PIC and FDPIC options.

2. This example also requires ``genromfs``. ``genromfs`` can be build as part of the
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
   and use that be build the ELF executables for the ROMFS file system.
