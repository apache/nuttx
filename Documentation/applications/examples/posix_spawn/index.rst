``posix_spawn``
===============

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

1. ``CFLAGS`` should be provided in ``CELFFLAGS``. RAM and FLASH memory regions may
    require long allcs. For ARM, this might be::

      CELFFLAGS = $(CFLAGS) -mlong-calls

    Similarly for C++ flags which must be provided in ``CXXELFFLAGS``.

2. Your top-level ``nuttx/Make.defs`` file must also include an appropriate
   definition, ``LDELFFLAGS``, to generate a relocatable ELF object. With GNU LD,
   this should include ``-r`` and ``-e main`` (or ``_main`` on some platforms).::

     LDELFFLAGS = -r -e main

   If you use GCC to link, you make also need to include ``-nostdlib``.

3. This example also requires ``genromfs``. ``genromfs`` can be build as part of the
   nuttx toolchain. Or can built from the ``genromfs`` sources that can be found
   in the NuttX tools repository (``genromfs-0.5.2.tar.gz``). In any event, the
   ``PATH`` variable must include the path to the ``genromfs`` executable.

4. ELF size: The ELF files in this example are, be default, quite large because
   they include a lot of build garbage. You can greatly reduce the size of the
   ELF binaries are using the ``objcopy --strip-unneeded`` command to remove
   un-necessary information from the ELF files.

5. Simulator. You cannot use this example with the NuttX simulator on Cygwin.
   That is because the Cygwin GCC does not generate ELF file but rather some
   Windows-native binary format.

   If you really want to do this, you can create a NuttX x86 buildroot toolchain
   and use that be build the ELF executables for the ROMFS file system.

6. Linker scripts. You might also want to use a linker scripts to combine
   sections better. An example linker script is at
   ``nuttx/binfmt/libelf/gnu-elf.ld``. That example might have to be tuned for
   your particular linker output to position additional sections correctly. The
   GNU LD ``LDELFFLAGS`` then might be::

     LDELFFLAGS = -r -e main -T$(TOPDIR)/binfmt/libelf/gnu-elf.ld
