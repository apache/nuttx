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

1. ``CFLAGS`` should be provided in ``CELFFLAGS``. RAM and FLASH memory regions may
   require long allcs. For ARM, this might be::

     CELFFLAGS = $(CFLAGS) -mlong-calls

   Similarly for C++ flags which must be provided in ``CXXELFFLAGS``.

2. Your top-level ``nuttx/Make.defs`` file must also include an appropriate
   definition, ``LDELFFLAGS``, to generate a relocatable ELF object. With GNU LD,
   this should include ``-r`` and ``-e main`` (or ``_main`` on some platforms).::

     LDELFFLAGS = -r -e main

   If you use GCC to link, you make also need to include ``-nostdlib``.

3. ELF size: The ELF files in this example are, be default, quite large because
   they include a lot of build garbage. You can greatly reduce the size of the
   ELF binaries are using the ``objcopy --strip-unneeded`` command to remove
   un-necessary information from the ELF files.

4. Simulator. You cannot use this example with the NuttX simulator on Cygwin.
   That is because the Cygwin GCC does not generate ELF file but rather some
   Windows-native binary format.

   If you really want to do this, you can create a NuttX x86 buildroot toolchain
   and use that be build the ELF executables for the ROMFS file system.

5. Linker scripts. You might also want to use a linker scripts to combine
   sections better. An example linker script is at
   ``nuttx/binfmt/elf/gnu-elf.ld``. That example might have to be tuned for
   your particular linker output to position additional sections correctly. The
   GNU LD ``LDELFFLAGS`` then might be::

     LDELFFLAGS = -r -e main -T$(TOPDIR)/binfmt/elf/gnu-elf.ld

6. When generating the ``romfs.img`` for external filesystem the image (``romfs.img``)
   needs to be manually copied to the start of the configured ``CONFIG_EXAMPLES_ELF_DEVPATH``
   using ``openocd`` or your preferred programmer.
