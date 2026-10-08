=================
``nuttx-cc.sh``
=================

A compiler wrapper for programs that are built outside the NuttX tree.
``make export`` installs it in the export package as ``bin/nuttx-cc`` (C)
and ``bin/nuttx-c++`` (C++), and writes the flags of the exported
configuration to ``scripts/nuttx-cc.conf``.

Use the wrapper like a normal cross compiler. It finds the package from its
own location, so you can move the package to another directory or another
computer. The compiler named in the package (for example
``aarch64-none-elf-gcc``) must be on ``PATH`` there. To use a different one,
set ``NUTTX_CC`` or ``NUTTX_CXX``.

.. code-block:: console

   $ make export
   $ tar xzf nuttx-export-*.tar.gz
   $ E=$PWD/nuttx-export-13.0.1
   $ $E/bin/nuttx-cc -O2 -o hello hello.c

An autoconf package needs only the compiler and a host triple that
``config.sub`` knows:

.. code-block:: console

   $ ./configure --host=aarch64-none-elf CC=$E/bin/nuttx-cc
   $ make

Copy the result to the target, for example to the directory that a hostfs or
v9fs mount shows, and run it from NSH.

What the wrapper does
=====================

- It compiles with the exported headers only (``-nostdinc``), plus the
  compiler's own headers (``stddef.h``, ``arm_neon.h``, ...). A header that
  NuttX does not have is reported missing. It is not taken from the C
  library of the toolchain.
- It uses the CPU, ABI and define options of the configuration. It does not
  use the warning options (for example ``-Werror``) of the NuttX build.
- When it links, it adds ``startup/crt0.o``, ``scripts/gnu-elf.ld`` and, in a
  kernel build (``CONFIG_BUILD_KERNEL``), the user-space libraries of the
  package.
- In a kernel build, a link with an undefined symbol fails. A kernel-build
  program is a relocatable (``-r``) ELF, and a relocatable link does not
  report undefined symbols by itself. Without this check, every configure
  test for a function would pass.

Limits
======

- Only the GNU toolchain and kernel builds (``qemu-armv8a:knsh``) are
  tested.
- In other builds the wrapper adds no libraries and does not check for
  undefined symbols.
