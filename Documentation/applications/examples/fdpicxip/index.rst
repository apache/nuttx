================================================
``fdpicxip`` FDPIC Executed In Place from XIPFS
================================================

Writes FDPIC modules into a :doc:`XIPFS </components/filesystem/xipfs>` volume
at run time, the way a download would, and runs them. Their text is executed
directly out of flash and shared between concurrent instances; each instance
gets its own data.

This is the FDPIC counterpart of :doc:`nxflatxip <../nxflatxip/index>`, kept
separate from it because the two module formats have little in common beyond
running in place, and one program demonstrating both would demonstrate
neither clearly. What FDPIC adds over NXFLAT is shared libraries and C++ with
global constructors — see :doc:`/components/fdpic`.

It demonstrates; it does not assert. The assertions live in the ``fdpic`` and
``reject`` sections of :doc:`the xipfs test suite </applications/testing/xipfs/index>`.

Usage::

    fdpicxip [qsort | solib | cxx | jmprel]

``qsort``
  The simplest case the loader has: one self-contained module, no library
  behind it, run as two concurrent instances. This is what runs with no
  argument::

    nsh> fdpicxip
    === FDPIC module executed in place from xipfs ===

    staged qsorter (2768 bytes)
    extent: block 98 x1, size 2768, flash addr 0x10162000

    spawning two concurrent instances...

    [while running] pins on the shared text = 2

    [inst 1] 11 21 31 41 51 61 71 81
    [inst 2] 12 22 32 42 52 62 72 82

    [after exit]    pins on the shared text = 0

  One copy of the text is mapped in flash and pinned once per instance, which
  is what stops the defragmenter relocating code that is executing. The two
  rows differ because each instance has its own data; shared data would show
  up as interleaved or corrupted output.

``solib``
  Adds a shared library, so two objects are mapped and each instance gets its
  own copy of *both* objects' data. If the library's state were shared, the
  two totals would interleave instead of each reaching ``seed * 3``.

``cxx``
  The same in C++, which additionally requires each object's global
  constructors to have run, in dependency order, before ``main``, and its
  destructors to run on unload with that instance's own state.

``jmprel``
  A module whose imports are all bound through ``DT_JMPREL`` rather than
  ``DT_REL``. A loader that ignored that table would load the module happily
  and fault only when it first called out.

The modules are embedded in the image as C headers.  Their sources are in
``apps/examples/fdpicxip/modules``, along with the makefile that rebuilds
every header from them::

  make -C apps/examples/fdpicxip/modules regen NUTTX_DIR=/path/to/nuttx

That is never part of the application build, because linking a module needs
``arm-uclinuxfdpiceabi`` binutils, which building NuttX does not.  See
:ref:`fdpic_tooling`.

Configuration
=============

``CONFIG_EXAMPLES_FDPICXIP``
  Enable the example. Depends on ``CONFIG_FS_XIPFS``, ``CONFIG_FDPIC`` and
  ``CONFIG_LIBC_EXECFUNCS``.

``CONFIG_EXAMPLES_FDPICXIP_MOUNTPT``
  Where the xipfs volume is mounted. Defaults to ``/mnt/xipfs``.
