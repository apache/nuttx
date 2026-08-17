.. _kernel-modules:

==================================
Kernel Modules vs Shared Libraries
==================================
 
Kernel Modules
==============

NuttX has had support for Kernel Modules for some time.
A kernel module allows you to extend the functionality of the OS
at runtime by installing ELF modules into the kernel.
A kernel module might be used, to example, to load device drivers
into RAM at runtime.

Here are some general properties of kernel modules:

* The first kernel module is loaded into the kernel address space
  by ``insmod()`` using only a symbol table exported by the OS.
* Kernel modules can also (optionally) export a symbol table.
  Such symbol tables are remembered and will be used by ``insmod()``
  to resolved undefined symbols in subsequently loaded modules.
* This requires dependency checking logic: A module that imports symbols
  from another module must be added after the modules that it depends upon.
  And a module exports a symbol may not be removed while there are such
  inter-module dependencies in place.
  A module that imports symbols from another module must be removed before
  the module that exports the symbol can be removed.
* There is a (non-standard) OS interface ``modsym()`` that will allow kernel
  logic to look up symbols within a kernel module.
* Handles are used at the kernel module interfaces: ``insmod()`` returns
  a handle to the module data structure. That handle can subsequently be used
  to retrieve symbols with ``modsym()``.
* ``rmmod()`` uses the handle to remove the module.
* ``modhandle()`` is also available for backward compatibility:
  Given the assigned module name, you can use this to retrieve
  the module handle at any time.
* There is a test case at ``nuttx-apps/examples/module``.

In the FLAT build, ELF kernel modules are simply loaded into RAM and linked
with the base firmware. But things get a little more complex with PROTECTED
and KERNEL builds.

In those case, there are separate address spaces for the kernel
and for applications.
Kernel modules are only loaded in the kernel address space and, hence,
are not available to applications.


Shared Libraries
================

A shared library is another software module with these properties:
The ``.text`` address space is accessible to all applications.
The ``.data`` and ``.bss`` address space is in the same address space
as the application that uses the shared library.
So, they are shared in the sense that the ``.text`` is shared.


Building a Shared Library
=========================

Applications can declare a loadable shared library with the Android.mk-style
helper in ``nuttx-apps``. Include ``Make.defs``, set the module name and its C
or C++ sources, then include ``BUILD_SHARED_LIBRARY``::

  include $(APPDIR)/Make.defs

  LOCAL_PATH := $(CURDIR)
  LOCAL_MODULE := my-new-lib
  LOCAL_SRC_FILES := my-source-file.cpp another-file.cpp

  include $(BUILD_SHARED_LIBRARY)

The build writes the relocatable ELF library to
``$(BINDIR)/$(LOCAL_MODULE)``. Set ``LOCAL_MODULE_FILENAME`` when the runtime
interface requires another name, such as ``libmy-new-lib.so``.

``LOCAL_CFLAGS``, ``LOCAL_CXXFLAGS``, ``LOCAL_LDFLAGS``, and ``LOCAL_LDLIBS``
add module-specific options. The selected architecture must provide the NuttX
module compiler and linker flags. Exported symbols must have default
visibility for ``dlsym()`` or use by another library.


FLAT Build
==========

In the FLAT build environment, there is only one address space.
So what is the difference between a kernel module and a shared library
in this case? Certainly a kernel module meets all of the requirements
of a shared library in that environment.
In this case kernel modules really only differ from shared
libraries in their usage semantics:

For the FLAT build, the standard ``include/dlfcn.h`` interfaces are
implemented as a thin wrapper around the same module library that the kernel
module support uses:

* ``dlopen()`` loads the library, or takes an additional reference on it if
  it is already loaded, and returns a handle to it. See
  `Opening a Library More Than Once`_.
* ``dlclose()`` releases one reference. The library is unloaded, as
  ``rmmod()`` would, only when the last handle is closed.
* ``dlsym()`` maps to ``modsym()``.
* ``dlerror()`` is only a stub at the present time.

There is a shared library test case at ``nuttx-apps/examples/sotest``.


PROTECTED Build
===============

The PROTECTED build is equivalent to the FLAT build except that there are
two address spaces: The kernel address space and the user address space.
But all applications still share the same user address spaces.

As a result ``.text`` along with ``.data`` and ``.bss`` are naturally shared.
This requires using two copies of the module logic: One residing in kernel
address space and using the kernel symbol table and one residing in user space
using the user space symbol table.
The first provides only kernel module support; the second only PROTECTED mode
shared library support.

This is accomplished by breaking the kernel module logic in two components
with OS kernel module interfaces in sched/module but with a sharable module
library at libc/modlib.

The shared library functions no longer call the kernel module logic but rather
implement their one top-level management logic using the lower-level routines
in the module library.

The user space copy of the module library keeps the name of each loaded
library whenever ``CONFIG_LIBC_DLFCN`` is enabled, since the name is the only
way to tell that a library is already loaded. This costs ``NAME_MAX`` bytes
per loaded library, but it makes ``dlopen()`` behave exactly as it does in the
FLAT build.


Opening a Library More Than Once
================================

In the FLAT and PROTECTED builds, ``dlopen()`` of a library that is already
loaded does not load a second copy and does not fail. It returns a handle to
the library that is already loaded and takes an additional reference on it.
Each successful ``dlopen()`` must be matched by a ``dlclose()``; the library
is unloaded only when the last handle is closed. Up to 255 handles may be
outstanding on one library; beyond that ``dlopen()`` fails with ``EMFILE``.

Some consequences worth keeping in mind:

* A library is identified by the *basename* of the path passed to
  ``dlopen()``. Two files with the same basename in different directories
  are treated as the same library, and the second ``dlopen()`` will return
  the first one.
* There is only one instance of the library's ``.data`` and ``.bss``. Global
  and static data are shared by every user of the library, and by every task
  group in the system.
* Constructors in ``.init_array`` run once, when the library is first loaded,
  and destructors in ``.fini_array`` run once, when the last handle is
  closed. They do not run per ``dlopen()``/``dlclose()`` pair.
* Symbols obtained with ``dlsym()`` remain valid until the last handle is
  closed, not until the caller's own handle is closed.

Kernel modules deliberately behave differently: ``insmod()`` fails with
``EEXIST`` if a module of that name is already installed, and ``rmmod()``
removes it immediately. A kernel module is a singleton and is not reference
counted.


Better FLAT and PROTECTED Mode Shared Libraries
===============================================

A better implementation of shared libraries in the FLAT and PROTECTED builds
would, however, have a separate copy of the ``.bss`` and ``.data`` region
for each NuttX task group.

A task group is the moral equivalent of a Unix process.
That is how a shared library would have to work in uClinux, for example.
But that would be a substantial effort! For example, since each
``.bss``/``.data`` would lie at a different physical address,
the ``.text`` section logic would need support
Position-Independent-Data (PID).
Embedded PID support, however, is pretty much broken on all current GCC
implementations. See NxFlat Compatibility Problem.

Perhaps the xFLAT work that I did for uClinux shared libraries could be
ported to Nuttx: `xFLAT Web Page <https://sourceforge.net/projects/xflat/>`_.

For more information about task groups see :ref:`nuttx-tasking`
or :ref:`tasks-vs-threads`.

The current implementation also assumes that all firmware resides in base
FLASH and hence is fully linked prior to loading any modules or shared
libraries. There is no support for loading programs into RAM and binding
them to symbols exported by modules or shared libraries.

This extension would, however, not be so difficult for the FLAT build;
it would be a simple matter of integrating the exported module symbol
tables into the symbol lookup.
That is already done in ``sched/module`` files, but not yet in the
very similar ``binfmt/libelf`` files.

In the PROTECTED build, this would require some special start-up logic
in the user address space as the initial steps of the newly started task.
Some kind of dynamic loader, such as ``ld.so``, would have to integrate
with ``crt0`` logic to automatically bind user space tasks to shared libraries
as they are loaded into memory before the programs ``main()`` function
is called.


KERNEL Build
============

The KERNEL build, however, is a completely different creature.
In that build, the kernel and each process has its own address space.

This means that a shared library in the kernel build has to be considerably
more complex: In order to be shared, the ``.text`` portion of the module
(1) must lie in a single shared memory region accessible from all processes
and (2) built for Position-Independent-Code (PIC) operation since
it must execute from an arbitrarily different virtual address in each process
address space.

The ``.data``/``.bss`` portion of the module must be allocated
in the user address space of each process, but either:

1. ``.data``/``.bss`` section must lie at a consistent virtual address
   so that it can be referenced from the one copy of the ``.text`` in
   the shared memory region, OR 
2. The ``.text`` section logic must support Position-Independent-Data (PID).
   The latter approach provides for a simpler build, but embedded PID support
   is pretty much broken on all current GCC implementations.
   See :ref:`nxflat` Compatibility Problem.

Some kind of dynamic loader, such as ``ld.so``, would have to integrate with
``crt0`` logic to automatically bind processes to shared libraries as they
are loaded into memory before the programs ``main()`` logic is called.

.. note:: There is not yet any shared library support in the KERNEL build mode.
          This would be quite a large effort and not on the plan of record
          at the present time.
          ``dlopen()`` always fails and returns ``NULL`` in the KERNEL build,
          so none of the reference counting behaviour described above applies
          there.
