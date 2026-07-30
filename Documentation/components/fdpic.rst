=============
FDPIC Modules
=============

FDPIC modules are ELF shared objects that execute **in place** out of
memory-mapped flash.  The read-only segment is mapped where it already sits on
the media and never copied to RAM; only the writable segment is copied, once
per running instance.  Shared libraries work, and so does C++ with global
constructors.

The loader is ``binfmt/fdpic.c``, enabled by ``CONFIG_FDPIC``.  Modules are
built out of tree with the tooling in ``tools/fdpic``.

How it works
============

An FDPIC object has two ``PT_LOAD`` segments that may be placed independently
of one another.  Nothing in the code depends on the distance between them,
which is what lets the loader leave the read-only segment in flash and put the
writable one wherever RAM is available.

Code reaches its own data through a base register -- **r9** on ARM -- holding
the address of that object's GOT.  A function pointer is therefore not enough
to call a function: the callee needs its data base too.  FDPIC represents a
function pointer as a two-word **descriptor**:

===========  ==============================================================
Word         Contents
===========  ==============================================================
``entry``    code address, including its Thumb bit
``got``      data base to install in r9 before branching
===========  ==============================================================

Building those descriptors is most of what the loader does.  Because each one
names its own base, two objects can be in use at once with separate writable
segments, and a pointer handed to the firmware carries everything needed to
call back into the module later.

A module links against nothing.  libc and everything else are undefined
imports, resolved at load time against other loaded objects first and the
firmware's exported symbol table second.

Load sequence
-------------

``binfmt/fdpic.c``, in order:

#. Parse the ELF and program headers and check the FDPIC marker.  Anything
   else is declined quietly so another binfmt handler can try.

#. Read ``PT_DYNAMIC`` from the file before either segment is placed: the
   relocation count bounds the descriptor pool the writable allocation needs.

#. Pin the read-only segment on the media with ``XIPFSIOC_PIN``.  The returned
   pointer *is* the execution address.  The pin stops the filesystem
   relocating code that is executing.

#. Allocate the writable segment, copy ``.data`` from flash, zero ``.bss``.

#. Walk ``DT_NEEDED`` and load each shared library the same way, depth capped
   by ``FDPIC_MAX_DEPTH``.

#. Apply relocations for every object, from both ``DT_REL`` and ``DT_JMPREL``.

#. Run ``DT_INIT_ARRAY`` for every object, dependencies first.

#. Hand the scheduler a ``struct dspace_s`` holding the module's GOT, which
   ``up_initial_state()`` installs into r9.

On unload the loader runs ``DT_FINI_ARRAY`` -- module first, then its
libraries -- while their writable segments still exist, drops the pin, and
frees the allocation.  The pin release does not depend on the module calling
``munmap``: ``mm_map_destroy()`` invokes each mapping's callback during group
release, so a module that faults or is killed still drops its pin.

Differences from NXFLAT and ELF PIC
===================================

All three run position-independent code from flash on a target with no MMU,
and all three give several instances of one module a shared ``.text`` with
private ``.data``.  They differ in what a *pointer* can express and in what
the toolchain has to provide.

=========================  ==============  ==============  ============
Property                   NXFLAT          ELF PIC (XIP)   FDPIC
=========================  ==============  ==============  ============
Format                     NuttX only      ELF             ELF
Extra build tools          yes             none            linker only
Base register              r10             r10             r9
Data base per              task            task            object
Shared libraries           no              no              yes
Foreign-thread callback    no              no              yes
``.rodata`` in flash       linker script   automatic       automatic
Cores                      Cortex-M3/M4    CONFIG_PIC      Thumb-2
=========================  ==============  ==============  ============

**NXFLAT** is a NuttX-specific format.  A module imports symbols from the base
firmware but cannot export any, so shared libraries are not possible, and the
build needs ``mknxflat`` to generate a thunk, ``ldnxflat`` to link, and one of
the ``binfmt/libnxflat`` linker scripts to place the sections -- with which
script depending on how the compiler addresses ``.rodata``.

**ELF PIC** needs no extra tools at all.  With ``CONFIG_PIC`` the loader
allocates the writable sections separately and, when the filesystem answers
``FIOC_XIPBASE``, leaves the read-only ones on the media
(``libs/libc/elf/elf_load.c``); ``mps3-an547:picostest`` runs ``ostest`` that
way.  Two limits follow from having one base register per task.  A shared
object is loaded as a single allocation, because "the relative positions
between text and data must be maintained due to references to the GOT", so its
text cannot stay in flash.  And ``tcb->dspace`` is installed into ``REG_PIC``
once per task, so every object in a task shares one data base and there is no
``DT_NEEDED`` walk to load a second one.

**FDPIC** pays for its descriptors with an ``arm-uclinuxfdpiceabi`` linker, and
gets back the two things a single register cannot express:

* **A module can be called back on a thread it never created.**  A task or
  pthread the module starts inherits its data base through
  ``nxtask_dup_dspace()``, so a register is enough there.  A work-queue worker
  is not: it was created at boot and carries no module base.  A descriptor
  supplies one, which is how ``SIGEV_THREAD`` notifications from
  ``timer_create()`` and ``mq_notify()`` reach module code.

* **Two objects hold distinct data bases at once.**  A module and the
  libraries it imports each have their own writable segment and still call one
  another, so two instances of a module share a library's text while each gets
  its own copy of the library's data.

Requirements
============

**An ARM Thumb-2 core.**  The boundary is the instruction set, not the core
profile: GCC rejects FDPIC in Thumb-1 mode.

=========================  ==========================  =====
Core                       Architecture                FDPIC
=========================  ==========================  =====
Cortex-M3 / M4 / M7        ARMv7-M / ARMv7E-M          yes
Cortex-M33                 ARMv8-M Mainline            yes
Cortex-M0 / M0+ / M23      ARMv6-M / ARMv8-M Baseline  no
=========================  ==========================  =====

If the toolchain's multilib list includes Thumb-1 variants, the Thumb-1 error
appears even when the target is a Thumb-2 core.  Configure the multilib set to
exclude them.

RISC-V has no FDPIC ABI -- the psABI addendum is an unmerged RFC and no
``EI_OSABI`` value is assigned -- so a RISC-V target cannot use this loader.

**Flash that is memory mapped and executable**, exposed by a filesystem that
answers ``BIOC_XIPBASE``: :doc:`XIPFS <filesystem/xipfs>` or ROMFS.

**The linker.**  A stock ``arm-none-eabi`` GCC compiles correct FDPIC objects
for both C and C++.  What it cannot do is link them: ``arm-none-eabi-ld`` is
configured with the ``armelf`` emulation alone, and rather than failing it
marks the output ``UNIX - System V``, which the loader refuses at run time.
Only ``arm-uclinuxfdpiceabi`` carries ``armelf_linux_fdpiceabi``.

No distribution packages it, so build it -- binutils alone, about a minute::

  tools/fdpic/build-binutils.sh ~/fdpic
  export PATH=~/fdpic/toolchain/bin:$PATH

An FDPIC GCC is not needed and nothing here uses one.

GNU ld's ARM FDPIC support has open defects, notably a failure on
``R_ARM_GOTOFFFUNCDESC`` against a hidden function symbol (binutils PR31407,
PR31408, PR31409).

Configuration
-------------

``CONFIG_FDPIC`` selects ``BINFMT_LOADABLE`` and ``PIC``, and depends on
``CONFIG_ARCH_ARM``.  A working configuration also needs a symbol table for
modules to import from and a filesystem that can expose its media::

  CONFIG_FDPIC=y
  CONFIG_LIBC_EXECFUNCS=y
  CONFIG_EXECFUNCS_HAVE_SYMTAB=y
  CONFIG_EXECFUNCS_SYSTEM_SYMTAB=y
  CONFIG_FS_XIPFS=y

The options under ``if FDPIC``:

=====================  ===========  =====================================
Option                 Default      Meaning
=====================  ===========  =====================================
``FDPIC_LIBPATH``      /mnt/xipfs   Where ``DT_NEEDED`` objects are found
``FDPIC_STACKSIZE``    2048         Stack for a task made from a module
``FDPIC_ALLOW_COPY``   n            Copy text to RAM when the media
                                    cannot be mapped
=====================  ===========  =====================================

``FDPIC_ALLOW_COPY`` is for bring-up.  Left off, a module that cannot execute
in place fails to load instead of silently costing the RAM this mechanism
exists to save.

**The base firmware must reserve r9.**  It is not enough for the module to be
well behaved: a firmware routine calling back into module code arrives with
the module's GOT in r9 only if the compiler was never free to allocate that
register elsewhere.  ``arch/arm/src/common/Toolchain.defs`` adds
``--fixed-r9`` to ``ARCHCPUFLAGS`` under ``CONFIG_FDPIC`` -- not to ``CFLAGS``,
which most board ``Make.defs`` files assign with ``:=`` afterwards and would
discard.  A lost flag is silent, so check it arrived::

  make V=1 2>&1 | grep -m1 fixed-r9

.. _fdpic_tooling:

Building a module
=================

A whole module is three lines of makefile beside the source.  Taking
``apps/examples/fdpicxip/modules/qsorter.c`` as the source::

  MODULE = qsorter
  SRCS   = qsorter.c

  include /path/to/nuttx/tools/fdpic/nuttx-fdpic.mk

::

  make NUTTX_DIR=/path/to/nuttx
    CC   qsorter.c
    LD   qsorter.fdpic
  OK    qsorter.fdpic: FDPIC, entry 0x2a1, 4 imports resolved

``NUTTX_DIR`` must be a configured, **built** tree: the compile needs its
headers and the verification step needs ``libs/libc/exec_symtab.c``, which the
build generates.

C++ sources go in ``CXXSRCS``.  ``ARM_TOOLCHAIN=`` and ``FDPIC_TOOLCHAIN=``
override the two toolchain prefixes, and ``BINDNOW=`` empties the default
``-z now``.  ``make`` will run the verification step alone against a stale
``.fdpic`` and report ``OK``, so ``make clean`` when changing toolchains.

The commands it runs are::

  arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -mfdpic -fPIC -Os \
      -fno-builtin -I$NUTTX/include -D__NuttX__ -c qsorter.c -o qsorter.o

  arm-uclinuxfdpiceabi-ld -m armelf_linux_fdpiceabi -shared -z now \
      -e main -o qsorter.fdpic qsorter.o

Four flags carry weight:

* **-mfdpic** is stated rather than assumed, so a mis-set toolchain fails
  loudly instead of producing a plain ELF the loader refuses.

* **-fPIC** is not implied by ``-mfdpic`` on the bare-metal target, and
  without it the link emits ``TEXTREL``.  Text relocations cannot work
  against text executed from read-only flash.

* **-shared** preserves the ``R_ARM_FUNCDESC_VALUE`` relocations for imported
  symbols.  A PIE link with ``--unresolved-symbols=ignore-all`` appears to
  work but degrades every import to ``R_ARM_NONE``, and the module branches to
  zero on its first call into the firmware.

* **-m armelf_linux_fdpiceabi** is required: this linker supports four
  emulations and will not guess.

Verifying imports
-----------------

A module links with ``-shared``, so undefined symbols are permitted: importing
something the firmware does not export links cleanly and fails on the target
as a bare ``-ENOENT`` naming no symbol.  ``fdpic-verify.sh`` runs as part of the
default target and turns that into a build failure::

  FAIL  imports the firmware does not export:
          this_symbol_does_not_exist

The export list comes from ``CONFIG_EXECFUNCS_SYSTEM_SYMTAB``, which generates
``libs/libc/exec_symtab.c``.  Most entries there sit behind
``#if defined(CONFIG_...)`` guards, so the file has to go through the
preprocessor before its names mean anything -- read textually it offers every
symbol that *could* be exported rather than the ones that were::

  tools/fdpic/nuttx-exports.sh /path/to/nuttx   # the list
  make exports NUTTX_DIR=/path/to/nuttx         # the count

Shared libraries
----------------

A library is built the same way with a soname and no entry point, and the
module names it on the link line::

  arm-uclinuxfdpiceabi-ld ... -shared -soname libcounter.so \
      -e 0 -o libcounter.so libcounter.o

  arm-uclinuxfdpiceabi-ld ... -shared -e main \
      -o user.fdpic user.o libcounter.so

At run time the library must be present at ``CONFIG_FDPIC_LIBPATH`` under the
name in the module's ``DT_NEEDED`` entry.  Each object gets its own GOT and its
own writable data, so two modules using one library see independent library
state while sharing one copy of its code in flash.

A leaf library -- one that calls nothing outside itself -- has no
``DT_PLTGOT``, because with no external calls there is no PLT.  It still has a
GOT, and the loader falls back to ``PT_DYNAMIC`` vaddr plus memsz.

C++ modules
-----------

Three compile flags are not optional:

* ``-fno-use-cxa-atexit``.  The default registers each static object's
  destructor with ``__cxa_atexit(dtor, obj, &__dso_handle)``, and
  ``__dso_handle`` comes from ``crtbegin``, which ``-nostartfiles`` excludes;
  the link fails outright.  Turning it off also puts destructors in
  ``.fini_array``, which is what the loader walks on unload.

* ``-fno-exceptions -fno-rtti``.  Both need ``libsupc++``, which a module
  linking ``-nostdlib`` cannot reach.

C++ is compiled with ``arm-none-eabi-g++`` and linked by the FDPIC linker,
exactly as C is.

``DT_INIT_ARRAY`` runs after relocations and before the module is entered,
``DT_FINI_ARRAY`` on unload, both per object and in dependency order.
Constructors run in the task that called the loader, not the task that will
run the module, so a constructor sees the loading task's identity.

Getting this wrong is quiet: a loader that ignores ``DT_INIT_ARRAY`` still
loads a C++ module, resolves every symbol and runs it, with every global left
zero.

Calling back into a module
==========================

A module's function pointer is the address of a descriptor in its **writable
segment**.  Firmware that stores one and later branches to it jumps into RAM
data.  ``fdpic_callback()`` resolves such a pointer to a code address,
deciding whether it is a descriptor by testing r9: a module task runs with its
GOT there, a kernel task with zero.

It is applied at every entry point a module can reach today: ``qsort``,
``bsearch``, ``pthread_create``, ``signal``, ``sigaction``,
``task_create``/``task_create_with_stack``, ``task_spawn``, ``pthread_once``,
``scandir`` (its filter), and ``mq_notify``/``timer_create`` with
``SIGEV_THREAD``.

A new entry point that takes a module callback must resolve it too, under
three rules:

* **Resolve once, in the innermost common routine.**  Resolving twice treats a
  code address as a descriptor.  ``qsort`` recurses, so its wrapper resolves
  and the recursive body does not; ``signal()`` does not resolve because
  ``nxsig_action()`` does it for both paths; ``scandir`` resolves its filter
  but not the comparison function it hands to ``qsort``.

* **Exclude sentinel values by hand.**  ``fdpic_callback()`` declines to
  dereference NULL and nothing else.  ``sigaction`` excludes ``SIG_IGN``,
  ``SIG_DFL``, ``SIG_HOLD`` and ``SIG_ERR`` -- the integers 0, 1, 2 and -1.

* **A callback on a shared thread needs its base installed.**  A
  ``SIGEV_THREAD`` notification runs on a work-queue worker with no module
  base, so resolving the entry is not enough.  Capture the base at
  registration with ``fdpic_base()``, in the module's context, and install it
  around the call with ``fdpic_invoke()``.

Everywhere else the callback runs in a task that inherited the module's
D-Space, so only the code address needs resolving.

Reference
=========

Object layout
-------------

The FDPIC marker is ``e_ident[EI_OSABI] == ELFOSABI_ARM_FDPIC`` (65), which
``readelf -h`` shows as "OS/ABI: ARM FDPIC".  It is *not* in ``e_flags``,
which read an ordinary ``0x5000000, Version5 EABI``.  Validate the OS/ABI byte
and nothing else.

A linked module has the shape execute-in-place needs with no linker script::

  LOAD  vaddr 0x00000000  R E   .text .rodata .hash .dynsym .dynstr
  LOAD  vaddr 0x000012b4  RW    .dynamic .got .data .bss
  DYNAMIC                       DT_PLTGOT -> .got

``.rodata`` lands in the RX segment on its own, resolved PC-relative or
GOT-indirect.  That matters: in the RW segment it would be copied to RAM with
``.data`` and most of the saving would evaporate, silently and with everything
still working.

``DT_PLTGOT`` gives the GOT base, which is the value r9 must hold.

Relocations
-----------

The static link resolves ``R_ARM_GOT_BREL`` and ``R_ARM_GOTFUNCDESC`` into the
GOT already, so only three types carry work into a linked module.
``R_ARM_NONE`` is accepted and skipped.

``R_ARM_RELATIVE``
  An address needing its segment's base added.

``R_ARM_FUNCDESC_VALUE``
  A descriptor the linker has laid out, for the loader to fill in with
  {resolved code, GOT base}.  This is what a *call* to an imported function
  produces, and where the firmware's symbol table is consulted.

``R_ARM_FUNCDESC``
  A pointer to a descriptor that does not exist yet, which the loader
  manufactures from a pool behind the writable segment.  This is what *taking
  the address* of an imported function produces -- a different thing from
  calling one, and both can appear for the same symbol.  Unlike NXFLAT, a
  pointer to a ``static`` function needs no workaround.

Constants, from binutils ``include/elf/arm.h`` and now in
``arch/arm/include/elf.h``: ``R_ARM_GOTFUNCDESC`` 161,
``R_ARM_GOTOFFFUNCDESC`` 162, ``R_ARM_FUNCDESC`` 163,
``R_ARM_FUNCDESC_VALUE`` 164.

Relocation tables
-----------------

Which table an imported function's descriptor lands in is the linker's
decision.  With ``-z now`` imports stay in ``DT_REL``; without it they go to
``DT_JMPREL`` and a module may have no ``DT_REL`` at all.

The loader binds **both**, eagerly.  There is no lazy resolver, so an unwalked
table would not be deferred work -- the descriptors would stay unrelocated
until the module branched through one.  Nothing is lost by binding early: a
module carries a handful of relocations.

The two are not walked identically.  In ``DT_REL`` the word being overwritten
is the addend and is added to the resolved symbol value.  In ``DT_JMPREL`` it
is the lazy-binding bootstrap -- the address of the entry's own PLT stub, with
a GOT half of ``-1`` -- and the binder overwrites the descriptor outright.

``DT_PLTREL`` must say ``REL``.  Nothing reads ``RELA``, whose entries are
twelve bytes rather than eight, so an object declaring that layout is refused
rather than misread.

``.rofixup`` is skipped.  It is the self-relocation list a *static*
executable's ``crt0`` walks to find its own GOT; a module links ``-shared
-nostartfiles``, so no ``crt0`` runs, and the section holds only the GOT
self-pointer that the loader supplies from r9 instead.  Everything else
load-dependent is in ``.rel.dyn``/``.rel.plt``, and a PIC object cannot keep an
absolute pointer in its read-only segment.

binfmt integration
------------------

The loader registers a ``struct binfmt_s``, so modules load through the
standard dispatch alongside ELF and NXFLAT.  Two details of that contract are
easy to get wrong:

* ``binp->picbase`` is a ``struct dspace_s *``, not an address.
  ``up_initial_state()`` dereferences ``->region``, so a bare GOT puts a kernel
  address in r9 and panics.

* ``binp->stacksize`` must be set, or ``binfmt_execmodule`` rejects the load
  with ``-EINVAL``.

``binfmt`` reports any loader error as ``-ENOENT``, because at the dispatch
layer "refused it" and "not my format" are the same answer.  The reason
reaches the syslog only, so do not read ``-ENOENT`` from ``exec`` as "file not
found".

Testing
=======

``apps/testing/fs/xipfs`` stages modules out of the filesystem and asserts on
them.  ``xipfs_test fdpic`` runs that part in about twenty seconds, and
``xipfs_test reject`` the loader's refusal paths.

It covers what fails silently: constructors in dependency order, per-instance
data, manufactured descriptors, both relocation tables, the GOT fallback for a
library with no PLT, and every callback entry point.  Each check has been
verified against a deliberate one-line breakage of the loader.

**A run that ends without its summary line is a failure, not a hang.**  These
tests call into modules whose relocations must be correct; when they are not,
the module branches to an unrelocated address and the HardFault takes the
system down mid-test, because on Cortex-M a HardFault is system-wide rather
than per-task.

The module sources are in ``apps/examples/fdpicxip/modules``: ``libshape.cpp``
for the init/fini arrays, ``funcdesc.c`` for the descriptor pool, ``lazymod.c``
for ``DT_JMPREL``, ``libcounter.c`` for a leaf library with no ``DT_PLTGOT``.
An opt-in ``make regen`` there rebuilds the headers both apps embed; it is
never part of the application build, so the tree stays buildable with a plain
toolchain.

``apps/examples/fdpicxip`` is a runnable demonstration of the same modules --
see :doc:`/applications/examples/fdpicxip/index`.
