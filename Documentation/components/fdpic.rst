.. _fdpic:

=============
FDPIC Modules
=============

Overview
========

An FDPIC module is an ELF shared object whose read-only and writable
segments are placed independently of one another.  NuttX uses the
read-only segment where it already lies on the media and never copies
it; only the writable segment is copied to RAM, once per running
instance.  A module's code and ``.rodata`` therefore cost no RAM at all,
and several instances of one module share them.

FDPIC is not a separate binary format and has no loader of its own.  An
object announces itself in its OS/ABI byte,
``e_ident[EI_OSABI] == ELFOSABI_ARM_FDPIC`` (65), which ``readelf -h``
reports as *OS/ABI: ARM FDPIC*, and the ELF loader takes it from there.
Everything else -- ``exec()``, ``posix_spawn()``, ``dlopen()``, the
symbol table -- is the ordinary ELF path.

What FDPIC adds over the position independent ELF support already in the
tree is a function pointer that carries its own data base.  That is what
lets a module be called back on a thread it did not create, and what
lets a module and the libraries it uses hold distinct data bases at the
same time.

Function descriptors
--------------------

Code reaches its own data through a base register -- **r9** on ARM --
holding the address of that object's GOT.  Because code and data are
placed independently, a bare code address is not enough to call a
function: the callee needs its data base too.  FDPIC therefore
represents a function pointer as a two word *descriptor*:

===========  ==============================================================
Word         Contents
===========  ==============================================================
``entry``    Code address, including its Thumb bit
``got``      Data base to install in the PIC base register before
             branching
===========  ==============================================================

Building those descriptors is most of what relocation does.  Because
each one names its own base, a pointer handed to the base firmware
carries everything needed to call back into the module later, from any
thread.

A module links against nothing.  libc and everything else are undefined
imports, resolved at load time against the globally registered symbols
first, then any shared libraries the module names, then the symbol table
``exec()`` supplied.

Placement
---------

The loader asks the filesystem where the file lies on its media.  Two
mechanisms exist and they are not interchangeable:

* ``XIPFSIOC_PIN`` is for a filesystem that can move a file's blocks.  It
  returns an address together with a pin that holds the extent still, and
  the pin is given back with ``XIPFSIOC_UNPIN`` when the module is
  unloaded.  :doc:`XIPFS <filesystem/xipfs>` is the one in tree.

* ``FIOC_XIPBASE`` is for a filesystem whose layout never changes, which
  has nothing to hold and answers with a bare address.  ROMFS and TMPFS
  are those.

The pin is asked for first, because a filesystem that needs one is not
safe without it.  The loader asks for a pin only if it can hold one,
which is the flat build, or the pin would stay for ever.

A filesystem that answers neither is still usable.  The loader then
copies the text to RAM, as it does for any other module.  The module
loses the shared text and the flash saving, but it runs.

The writable segment is allocated and copied per instance, and a pool of
function descriptors is reserved behind it for the relocations that ask
the loader to manufacture one.  When the task starts,
``up_initial_state()`` installs the object's data base -- ``DT_PLTGOT``,
or the GOT immediately after ``PT_DYNAMIC`` in an object with no
imports -- into the PIC base register.

Shared libraries
----------------

A module may name shared libraries in ``DT_NEEDED``.  Each is loaded
during relocation by calling ``dlopen()`` on the name, and the module's
undefined symbols are then bound against that library's exports.  This
requires ``CONFIG_LIBC_DLFCN``; without it, a module carrying
``DT_NEEDED`` is refused, because there is no way to bring in what it
asks for.  ``CONFIG_LIBC_ELF_MAXNEEDED`` caps how many one module may
name.

Because ``dlopen()`` does the work, libraries are found the way it finds
them: an absolute path is used as given, and a bare name is searched for
along ``LD_LIBRARY_PATH``, which needs ``CONFIG_LIBC_ENVPATH`` and is
seeded from ``CONFIG_LDPATH_INITIAL``.

A library lands in the module registry, which holds one instance per
name, so its data is shared by everything that opens it.  A module
started with ``exec()`` is different: that path loads a fresh copy each
time, so two running instances of one module have separate data while
sharing one copy of the text in flash.

Comparison with NXFLAT and PIC ELF
==================================

All three run position independent code from flash on a target with no
MMU, and all three give several instances of one module a shared
``.text`` with private ``.data``.  They differ in what a *pointer* can
express and in what the toolchain has to provide.

=========================  ==============  ==============  =============
Property                   NXFLAT          PIC ELF         FDPIC
=========================  ==============  ==============  =============
Format                     NuttX only      ELF             ELF
Extra build tools          yes             none            assembler and
                                                           linker
Data base per              task            task            object
Shared libraries           no              no              yes
Foreign-thread callback    no              no              yes
Instruction set            ARM, Thumb-2    unrestricted    Thumb-2 only
=========================  ==============  ==============  =============

:ref:`NXFLAT <nxflat>` is a NuttX-specific format.  A module imports
symbols from the base firmware but cannot export any, so shared
libraries are not possible, and the build needs ``mknxflat`` to generate
a thunk, ``ldnxflat`` to link, and one of the ``binfmt/libnxflat`` linker
scripts to place the sections.

**PIC ELF** needs no extra tools.  With ``CONFIG_PIC`` the ELF loader
allocates the writable sections separately and, when the filesystem
answers ``FIOC_XIPBASE``, leaves the read-only ones on the media.  Two
limits follow from having one base register per task: a shared object is
loaded as a single allocation, because the distance between its text and
its data is compiled into it, and the data base is installed once per
task, so every object in a task shares one.

**FDPIC** pays for its descriptors with an ``arm-uclinuxfdpiceabi``
assembler and linker, and gets back the two things a single register
cannot express.  A task or pthread that a module starts inherits the
module's D-Space, so a register would be enough there; a work queue
worker was created at boot and carries no module base, and a descriptor
supplies one, which is how ``SIGEV_THREAD`` notifications reach module
code.

Requirements
============

**An ARM Thumb-2 core.**  The boundary is the instruction set, not the
core profile: GCC rejects FDPIC in Thumb-1 mode.

=========================  ==========================  =====
Core                       Architecture                FDPIC
=========================  ==========================  =====
Cortex-M3 / M4 / M7        ARMv7-M / ARMv7E-M          yes
Cortex-M33                 ARMv8-M Mainline            yes
Cortex-M0 / M0+ / M23      ARMv6-M / ARMv8-M Baseline  no
=========================  ==========================  =====

RISC-V has no FDPIC ABI -- the psABI addendum is an unmerged proposal and
no ``EI_OSABI`` value is assigned -- so a RISC-V target cannot use this.

**Flash that is memory mapped and executable**, exposed by a filesystem
that answers ``XIPFSIOC_PIN`` or ``FIOC_XIPBASE``.  This is what gives
execute in place.  Without it the module still loads, but from RAM.

**An FDPIC assembler and linker.**  A stock ``arm-none-eabi`` GCC
compiles correct FDPIC code for both C and C++, but the assembler has to
be in FDPIC mode to accept the relocations that code produces, and only
``arm-uclinuxfdpiceabi`` binutils carry the ``armelf_linux_fdpiceabi``
emulation the link needs.  ``arm-none-eabi-ld``, rather than failing,
marks its output *UNIX - System V*, which the loader will not treat as
FDPIC.

No distribution packages that target, so build binutils for it -- which
takes about a minute and needs nothing else::

  configure --target=arm-uclinuxfdpiceabi --prefix=$HOME/fdpic \
      --disable-nls --disable-werror
  make && make install
  export PATH=$HOME/fdpic/bin:$PATH

An FDPIC GCC is not needed.

**The base firmware must reserve r9.**  It is not enough for the module
to be well behaved: a firmware routine calling back into module code
arrives with the module's data base in r9 only if the compiler was never
free to allocate that register elsewhere.  ``CONFIG_FDPIC`` selects
``CONFIG_PIC``, under which ``arch/arm/src/common/Toolchain.defs`` adds
``--fixed-r9``; see :ref:`nxflat` for why it goes into ``ARCHCFLAGS``
rather than ``CFLAGS`` and how to check that it arrived.

Configuration
=============

``CONFIG_FDPIC`` lives under ``CONFIG_ELF``.  A working configuration
also needs a symbol table for modules to import from and a filesystem
that can expose its media::

  CONFIG_ELF=y
  CONFIG_FDPIC=y
  CONFIG_LIBC_EXECFUNCS=y
  CONFIG_EXECFUNCS_HAVE_SYMTAB=y
  CONFIG_EXECFUNCS_SYSTEM_SYMTAB=y
  CONFIG_FS_XIPFS=y

Shared libraries need three more, the last two so that a library can be
named rather than spelled out as an absolute path::

  CONFIG_LIBC_DLFCN=y
  CONFIG_LIBC_ENVPATH=y
  CONFIG_LDPATH_INITIAL="/mnt/xipfs"

``CONFIG_ELF_STACKSIZE`` gives the stack a module runs with.  A module
that needs a different one can export an ``nx_stacksize`` symbol, which
the loader prefers when present.

Building a module
=================

The steps below are what the build does; ``tools/fdpic`` wraps them so that a
module is three lines of makefile.  See :doc:`/components/tools/fdpic`.

Three steps: compile to assembly with the stock compiler, assemble with
the FDPIC assembler, link with the FDPIC linker::

  arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfdpic -fPIC -Os \
      -fno-builtin -D__NuttX__ -I$NUTTX/include -S mod.c -o mod.s

  arm-uclinuxfdpiceabi-as --fdpic -mthumb -mcpu=cortex-m3 \
      mod.s -o mod.o

  arm-uclinuxfdpiceabi-ld -m armelf_linux_fdpiceabi -shared -z now \
      -e main -o mod.fdpic mod.o

The detour through assembly is what makes this work with any toolchain.
Whether ``arm-none-eabi-gcc -c`` can assemble FDPIC code itself depends
on the release: newer ones pass ``--fdpic`` down to the assembler, older
ones do not and fail with *"Relocation supported only in FDPIC mode"*.
Assembling separately never depends on that.  Note that the assembler's
option is ``--fdpic``, not ``-mfdpic``.

Five flags carry weight:

* ``-mfdpic`` is stated rather than assumed, so a mis-set toolchain fails
  loudly instead of producing a plain ELF the loader will not recognize.

* ``-fPIC`` is not implied by ``-mfdpic`` on a bare-metal target, and
  without it the link emits ``TEXTREL``.  Text relocations cannot work
  against text executed from read-only flash.

* ``-shared`` preserves the ``R_ARM_FUNCDESC_VALUE`` relocations for
  imported symbols.  A PIE link with ``--unresolved-symbols=ignore-all``
  appears to work but degrades every import to ``R_ARM_NONE``, and the
  module branches to zero on its first call into the firmware.

* ``-m armelf_linux_fdpiceabi`` is required: this linker supports several
  emulations and will not guess.

* ``-e main`` names the entry point.  There is no ``crt0``; the module is
  entered directly.

A module links with ``-shared``, so importing something the firmware does
not export links cleanly and fails only on the target.  Checking the
module's undefined symbols against the generated
``libs/libc/exec_symtab.c`` is worth doing as part of the module build.

Building a shared library
-------------------------

A library is built the same way, with a soname and no entry point, and
the module names it on its link line::

  arm-uclinuxfdpiceabi-ld -m armelf_linux_fdpiceabi -shared -z now \
      -e 0 -soname libfoo.so -o libfoo.so libfoo.o

  arm-uclinuxfdpiceabi-ld -m armelf_linux_fdpiceabi -shared -z now \
      -e main -o mod.fdpic mod.o libfoo.so

At run time the library must be reachable under its soname along
``LD_LIBRARY_PATH``.

Calling back into a module
==========================

A module's function pointer is the address of a descriptor in its
writable segment.  Firmware that stores one and later branches to it
would jump into RAM data, so an entry point that accepts a callback from
a module has to resolve the descriptor first.  ``CONFIG_FDPIC`` makes
these do so:

``qsort``, ``bsearch``, ``pthread_create``, ``signal``/``sigaction``,
``task_create``/``task_create_with_stack``, ``task_spawn``,
``pthread_once``, ``scandir``, and ``mq_notify``/``timer_create`` with
``SIGEV_THREAD``.

Whether a pointer is a descriptor is decided by reading the PIC base
register: a module's task runs with its data base there, a firmware task
with zero, so a kernel caller is unaffected.

A new entry point that takes a module callback must resolve it too, under
three rules:

* **Resolve once, in the innermost common routine.**  Resolving twice
  treats a code address as a descriptor.  ``qsort()`` recurses, so its
  public entry resolves and the recursive body does not; ``signal()``
  does not resolve because ``nxsig_action()`` does it for both paths;
  ``scandir()`` resolves its filter but not the comparison function it
  hands to ``qsort()``.

* **Exclude sentinel values by hand.**  ``fdpic_callback()`` declines to
  dereference NULL and nothing else.  ``sigaction()`` excludes
  ``SIG_IGN``, ``SIG_DFL``, ``SIG_HOLD`` and ``SIG_ERR`` -- the integers
  0, 1, 2 and -1.

* **A callback on a shared thread needs its base installed.**  A
  ``SIGEV_THREAD`` notification runs on a work queue worker that carries
  no module base, so resolving the entry is not enough.  Capture the base
  at registration with ``fdpic_base()``, in the module's own context, and
  install it around the call with ``fdpic_invoke()``.

Everywhere else the callback runs in a task that inherited the module's
D-Space, so only the code address needs resolving.

Limitations
===========

**Constructors run on the loading task, not the module's own.**
``DT_INIT_ARRAY`` runs at the end of the load and ``DT_FINI_ARRAY`` at
unload, which needs ``CONFIG_BINFMT_CONSTRUCTORS``.  Both are entered
through ``fdpic_invoke()`` with the object's own data base, so a global
object reaches its own storage; but the task they run on is whichever one
called the loader, so a constructor that reads task-local state -- its own
pid, its environment -- sees that task's, not the one that will run
``main()``.

A library named in ``DT_NEEDED`` is constructed before the module that
needs it, because the module's own relocation is what opens it, and
destroyed after, at the last ``dlclose()``.  Since the library is one
instance, its constructors run once however many modules name it.

Reference
=========

Object layout
-------------

A linked module already has the layout execute in place needs, with no
linker script::

  LOAD  vaddr 0x00000000  R E   .text .rodata .hash .dynsym .dynstr
  LOAD  vaddr 0x00001244  RW    .dynamic .got .data .bss
  DYNAMIC                       DT_PLTGOT -> .got

``.rodata`` lands in the read-only segment on its own, reached PC
relative or GOT indirect.  That matters: in the writable segment it would
be copied to RAM with ``.data``, and most of the saving would evaporate
silently, with everything still working.

The FDPIC marker is the OS/ABI byte alone.  ``e_flags`` reads as an
ordinary ``0x5000000, Version5 EABI``.

Relocations
-----------

The static link resolves ``R_ARM_GOT_BREL`` and ``R_ARM_GOTFUNCDESC``
into the GOT already, so only three types carry work into a linked
module.

``R_ARM_RELATIVE``
  An address needing its segment's base added.

``R_ARM_FUNCDESC_VALUE``
  A descriptor the linker has laid out, for the loader to fill in.  This
  is what a *call* to an imported function produces.  When the symbol
  resolves to a function in another FDPIC object, both words are copied
  from that object's own descriptor, so the callee runs with its own data
  base; otherwise the entry is the resolved address and the base is this
  object's.

``R_ARM_FUNCDESC``
  A pointer to a descriptor that does not exist yet, which the loader
  manufactures from the pool behind the writable segment.  This is what
  *taking the address* of a function produces -- a different thing from
  calling one, and both can appear for the same symbol.

Constants, from binutils ``include/elf/arm.h`` and mirrored in
``arch/arm/include/elf.h``: ``R_ARM_GOTFUNCDESC`` 161,
``R_ARM_GOTOFFFUNCDESC`` 162, ``R_ARM_FUNCDESC`` 163,
``R_ARM_FUNCDESC_VALUE`` 164.

Which table an imported function's descriptor lands in is the linker's
decision: with ``-z now`` imports stay in ``DT_REL``, without it they go
to ``DT_JMPREL``.  Both are bound eagerly, so either link works, but the
two are not walked identically.  In ``DT_REL`` the word being overwritten
is the addend and is added to the resolved value; in ``DT_JMPREL`` it is
the lazy binding bootstrap and the descriptor is overwritten outright.

``.rofixup`` is skipped.  It is the self-relocation list a *static*
executable's ``crt0`` walks to find its own GOT; a module has no
``crt0``, and the loader supplies the data base instead.

Exported functions
------------------

A function exported by a module or library is published to ``dlsym()``
as a descriptor rather than a code address, taken from the same pool, so
that an FDPIC caller can branch through what it gets back.
