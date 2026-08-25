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
  unloaded.  :doc:`XIPFS </os/filesystem/xipfs>` is the one in tree.

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

A module may name shared libraries in ``DT_NEEDED``.  The loader loads
each one during relocation, through ``libelf_insert()`` as ``dlopen()``
does, and then binds the module's undefined symbols against that
library's exports.  ``CONFIG_LIBC_ELF_MAXDEPEND`` caps how many one module
may name, and a module that names more is refused.  With
``CONFIG_ARCH_ADDRENV`` a module carrying ``DT_NEEDED`` is refused, because
the library would not be in the address space of the program.

Libraries are found the way ``dlopen()`` finds them: an absolute path is
used as given, and a bare name is searched for along ``LD_LIBRARY_PATH``,
which needs ``CONFIG_LIBC_ENVPATH`` and is seeded from
``CONFIG_LDPATH_INITIAL``.

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
Extra build tools          yes             none            linker
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
linker, and gets back the two things a single register
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

**An FDPIC linker.**  A stock ``arm-none-eabi`` GCC compiles correct
FDPIC code for both C and C++.  Its assembler accepts the relocations that
code produces in FDPIC mode only: GCC 14 and later select that mode for
``-mfdpic``, and for an older GCC the build passes ``-Wa,--fdpic``.  Only
``arm-uclinuxfdpiceabi`` binutils carry the ``armelf_linux_fdpiceabi``
emulation that the link needs, and ``arm-none-eabi-ld`` rejects it.

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

``crt0`` runs the constructors of a module only with
``CONFIG_HAVE_CXXINITIALIZE``.

Shared libraries need three more.  A library resolves its own imports
against the table of ``CONFIG_LIBC_ELF_HAVE_SYMTAB``, and the last two let
a library be named rather than spelled out as an absolute path::

  CONFIG_LIBC_ELF_HAVE_SYMTAB=y
  CONFIG_LIBC_ENVPATH=y
  CONFIG_LDPATH_INITIAL="/mnt/xipfs"

``dlopen()`` needs ``CONFIG_LIBC_DLFCN`` too.

Only the make build generates the table of
``CONFIG_EXECFUNCS_SYSTEM_SYMTAB``.  A CMake configuration needs a symbol
table from elsewhere.

``CONFIG_ELF_STACKSIZE`` gives the stack a module runs with.  A module
that needs a different one can export an ``nx_stacksize`` symbol, which
the loader prefers when present.

Building a module
=================

Select ``CONFIG_FDPIC`` and a module is built by the ordinary in-tree ELF
build: the same ``MODULE = m`` in the same application Makefile as any
other, and the same ``crt0``.  Nothing else is needed.

The flags come from ``arch/arm/src/common/Toolchain.defs``.  A board whose
``scripts/Make.defs`` sets ``LDELFFLAGS`` after it includes that file
replaces them, and its modules do not link as FDPIC.  Remove that setting
from the board.

What the build does differently is add compiler flags and use a
different linker::

  arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -mfdpic -fPIC -Wa,--fdpic \
      -fno-optimize-sibling-calls -Os -fno-builtin -D__NuttX__ \
      -I$NUTTX/include -c mod.c -o mod.o

  arm-uclinuxfdpiceabi-ld -m armelf_linux_fdpiceabi -shared -z now \
      -e _start -T $NUTTX/libs/libc/elf/gnu-elf.ld \
      -o mod $NUTTX/arch/arm/src/crt0.o mod.o

Only the link needs the FDPIC toolchain.  The stock compiler emits correct
FDPIC objects for both C and C++, its assembler included once it is in
FDPIC mode.  That linker is in the NuttX CI image;
``tools/ci/docker/linux/Dockerfile`` shows how it is built.
``FDPIC_CROSSDEV`` names a different prefix, and the build says so if it
is missing.

Seven flags carry weight:

* ``-mfdpic`` is stated rather than assumed, so a mis-set toolchain fails
  loudly instead of producing a plain ELF the loader will not recognize.

* ``-fPIC`` is not implied by ``-mfdpic`` on a bare-metal target, and
  without it the link emits ``TEXTREL``.  Text relocations cannot work
  against text executed from read-only flash.

* ``-Wa,--fdpic`` puts the assembler in FDPIC mode, which GCC before 14
  does not do for ``-mfdpic``.  Without it the assembler stops with
  "Relocation supported only in FDPIC mode".

* ``-fno-optimize-sibling-calls`` keeps GCC from making tail calls.  With
  ``-mlong-calls``, GCC turns a call in tail position into a branch to the
  function descriptor rather than through it, and the module faults.  Only
  an optimized build makes tail calls, so a build without optimization
  hides the problem.

* ``-shared`` preserves the ``R_ARM_FUNCDESC_VALUE`` relocations for
  imported symbols.  A PIE link with ``--unresolved-symbols=ignore-all``
  appears to work but degrades every import to ``R_ARM_NONE``, and the
  module branches to zero on its first call into the firmware.

* ``-m armelf_linux_fdpiceabi`` is required: this linker supports several
  emulations and will not guess.

* ``-e _start`` names the entry point.  ``crt0.c`` is the module's own
  start-up file, the one every other module uses: it walks ``.init_array``
  and then calls ``main``.  A shared library is never entered, so it is
  linked without ``crt0``.

A module links with ``-shared``, so importing something the firmware does
not export links cleanly and fails only on the target.  Checking the
undefined symbols of the module against the generated
``libs/libc/exec_symtab.c`` catches that at build time.

When the toolchain cannot make an FDPIC module, the build stops with one
of these:

``CONFIG_FDPIC needs arm-uclinuxfdpiceabi-ld, which is not on PATH``
  The FDPIC linker is missing.  The CMake build stops with the same
  message when it configures.

``unrecognised emulation mode: armelf_linux_fdpiceabi``
  ``FDPIC_CROSSDEV`` names a linker without the FDPIC emulation, such as
  ``arm-none-eabi-``.

``Relocation supported only in FDPIC mode``
  The assembler is not in FDPIC mode.  A compile that does not use the
  module flags has lost ``-Wa,--fdpic``.

``unknown argument: '-mfdpic'``
  The compiler cannot make FDPIC code.  Clang is one.

Building a shared library
-------------------------

A library is an ordinary shared library of the application build,
``BUILD_SHARED_LIBRARY`` in make and ``DYNLIB y`` in CMake, with a
soname::

  LOCAL_MODULE := libfoo
  LOCAL_MODULE_FILENAME := libfoo.so
  LOCAL_SRC_FILES := libfoo.c
  LOCAL_LDFLAGS := -soname libfoo.so

  include $(BUILD_SHARED_LIBRARY)

The module flags hide every symbol, so a library marks what it exports
with ``visibility_default``.  A module that uses the library names it on
its link line, which records the soname in ``DT_NEEDED``; in a module
Makefile, ``LDLIBS`` for the module target does that.
``apps/examples/fdpicxip/modules`` has both kinds.

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

Examples and tests
==================

``apps/examples/fdpicxip`` writes modules into XIPFS at run time and runs
them.  ``fdpicxip qsort`` runs one module twice, with one copy of its
text; ``solib`` adds a shared library; ``cxx`` does the same in C++ and
checks that the constructors ran; ``jmprel`` runs a module whose imports
are bound through ``DT_JMPREL``.

``apps/testing/fs/xipfs`` asserts what the demo shows.  ``xipfs_test
fdpic`` loads modules and checks the loader, and ``xipfs_test reject``
checks that a malformed or unloadable module is refused.  Both apps build
their modules from ``apps/examples/fdpicxip/modules`` with the module
flags of the tree.

``pimoroni-pico-2-plus:xipfs-fdpic`` is a board configuration with all of
this.  Without hardware, ``mps2-an500:xipfs`` runs it under QEMU with
these added::

  CONFIG_ELF=y
  CONFIG_FDPIC=y
  CONFIG_EXAMPLES_FDPICXIP=y
  CONFIG_LIBC_EXECFUNCS=y
  CONFIG_EXECFUNCS_HAVE_SYMTAB=y
  CONFIG_EXECFUNCS_SYSTEM_SYMTAB=y
  CONFIG_LIBC_DLFCN=y
  CONFIG_LIBC_ELF_HAVE_SYMTAB=y
  CONFIG_LIBC_ENVPATH=y
  CONFIG_LDPATH_INITIAL="/mnt/xipfs"
  CONFIG_SIG_EVTHREAD=y
  CONFIG_SCHED_HPWORK=y
  CONFIG_INIT_STACKSIZE=16384

and with ``CONFIG_DISABLE_POSIX_TIMERS``, ``CONFIG_PROFILE_ALL``,
``CONFIG_PROFILE_MINI`` and ``CONFIG_SYSTEM_GPROF`` off.

Constructors and destructors
============================

A module that ``exec()`` runs is entered at its own ``crt0``, which walks
``.init_array`` on the task that runs the module.  A library is
constructed by ``dlopen()`` through ``libelf_insert()``, on the task that
loads it, as for any shared library.  It is entered through
``fdpic_invoke()``, so that a global object reaches the library's own
storage.

A library named in ``DT_NEEDED`` is constructed before the module that
needs it, because the module's own relocation is what opens it, and
destroyed after, at the last ``dlclose()``.  Since the library is one
instance, its constructors run once however many modules name it.

Destructors are walked from ``DT_FINI_ARRAY`` at unload, in
``libelf_uninit()``, and not from ``crt0``, so they run once whichever way
the object was loaded.

Limitations
===========

**Tested in the flat build only.**  In a protected build, a module that
``exec()`` started faulted at its entry point when last tested, because
the kernel side of the loader places the module in the kernel heap.  A
kernel build needs an MMU, which the FDPIC cores do not have, and an
address environment refuses ``DT_NEEDED``.

Reference
=========

Object layout
-------------

``libs/libc/elf/gnu-elf.ld`` gives a module the layout that execute in
place needs::

  LOAD  vaddr 0x00000000  R E   .text .rodata .dynsym .dynstr .hash
                                .rofixup .rel.dyn .rel.plt
  LOAD  vaddr 0x00001000  RW    .data .dynamic .got .bss
  DYNAMIC                       DT_PLTGOT -> .got

The writable segment starts on the next 4 KiB boundary.  ``.rel.plt`` is
there only when the module calls through a PLT.  In ``.got``,
``.got.plt`` comes first, as in the linker's own script.

``.rodata`` lands in the read-only segment on its own, reached PC
relative or GOT indirect.  That matters: in the writable segment it would
be copied to RAM with ``.data``, and most of the saving would evaporate
silently, with everything still working.

The FDPIC marker is the OS/ABI byte alone.  ``e_flags`` holds the
ordinary EABI version and float ABI, such as ``0x5000000, Version5 EABI``.

Relocations
-----------

The static link resolves ``R_ARM_GOT_BREL`` and ``R_ARM_GOTFUNCDESC``
into the GOT already, so only four types carry work into a linked
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

``R_ARM_GLOB_DAT``
  The address of a symbol, stored in a GOT entry: an imported data
  object, or a symbol of the module itself, such as the bounds of
  ``.init_array`` that ``crt0`` reads.

Constants, from binutils ``include/elf/arm.h`` and mirrored in
``arch/arm/include/elf.h``: ``R_ARM_GOTFUNCDESC`` 161,
``R_ARM_GOTOFFFUNCDESC`` 162, ``R_ARM_FUNCDESC`` 163,
``R_ARM_FUNCDESC_VALUE`` 164.

Which table an imported function's descriptor lands in is the linker's
decision.  The module flags have ``-mlong-calls`` and ``-z now``, so a
call goes through the GOT and its descriptor is in ``DT_REL``.  A module
built without ``-mlong-calls`` and linked with ``-z lazy`` calls through a
PLT, and the descriptors are in ``DT_JMPREL``.  Both are bound eagerly,
so either link works, but the two are not walked identically.  In
``DT_REL`` the word being overwritten is the addend and is added to the
resolved value; in ``DT_JMPREL`` it is the lazy binding bootstrap and the
descriptor is overwritten outright.

``.rofixup`` is skipped.  It is the self-relocation list a *static*
executable's ``crt0`` walks to find its own GOT.  A module's ``crt0`` does
not: the loader supplies the data base, in the PIC base register, before
the module is entered.

Exported functions
------------------

A function exported by a module or library is published to ``dlsym()``
as a descriptor rather than a code address, taken from the same pool, so
that an FDPIC caller can branch through what it gets back.
