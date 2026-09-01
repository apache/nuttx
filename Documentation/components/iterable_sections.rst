=================
Iterable Sections
=================

Iterable sections provide **link-time registration** of ``struct``
instances: an instance defined with :c:macro:`STRUCT_SECTION_ITERABLE` in
any compilation unit is placed in a dedicated linker input section.  The
linker collects all instances into a contiguous, name-sorted array
delimited by ``_<type>_list_start``/``_<type>_list_end`` symbols, which
the code can then iterate like a plain C array -- no runtime registration
calls, no central list to maintain.

This is the same mechanism used by the Zephyr RTOS ``STRUCT_SECTION_*``
macros.  The first user of this infrastructure is the zbus message bus
port (``apps/system/zbus``, from nuttx-apps).

C API
=====

The macros are provided by ``include/nuttx/iterable_sections.h``:

.. code-block:: c

   #include <nuttx/iterable_sections.h>

   struct my_entry
   {
     const char *name;
     int value;
   };

   /* In any .c file (const places the instance in ROM): */

   const STRUCT_SECTION_ITERABLE(my_entry, entry_foo) =
   {
     .name = "foo",
     .value = 42,
   };

   /* In the file that iterates: declare the section boundaries once, at
    * file scope, then loop with a caller-declared pointer.
    */

   STRUCT_SECTION_DECLARE(my_entry);

   void print_entries(void)
   {
     FAR struct my_entry *entry;

     STRUCT_SECTION_FOREACH(my_entry, entry)
       {
         printf("%s = %d\n", entry->name, entry->value);
       }
   }

Available macros:

* ``STRUCT_SECTION_ITERABLE(type, varname)`` -- define an instance inside
  the iterable section ``._<type>.static.<varname>``.  The variable name
  is part of the input section name, so the linker's ``SORT_BY_NAME()``
  defines the iteration order.  A subsystem that needs a specific order
  encodes it in the name, usually with a fixed width index:
  ``FOREACH_ARG()`` from ``nuttx/macro.h`` hands the position of each item
  to the definition macro as a literal that can be pasted into the name.
* ``STRUCT_SECTION_DECLARE(type)`` -- declare the boundary symbols (file
  scope), required before iterating.
* ``STRUCT_SECTION_FOREACH(type, iterator)`` -- for-loop over all
  instances; ``iterator`` is a pointer declared by the caller, as with
  ``list_for_every_entry()``.
* ``STRUCT_SECTION_GET(type, i, dst)`` -- random access by index.
* ``STRUCT_SECTION_COUNT(type, dst)`` -- number of instances.
* ``STRUCT_SECTION_START/END/START_EXTERN/END_EXTERN`` -- direct access
  to the boundary symbols.

Linker integration
==================

The collection step needs linker script support.  Two mechanisms are
available; both rely on the fact that the linker scripts listed in
``ARCHSCRIPT`` are preprocessed with CPP (arm, arm64, risc-v, xtensa,
x86_64 and tricore), so ``#include`` and ``#ifdef CONFIG_*`` work inside
them.

Board script include (first-class mechanism)
--------------------------------------------

The board linker script includes the central fragments, which expand to
nothing unless a subsystem using iterable sections is enabled:

.. code-block:: text

   .text :
   {
       ...
       *(.gnu.linkonce.r.*)
   #include <nuttx/linker/common-rom.ld>
       _etext = ABSOLUTE(.);
   } > flash

   .data :
   {
       _sdata = ABSOLUTE(.);
       ...
   #include <nuttx/linker/common-ram.ld>
       . = ALIGN(4);
       _edata = ABSOLUTE(.);
   } > sram AT > flash

* ``common-rom.ld`` collects the read-only (``const``) iterable sections
  and must be included inside the read-only output section (typically
  ``.text``, before ``_etext``).
* ``common-ram.ld`` collects mutable *initialized* iterable sections and
  must be included inside ``.data`` (between ``_sdata`` and ``_edata``)
  so the startup FLASH-to-RAM copy initializes the entries.
* Subsystems add their sections to these central files, guarded by their
  Kconfig option (see ``include/nuttx/linker/common-rom.ld`` for the zbus
  example).

Supplementary INSERT script (zero-touch mode)
---------------------------------------------

With ``CONFIG_ITERABLE_SECTIONS_LINKER_INSERT`` the central script
``include/nuttx/linker/common-insert.ld`` is added before the board
script by the build system (``tools/Config.mk`` for Make, the top-level
``CMakeLists.txt`` for CMake) and supplements it through the GNU ld
``INSERT AFTER`` command, so **no board script modification is needed**.
The file defines one output section, ``.iterable_sections``, inserted
after ``.text``; subsystems add their ``ITERABLE_SECTION()`` blocks
inside it, guarded by their Kconfig option, exactly as in
``common-rom.ld`` (which expands to nothing in this mode).

This mode has constraints, discovered the hard way and worth knowing
before choosing it:

* GNU ld only (``INSERT`` is not supported by the macOS ld64).
* The INSERT script must come *before* the board script on the linker
  command line.  Adding it via ``ARCHSCRIPT`` from ``tools/Config.mk``
  guarantees that, because ``Config.mk`` is included by the board
  ``Make.defs`` before it appends its own script.  (The reversed order
  fails with ``.text not found for insert``.)
* GNU ld assigns an INSERTed output section to a ``MEMORY`` region by
  *attribute matching in declaration order*, not by inheriting the anchor
  section's region.  The ROM/flash region must therefore be the first
  region compatible with read-only sections.  Boards declaring a generic
  ``rwx`` region at a lower address first (e.g. an ITCM at ``0x0``) are
  incompatible with this mode and must use the board script include.
* Giving the inserted section an explicit address is **not** a fix: a
  section with an explicit address does not consume the memory region,
  so the next region-allocated section overlaps it.

Alignment rules
===============

Instances are aligned to the natural alignment of their type
(``STRUCT_SECTION_ITERABLE`` adds ``__aligned__(__alignof__(type))``), and
``sizeof`` is always a multiple of ``alignof``, so the collected section
can be indexed as a plain array with no padding between entries from
different compilation units.  The fragments additionally align the list
boundaries to 4 bytes.

Adding a new iterable type
==========================

1. Define the instances with ``STRUCT_SECTION_ITERABLE(mytype, name)``.
2. Add ``ITERABLE_SECTION(mytype)`` to
   ``include/nuttx/linker/common-rom.ld`` (const) or ``common-ram.ld``
   (mutable initialized), guarded by the subsystem Kconfig option.
3. Iterate with ``STRUCT_SECTION_FOREACH(mytype, it)`` after
   ``STRUCT_SECTION_DECLARE(mytype);`` at file scope.

Caveat on generated linker scripts: the preprocessed ``.ld.tmp`` files
only depend on the board script and ``.config``; after editing the
central fragments during development, remove the ``.tmp`` files (or run
``make clean``) to force regeneration.
