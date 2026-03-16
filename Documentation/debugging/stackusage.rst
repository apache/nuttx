===========================
Static Stack Usage Analysis
===========================

Overview
========

``tools/stackusage.py`` performs static stack usage analysis by reading
DWARF ``.debug_frame`` data from an ELF file.  It extracts per-function
stack sizes from CFA (Canonical Frame Address) offsets and optionally
builds a call graph via disassembly to compute worst-case total stack
depth.

- **Self** – stack bytes used by the function itself (max CFA offset).
- **Total** – worst-case stack depth through the deepest call chain
  (self + callees).  A marker prefix flags uncertain values.

Dependencies
============

The tool invokes standard toolchain binaries:

- **readelf** – symbol table and DWARF frame info
- **objdump** – disassembly for call graph analysis
- **addr2line** – source file and line resolution

Both GNU and LLVM toolchains are supported.  Use ``-p`` to set the
toolchain prefix (e.g. ``-p arm-none-eabi-`` for GCC,
``-p llvm-`` for LLVM).

The ELF must contain DWARF debug info (``-g`` or ``-gdwarf``).
No special Kconfig option is needed.

Usage
=====

Analyze a native ELF (no prefix needed)::

   python3 tools/stackusage.py nuttx

Cross-compiled ELF with GCC toolchain::

   python3 tools/stackusage.py -p arm-none-eabi- nuttx

Cross-compiled ELF with LLVM toolchain::

   python3 tools/stackusage.py -p llvm- nuttx

Show top 20 functions::

   python3 tools/stackusage.py -p arm-none-eabi- -n 20 nuttx

Estimate recursion depth of 10::

   python3 tools/stackusage.py -p arm-none-eabi- -r 10 nuttx

Command Line Options
====================

.. code-block:: text

   positional arguments:
     elf                   path to ELF file with DWARF debug info

   options:
     -p, --prefix PREFIX   toolchain prefix (e.g. arm-none-eabi- or llvm-)
     -n, --rank N          show top N functions (default: 0 = all)
     -r, --recursion-depth N
                           assumed recursion depth (default: 0)

Text Output
===========

The default output is an aligned table.  Each function's deepest
backtrace is shown with one frame per row.  The ``Self`` column shows
each frame's own stack cost.  The ``Backtrace`` column shows the
function name followed by its code size in parentheses (when available
from the symbol table), e.g. ``main(128)``.  The entry point of each
call chain is suffixed with ``~``.

Example (``nucleo-f429zi:trace``, ``-n 3``)::

   Total  Self  Backtrace                    File:Line
   -----  ----  ---------------------------  -------------------------------------------
   @2344    56  telnetd_main(236)~           apps/system/telnetd/telnetd.c:42
           ^24  nsh_telnetmain(128)          apps/nshlib/nsh_telnetd.c:48
           ^48  nsh_session(400)             apps/nshlib/nsh_session.c:73
                ...
          @224  nsh_parse_cmdparm(1024)      apps/nshlib/nsh_parse.c:2362
           @96  nsh_execute(512)             apps/nshlib/nsh_parse.c:510
           ^56  nsh_builtin(320)             apps/nshlib/nsh_builtin.c:76
            88  exec_builtin(256)            apps/builtin/exec_builtin.c:61
                ...
           ^64  file_vopen(192)              nuttx/fs/vfs/fs_open.c:124
                ...
   @2328    16  sh_main(64)~                 apps/system/nsh/sh_main.c:40
            16  nsh_system_ctty(96)          apps/nshlib/nsh_system.c:105
           ^32  nsh_system_(160)             apps/nshlib/nsh_system.c:41
           ^48  nsh_session(400)             apps/nshlib/nsh_session.c:73
                ...
   @2312    24  nsh_main(80)~                apps/system/nsh/nsh_main.c:54
           ^24  nsh_consolemain(48)          apps/nshlib/nsh_consolemain.c:65
           ^48  nsh_session(400)             apps/nshlib/nsh_session.c:73
                ...

Uncertainty markers on both Total and Self columns indicate the most
significant reason:

=======  ==========================================
Marker   Meaning
=======  ==========================================
``~``    entry point of the call chain (suffix)
``?``    no DWARF data (self counted as zero)
``*``    dynamic stack (alloca or VLA)
``@``    recursion detected
``^``    indirect call (function pointer)
=======  ==========================================

Uncertainty Reasons
===================

======================================  =========================================
Reason                                  Description
======================================  =========================================
recursion: A->B->...->A                 Recursive cycle detected.  Use ``-r N``
                                        to estimate.
indirect call (function pointer)        Callee unknown at compile time.
no DWARF data                           No ``.debug_frame`` entry; self counted
                                        as zero.
dynamic stack (alloca/VLA)              Function uses ``alloca()`` or
                                        variable-length arrays; self is a
                                        minimum.
======================================  =========================================

Uncertainty propagates upward: if any callee in the deepest path is
uncertain the caller is also marked uncertain.

Recursion Depth Estimation
==========================

By default (``-r 0``) recursive back-edges contribute zero stack.
With ``-r N`` (N > 0) the tool estimates::

   cycle_body_cost × N

For example ``A(64) -> B(32) -> A``::

   cycle_body_cost = 64 + 32 = 96
   -r 10 → 96 × 10 = 960 bytes

The result is still marked uncertain.

Supported Architectures
=======================

Any architecture supported by the toolchain's ``readelf``,
``objdump``, and ``addr2line`` is supported.  This includes
ARM, AArch64, x86, x86_64, MIPS, RISC-V, Xtensa, PowerPC, SPARC,
TriCore, SuperH, and others.
