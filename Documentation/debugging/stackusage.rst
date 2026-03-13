===========================
Static Stack Usage Analysis
===========================

Overview
========

NuttX supports compile-time static stack usage analysis through GCC's
``-fstack-usage`` flag. When ``CONFIG_STACK_USAGE`` is enabled, the compiler
generates ``.su`` files alongside each object file, reporting the maximum stack
usage of every function.

The ``tools/stackusage.py`` script aggregates these ``.su`` files and optionally
combines them with ``objdump`` disassembly to build a call graph, computing both:

- **Self**: The stack used by the function itself.
- **Total**: The worst-case stack depth through the entire call chain
  (self + deepest callee path).

Functions whose stack depth cannot be fully determined are flagged as
uncertain with specific reasons (e.g., ``alloca``/VLA, recursion, function
pointers, missing ``.su`` data).

Configuration
=============

Enable stack usage information generation in your board's defconfig::

   CONFIG_STACK_USAGE=y

Optionally set a compile-time warning threshold (in bytes)::

   CONFIG_STACK_USAGE_WARNING=1024

Usage
=====

Self-only analysis (no call graph, only per-function stack usage)::

   python3 tools/stackusage.py .

Full analysis with call graph (requires ``objdump`` and ELF binary)::

   python3 tools/stackusage.py -d arm-none-eabi-objdump -e nuttx .

Show only top 50 functions::

   python3 tools/stackusage.py -n 50 -d arm-none-eabi-objdump -e nuttx .

Output in CSV format::

   python3 tools/stackusage.py --csv -d arm-none-eabi-objdump -e nuttx .

Specify recursion depth for estimation (e.g., assume depth of 10)::

   python3 tools/stackusage.py -r 10 -d arm-none-eabi-objdump -e nuttx .

Search multiple directories::

   python3 tools/stackusage.py -d arm-none-eabi-objdump -e nuttx arch/ sched/ drivers/

Command Line Options
====================

.. code-block:: text

   positional arguments:
     dirs                  directories to search for .su files

   options:
     -d, --objdump PATH    path to objdump binary (enables callee analysis)
     -e, --elf PATH        path to ELF file (required with --objdump)
     -n, --rank N          show top N functions (default: 0 = all)
     -r, --recursion-depth N
                           assumed recursion depth for stack estimation
                           (default: 0, recursive back-edges contribute nothing)
     --csv                 output in CSV format
     --detail              show detailed reasons (default: summary only)

Output Columns
==============

=========  ==========================================================
Column     Description
=========  ==========================================================
Function   Function name
Self       Stack bytes used by the function itself
Total      Worst-case total stack depth (self + deepest callee chain)
\*         Uncertainty marker (``*`` if total cannot be precisely determined)
File:Line  Source file and line number where the function is defined
Reasons    Comma-separated list of uncertainty reasons (summary by default,
           use ``--detail`` for full call chain details)
=========  ==========================================================

Example output::

   Function                  Self  Total     File:Line                 Reasons
   --------                  ----  -----     ---------                 -------
   tcp_input                  512   2048     net/tcp/tcp_input.c:342
   some_alloca_func           128    896  *  drivers/net/x.c:55        dynamic stack (alloca/VLA)
   recursive_parser            64    320  *  libs/libc/parse.c:12      recursion: parse->parse_expr->parse
   handler_dispatch           256   1024  *  sched/handler.c:80        indirect call (function pointer)

Uncertainty Reasons
===================

======================================  =========================================
Reason                                  Description
======================================  =========================================
dynamic stack (alloca/VLA)              Function uses ``alloca()`` or variable-length arrays;
                                        GCC reports ``dynamic`` qualifier and stack size
                                        cannot be statically determined.
dynamic stack (bounded estimate)        Dynamic allocation but GCC can determine an upper
                                        bound; uses the bounded value as estimate.
recursion: A->B->...->A                 Recursive call cycle detected. Use ``-r N`` to
                                        estimate by multiplying the cycle body cost by N.
indirect call (function pointer)        Function calls through a pointer; callee is unknown
                                        at compile time and cannot be included in total.
no .su data                             Function appears in the call graph (from objdump)
                                        but has no matching ``.su`` file entry. Its self
                                        stack usage is counted as zero.
======================================  =========================================

All uncertainty propagates upward: if any callee in the worst-case path is
uncertain, the caller is also marked uncertain.

Recursion Depth Estimation
==========================

By default (``-r 0``), recursive back-edges contribute zero additional stack.
When ``-r N`` is specified with N > 0, the tool estimates the recursive cost as::

   cycle_body_cost * N

Where ``cycle_body_cost`` is the sum of self stack sizes of all functions in the
detected cycle. For example, with ``A(64) -> B(32) -> A`` forming a cycle::

   cycle_body_cost = 64 + 32 = 96
   -r 10 → recursive contribution = 96 × 10 = 960 bytes

The result is still marked as uncertain since the actual recursion depth depends
on runtime behavior.

Supported Architectures
=======================

The call instruction detection covers all NuttX-supported architectures:
ARM, ARM64, x86, x86_64, RISC-V, Xtensa, MIPS, AVR, z80, z16,
OpenRISC, SPARC, TriCore, HC, Renesas, CEVA, and Simulator.
