==============================
Debugging ELF Loadable Modules
==============================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Debugging+ELF+Loadable+Modules

Debugging ELF modules loaded in memory can be tricky because the load address
in memory does not match the addresses in the ELF file. This challenge has long
existed for debugging uClinux programs and Linux kernel modules; the same
solution can be used with NuttX ELF files (and probably with NxFLAT modules as
well). Below is a summary of one way to approach this:

1. Get ELF Module Load Address
==============================

Put a change in ``nuttx/binfmt`` so that you print the address where the ELF
text was loaded into memory.

Turning on BINFMT debug (``CONFIG_DEBUG_BINFMT=y``) should give you the same
information, although it may also provide more output than you really want.

Alternatively, you could place a ``printf()`` at the beginning of your ``main()``
function so that your ELF module can print its own load address. For example,
the difference between the address of ``main()`` in your object file and the
address of ``main()`` at run time reveals the actual load address.

2. Make the ELF Module Wait for You
===================================

Insert an infinite loop in the ``main()`` routine of your ELF program. For
example:

.. code-block:: c

    volatile bool waitforme;
    int main (int arc, char **argv)
    {
        while (!waitforme);
        ...

When you start the ELF program, you will see where it was loaded in memory, and
the ELF program will remain stuck in the infinite loop. It will continue to
wait for ``waitforme`` to become true before proceeding.

3. Start the Debugger
=====================

Start the debugger, connect to the GDB server, and halt the program. If your
debugger is well-behaved, it should stop at the infinite loop in ``main()``.

4. Load Offset Symbols
======================

Load symbols using the offset where the ELF module was loaded:

.. code-block:: shell

   (gdb) add-symbol-file <myprogram> <load-address>

Here, ``<myprogram>`` is your ELF file containing symbols, and
``<load-address>`` is the address where the program text was actually loaded (as
determined above). Single-step a couple of times and confirm that you are in the
infinite loop.

5. And Debug
============

Set ``waitforme`` to a non-zero value. Execution should exit the infinite loop,
and now you can debug the ELF program loaded into RAM in the usual way.

An Easier Way?
==============

There might be an alternative that allows you to step into the ELF module
without modifying the code to include the ``waitforme`` loop. You could place a
breakpoint on the OS function ``task_start()``. That function runs before your
ELF program starts, so you should be able to single-step from the OS code
directly into your loaded ELF application—no changes to the ELF application
required.

When you step into the application's ``main()``, you have the relocated address
of ``main()`` and can use that address (see step #1) to compute the load offset.
