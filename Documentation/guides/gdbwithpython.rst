===============
GDB with Python
===============

Introduction
============

The NuttX kernel can be effectively debugged using GDB's Python extension.
Commonly used classes and utilities are implemented in the nuttx/tools/gdb/nuttxgdb directory.
Users can also create custom Python scripts tailored to their debugging needs to analyze and troubleshoot the NuttX kernel more efficiently.

Usage
=====

1. Compile NuttX with CONFIG_DEBUG_SYMBOLS=y enabled and change `CONFIG_DEBUG_SYMBOLS_LEVEL` to -g3.
2. Use GDB to debug the NuttX ELF binary (on a real device, a simulator, or with a coredump).
3. Add the following argument to the GDB command line: `-ix="nuttx/tools/gdb/gdbinit.py"`
4. GDB will automatically load the Python script, enabling the use of custom commands.

How to write a GDB python script
================================

Here is an article that introduces the fundamental principles of Python in GDB. Read it to gain a basic understanding.
`Automate Debugging with GDB Python API <https://interrupt.memfault.com/blog/automate-debugging-with-gdb-python-api>`_.

For more documentation on gdb python, please refer to the official documentation of GDB.
`GDB Python API <https://sourceware.org/gdb/current/onlinedocs/gdb.html/Python-API.html#Python-API>`_.
