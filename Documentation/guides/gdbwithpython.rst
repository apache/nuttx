====================
GDB with Python
====================

Introduction
============

We can better debug the nuttx kernel through GDB's python extension.
Some of the most common class usages are implemented under the nuttx/tools/gdb directory.
Users can write their own python scripts to debug the nuttx kernel according to their needs

Usage
=====

1. Compile nuttx with CONFIG_DEBUG_SYMBOLS=y
2. Use gdb to debug nuttx elf.(real device, or sim, or coredump)
3. add args to gdb command line: -ix="nuttx/tools/gdb/__init__.py"
4. Then gdb will load the python script automatically.you can use the custom commands.

How to write a GDB python script
================================

Here is an article to introduce, read it to understand the most basic principles of python,
`Automate Debugging with GDB Python API <https://interrupt.memfault.com/blog/automate-debugging-with-gdb-python-api>`_.

For more documentation on gdb python, please refer to the official documentation of gdb
`GDB with python <https://interrupt.memfault.com/blog/automate-debugging-with-gdb-python-api>`_.
