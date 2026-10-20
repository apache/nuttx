==================================
``berry`` Berry scripting language
==================================

Berry is a small embedded scripting language implemented in ANSI C99.  The
``berry`` command provides a compiler, register-based virtual machine, garbage
collector, file I/O, bytecode loading and saving, and built-in modules for
small on-device scripts.

Configuration
=============

Enable Berry with ``CONFIG_INTERPRETERS_BERRY``.  The interpreter is built as an
NSH built-in command with the following options:

``CONFIG_INTERPRETERS_BERRY_PROGNAME``
  Command name used to start Berry.  The default is ``berry``.

``CONFIG_INTERPRETERS_BERRY_PRIORITY``
  Task priority for the interpreter.  The default is ``100``.

``CONFIG_INTERPRETERS_BERRY_STACKSIZE``
  Stack size for the interpreter task.  The default is ``12288`` bytes.

Berry depends on ``CONFIG_ARCH_SETJMP_H``, ``CONFIG_LIBC_FLOATINGPOINT``, and
``CONFIG_SYSTEM_SYSTEM``.  The default NuttX Berry configuration enables the
file, JSON, math, time, OS, GC, debug, strict, introspection, and bytecode
modules.  Shared-library module loading is disabled, so Berry does not require
``CONFIG_LIBC_DLFCN``.

The math module requires a math library.  Use the NuttX ``LIBM`` config or a
toolchain-provided math library.

Usage
=====

Run a script from any mounted filesystem:

.. code-block:: console

   nsh> berry hello.be
   nsh> berry /scripts/blink.be

Run inline source:

.. code-block:: console

   nsh> berry -e print(40+2)
   42

Start the interactive prompt:

.. code-block:: console

   nsh> berry
   Berry 1.1.0 (build in Jun 26 2026, 09:07:01)
   [GCC 13.2.0] on NuttX (default)
   >
