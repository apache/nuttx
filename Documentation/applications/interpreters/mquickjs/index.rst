================================================
``mquickjs`` MicroQuickJS JavaScript interpreter
================================================

This is a port of `MicroQuickJS <https://github.com/bellard/mquickjs>`_, a lightweight JavaScript
interpreter created by Fabrice Bellard. MicroQuickJS is designed for resource-constrained embedded
systems, offering a minimal footprint while still providing JavaScript language support.

Strict Mode
===========

MQuickJS operates in a **stricter mode** where certain error-prone or inefficient JavaScript
features are disabled. Key restrictions include:

- Only strict mode constructs are supported (no ``with`` keyword, global variables must use ``var``)
- Arrays cannot have holes; out-of-bounds array writes (except at array end) throw TypeError
- Only global ``eval`` is supported; indirect eval cannot access local variables
- No value boxing (e.g., ``new Number(1)`` is not supported)

Configuration Options
=====================

``CONFIG_INTERPRETERS_MQJS``
  Enable the MicroQuickJS JavaScript interpreter (default: n)

``CONFIG_INTERPRETERS_MQJS_PRIORITY``
  Task priority for the interpreter (default: 100)

``CONFIG_INTERPRETERS_MQJS_STACKSIZE``
  Stack size for the interpreter in bytes (default: 8192)

Usage
=====

The ``mqjs`` command is available in NSH with the following options::

  mqjs [options] [file [args]]

  Options:
    -h, --help           List options
    -e, --eval EXPR      Evaluate expression
    -i, --interactive    Go to interactive mode
    -I, --include file   Include additional file
    -d, --dump           Dump memory usage stats
    --memory-limit n     Limit memory usage to n bytes
    --no-column          No column number in debug info
    -o FILE              Save bytecode to FILE
    -m32                 Force 32-bit bytecode output
    -b, --allow-bytecode Allow bytecode in input file

Examples:

- Execute a script: ``mqjs script.js``
- Interactive REPL: ``mqjs -i``
- Run with memory limit: ``mqjs --memory-limit 10k script.js``
- Evaluate expression: ``mqjs -e 'print("Hello")'``

.. warning::
   **Important**: The default memory limit is 16 MB (``16 << 20`` bytes), which is too large
   for most embedded systems. **Always set a memory limit** when running mquickjs on NuttX,
   typically using ``--memory-limit 10k`` or similar small value. For example::

     nsh> mqjs --memory-limit 10k script.js

   Without a memory limit, mquickjs may exhaust system memory and cause instability.

Features
========

- **JavaScript subset**: Implements a strict ES5-compatible subset of JavaScript with stricter mode that disables error-prone or inefficient constructs
- **Tracing garbage collector**: Compactive GC for smaller objects and reduced memory fragmentation
- **ROM-based standard library**: Standard library resides in ROM, generated at compile time for fast instantiation
- **Bytecode compilation**: Can compile to bytecode and save to persistent storage (file or ROM) for embedded systems
