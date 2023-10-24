=======================
``lua`` Lua interpreter
=======================

Fetch and build a Lua interpreter. Versions 5.2 through 5.4 are supported. The
``lua`` command will be added to NSH. Lua can run a script for a given path,
execute a string of code, or open a readline compatible REPL on the NSH console.
The ``<lua.h>`` and ``<lauxlib.h>`` headers are available to start a new embedded
interpreter or extend Lua with C modules. See the ``luamod_hello`` example for how
to include a built-in module.

A math library is required to build. Enable the ``LIBM`` config or use a
toolchain provided math library.

The following configs are recommended for a full featured Lua interpreter:
- ``LIBC_FLOATINGPOINT``
- ``SYSTEM_READLINE``


Lua modules:

- cjson
- lfs
- luasyslog
- luv
