=================================
``bastest`` Bas BASIC Interpreter
=================================

This directory contains a small program that will mount a ROMFS file system
containing the BASIC test files extracted from the Bas ``2.4`` release.

- ``CONFIG_EXAMPLES_BASTEST_DEVMINOR`` – The minor device number of the ROMFS
  block driver. For example, the ``N`` in ``/dev/ramN``. Used for registering the
  RAM block driver that will hold the ROMFS file system containing the BASIC
  files to be tested. Default: ``0``.

- ``CONFIG_EXAMPLES_BASTEST_DEVPATH`` – The path to the ROMFS block driver device.
  This must match ``EXAMPLES_BASTEST_DEVMINOR``. Used for registering the RAM
  block driver that will hold the ROMFS file system containing the BASIC files
  to be tested. Default: ``/dev/ram0``.

Background
----------

Bas is an interpreter for the classic dialect of the programming language BASIC.
It is pretty compatible to typical BASIC interpreters of the 1980s, unlike some
other UNIX BASIC interpreters, that implement a different syntax, breaking
compatibility to existing programs. Bas offers many ANSI BASIC statements for
structured programming, such as procedures, local variables and various loop
types. Further there are matrix operations, automatic LIST indentation and many
statements and functions found in specific classic dialects. Line numbers are
not required.

The interpreter tokenises the source and resolves references to variables and
jump targets before running the program. This compilation pass increases
efficiency and catches syntax errors, type errors and references to variables
that are never initialised. Bas is written in ANSI C for UNIX systems.

License
-------

BAS `2.4` is released as part of NuttX under the standard 3-clause BSD license
use by all components of NuttX. This is not incompatible with the original BAS
`2.4` licensing

Copyright (c) 1999-2014 Michael Haardt

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
