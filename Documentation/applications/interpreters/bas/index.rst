=============================
``bas`` Bas BASIC Interpreter
=============================

Introduction
------------

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

BAS 2.4 is released as part of NuttX under the standard 3-clause BSD license use
by all components of NuttX. This is not incompatible with the original BAS 2.4
licensing

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

Bas 2.4 Release Notes
---------------------

Changes compared to version ``2.3``

- Matrix inversion on integer arrays with option base 1 fixed.
- ``PRINT USING`` behaviour for ``!`` fixed.
- ``PRINT``, separator should advance to the next zone, even if the current
  position is at the start of a zone.
- Added ``ip()``, ``frac()``, ``fp()``, ``log10()``, ``log2()``, ``min()`` and ``max()``.
- Fixed ``NEXT`` checking the variable case sensitive.
- Use ``terminfo`` capability cr to make use of its padding.
- ``LET`` segmentation fault fixed.
- ``PRINT`` now uses print items.
- ``-r`` for restricted operation.
- ``MAT INPUT`` does not drop excess arguments, but uses them for the next row.
- License changed to MIT.
