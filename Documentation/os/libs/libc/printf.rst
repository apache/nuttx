======
printf
======

This page explains what ``printf()`` does in NuttX: each conversion, flag,
length, width and precision, each option and profile, and the flash and stack
that each one costs.

``printf()``, ``fprintf()``, ``dprintf()``, ``sprintf()``, ``snprintf()``,
``asprintf()``, ``syslog()`` and their ``v`` forms all use one function:
``lib_vsprintf()`` in ``libs/libc/stream/lib_libvsprintf.c``.  So this page
applies to all of them.

All outputs on this page come from a run of the examples on NuttX
(``rv-virt:nsh``, 32-bit RISC-V, under QEMU).  The first sections show the
Full profile.  Each example shows the output between quotes.

The rule
========

A correct call of ``printf()`` never crashes or hangs, in any configuration.
A *correct call* is a call that GCC and Clang accept with
``-Wall -Wextra -Wformat=2``.

- The grammar of the format is always on.  An option removes an output, and
  never the parser.
- Each argument is read at its right size, also when its conversion is off.
- A conversion that is off prints one ``?``, padded to the width, so the rest
  of the line stays correct.
- Width and precision counters cannot wrap.  No read or write goes outside a
  buffer.  ``NULL`` for ``%s`` prints ``(null)``.  The stack use is bounded.

Conversions
===========

================  =========================================================
Conversion        Output
================  =========================================================
``%d``, ``%i``    A signed decimal integer.
``%u``            An unsigned decimal integer.
``%o``            An unsigned octal integer.
``%x``, ``%X``    An unsigned hexadecimal integer.
``%b``, ``%B``    An unsigned binary integer (C23, ``LIBC_PRINTF_C23``).
``%c``            One character.
``%s``            A string.
``%p``            A pointer, in hexadecimal, with ``0x``.
``%%``            One ``%``.
``%e``, ``%E``    A float, in the form ``d.ddde+dd``.
``%f``, ``%F``    A float, in the form ``ddd.ddd``.
``%g``, ``%G``    A float, in the form of ``%e`` or ``%f``, whichever is
                  shorter, without trailing zeros.
``%a``, ``%A``    A float, exactly, in hexadecimal
                  (``LIBC_PRINTF_HEXFLOAT``).
``%n``            Reads its pointer, stores nothing and prints ``?``, so a
                  format never writes to memory.  ``LIBC_PRINTF_PERCENT_N``
                  stores the count: it is dangerous, and no profile has it.
================  =========================================================

.. code-block:: text

   printf("%d", 42)              "42"
   printf("%i", -42)             "-42"
   printf("%u", 42)              "42"
   printf("%o", 8)               "10"
   printf("%x", 255)             "ff"
   printf("%X", 255)             "FF"
   printf("%c", 'A')             "A"
   printf("%s", "text")          "text"
   printf("%p", (void *)0x1000)  "0x1000"
   printf("100%%")               "100%"
   printf("%e", 3.14159)         "3.141590e+00"
   printf("%E", 3.14159)         "3.141590E+00"
   printf("%f", 3.14159)         "3.141590"
   printf("%F", 3.14159)         "3.141590"
   printf("%g", 3.14159)         "3.14159"
   printf("%G", 0.000001)        "1E-06"
   printf("%a", 1.5)             "0x1.8p+0"
   printf("%A", 1.5)             "0X1.8P+0"
   printf("%b", 5)               "101"
   printf("%B", 5)               "101"
   printf("ab%n|", &n)           "ab?|"

Flags
=====

=========  ==============================================================
Flag       Effect
=========  ==============================================================
``-``      Pads on the right (``LIBC_PRINTF_LEFT_JUSTIFY``).
``+``      Prints the sign of a positive number.
space      Prints a space in place of the sign of a positive number.
``#``      The alternative form (``LIBC_PRINTF_ALT_FORM``): ``0x``, ``0X``,
           ``0b``, ``0B``, a first ``0`` for ``%o``, and the point of a
           float that has no digits after it.
``0``      Pads with zeros after the sign and the prefix.
``'``      Accepted and ignored: the C locale has no separator.
=========  ==============================================================

.. code-block:: text

   printf("%-5d|", 42)     "42   |"
   printf("%+d", 42)       "+42"
   printf("% d", 42)       " 42"
   printf("%#x", 255)      "0xff"
   printf("%#X", 255)      "0XFF"
   printf("%#o", 8)        "010"
   printf("%#b", 5)        "0b101"
   printf("%#.0f", 3.0)    "3."
   printf("%05d", 42)      "00042"
   printf("%'d", 1234567)  "1234567"

Width
=====

The width is the smallest number of bytes.  A longer output is not cut.  ``*``
takes the width from an ``int`` argument, and a negative value means the ``-``
flag and a positive width.

.. code-block:: text

   printf("%5d", 42)       "   42"
   printf("%*d", 5, 42)    "   42"
   printf("%*d|", -5, 42)  "42   |"
   printf("%5s", "ab")     "   ab"
   printf("%-5s|", "ab")   "ab   |"
   printf("%3d", 12345)    "12345"

Precision
=========

For an integer, the precision is the smallest number of digits
(``LIBC_PRINTF_INT_PRECISION``).  An integer with the value 0 and a precision
of 0 prints no digits.  For ``%e``, ``%f`` and ``%a`` it is the number of digits
after the point.  For ``%g`` it is the number of significant digits.  For
``%s`` it is the largest number of bytes.  ``*`` takes it from an ``int``
argument.

.. code-block:: text

   printf("%.3d", 5)         "005"
   printf("%.0d|", 0)        "|"
   printf("%8.3d", 5)        "     005"
   printf("%.2f", 3.14159)   "3.14"
   printf("%.3e", 3.14159)   "3.142e+00"
   printf("%.3g", 3.14159)   "3.14"
   printf("%.3s", "abcde")   "abc"
   printf("%.*f", 1, 2.25)   "2.3"
   printf("%8.2f", 3.14159)  "    3.14"
   printf("%.0f", 2.5)       "2"

Length modifiers
================

=============================  ==============================================
Length                         Argument of an integer conversion
=============================  ==============================================
``hh``                         ``char`` (``LIBC_PRINTF_SHORT``)
``h``                          ``short`` (``LIBC_PRINTF_SHORT``)
``l``                          ``long``
``ll``                         ``long long`` (``LIBC_PRINTF_LONG_LONG``)
``j``                          ``intmax_t`` (``LIBC_PRINTF_LONG_LONG``)
``z``, ``t``                   ``size_t``, ``ptrdiff_t``
``L``                          ``long double`` for a float, and ``long long``
                               for an integer, as in GNU libc
``w8``, ``w16``, ``w32``,      The exact-width types (C23,
``w64``                        ``LIBC_PRINTF_C23``)
``wf8``, ``wf16``, ``wf32``,   The fastest types of that width (C23,
``wf64``                       ``LIBC_PRINTF_C23``)
=============================  ==============================================

``LIBC_PRINTF_LONG_LONG`` applies where ``long long`` is longer than ``long``.
A ``long double`` prints with the precision and range of a ``double``, but
``%La`` prints all its bits where it has more than a ``double``.

.. code-block:: text

   printf("%hhd", 300)                       "44"
   printf("%hd", 70000)                      "4464"
   printf("%ld", 1234567L)                   "1234567"
   printf("%lld", 123456789012LL)            "123456789012"
   printf("%llx", 0x123456789abULL)          "123456789ab"
   printf("%jd", (intmax_t)-1)               "-1"
   printf("%zu", (size_t)42)                 "42"
   printf("%td", (ptrdiff_t)-42)             "-42"
   printf("%Lf", (long double)1.5)           "1.500000"
   printf("%La", (long double)1.5)           "0x1.8p+0"
   printf("%w8d", 300)                       "44"
   printf("%w16x", 0x12345)                  "2345"
   printf("%w32u", 42)                       "42"
   printf("%w64d", (int64_t)123456789012LL)  "123456789012"
   printf("%wf16d", 42)                      "42"

Numbered arguments
==================

``%n$`` takes argument number *n* (``LIBC_NUMBERED_ARGS``), up to
``LIBC_NL_ARGMAX``.  ``*m$`` takes a width or precision from argument *m*.
POSIX does not allow a mix of numbered and sequential conversions in one format.

.. code-block:: text

   printf("%2$s %1$s", "world", "hello")  "hello world"
   printf("%1$*2$d|", 42, 5)              "   42|"
   printf("%2$.*1$f", 2, 3.14159)         "3.14"

Each argument before the highest number that the format uses must have a
number in the format: otherwise its size is not known, so ``printf()`` cannot
find the arguments after it.  Then it reads only the arguments before the gap,
and a conversion with a later number ends the output.

Floating point values
=====================

.. code-block:: text

   printf("%f", 1.0 / 0.0)        "inf"
   printf("%F", -1.0 / 0.0)       "-INF"
   printf("%e", 0.0 / 0.0 * 0.0)  "nan"
   printf("%f", -0.0)             "-0.000000"
   printf("%g", -0.0)             "-0"
   printf("%a", -0.0)             "-0x0p+0"
   printf("%s", (char *)NULL)     "(null)"
   printf("%a", 0.1)              "0x1.999999999999ap-4"
   printf("%.0a", 1.5)            "0x2p+0"
   printf("%.1a", 1.09375)        "0x1.2p+0"
   printf("%a", 5e-324)           "0x0.0000000000001p-1022"
   printf("%a", 1.0 / 0.0)        "inf"
   printf("%10.3a|", 1.5)         "0x1.800p+0|"
   printf("%#.0a", 1.0)           "0x1.p+0"
   printf("%010a", 1.5)           "0x001.8p+0"

``%a`` has no limit on precision and takes all digits from the bits of the
value.  With a precision it rounds to nearest, ties to even.  A subnormal value
has a first digit of 0 and the exponent of the smallest normal value, as in GNU
libc.

Unknown conversions and truncated formats
=========================================

An unknown conversion has no known argument size.  So ``printf()`` prints ``%``
and the letter, then the rest of the format as text, reads no more arguments,
and ends.  A format that ends in the middle of a conversion prints ``%`` and
ends, and no read goes past the terminator.  This is always on.  A valid C23
format that this configuration does not know takes the same path, so it cannot
misread a later ``%s``.

.. code-block:: text

   printf("%d %y %d", 1, 2)    "1 %y %d"
   printf("a%-5.3kb%d", 1)     "a%kb%d"
   printf("abc%")              "abc%"
   printf("abc%5")             "abc%"
   printf("abc%l")             "abc%"
   printf("%Hf|%s", 1.5, "x")  "%Hf|%s"
   printf("%w7d|%d", 1, 2)     "%w7d|%d"

Large output
============

There is no cap on width or precision.  ``%2147483647d`` is legal and prints all
of it, which takes about 2 days on a 115200 baud console.  A caller with a small
buffer must use ``snprintf()``.  If the total passes ``INT_MAX``, the result
is -1 with ``errno`` set to ``EOVERFLOW``.  The output written before that stays.
A conversion that is longer than ``INT_MAX`` fails before its output.  A width
or precision with too many digits stops at ``INT_MAX``.

The next examples use ``snprintf()`` with a small buffer, and show the return
value.

.. code-block:: text

   snprintf(buf, sizeof(buf), "x%2147483647d", 1)              returns -1, errno 75 (EOVERFLOW)
   snprintf(buf, sizeof(buf), "%.2147483647f", 1.5)            returns -1, errno 75 (EOVERFLOW)
   snprintf(buf, sizeof(buf), "%.99999999999s|", "abc")        returns 4
   snprintf(buf, sizeof(buf), "%300d", 1)                      returns 300

Options and profiles
====================

Each feature is an option with its own prompt.  A profile only sets the default
of each option: select one in *Library Routines*, *printf profile*.  You can add
or remove any feature in any profile.

=============================  ==========  ===========  ======
Option                         Minimal     Default      Full
=============================  ==========  ===========  ======
``LIBC_FLOATINGPOINT``         off         see below    on
``LIBC_PRINTF_HEXFLOAT``       off         off          on
``LIBC_PRINTF_C23``            off         off          on
``LIBC_PRINTF_PERCENT_N``      off         off          off
``LIBC_NUMBERED_ARGS``         off         off          on
``LIBC_LOCALE``                off         off          on
``LIBC_PRINTF_LONG_LONG``      see below   on           on
``LIBC_PRINTF_ALT_FORM``       off         on           on
``LIBC_PRINTF_INT_PRECISION``  off         on           on
``LIBC_PRINTF_OCTAL``          on          on           on
``LIBC_PRINTF_LEFT_JUSTIFY``   on          on           on
``LIBC_PRINTF_SHORT``          on          on           on
``STREAM_OUT_BUFFER_SIZE``     0           64           64
=============================  ==========  ===========  ======

In Default, ``LIBC_FLOATINGPOINT`` is on when ``DEFAULT_SMALL`` is off and the
CPU has an FPU, as before the profiles existed.  In Minimal,
``LIBC_PRINTF_LONG_LONG`` stays on with ``SYSLOG_TIMESTAMP`` or ``FS_PROCFS``,
because the log time stamps and ``/proc`` print ``time_t`` and ``uint64_t``.

``LIBC_PRINTF_HEXFLOAT`` does not need ``LIBC_FLOATINGPOINT``.  It uses no
floating point operation and pulls in no soft-float code, so a small board with
floating point off can still print the exact value of a ``double``.

``LIBC_PRINTF_PERCENT_N`` is dangerous.  With it, ``%n`` stores the count of the
bytes so far through its pointer, at the size of its length modifier, and prints
nothing.  A format that an attacker can change then gives a write to any address.
A ``NULL`` pointer stores nothing.  With it on, the call below stores 2 in ``n``:

.. code-block:: text

   printf("ab%n|", &n)  "ab|"

What a feature prints when it is off
------------------------------------

Each line shows the same call with the feature on, then with it off.  The ``?``
comes from a conversion that is off: the argument is read at its right size, so
the next conversion is correct.  With ``LIBC_PRINTF_ALT_FORM`` off the flag has
no effect, but ``%p`` keeps ``0x``.  With ``LIBC_PRINTF_LEFT_JUSTIFY`` off the
output pads on the left.  With ``LIBC_PRINTF_C23`` off ``%b`` and ``wN`` are
unknown conversions, so they print as text and end the conversions.  Numbered
arguments off have the same result.

.. code-block:: text

   Call                                      Full            All off
   printf("ab%n|", &n)                       "ab?|"          "ab?|"
   printf("%f", 3.14159)                     "3.141590"      "?"
   printf("%e", 3.14159)                     "3.141590e+00"  "?"
   printf("%g", 3.14159)                     "3.14159"       "?"
   printf("%a", 1.5)                         "0x1.8p+0"      "?"
   printf("%b", 5)                           "101"           "%b"
   printf("%#x", 255)                        "0xff"          "ff"
   printf("%#o", 8)                          "010"           "?"
   printf("%p", (void *)0x1000)              "0x1000"        "0x1000"
   printf("%.3d", 5)                         "005"           "?"
   printf("%8.3d", 5)                        "     005"      "       ?"
   printf("%.3s", "abcde")                   "abc"           "abc"
   printf("%o", 8)                           "10"            "?"
   printf("%-5d|", 42)                       "42   |"        "   42|"
   printf("%-5s|", "ab")                     "ab   |"        "   ab|"
   printf("%hhd", 300)                       "44"            "?"
   printf("%hd", 70000)                      "4464"          "?"
   printf("%lld", 123456789012LL)            "123456789012"  "?"
   printf("%llx", 0x123456789abULL)          "123456789ab"   "?"
   printf("%jd", (intmax_t)-1)               "-1"            "?"
   printf("%ld", 1234567L)                   "1234567"       "1234567"
   printf("%Lf", (long double)1.5)           "1.500000"      "?"
   printf("%La", (long double)1.5)           "0x1.8p+0"      "?"
   printf("%w64d", (int64_t)123456789012LL)  "123456789012"  "%w64d"
   printf("%2$s %1$s", "world", "hello")     "hello world"   "%$s %1$s"
   printf("%05d", 42)                        "00042"         "00042"

Correct and wrong usage
=======================

If the compiler accepts the call with ``-Wall -Wextra -Wformat=2``, it is correct
usage and ``printf()`` must handle it.  If the compiler warns, it is wrong usage.

These are outside the guarantee, because ``printf()`` cannot detect them:

- A wild pointer for ``%s``.
- An argument of the wrong type, such as a ``double`` for ``%d`` or an ``int``
  for ``%s``.
- ``printf(NULL)``: ``NULL`` is not a format string.
- A mix of numbered and sequential conversions.
- A wild pointer for ``%n`` with ``LIBC_PRINTF_PERCENT_N`` on.

What is outside the guarantee
=============================

The guarantee is about safety.  The output of the floating point engine is not
exact, and some forms do not match the standard yet:

.. code-block:: text

   printf("%.0f", 1e22)     "10000000000000000000000"
   printf("%.20f", 0.1)     "0.10000000000000000000"
   printf("%.17e", 0.1)     "1.00000000000000000e-01"
   printf("%f", 1e300)      "1000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000.000000"
   printf("%g", 1234567.0)  "1.23457e+06"
   printf("%#g", 1.0)       "1."
   printf("%Lf", 1e300L)    "1000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000.000000"

- ``%e``, ``%f`` and ``%g`` have at most 15 significant digits.  The digits after
  them print as 0.  The last digits can differ from a correctly rounded result,
  and ``%.0f`` does not round.
- ``%#g`` does not keep the trailing zeros.
- ``%lc`` and ``%ls`` without ``LIBC_LOCALE`` print narrow characters.
- A negative precision from ``*`` counts as 0, and not as omitted.
- ``lib_bsprintf()`` has its own smaller format: see :doc:`stream`.

Flash and stack
===============

Each number comes from a build of the board's configuration with ``-Os``, the
``size`` tool, the linker map and ``-fstack-usage``.  The boards are
``nucleo-f072rb:nsh`` (Cortex-M0), ``nucleo-f103rb:nsh`` (Cortex-M3),
``rv-virt:nsh`` (32-bit RISC-V, with an FPU) and ``qemu-armv8a:nsh`` (arm64).
Other boards and compilers give other numbers, so use them to compare.


Image text, in bytes:

.. list-table::
   :header-rows: 1

   * - 
     - Cortex-M0
     - Cortex-M3
     - rv32
     - arm64
   * - Minimal
     - 46,380
     - 33,860
     - 227,988
     - 325,584
   * - Default
     - 46,724
     - 34,436
     - 230,462
     - 328,268
   * - Full
     - 58,856
     - 40,460
     - 233,046
     - 331,896
   * - ``LIBC_PRINTF_HEXFLOAT`` added to Minimal
     - +836
     - +820
     - +1048
     - +1472
   * - ``LIBC_PRINTF_C23`` added to Minimal
     - +160
     - +228
     - +166
     - +192

The objects of printf (text and data), in bytes:

.. list-table::
   :header-rows: 1

   * - 
     - Cortex-M0
     - Cortex-M3
     - rv32
     - arm64
   * - Minimal
     - 1,511
     - 1,291
     - 1,933
     - 2,781
   * - Default
     - 1,839
     - 1,797
     - 4,445
     - 5,465
   * - Full
     - 5,439
     - 5,281
     - 6,901
     - 8,849

``libgcc`` in the image, in bytes:

.. list-table::
   :header-rows: 1

   * - 
     - Cortex-M0
     - Cortex-M3
     - rv32
     - arm64
   * - Minimal
     - 2,028
     - 964
     - 4,470
     - 361
   * - Default
     - 2,028
     - 1,012
     - 4,546
     - 361
   * - Full
     - 10,432
     - 3,400
     - 4,546
     - 361

Stack frame of ``vsprintf_internal()``, in bytes:

.. list-table::
   :header-rows: 1

   * - 
     - Cortex-M0
     - Cortex-M3
     - rv32
     - arm64
   * - Minimal
     - 80
     - 72
     - 96
     - 160
   * - Default
     - 88
     - 72
     - 144
     - 160
   * - Full
     - 184
     - 176
     - 208
     - 256

Floating point is off by default on the two Cortex boards, and on by default on the other two.  Full has it on everywhere.

Floating point on a CPU with no FPU pulls in the soft-float code of ``libgcc``,
which is most of its cost: see the row of ``libgcc``.  A program that already
uses ``double`` has that code anyway.  ``%lld`` needs a 64-bit division on a
32-bit CPU, and the kernel clock code often needs it too.

``LIBC_NUMBERED_ARGS`` also adds a variable length array to the stack of
``lib_vsprintf()``: one union for each argument that the format uses.  The size
of the union is 8 bytes, or the size of ``long double`` with
``LIBC_PRINTF_HEXFLOAT`` on a CPU where that is larger.
