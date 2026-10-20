=======
fnmatch
=======

``fnmatch()`` in ``libs/libc/misc/lib_fnmatch.c`` matches a string against
a shell-style pattern.  It is a small matcher, not a full POSIX
implementation.

Patterns
--------

- ``?`` matches one character.
- ``*`` matches any number of characters, but not ``/``.
- ``**`` matches any number of characters, ``/`` included.
- ``[...]`` matches one character of a set.  A set can hold characters and
  ranges, for example ``[a-zA-Z_]``.  ``[!...]`` is not supported.
- ``|`` separates patterns: the string matches if one of them matches.
- The other characters match themselves.  A backslash is not special.

Flags
-----

- ``FNM_CASEFOLD`` (or ``FNM_IGNORECASE``): upper and lower case letters
  are the same, also in sets and ranges.
- ``FNM_LEADING_DIR``: the pattern can match the string up to a ``/``, so
  ``usr/lib`` matches ``usr/lib/x``.
- ``FNM_PATHNAME``, ``FNM_PERIOD`` and ``FNM_NOESCAPE`` are defined but
  have no effect.
