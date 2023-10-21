``pdcurses``
============

This directory contains demonstration programs to show and test the capabilities
of ``pdcurses`` libraries. Some of them predate PDCurses, PCcurses or even
``pcurses``/``ncurses``. Although some PDCurses-specific code has been added, all
programs remain portable to other implementations (at a minimum, to ``ncurses``).

## Building

The demos are built by the platform-specific makefiles, in the platform
directories. There are no dependencies besides curses and the standard C
library, and no configuration is needed.

## Distribution Status

Public Domain, except for ``rain_main.c`` and ``worm_main.c``, which are under the
ncurses license (MIT-like).
