==============================================
``termcurses`` Terminal Curses control support
==============================================

Terminal emulation library for NuttX

The Termcurses library provides terminal emulation support for performing common
screen actions such as cursor movement, foreground / background color control
and keyboard escape sequence mapping. The initial release supports only ``vt100``
/ ``ansi`` terminal types, but the library architecture has an extensible
interface to allow support for additional emulation types if needed.

The library can be used standalone or in conjunction with the
``apps/graphics/pdcurses`` libraries. The pdcurses libraries have been updated
with a _termcurses_ config option which fully integrates the termcurses library
automatically.

Usage
-----

To use the termcurses library, the routines must be initialized by calling the
``termcurses_initterm()`` function. This routine accepts a terminal type string
identifying the type of terminal emulation support requested. If a ``NULL``
pointer is passed, then the routine will check for a ``TERM`` environment variable
and set the terminal type based on that string. If the emulation type still
cannot be determined, the routine will default to ``vt100`` emulation type.

Upon successful initialization, the ``termcurses_initterm()`` function will
allocate an new terminal context which must be passed with all future termcurses
library functions. When this context is no longer needed, the
``termcurses_deinitterm()`` routine should be called for proper freeing and
terminal teardown.

Use with ``telnetd``
--------------------

When using termcurses with the telnet daemon, the telnet config option
``CONFIG_TELNET_SUPPORT_NAWS`` should be enabled. This option adds code to the
telnet library for terminal size negotiation. Without this option, the telnet
routines have no concept of the terminal size, and therefore the termcurses
routines must default to ``80x24`` screen mode.

Use with ``pdcurses``
---------------------

When using the pdcurses termcurses support (i.e you have enabled both the
``CONFIG_PDCURSES`` and ``CONFIG_TERMCURSES`` options),, the pdcurses input device
should be selected to be ``TERMINPUT`` (i.e. set ``CONFIG_PDCURSES_TERMINPUT=y``).
This causes the pdcurses keyboard input logic to use ``termcurses_getkeycode()``
routine for curses input.


Author: Ken Pettit
Date: 2018-2019
