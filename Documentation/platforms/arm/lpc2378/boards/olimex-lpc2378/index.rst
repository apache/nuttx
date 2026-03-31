==============
olimex-lpc2378
==============

The Olimex LPC2378 board has only a small amount of board-specific porting
information recorded in the original notes.

Board Notes
===========

* ``EXT1``: ``P3[0:7]`` are used for LEDs.
* ``UEXT`` pins 3 and 4 provide ``TXD2`` and ``RXD2`` for UART2.

Known Limitations
=================

The board used for this port still had the ``-`` LPC2378 revision, even though
it was purchased more than two years after the updated chip revision was
released. With that hardware, NuttX would not run correctly with the MAM fully
enabled.

Console Setup
=============

Testing was done with an FTDI ``ft232`` USB-to-serial adapter and NSH piping,
as shown in the original board screenshot.
