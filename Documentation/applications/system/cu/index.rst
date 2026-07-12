=================================
``cu`` CU minimal serial terminal
=================================

Description
-----------

The ``cu`` application is a minimalistic implementation of the classic
``cu`` ("call up") terminal program from Taylor UUCP. It connects the
current console bidirectionally to another serial device: everything
typed on the console is sent to the device, and everything received
from the device is printed on the console.

This effectively turns the board into a serial bridge, which is handy
to talk directly to devices attached to the system (GNSS receivers,
GSM/LTE modems, RS-485 buses, or another board) using the NuttX
console as a passthrough, without writing any application code.

Usage
-----

Basic Syntax
^^^^^^^^^^^^

.. code-block:: bash

   cu [options]

Options:

============== ==========================================================
``-l <dev>``   Use named device (default
               ``CONFIG_SYSTEM_CUTERM_DEFAULT_DEVICE``, normally
               ``/dev/ttyS0``)
``-s <baud>``  Use given speed (default
               ``CONFIG_SYSTEM_CUTERM_DEFAULT_BAUD``)
``-e``         Set even parity
``-o``         Set odd parity
``-f``         Disable RTS/CTS flow control (default: on)
``-c``         Disable LF -> CRLF conversion (default: off)
``-E <char>``  Set the escape character (default ``~``). Use ``-E ''``
               to eliminate the escape character (raw 8-bit transfers)
``-?``         Show help
============== ==========================================================

The ``-s``, ``-e``, ``-o`` and ``-f`` options require
``CONFIG_SERIAL_TERMIOS``.

Escape Sequences
^^^^^^^^^^^^^^^^

At the beginning of a line, the escape character (default ``~``)
introduces a command:

======= ====================================
``~.``  Hang up and drop back to the shell
``~?``  Show the available escape sequences
======= ====================================

When talking to binary protocols (firmware upload, binary GPS
protocols, etc.), disable the escape character with ``-E ''`` so
every byte is forwarded untouched.

Example
^^^^^^^

Bridging the console to a device attached to a second serial port:

.. code-block:: bash

   nsh> cu -l /dev/ttyS1 -s 115200
   [everything typed here is transmitted on the serial port,
    and received data is printed to the console]
   ~.
   nsh>

Bridging two serial ports
^^^^^^^^^^^^^^^^^^^^^^^^^

``cu`` always bridges the *console* to one device. To bridge two
serial ports directly to each other, leaving the console free, use
the shell output redirection in background instead:

.. code-block:: bash

   nsh> cat /dev/ttyS1 > /dev/ttyS2 &
   nsh> cat /dev/ttyS2 > /dev/ttyS1 &

With both commands running the bridge is bidirectional: everything
received on one port is retransmitted on the other, in both
directions. Running only one of them creates a unidirectional
bridge, which is useful to forward or monitor a single direction.
Use ``kill`` on the background task PIDs to undo the bridge.

The two ports do not need the same line configuration: each one
keeps its own baud rate and framing (as set by ``CONFIG_UARTx_BAUD``
or changed by an application through termios), so the bridge also
works as a rate adapter between buses running at different speeds.

Configuration
-------------

**CONFIG_SYSTEM_CUTERM**

Options:

* **CONFIG_SYSTEM_CUTERM_DEFAULT_DEVICE** - Serial device used when
  ``-l`` is not given (default ``/dev/ttyS0``)
* **CONFIG_SYSTEM_CUTERM_DEFAULT_BAUD** - Baud rate used when ``-s``
  is not given
* **CONFIG_SYSTEM_CUTERM_DISABLE_ERROR_PRINT** - Suppress error
  messages to reduce size

Dependencies
^^^^^^^^^^^^

* **CONFIG_SERIAL_TERMIOS** - Optional, required for the baud rate,
  parity and flow control options; without it ``cu`` uses the device
  as configured by the driver

Limitations
-----------

* File transfer escape commands (``~>``/``~<``) are not implemented;
  for file transfers see ``apps/system/zmodem`` or ``ymodem``.
* ``cu`` bridges the *current console* to one device. It does not run
  as a background daemon bridging two arbitrary ports; for that, see
  `Bridging two serial ports`_ above.
