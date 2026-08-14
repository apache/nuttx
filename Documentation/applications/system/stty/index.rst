=================================
``stty`` Terminal Configuration
=================================

Description
-----------

The ``stty`` command provides a standard Unix-like interface for viewing
and configuring terminal (TTY) device settings at runtime via termios.
It supports raw/cooked mode switching, individual flag control, baud
rate configuration, and VMIN/VTIME parameters.

Usage
-----

Basic Syntax
^^^^^^^^^^^^

.. code-block:: bash

   stty [options] [settings...]

Options:

============== ==========================================================
``-F <dev>``   Operate on the specified device instead of stdin
``-h``         Show help message
============== ==========================================================

Settings
^^^^^^^^

=============  ==========================================
Setting        Description
=============  ==========================================
``raw``        Set raw mode (no input/output processing)
``cooked``     Set cooked mode (canonical with echo)
``echo``       Enable echo
``-echo``      Disable echo
``icanon``     Enable canonical mode
``-icanon``    Disable canonical mode
``icrnl``      Map CR to NL on input
``-icrnl``     Don't map CR to NL
``onlcr``      Map NL to CR-NL on output
``-onlcr``     Don't map NL to CR-NL
``opost``      Enable output processing
``-opost``     Disable output processing
``isig``       Enable signal characters
``-isig``      Disable signal characters
``inlcr``      Map NL to CR on input
``-inlcr``     Don't map NL to CR on input
``igncr``      Ignore CR on input
``-igncr``     Don't ignore CR on input
``ocrnl``      Map CR to NL on output
``-ocrnl``     Don't map CR to NL on output
``speed N``    Set baud rate
``min N``      Set VMIN (minimum chars for read, 0-255)
``time N``     Set VTIME (read timeout in 0.1s, 0-255)
=============  ==========================================

Examples
^^^^^^^^

.. code-block:: bash

   # Display current settings for /dev/ttyS0
   nsh> stty -F /dev/ttyS0

   # Set raw mode for binary communication
   nsh> stty -F /dev/ttyS0 raw -echo

   # Set cooked mode for interactive terminal
   nsh> stty -F /dev/ttyS0 cooked

   # Configure baud rate
   nsh> stty -F /dev/ttyS1 speed 115200

   # Set non-blocking read with timeout
   nsh> stty -F /dev/ttyS0 min 0 time 10

   # Display current settings for stdin
   nsh> stty

Configuration
-------------

**CONFIG_SYSTEM_STTY**

Options:

* **CONFIG_SYSTEM_STTY_PRIORITY** - Task priority (default 100)
* **CONFIG_SYSTEM_STTY_STACKSIZE** - Stack size (default
  ``DEFAULT_TASK_STACKSIZE``)

Dependencies
^^^^^^^^^^^^

* **CONFIG_SERIAL_TERMIOS** - Required for termios support

