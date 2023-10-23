=========================================
``termios`` Simple Termios interface test
=========================================

This directory contains a simple application that uses the termios interface
to change serial parameters. Just import a ``nsh`` config and enable the
following symbols:

- ``CONFIG_SERIAL_TERMIOS``   – Enable the termios support.
- ``CONFIG_EXAMPLES_TERMIOS`` – Enable the example itself.
