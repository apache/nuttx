=============
Vexriscv Core
=============

The vexriscv core only supports standard "Flat builds", consisting of a single binary.

Configuration
-------------

For vexriscv, the linux CPU variant is required.  Please consult the appropriate board
documentation for flashing gateware.

If you use the secure CPU variant, you may encounter a kernel panic on startup - please see
https://github.com/apache/nuttx/pull/17494 for an example.

Building
--------

Build the minimal NSH application::

   # Configure for NSH
   $ ./tools/configure.sh arty_a7:nsh

   # Build Nuttx
   $ make


Booting
--------

Create a file, 'boot.json' in the Nuttx root directory, with the following content::

  {
    "nuttx.bin": "0x40000000",
    "board.dtb": "0x41ec0000"
  }

Load the application over serial with::

   $ litex_term --images=boot.json --speed=1e6 /dev/ttyUSB0

Update the baud rate and serial port to suit your configuration.



