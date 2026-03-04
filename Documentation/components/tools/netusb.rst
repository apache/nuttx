=============
``netusb.sh``
=============

Helper script used to set up the CDC ECM Ethernet Over USB driver,
host routes, and IP Tables rules to support networking with a NuttX
system that has a CDC ECM Ethernet Over USB driver configured. Only
supported on Linux.

General usage:

.. code:: console

   $ ./tools/netusb.sh
   Usage: tools/netusb.sh <main-interface> <usb-net-interface> <on|off>

This has been tested on the SAMA5D3-Xplained board; see
`Documentation/platforms/arm/sama5/boards/sama5d3-xplained/README.txt`
for more information on how to configure the CDC ECM driver for that board.
