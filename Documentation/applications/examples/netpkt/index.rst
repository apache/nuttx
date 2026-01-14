====================================
``netpkt`` ``AF_PACKET`` Raw Sockets
====================================

A test of ``AF_PACKET``, raw sockets. Contributed by Lazlo Sitzer.

Overview
========

The ``netpkt`` example demonstrates the use of raw packet sockets (``AF_PACKET``)
for sending and receiving Ethernet frames at the link layer, bypassing the
TCP/IP protocol stack. This is useful for network protocol testing, driver
verification, and low-level network analysis.

Configuration
=============

- ``CONFIG_EXAMPLES_NETPKT=y`` – Enables the netpkt example.

Usage
=====

The ``netpkt`` program supports the following command-line options:

.. code-block:: bash

   netpkt [options]

Options:

- ``-a`` – Transmit and receive packets
- ``-r`` – Receive packets only
- ``-t`` – Transmit packets only
- ``-v`` – Verbose mode (display packet contents in hexadecimal)
- ``-i <IF>`` – Specify network interface name (e.g., ``eth0``)

Examples:

.. code-block:: bash

   # Send 3 packets on eth0 interface with verbose output
   netpkt -i eth0 -t -v

   # Receive packets on eth0 interface
   netpkt -i eth0 -r -v

   # Both send and receive on eth0
   netpkt -i eth0 -a -v

   # Send packets without specifying interface (uses default)
   netpkt -t
