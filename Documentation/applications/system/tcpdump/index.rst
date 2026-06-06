===========================
``tcpdump`` tcpdump command
===========================

Captures network packets from a specified interface and writes them to a file
in `pcap <https://www.tcpdump.org/>`__ format. The resulting capture file can
be analyzed with tools such as Wireshark or ``tcpdump`` on a host machine.

Configuration
=============

Enable the command with ``CONFIG_SYSTEM_TCPDUMP``. This option requires
``CONFIG_NET_PKT`` (raw packet socket support) and automatically selects
``CONFIG_SYSTEM_ARGTABLE3`` for command-line argument parsing.

The following configuration options are available:

``CONFIG_SYSTEM_TCPDUMP_PROGNAME``
  Program name for the ``tcpdump`` command. Default: ``tcpdump``.

``CONFIG_SYSTEM_TCPDUMP_PRIORITY``
  Task priority. Default: ``100``.

``CONFIG_SYSTEM_TCPDUMP_STACKSIZE``
  Stack size. Default: ``4096``.

Usage
=====

.. code-block:: console

   nsh> tcpdump -i <interface> -w <file> [-s <snaplen>]

Options
=======

``-i <interface>``, ``--interface <interface>``
  Network interface to capture from (e.g. ``eth0``). Required.

``-w <file>``
  Path to the output pcap file. Required.

``-s <snaplen>``, ``--snapshot-length <snaplen>``
  Maximum number of bytes to capture per packet. Optional.
  Default: ``262144``.

Examples
========

Capture all packets on ``eth0`` and save to a file:

.. code-block:: console

   nsh> tcpdump -i eth0 -w /tmp/capture.pcap
   ^C

Capture with a limited snapshot length:

.. code-block:: console

   nsh> tcpdump -i eth0 -w /tmp/capture.pcap -s 1500
   ^C

Copy the capture file to a host machine for analysis with Wireshark:

.. code-block:: console

   nsh> cp /tmp/capture.pcap /mnt/capture.pcap

Notes
=====

- The output file uses the pcap format (version 2.4, nanosecond resolution)
  which is compatible with Wireshark, ``tcpdump``, and other standard capture
  analysis tools.
- The command captures on the specified interface until interrupted with
  ``Ctrl-C`` (``SIGINT``).
- The link-layer type is detected automatically: ``LINKTYPE_ETHERNET`` (1)
  for Ethernet interfaces, or ``LINKTYPE_RAW`` (101) for other interfaces
  such as SLIP or tun.
- Packets are timestamped using ``CLOCK_REALTIME``. Ensure the system clock
  is set correctly for meaningful timestamps in the capture file.
- The capture requires ``CONFIG_NET_PKT`` to be enabled for raw packet
  socket support.
