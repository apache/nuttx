===================================
``ping6`` ICMPv6 ECHO request tool
===================================

Overview
========

The ``ping6`` command sends ICMPv6 ECHO_REQUEST packets to a remote
IPv6 host and reports the replies.  It is the IPv6 counterpart of the
IPv4 ``ping`` command and is used to verify IPv6 connectivity and
measure round-trip times.

The command takes a single required argument: either a numeric IPv6
address or, when DNS resolution is enabled, a hostname.  It sends a
configurable number of packets at a fixed interval and prints a
per-reply summary followed by aggregate statistics (packet loss and
round-trip time min/avg/max/mdev).

Configuration
=============

Enable the command with ``CONFIG_SYSTEM_PING6`` (tristate).  This
option selects ``CONFIG_NETUTILS_PING6``, which provides the
underlying ICMPv6 ping library.

Additional configuration symbols:

- ``CONFIG_SYSTEM_PING6_PROGNAME`` — Program name registered with
  NSH (default ``"ping6"``).
- ``CONFIG_SYSTEM_PING6_PRIORITY`` — Task priority (default
  ``100``).
- ``CONFIG_SYSTEM_PING6_STACKSIZE`` — Task stack size (default
  ``DEFAULT_TASK_STACKSIZE``).

The ``-I`` option for binding to a specific network interface requires
``CONFIG_NET_BINDTODEVICE``.

When both ``CONFIG_LIBC_NETDB`` and ``CONFIG_NETDB_DNSCLIENT`` are
enabled, the ``<host>`` argument is resolved via DNS; otherwise only
numeric IPv6 addresses are accepted.

Usage
=====

.. code-block:: console

   ping6 [-c <count>] [-i <interval>] [-W <timeout>] [-s <size>]
         [-I <interface>] <host>
   ping6 -h

The ``<host>`` argument is required.  It must be an IPv6 address
(e.g. ``fe80::1``) or, when DNS is available, a hostname.

Options
=======

.. list-table::
   :header-rows: 1

   * - Option
     - Description
   * - ``-c <count>``
     - Number of ECHO requests to send.  Range: 1–65535.
       Default: ``10``.
   * - ``-i <interval>``
     - Delay between successive requests in milliseconds.  Range:
       1–65535.  Default: ``1000`` (1 second).
   * - ``-W <timeout>``
     - Time to wait for a reply before reporting a timeout, in
       milliseconds.  Range: 1–65535.  Default: ``1000`` (1 second).
   * - ``-s <size>``
     - Number of data bytes in each ECHO request packet.  Range:
       1–65535.  Default: ``56``.
   * - ``-I <interface>``
     - Bind the ping traffic to the specified network device (e.g.
       ``eth0``).  Only available when ``CONFIG_NET_BINDTODEVICE`` is
       enabled.
   * - ``-h``
     - Show usage information and exit.

Examples
========

Ping an IPv6 address with default settings (10 packets, 1-second
interval):

.. code-block:: console

   nsh> ping6 fe80::1

Send 5 pings with a 500 ms interval:

.. code-block:: console

   nsh> ping6 -c 5 -i 500 fe80::1

Ping with a larger packet size and 2-second timeout:

.. code-block:: console

   nsh> ping6 -s 1024 -W 2000 fe80::1

Bind ping traffic to a specific interface:

.. code-block:: console

   nsh> ping6 -I eth0 fe80::1

Ping a hostname (requires DNS):

.. code-block:: console

   nsh> ping6 myhost.example.com

Show usage:

.. code-block:: console

   nsh> ping6 -h

Output format
=============

Each successful reply prints a line showing the reply size, source
address, ICMP sequence number, and round-trip time::

   64 bytes from fe80::1 icmp_seq=0 time=1.234 ms

After all packets are sent, the command prints aggregate statistics::

   10 packets transmitted, 10 received, 0% packet loss, time 9001 ms
   rtt min/avg/max/mdev = 1.100/1.234/1.500/0.123 ms

Notes
=====

- The interval and timeout values are specified in milliseconds, not
  seconds (unlike some other ``ping`` implementations).
- The ``-I`` option is silently ignored if ``CONFIG_NET_BINDTODEVICE``
  is not enabled; an error message is printed in that case.
- The command uses the ``icmp6_ping()`` library function from
  ``netutils/icmpv6_ping.h``.
- Round-trip times are measured in microseconds internally and
  displayed with millisecond precision.
