============================
``ping`` ICMP "ping" command
============================

Overview
--------
The ``ping`` application sends ICMPv4 Echo Request packets to a target host
and reports replies, packet loss, round-trip time (RTT) statistics, and
basic errors. It is useful for verifying IP connectivity and measuring
latency.

Requirements
------------
- Networking enabled: ``CONFIG_NET=y``
- ICMP and raw socket support in the OS
- Build the app: ``CONFIG_SYSTEM_PING=y``
- Optional DNS resolution: ``CONFIG_LIBC_NETDB`` and ``CONFIG_NETDB_DNSCLIENT``
- Optional device binding support for ``-I``: ``CONFIG_NET_BINDTODEVICE``

Synopsis
--------
::

	ping [-c <count>] [-i <interval>] [-W <timeout>] [-s <size>] [-I <interface>] <hostname|ip-address>

Where ``<hostname>`` may be a DNS name (with DNS client enabled) or an IPv4
address. Without DNS, an IPv4 address is required.

Options
-------
- ``-c <count>``: Number of echo requests to send (default: implementation-defined, typically 10).
- ``-i <interval>``: Delay between requests in milliseconds (default: 1000 ms).
- ``-W <timeout>``: Per-reply timeout in milliseconds (default: 1000 ms).
- ``-s <size>``: Number of ICMP payload bytes to send (default: 56).
- ``-I <interface>``: Bind socket traffic to a specific network device name (requires ``CONFIG_NET_BINDTODEVICE``).
- ``-h``: Show help and exit.

Output
------
For each reply, ``ping`` prints a line similar to:

::

	56 bytes from 10.0.2.2: icmp_seq=3 time=6.0 ms

On timeout:

::

	No response from 10.0.2.2: icmp_seq=3 time=1000 ms

At completion, summary statistics are printed, including packets transmitted,
received, loss percentage, total time, and RTT min/avg/max/mdev:

::

	10 packets transmitted, 10 received, 0% packet loss, time 10011 ms
	rtt min/avg/max/mdev = 0.000/0.600/6.000/1.800 ms

Exit Status
-----------
- Success (0): Completed without fatal errors.
- Failure (!=0): A fatal error was reported (e.g., socket/DNS error, invalid arguments).

Examples
--------
Basic connectivity to an IP:

.. code-block:: bash

	ping 1.1.1.1

Ping a hostname with custom request count and payload size:

.. code-block:: bash

	ping -c 3 -s 100 example.com

Reduce the interval and timeout for faster probing:

.. code-block:: bash

	ping -i 500 -W 500 10.0.2.2

Bind traffic to a specific interface (requires ``CONFIG_NET_BINDTODEVICE``):

.. code-block:: bash

	ping -I wlan0 10.0.2.2

If the device name is invalid, ``ping`` reports a bind error and terminates.

Notes
-----
- Device binding (``-I``) is useful when multiple interfaces exist or during
	early network setup when routing is ambiguous. It forces traffic to use the
	specified device.
- DNS resolution of ``<hostname>`` requires the DNS client configuration;
	otherwise, provide an IPv4 address.
- ICMPv6 support is provided by a separate ``ping6`` application when enabled
	(``CONFIG_NETUTILS_PING6``) with similar options and output.

Troubleshooting
---------------
- ``ERROR: ping_gethostip(...) failed``: DNS lookup failed or invalid address.
- ``ERROR: socket() failed: <errno>``: Raw socket creation failed.
- ``ERROR: setsockopt error: <errno>``: Device bind (``-I``) failed; check interface name and ``CONFIG_NET_BINDTODEVICE``.
- ``ERROR: poll/recvfrom failed``: Link issues or network stack errors.

Implementation Details
----------------------
Internally, the app uses ``apps/netutils/ping/icmp_ping.c`` to drive ICMP
Echo requests and parse replies. The command-line interface and printing
logic are implemented in ``apps/system/ping/ping.c``.
