============================
``conntrack`` connection tracking
============================

The ``conntrack`` command is used to list and monitor connection tracking
entries in the NuttX kernel, similar to the Linux conntrack tool. It
communicates with the kernel via Netlink (``NETLINK_NETFILTER``).

Configuration
=============

- ``CONFIG_SYSTEM_CONNTRACK``
- ``CONFIG_NETLINK_NETFILTER``

The following additional options are available:

- ``CONFIG_SYSTEM_CONNTRACK_PRIORITY`` - Task priority
  (default: 100)
- ``CONFIG_SYSTEM_CONNTRACK_STACKSIZE`` - Stack size
  (default: ``DEFAULT_TASK_STACKSIZE``)

Usage
=====

.. code-block:: text

   conntrack -L [-f family]
   conntrack -E

Options
=======

``-L, --dump``
   List all connection tracking entries. For each entry, the protocol,
   original tuple (source, destination, ports), and reply tuple are
   displayed.

``-E, --event``
   Display a real-time event log of connection tracking changes. New
   connections are shown with ``[NEW]`` and destroyed connections with
   ``[DESTROY]``. Press Ctrl+C to stop monitoring.

``-f, --family PROTO``
   Specify the L3 protocol family for the ``-L`` (dump) option. Valid
   values are ``ipv4`` (default) and ``ipv6``. This option is only
   valid with ``-L``.

Output Format
=============

Each connection tracking entry is displayed in the following format:

.. code-block:: text

   proto orig reply

Where:

- ``proto``: Protocol name (``tcp``, ``udp``, ``icmp``, or ``icmp6``)
- ``orig``: Original direction tuple (``src=``, ``dst=``, ``sport=``/``type=``,
  ``dport=``/``code=``/``id=``)
- ``reply``: Reply direction tuple (same format as orig)

For TCP/UDP entries, the port numbers are shown. For ICMP/ICMPv6 entries,
the type, code, and id are shown instead.

Examples
========

List all IPv4 connection tracking entries:

.. code-block:: text

   nsh> conntrack -L

List all IPv6 connection tracking entries:

.. code-block:: text

   nsh> conntrack -L -f ipv6

Monitor connection tracking events in real-time:

.. code-block:: text

   nsh> conntrack -E

Sample output for ``conntrack -L``:

.. code-block:: text

   tcp   src=10.0.0.1 dst=10.0.0.2 sport=12345 dport=80 src=10.0.0.2 dst=10.0.0.1 sport=80 dport=12345
   udp   src=10.0.0.1 dst=10.0.0.2 sport=54321 dport=53 src=10.0.0.2 dst=10.0.0.1 sport=53 dport=54321
   conntrack: 2 flow entries have been shown.

Sample output for ``conntrack -E``:

.. code-block:: text

       [NEW] tcp   src=10.0.0.1 dst=10.0.0.2 sport=12345 dport=80 src=10.0.0.2 dst=10.0.0.1 sport=80 dport=12345
   [DESTROY] tcp   src=10.0.0.1 dst=10.0.0.2 sport=12345 dport=80 src=10.0.0.2 dst=10.0.0.1 sport=80 dport=12345

See Also
========

- :doc:`../iptables/index`
- :doc:`../ip6tables/index`
