==============================
``conntrack`` connection track
==============================

The ``conntrack`` command is used to display and monitor connection tracking
entries in the NuttX kernel. It is similar to Linux's ``conntrack`` tool.

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

Commands
========

``-L, --dump``
   List all connection tracking entries.

``-E, --event``
   Display a real-time event log of connection tracking changes.
   Press Ctrl+C to stop monitoring.

Options
=======

``-f, --family PROTO``
   Specify the L3 protocol family. Only valid with ``-L``.

   Supported values:

   - ``ipv4`` (default): Show IPv4 connection tracking entries.
   - ``ipv6``: Show IPv6 connection tracking entries.

Output Format
=============

Each connection tracking entry is displayed in the following format:

.. code-block:: text

   PROTO src=SRC_ADDR dst=DST_ADDR sport=SPORT dport=DPORT src=REPLY_SRC dst=REPLY_DST sport=REPLY_SPORT dport=REPLY_DPORT

For ICMP/ICMPv6 entries, the format uses ``type``, ``code``, and ``id``
instead of ``sport`` and ``dport``:

.. code-block:: text

   icmp src=SRC_ADDR dst=DST_ADDR type=TYPE code=CODE id=ID src=REPLY_SRC dst=REPLY_DST type=REPLY_TYPE code=REPLY_CODE id=REPLY_ID

Event mode prefixes each entry with an event type:

- ``[NEW]``: A new connection tracking entry was created.
- ``[DESTROY]``: A connection tracking entry was removed.

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
       [NEW] tcp src=10.0.0.1 dst=10.0.0.2 sport=12345 dport=80 src=10.0.0.2 dst=10.0.0.1 sport=80 dport=12345
   [DESTROY] tcp src=10.0.0.1 dst=10.0.0.2 sport=12345 dport=80 src=10.0.0.2 dst=10.0.0.1 sport=80 dport=12345

See Also
========

- :doc:`../iptables/index`
- :doc:`../ip6tables/index`
