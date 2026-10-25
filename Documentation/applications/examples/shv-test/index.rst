==============================================
``shv-test`` Silicon Heaven Protocol showcase
==============================================

This example shows how to build a Silicon Heaven (SHV) device tree with
``libshvc`` (``apps/netutils/libshvc``) and how to connect it to an SHV
broker over TCP/IP. The same tree can be allocated in three different ways,
selected at runtime.

Configuration
-------------

- ``CONFIG_EXAMPLES_SHV_TEST`` – Enable the example. Depends on
  ``CONFIG_NETUTILS_LIBSHVC``.
- ``CONFIG_EXAMPLES_SHV_TEST_PROGNAME`` – Program name. Default: ``shv_test``.
- ``CONFIG_EXAMPLES_SHV_TEST_PRIORITY`` – Task priority. Default: ``100``.
- ``CONFIG_EXAMPLES_SHV_TEST_STACKSIZE`` – Task stack size. Default: ``4096``.

Created nodes
-------------

All nodes are mounted under the mount point given by ``-m``. Every node also
provides the standard ``dir`` and ``ls`` methods.

=================  ============================================================
Node               Methods
=================  ============================================================
``.app``           ``shvVersionMajor``, ``shvVersionMinor``, ``name``,
                   ``version``, ``ping``, ``date``
``.device``        ``name``, ``version``, ``serialNumber``, ``uptime``,
                   ``reset``
``nuttxTesting``   ``setTestingVal`` – store an integer,
                   ``getTestingVal`` – return the stored integer,
                   ``asciiArt`` – print an SHV logo to the device console
=================  ============================================================

Usage
-----

::

  shv_test -a <0-2> -m <mount-point> -i <ip-addr> -t <port> -u <user> -p <passwd>

- ``-a`` – tree allocation type: ``0`` GAVL, ``1`` GSA, ``2`` GSA with the
  nodes placed statically in ``.rodata``.
- ``-m`` – mount point in the SHV broker.
- ``-i`` – IPv4 address of the SHV broker.
- ``-t`` – TCP port of the SHV broker.
- ``-u`` / ``-p`` – user name and password for the broker.
- ``-h`` – print help.

All options except ``-a`` (default GAVL) and ``-h`` are mandatory. A typical
session looks like this::

  nsh> shv_test -a 2 -m test/nuttxdevice -i 192.168.1.10 -t 3755 -u xyz -p xyz &

The device can then be accessed through a SHV broker
(`<https://github.com/silicon-heaven/shvspy>`).
Connect it to the same broker and browse to the mount point to list the nodes
and call their methods.

The application runs until it receives ``SIGTERM`` (e.g. ``kill -15 <pid>``)
or until an error is reported by the SHV connection.
