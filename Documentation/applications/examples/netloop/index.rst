===================================
``netloop`` Network loopback device
===================================

This is a simple test of the netwok loopback device. ``examples/nettest`` can also
be configured to provide (better) test of local loopback transfers. This version
derives from ``examples/poll`` and is focused on testing ``poll()`` with loopback
devices.

- ``CONFIG_EXAMPLES_NETLOOP=y`` – Enables the nettest example.

Dependencies:

- ``CONFIG_NET_LOOPBACK`` – Requires local loopback support.
- ``CONFIG_NET_TCP`` – Requires TCP support with the following:
   - ``CONFIG_NET_TCPBACKLOG``
   - ``CONFIG_NET_TCP_WRITE_BUFFERS``
- ``CONFIG_NET_IPv4`` – Currently supports only IPv4.
