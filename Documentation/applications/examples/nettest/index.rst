==================================
``nettest`` Client/Server Over TCP
==================================

This is a simple network test for verifying client- and server- functionality in
a TCP/IP connection.

- ``CONFIG_EXAMPLES_NETTEST=y`` – Enables the nettest example.
- ``CONFIG_EXAMPLES_NETLIB=y`` – The networking library in needed.

Configurations:

- Server on target hardware; client on host.
- Client on target hardware; server on host.
- Server and Client on different targets.
- Loopback configuration with both client and server on the same target.

See also ``examples/tcpecho``.
