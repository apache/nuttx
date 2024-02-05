==============================
``udp`` Client/Server Over UDP
==============================

This is a simple network test for verifying client- and server- functionality
over UDP.

Applications using this example will need to enabled the following ``netutils``
libraries in the ``defconfig`` file:

- ``CONFIG_NETUTILS_NETLIB=y``

Possible configurations:

- Server on target hardware; client on host.
- Client on target hardware; Server on host.
- Server and Client on different targets.
