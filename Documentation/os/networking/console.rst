===================
Console over NETDEV
===================

Console over NETDEV provides a remote interactive system console (stdin/stdout)
over a network instead of a serial port. It use bidirectional communication over
UDP.

To connect to the console on the default port 55000 you can use ``socat``::

  socat -d -d UDP-LISTEN:55000,reuseaddr STDOUT

NOTE: you should first start ``socat``, then start NuttX, so that the console
driver has a chance to connect to your host.
