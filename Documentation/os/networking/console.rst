===================
Console over NETDEV
===================

Console over NETDEV provides a remote interactive system console (stdin/stdout)
over a network interface. At this moment UDP and ISO-TP protocols are supported.

UDP
===

It uses bidirectional communication over UDP.

To connect to the console on the default port 55000 you can use ``socat``::

  socat -d -d UDP-LISTEN:55000,reuseaddr STDOUT

NOTE: you should first start ``socat``, then start NuttX, so that the console
driver has a chance to connect to your host.

ISO-TP
======

It uses bidirectional communication over CAN network with ISO-TP protocol.

To create an ISO-TP to PTY bridge you can use ``tools/isotp_pty.py``
script. For default ISO-TP console configuration use::

  python tools/isotp_pty.py can0 7d0 7d1

Then you can connect to the created PTY console with Minicom::

  minicom -D /tmp/isotp-pty

