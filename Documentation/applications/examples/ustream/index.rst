===============================
``ustream`` Unix Stream Sockets
===============================

This is the same test as ``examples/udp`` and similar to ``examples/udgram``, but
using Unix domain stream sockets.

Dependencies:

- ``CONFIG_NET_LOCAL`` – Depends on support for Unix domain sockets.

Configuration:

- ``CONFIG_EXAMPLES_USTREAM`` – Enables the Unix domain socket example.
- ``CONFIG_EXAMPLES_USTREAM_ADDR`` – Specifics the Unix domain address. Default:
  ``/dev/fifo``.
