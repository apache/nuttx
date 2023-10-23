=====================
``poll`` Poll example
=====================

A test of the ``poll()`` and ``select()`` APIs using FIFOs and, if available,
``stdin``, and a TCP/IP socket. In order to use the TCP/IP select test, you must
have the following things selected in your NuttX configuration file:

- ``CONFIG_NET``                        – Defined for general network support.
- ``CONFIG_NET_TCP``                    – Defined for TCP/IP support.
- ``CONFIG_NET_NTCP_READAHEAD_BUFFERS`` – Defined to be greater than zero.
- ``CONFIG_EXAMPLES_POLL_NOMAC``        – (May be defined to use software assigned
  MAC)
- ``CONFIG_EXAMPLES_POLL_IPADDR``       – Target IP address.
- ``CONFIG_EXAMPLES_POLL_DRIPADDR``     – Default router IP address.
- ``CONFIG_EXAMPLES_POLL_NETMASK``      – Network mask.

In order to for select to work with incoming connections, you must also select:

- ``CONFIG_NET_TCPBACKLOG`` – Incoming connections pend in a backlog until
  ``accept()`` is called.

In additional to the target device-side example, there is also a host-side
application in this directory. It can be compiled under Linux or Cygwin as
follows::

  cd examples/usbserial
  make -f Makefile.host TOPDIR=<nuttx-directory> TARGETIP=<target-ip>

Where ``<target-ip>`` is the IP address of your target board.

This will generate a small program called 'host'. Usage:

1. Build the ``examples/poll`` target program with TCP/IP poll support and start
   the target.

2. Then start the host application::

   ./host

The host and target will exchange are variety of small messages. Each message
sent from the host should cause the select to return in target. The target
example should read the small message and send it back to the host. The host
should then receive the echo'ed message.

If networking is enabled, applications using this example will need to provide
the following definition in the ``defconfig`` file to enable the networking
library:

- ``CONFIG_NETUTILS_NETLIB=y``
