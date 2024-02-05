===================
``wget`` Web Client
===================

A simple web client example. It will obtain a file from a server using the HTTP
protocol. Settings unique to this example include:

- ``CONFIG_EXAMPLES_WGET_URL`` – The URL of the file to get
- ``CONFIG_EXAMPLES_WGET_NOMAC`` – (May be defined to use software assigned MAC)
- ``CONFIG_EXAMPLES_WGET_IPADDR`` – Target IP address
- ``CONFIG_EXAMPLES_WGET_DRIPADDR`` – Default router IP address
- ``CONFIG_EXAMPLES_WGET_NETMASK`` – Network mask

This example uses ``netutils/webclient``. Additional configuration settings apply
to that code as follows (but built-in defaults are probably OK):

- ``CONFIG_WEBCLIENT_GETMIMETYPE``
- ``CONFIG_WEBCLIENT_MAXHTTPLINE``
- ``CONFIG_WEBCLIENT_MAXMIMESIZE``
- ``CONFIG_WEBCLIENT_MAXHOSTNAME``
- ``CONFIG_WEBCLIENT_MAXFILENAME``

Of course, the example also requires other settings including ``CONFIG_NET`` and
``CONFIG_NET_TCP``. The example also uses the uIP resolver which requires
``CONFIG_UDP``.

**Warning**: As of this writing, ``wget`` is untested on the target platform. At
present it has been tested only in the host-based configuration described in the
following note. The primary difference is that the target version will rely on
the also untested uIP name resolver.

**Note**: For test purposes, this example can be built as a host-based ``wget``
function. This can be built as follows::

  cd examples/wget
  make -f Makefile.host

Applications using this example will need to enable the following ``netutils``
libraries in the ``defconfig`` file: ::

  CONFIG_NETUTILS_NETLIB=y
  CONFIG_NETDB_DNSCLIENT=y
  CONFIG_NETUTILS_WEBCLIENT=y
