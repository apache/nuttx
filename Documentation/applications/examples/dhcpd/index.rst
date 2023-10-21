``dhcpd`` DHCP Server
=====================

This examples builds a tiny DHCP server for the target system.

**Note**: For test purposes, this example can be built as a host-based DHCPD
server. This can be built as follows::

  cd examples/dhcpd
  make -f Makefile.host TOPDIR=<nuttx-directory>

NuttX configuration settings:

- ``CONFIG_NET=y`` – of course.
- ``CONFIG_NET_UDP=y`` – UDP support is required for DHCP (as well as various
  other UDP-related configuration settings).
- ``CONFIG_NET_BROADCAST=y`` – UDP broadcast support is needed.
- ``CONFIG_NETUTILS_NETLIB=y`` – The networking library is needed.
- ``CONFIG_EXAMPLES_DHCPD_NOMAC`` – (May be defined to use software assigned MAC)

See also ``CONFIG_NETUTILS_DHCPD_*`` settings described elsewhere and used in
``netutils/dhcpd/dhcpd.c``. These settings are required to described the behavior
of the daemon.
