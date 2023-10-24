=====================
``dhcpd`` DHCP server
=====================

See ``apps/include/netutils/dhcpd.h`` for interface information.

Tips for Using DHCPC
--------------------

If you use DHCPC/D, then some special configuration network options are
required. These include:

- ``CONFIG_NET=y``
- ``CONFIG_NET_UDP=y`` – UDP support is required for DHCP (as well as various
  other UDP-related configuration settings).
- ``CONFIG_NET_BROADCAST=y`` – UDP broadcast support is needed.
- ``CONFIG_NET_ETH_PKTSIZE=650`` or larger. The client must be prepared to receive
  DHCP messages of up to ``576`` bytes (excluding Ethernet, IP  or UDP headers and
  FCS). **Note**: Note that the actual MTU setting will depend upon the specific
  link protocol. Here Ethernet is indicated.
