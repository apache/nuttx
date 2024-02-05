=========================
``bridge`` Network Bridge
=========================

A simple test of a system with multiple networks. It simply echoes all UDP
packets received on network ``1`` and network ``2`` to network ``2`` and network ``1``,
respectively. Interface ``1`` and interface may or may not lie on the same
network.

- ``CONFIG_EXAMPLES_BRIDGE`` – Enables the simple UDP bridge test.

There identical configurations for each of the two networks, ``NETn`` where ``n``
refers to the network being configured ``n={1,2}``. Let ``m`` refer to the other
network.

- ``CONFIG_EXAMPLES_BRIDGE_NETn_IFNAME`` – The register name of the network ``n``
  device. Must match the previously registered driver name and must not be the
  same as other network device name, ``CONFIG_EXAMPLES_BRIDGE_NETm_IFNAME``.
- ``CONFIG_EXAMPLES_BRIDGE_NETn_RECVPORT`` – Network ``n`` listen port number.
- ``CONFIG_EXAMPLES_BRIDGE_NETn_SNDPORT`` – Network ``2`` send port number.
- ``CONFIG_EXAMPLES_BRIDGE_NETn_IOBUFIZE`` – Size of the network ``n`` UDP
  send/receive I/O buffer.
- ``CONFIG_EXAMPLES_BRIDGE_NETn_STACKSIZE`` – Network ``n`` daemon stacksize.
- ``CONFIG_EXAMPLES_BRIDGE_NETn_PRIORITY`` – Network ``n`` daemon task priority.

If used as a NSH add-on, then it is assumed that initialization of both networks
was performed externally prior to the time that this test was started.
Otherwise, the following options are available:

- ``CONFIG_EXAMPLES_BRIDGE_NETn_NOMAC`` – Select of the network ``n`` hardware does
  not have a built-in MAC address. If selected, the MAC address. provided by
  ``CONFIG_EXAMPLES_BRIDGE_NETn_MACADDR`` will be used to assign the MAC address
  to the network n device.
- ``CONFIG_EXAMPLES_BRIDGE_NETn_DHCPC`` – Use DHCP Client to get the network n IP
  address.
- ``CONFIG_EXAMPLES_BRIDGE_NETn_IPADDR`` – If ``CONFIG_EXAMPLES_BRIDGE_NETn_DHCPC``
  is not selected, then this is the fixed IP address for network ``n``.
- ``CONFIG_EXAMPLES_BRIDGE_NETn_DRIPADDR`` – Network ``n`` default router IP address
  (Gateway).
- ``CONFIG_EXAMPLES_BRIDGE_NETn_NETMASK`` – Network ``n`` mask.

