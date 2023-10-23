=====================
``igmp`` Trivial IGMP
=====================

This is a trivial test of the NuttX IGMP capability. It present it does not do
much of value – Much more is needed in order to verify the IGMP features!

- ``CONFIG_EXAMPLES_IGMP_NOMAC`` – Set if the hardware has no MAC address; one
  will be assigned.
- ``CONFIG_EXAMPLES_IGMP_IPADDR`` – Target board IP address.
- ``CONFIG_EXAMPLES_IGMP_DRIPADDR`` – Default router address.
- ``CONFIG_EXAMPLES_IGMP_NETMASK`` – Network mask.
- ``CONFIG_EXAMPLES_IGMP_GRPADDR`` – Multicast group address.
- ``CONFIG_EXAMPLES_NETLIB`` – The networking library is needed.
