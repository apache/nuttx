================================
``discover`` UDP Discover Daemon
================================

This example exercises ``netutils/discover`` utility. This example initializes and
starts the UDP discover daemon. This daemon is useful for discovering devices in
local networks, especially with DHCP configured devices. It listens for UDP
broadcasts which also can include a device class so that groups of devices can
be discovered. It is also possible to address all classes with a kind of
broadcast discover.

This example will automatically be built as an NSH built-in if
``CONFIG_NSH_BUILTIN_APPS`` is selected. Otherwise, it will be a standalone
program with entry point ``discover_main``.

NuttX configuration settings:

- ``CONFIG_EXAMPLES_DISCOVER_DHCPC`` – DHCP Client.
- ``CONFIG_EXAMPLES_DISCOVER_NOMAC`` – Use canned MAC address.
- ``CONFIG_EXAMPLES_DISCOVER_IPADDR`` – Target IP address.
- ``CONFIG_EXAMPLES_DISCOVER_DRIPADDR`` – Router IP address.
- ``CONFIG_EXAMPLES_DISCOVER_NETMASK`` – Network Mask.
