============================
``vconfig`` VLAN Config Tool
============================

vconfig is a utility for managing VLAN (Virtual LAN) interfaces in NuttX.
It allows users to add, remove, and display VLANs on network interfaces.

USAGE
-----

Usage is straightforward::

    nsh> help ; vconfig
    Usage: vconfig add <interface> <vlan_id> [priority]
           vconfig rem|del <vlan-interface>

Examples
~~~~~~~~

Add a VLAN interface::

    nsh> vconfig add eth0 10

Remove a VLAN interface::

    nsh> vconfig rem|del eth0.10

Resources
~~~~~~~~~

* https://en.wikipedia.org/wiki/Virtual_LAN
* https://www.nuttx.org/


