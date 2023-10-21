``xmlrpc`` XML-RPC Server
=========================

This example exercises the "Embeddable Lightweight XML-RPC Server" which is
discussed at:

http://www.drdobbs.com/web-development/an-embeddable-lightweight-xml-rpc-server/184405364

Configuration options:

- ``CONFIG_EXAMPLES_XMLRPC_BUFFERSIZE`` – HTTP buffer size. Default ``1024``
- ``CONFIG_EXAMPLES_XMLRPC_DHCPC`` – Use DHCP Client. Default ``n``. Ignored if
  ``CONFIG_NSH_NETINIT`` is selected.
- ``CONFIG_EXAMPLES_XMLRPC_NOMAC`` – Use Canned MAC Address. Default ``n``. Ignored
  if ``CONFIG_NSH_NETINIT`` is selected.
- ``CONFIG_EXAMPLES_XMLRPC_IPADDR`` – Target IP address. Default ``0x0a000002``.
  Ignored if ``CONFIG_NSH_NETINIT`` is selected.
- ``CONFIG_EXAMPLES_XMLRPC_DRIPADDR`` – Default Router IP address (Gateway).
  Default ``0x0a000001``. Ignored if ``CONFIG_NSH_NETINIT`` is selected.
- ``CONFIG_EXAMPLES_XMLRPC_NETMASK`` – Network Mask. Default ``0xffffff00``. Ignored
  if ``CONFIG_NSH_NETINIT`` is selected.

