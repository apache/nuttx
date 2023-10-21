``webserver`` Simple Webserver
==============================

This is a port of uIP tiny webserver example application. Settings specific to
this example include:

- ``CONFIG_EXAMPLES_WEBSERVER_NOMAC`` (may be defined to use software assigned
  MAC)
- ``CONFIG_EXAMPLES_WEBSERVER_IPADDR`` – Target IP address.
- ``CONFIG_EXAMPLES_WEBSERVER_DRIPADDR`` – Default router IP address.
- ``CONFIG_EXAMPLES_WEBSERVER_NETMASK`` – Network mask.
- ``CONFIG_EXAMPLES_WEBSERVER_DHCPC`` – Select to get IP address via DHCP.

If you use DHCPC, then some special configuration network options are required.
These include:

- ``CONFIG_NET=y`` – of course.
- ``CONFIG_NET_UDP=y`` – UDP support is required for DHCP (as well as various
  other UDP-related configuration settings).
- ``CONFIG_NET_BROADCAST=y`` – UDP broadcast support is needed.
- ``CONFIG_NET_ETH_PKTSIZE=650`` or larger. Per RFC2131 (p. 9), the DHCP client
  must be prepared to receive DHCP messages of up to ``576`` bytes (excluding
  Ethernet, IP, or UDP headers and FCS). **Note** that the actual MTU setting
  will depend upon the specific link protocol. Here Ethernet is indicated.

Other configuration items apply also to the selected ``webserver`` net utility.
Additional relevant settings for the uIP ``webserver`` net utility are:

- ``CONFIG_NETUTILS_HTTPDSTACKSIZE``
- ``CONFIG_NETUTILS_HTTPDFILESTATS``
- ``CONFIG_NETUTILS_HTTPDNETSTATS``

Applications using this example will need to enable the following ``netutils``
libraries in their ``defconfig`` file: ::

  CONFIG_NETUTILS_NETLIB=y
  CONFIG_NETUTILS_DHCPC=y
  CONFIG_NETDB_DNSCLIENT=y
  CONFIG_NETUTILS_WEBSERVER=y

**Note**: This example does depend on the ``perl`` script at
``nuttx/tools/mkfsdata.pl``. You must have ``perl`` installed on your development
system at ``/usr/bin/perl``.
