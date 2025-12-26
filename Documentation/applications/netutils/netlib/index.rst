==================================
``netlib`` Network support library
==================================

The ``netlib`` library provides a collection of network utility functions
for managing network interfaces, IP addresses, routing tables, and other
network-related operations. These functions are defined in
``netutils/netlib.h`` and are intended for internal use by NuttX applications.

Device Management
==================

  - :c:func:`netlib_get_devices`

.. c:function:: ssize_t netlib_get_devices(struct netlib_device_s *devlist, unsigned int nentries, sa_family_t family)

  Get a list of all network devices.

  :param devlist: Pointer to an array of ``netlib_device_s`` structures to
                  receive the device list.
  :param nentries: Maximum number of entries in the array.
  :param family: Address family filter (``AF_INET``, ``AF_INET6``, or
                 ``AF_UNSPEC`` for all).

  :return: Number of devices returned on success; -1 on error with ``errno``
           set appropriately.

Address Conversion Functions
=============================

  - :c:func:`netlib_ipv4addrconv`
  - :c:func:`netlib_ethaddrconv`

.. c:function:: bool netlib_ipv4addrconv(const char *addrstr, uint8_t *addr)

  Convert a textual representation of an IPv4 address to a numerical
  representation. This function takes an IP address in the form ``a.b.c.d``
  and converts it into a 4-byte array.

  :param addrstr: A pointer to a string containing the IP address in textual form.
  :param addr: A pointer to a 4-byte array that will be filled with the
               numerical representation of the address.

  :return: ``true`` if the IP address was parsed successfully; ``false`` otherwise.

.. c:function:: bool netlib_ethaddrconv(const char *hwstr, uint8_t *hw)

  Convert a textual representation of an Ethernet MAC address to a numerical
  representation.

  :param hwstr: A pointer to a string containing the MAC address in textual form
                (e.g., ``"00:11:22:33:44:55"``).
  :param hw: A pointer to a byte array that will be filled with the numerical
             representation of the MAC address.

  :return: ``true`` if the MAC address was parsed successfully; ``false`` otherwise.

MAC Address Management
=======================

  - :c:func:`netlib_setmacaddr`
  - :c:func:`netlib_getmacaddr`

.. c:function:: int netlib_setmacaddr(const char *ifname, const uint8_t *macaddr)

  Set the MAC address for an Ethernet network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param macaddr: Pointer to a 6-byte array containing the MAC address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_getmacaddr(const char *ifname, uint8_t *macaddr)

  Get the MAC address of an Ethernet network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param macaddr: Pointer to a 6-byte array to receive the MAC address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

IPv4 Address Management
========================

  - :c:func:`netlib_set_ipv4addr`
  - :c:func:`netlib_get_ipv4addr`
  - :c:func:`netlib_set_dripv4addr`
  - :c:func:`netlib_get_dripv4addr`
  - :c:func:`netlib_set_ipv4netmask`
  - :c:func:`netlib_get_ipv4netmask`
  - :c:func:`netlib_ipv4adaptor`

.. c:function:: int netlib_set_ipv4addr(const char *ifname, const struct in_addr *addr)

  Set the IPv4 address for a network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param addr: Pointer to an ``in_addr`` structure containing the IPv4 address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_get_ipv4addr(const char *ifname, struct in_addr *addr)

  Get the IPv4 address of a network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param addr: Pointer to an ``in_addr`` structure to receive the IPv4 address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_set_dripv4addr(const char *ifname, const struct in_addr *addr)

  Set the default router (gateway) IPv4 address for a network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param addr: Pointer to an ``in_addr`` structure containing the gateway address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_get_dripv4addr(const char *ifname, struct in_addr *addr)

  Get the default router (gateway) IPv4 address of a network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param addr: Pointer to an ``in_addr`` structure to receive the gateway address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_set_ipv4netmask(const char *ifname, const struct in_addr *addr)

  Set the IPv4 netmask for a network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param addr: Pointer to an ``in_addr`` structure containing the netmask.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_get_ipv4netmask(const char *ifname, struct in_addr *addr)

  Get the IPv4 netmask of a network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param addr: Pointer to an ``in_addr`` structure to receive the netmask.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_ipv4adaptor(in_addr_t destipaddr, in_addr_t *srcipaddr)

  Find the appropriate source IPv4 address to use for communicating with a
  destination address.

  :param destipaddr: Destination IPv4 address (in network byte order).
  :param srcipaddr: Pointer to receive the source IPv4 address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

IPv6 Address Management
========================

  - :c:func:`netlib_set_ipv6addr`
  - :c:func:`netlib_get_ipv6addr`
  - :c:func:`netlib_add_ipv6addr`
  - :c:func:`netlib_del_ipv6addr`
  - :c:func:`netlib_set_dripv6addr`
  - :c:func:`netlib_set_ipv6netmask`
  - :c:func:`netlib_ipv6adaptor`
  - :c:func:`netlib_ipv6netmask2prefix`
  - :c:func:`netlib_prefix2ipv6netmask`

.. c:function:: int netlib_set_ipv6addr(const char *ifname, const struct in6_addr *addr)

  Set the IPv6 address for a network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``). Can include slot
                 number for multiple addresses (e.g., ``"eth0:0"``).
  :param addr: Pointer to an ``in6_addr`` structure containing the IPv6 address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_get_ipv6addr(const char *ifname, struct in6_addr *addr)

  Get the IPv6 address of a network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``). Can include slot
                 number for multiple addresses (e.g., ``"eth0:0"``).
  :param addr: Pointer to an ``in6_addr`` structure to receive the IPv6 address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_add_ipv6addr(const char *ifname, const struct in6_addr *addr, uint8_t preflen)

  Add an IPv6 address to a network interface. This function is recommended for
  managing multiple IPv6 addresses on a single interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param addr: Pointer to an ``in6_addr`` structure containing the IPv6 address.
  :param preflen: Prefix length (0-128).

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_del_ipv6addr(const char *ifname, const struct in6_addr *addr, uint8_t preflen)

  Remove an IPv6 address from a network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param addr: Pointer to an ``in6_addr`` structure containing the IPv6 address
               to remove.
  :param preflen: Prefix length (0-128).

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_set_dripv6addr(const char *ifname, const struct in6_addr *addr)

  Set the default router (gateway) IPv6 address for a network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param addr: Pointer to an ``in6_addr`` structure containing the gateway address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_set_ipv6netmask(const char *ifname, const struct in6_addr *addr)

  Set the IPv6 netmask for a network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param addr: Pointer to an ``in6_addr`` structure containing the netmask.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_ipv6adaptor(const struct in6_addr *destipaddr, struct in6_addr *srcipaddr)

  Find the appropriate source IPv6 address to use for communicating with a
  destination address.

  :param destipaddr: Pointer to the destination IPv6 address.
  :param srcipaddr: Pointer to receive the source IPv6 address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: uint8_t netlib_ipv6netmask2prefix(const uint16_t *mask)

  Convert an IPv6 netmask to a prefix length.

  :param mask: Pointer to a 16-bit array representing the netmask.

  :return: Prefix length (0-128).

.. c:function:: void netlib_prefix2ipv6netmask(uint8_t preflen, struct in6_addr *netmask)

  Convert a prefix length to an IPv6 netmask.

  :param preflen: Prefix length (0-128).
  :param netmask: Pointer to an ``in6_addr`` structure to receive the netmask.

Network Interface Management
=============================

  - :c:func:`netlib_getifstatus`
  - :c:func:`netlib_ifup`
  - :c:func:`netlib_ifdown`
  - :c:func:`netlib_set_mtu`
  - :c:func:`netlib_check_ifconflict`

.. c:function:: int netlib_getifstatus(const char *ifname, uint8_t *flags)

  Get the status flags of a network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param flags: Pointer to receive the interface flags.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_ifup(const char *ifname)

  Bring a network interface up (activate it).

  :param ifname: Network interface name (e.g., ``"eth0"``).

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_ifdown(const char *ifname)

  Bring a network interface down (deactivate it).

  :param ifname: Network interface name (e.g., ``"eth0"``).

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_set_mtu(const char *ifname, int mtu)

  Set the Maximum Transmission Unit (MTU) for a network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).
  :param mtu: MTU value in bytes.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_check_ifconflict(const char *ifname)

  Check the IP address conflict status for a network interface. This function
  reads the conflict status from the procfs file system (``/proc/net/<ifname>``)
  which is populated by the ARP Address Conflict Detection (ACD) module.

  :param ifname: Network interface name (e.g., ``"eth0"``).

  :return: 0 if no conflict is detected; 1 if a conflict is detected; a negative
           value on error with ``errno`` set appropriately.

Network Buffer Statistics
==========================

  - :c:func:`netlib_get_iobinfo`

.. c:function:: int netlib_get_iobinfo(struct iob_stats_s *iob)

  Get the network IOB (I/O Buffer) statistics. This function reads IOB usage
  information from the procfs file system (``/proc/iobinfo``) which provides
  details about the network packet buffer pool.

  IOB buffers are used by the NuttX network stack to buffer network packets
  during transmission and reception. This function allows monitoring the IOB
  buffer pool to detect potential buffer exhaustion or throttling conditions.

  :param iob: Pointer to an ``iob_stats_s`` structure to receive the IOB
              statistics. The structure contains the following fields:

              - ``ntotal``: Total number of IOB buffers configured
                (``CONFIG_IOB_NBUFFERS``)
              - ``nfree``: Current number of free IOB buffers available
              - ``nwait``: Number of tasks waiting for IOB buffers (when
                ``nfree < 0``)
              - ``nthrottle``: Number of IOB buffers that are throttled (when
                below ``CONFIG_IOB_THROTTLE`` threshold)

  :return: 0 on success; a negative value on failure with ``errno`` set
           appropriately.

  **Note:** This function requires ``CONFIG_MM_IOB`` to be enabled. The IOB
  statistics are provided by the kernel's IOB buffer management system and
  reflect the current state of the network packet buffer pool.

Network Connectivity
=====================

  - :c:func:`netlib_check_ipconnectivity`

.. c:function:: int netlib_check_ipconnectivity(FAR const char *ip, int timeout, int retry)

  Check network connectivity to a specified IPv4 address using ICMP ping. This
  function sends ICMP echo requests to the target address and counts the number
  of replies received.

  :param ip: IPv4 address string to check (e.g., ``"192.168.1.1"``). If ``NULL``,
             the function will use the default DNS server address configured via
             ``CONFIG_NETDB_DNSSERVER_IPv4ADDR`` (if available).
  :param timeout: Maximum timeout for each ping attempt in milliseconds.
  :param retry: Number of ping attempts to send.

  :return: Number of replies received (0 or positive) on success. A value of 0
           indicates that no replies were received (network unreachable or timeout).
           A negative value indicates an error occurred (e.g., ``-EINVAL`` if ``ip``
           is ``NULL`` and no DNS server is configured).

  **Note:** This function requires ``CONFIG_NETUTILS_PING`` to be enabled. The
  function is blocking and will wait for all ping attempts to complete before
  returning.

ARP Table Support
==================

  - :c:func:`netlib_set_arpmapping`
  - :c:func:`netlib_get_arpmapping`
  - :c:func:`netlib_del_arpmapping`
  - :c:func:`netlib_get_arptable`

.. c:function:: int netlib_set_arpmapping(const struct sockaddr_in *inaddr, const uint8_t *macaddr, const char *ifname)

  Add or update an ARP table entry mapping an IPv4 address to a MAC address.

  :param inaddr: Pointer to a ``sockaddr_in`` structure containing the IPv4 address.
  :param macaddr: Pointer to a 6-byte array containing the MAC address.
  :param ifname: Network interface name (e.g., ``"eth0"``), or NULL for any interface.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_get_arpmapping(const struct sockaddr_in *inaddr, uint8_t *macaddr, const char *ifname)

  Retrieve the MAC address associated with an IPv4 address from the ARP table.

  :param inaddr: Pointer to a ``sockaddr_in`` structure containing the IPv4 address.
  :param macaddr: Pointer to a 6-byte array to receive the MAC address.
  :param ifname: Network interface name (e.g., ``"eth0"``), or NULL for any interface.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_del_arpmapping(const struct sockaddr_in *inaddr, const char *ifname)

  Delete an entry from the ARP table.

  :param inaddr: Pointer to a ``sockaddr_in`` structure containing the IPv4 address.
  :param ifname: Network interface name (e.g., ``"eth0"``), or NULL for any interface.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: ssize_t netlib_get_arptable(struct arpreq *arptab, unsigned int nentries)

  Retrieve the entire ARP table.

  :param arptab: Pointer to an array of ``arpreq`` structures to receive the
                 ARP table entries.
  :param nentries: Maximum number of entries in the array.

  :return: Number of entries returned on success; -1 on error with ``errno``
           set appropriately.

Routing Table Support
======================

  - :c:func:`netlib_ipv4router`
  - :c:func:`netlib_ipv6router`
  - :c:func:`netlib_open_ipv4route`
  - :c:func:`netlib_close_ipv4route`
  - :c:func:`netlib_read_ipv4route`
  - :c:func:`netlib_open_ipv6route`
  - :c:func:`netlib_close_ipv6route`
  - :c:func:`netlib_read_ipv6route`
  - :c:func:`netlib_get_route`

.. c:function:: int netlib_ipv4router(const struct in_addr *destipaddr, struct in_addr *router)

  Find the router (gateway) address to use for reaching a destination IPv4 address.

  :param destipaddr: Pointer to the destination IPv4 address.
  :param router: Pointer to receive the router IPv4 address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_ipv6router(const struct in6_addr *destipaddr, struct in6_addr *router)

  Find the router (gateway) address to use for reaching a destination IPv6 address.

  :param destipaddr: Pointer to the destination IPv6 address.
  :param router: Pointer to receive the router IPv6 address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: FILE *netlib_open_ipv4route(void)

  Open the IPv4 routing table for reading. This is a macro that opens
  ``/proc/net/route/ipv4``.

  :return: File pointer on success; NULL on error.

.. c:function:: void netlib_close_ipv4route(FILE *stream)

  Close the IPv4 routing table file. This is a macro that calls ``fclose()``.

  :param stream: File pointer returned by ``netlib_open_ipv4route()``.

.. c:function:: ssize_t netlib_read_ipv4route(FILE *stream, struct netlib_ipv4_route_s *route)

  Read one entry from the IPv4 routing table.

  :param stream: File pointer returned by ``netlib_open_ipv4route()``.
  :param route: Pointer to a ``netlib_ipv4_route_s`` structure to receive the
                route entry.

  :return: 1 on success; 0 on end of file; -1 on error.

.. c:function:: FILE *netlib_open_ipv6route(void)

  Open the IPv6 routing table for reading. This is a macro that opens
  ``/proc/net/route/ipv6``.

  :return: File pointer on success; NULL on error.

.. c:function:: void netlib_close_ipv6route(FILE *stream)

  Close the IPv6 routing table file. This is a macro that calls ``fclose()``.

  :param stream: File pointer returned by ``netlib_open_ipv6route()``.

.. c:function:: ssize_t netlib_read_ipv6route(FILE *stream, struct netlib_ipv6_route_s *route)

  Read one entry from the IPv6 routing table.

  :param stream: File pointer returned by ``netlib_open_ipv6route()``.
  :param route: Pointer to a ``netlib_ipv6_route_s`` structure to receive the
                route entry.

  :return: 1 on success; 0 on end of file; -1 on error.

.. c:function:: ssize_t netlib_get_route(struct rtentry *rtelist, unsigned int nentries, sa_family_t family)

  Retrieve routing table entries using Netlink.

  :param rtelist: Pointer to an array of ``rtentry`` structures to receive the
                  routing entries.
  :param nentries: Maximum number of entries in the array.
  :param family: Address family filter (``AF_INET``, ``AF_INET6``, or
                 ``AF_UNSPEC`` for all).

  :return: Number of entries returned on success; -1 on error with ``errno``
           set appropriately.

DHCP Support
=============

  - :c:func:`netlib_obtain_ipv4addr`
  - :c:func:`netlib_icmpv6_autoconfiguration`
  - :c:func:`netlib_obtain_ipv6addr`

.. c:function:: int netlib_obtain_ipv4addr(const char *ifname)

  Obtain an IPv4 address via DHCP for the specified network interface. This
  function blocks until an address is obtained or an error occurs.

  :param ifname: Network interface name (e.g., ``"eth0"``).

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_icmpv6_autoconfiguration(const char *ifname)

  Perform IPv6 stateless address autoconfiguration (SLAAC) for the specified
  network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_obtain_ipv6addr(const char *ifname)

  Obtain an IPv6 address via DHCPv6 for the specified network interface.

  :param ifname: Network interface name (e.g., ``"eth0"``).

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

DNS Server Address Management
===============================

  - :c:func:`netlib_set_ipv4dnsaddr`
  - :c:func:`netlib_del_ipv4dnsaddr`
  - :c:func:`netlib_del_ipv4dnsaddr_by_index`
  - :c:func:`netlib_set_ipv6dnsaddr`
  - :c:func:`netlib_del_ipv6dnsaddr`
  - :c:func:`netlib_del_ipv6dnsaddr_by_index`

.. c:function:: int netlib_set_ipv4dnsaddr(const struct in_addr *inaddr)

  Add an IPv4 DNS server address to the resolver configuration.

  :param inaddr: Pointer to an ``in_addr`` structure containing the DNS server
                 address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_del_ipv4dnsaddr(const struct in_addr *inaddr)

  Remove an IPv4 DNS server address from the resolver configuration.

  :param inaddr: Pointer to an ``in_addr`` structure containing the DNS server
                 address to remove.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_del_ipv4dnsaddr_by_index(int index)

  Remove an IPv4 DNS server from the resolver configuration by index.

  :param index: Index of the DNS server to remove (0-based).

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_set_ipv6dnsaddr(const struct in6_addr *inaddr)

  Add an IPv6 DNS server address to the resolver configuration.

  :param inaddr: Pointer to an ``in6_addr`` structure containing the DNS server
                 address.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_del_ipv6dnsaddr(const struct in6_addr *inaddr)

  Remove an IPv6 DNS server address from the resolver configuration.

  :param inaddr: Pointer to an ``in6_addr`` structure containing the DNS server
                 address to remove.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_del_ipv6dnsaddr_by_index(int index)

  Remove an IPv6 DNS server from the resolver configuration by index.

  :param index: Index of the DNS server to remove (0-based).

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

Wireless Configuration
=======================

  - :c:func:`netlib_getessid`
  - :c:func:`netlib_setessid`

.. c:function:: int netlib_getessid(const char *ifname, char *essid, size_t idlen)

  Get the ESSID (network name) of a wireless network interface.

  :param ifname: Wireless network interface name (e.g., ``"wlan0"``).
  :param essid: Buffer to receive the ESSID string.
  :param idlen: Size of the buffer.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_setessid(const char *ifname, const char *essid)

  Set the ESSID (network name) for a wireless network interface.

  :param ifname: Wireless network interface name (e.g., ``"wlan0"``).
  :param essid: ESSID string to set.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

URL Parsing
============

  - :c:func:`netlib_parsehttpurl`
  - :c:func:`netlib_parseurl`

.. c:function:: int netlib_parsehttpurl(const char *url, uint16_t *port, char *hostname, int hostlen, char *filename, int namelen)

  Parse an HTTP URL into its components.

  :param url: URL string to parse (e.g., ``"http://example.com:8080/path/file"``).
  :param port: Pointer to receive the port number (default 80 if not specified).
  :param hostname: Buffer to receive the hostname.
  :param hostlen: Size of the hostname buffer.
  :param filename: Buffer to receive the path/filename.
  :param namelen: Size of the filename buffer.

  :return: 0 on success; -1 on error.

.. c:function:: int netlib_parseurl(const char *str, struct url_s *url)

  Parse a generic URL into its components. This function handles various URL
  schemes beyond HTTP.

  :param str: URL string to parse.
  :param url: Pointer to a ``url_s`` structure to receive the parsed components.

  :return: 0 on success; -1 on error.

Server Support
===============

  - :c:func:`netlib_listenon`
  - :c:func:`netlib_server`

.. c:function:: int netlib_listenon(uint16_t portno)

  Create a TCP socket and listen on the specified port. This is a convenience
  function for setting up a server socket.

  :param portno: Port number to listen on.

  :return: Socket descriptor on success; -1 on error with ``errno`` set
           appropriately.

.. c:function:: void netlib_server(uint16_t portno, pthread_startroutine_t handler, int stacksize)

  Create a simple server that listens on the specified port and spawns a new
  thread for each connection using the provided handler function.

  :param portno: Port number to listen on.
  :param handler: Function to call for each new connection. The handler receives
                  the client socket descriptor as an argument.
  :param stacksize: Stack size for handler threads.

Neighbor Table (IPv6)
======================

  - :c:func:`netlib_get_nbtable`

.. c:function:: ssize_t netlib_get_nbtable(struct neighbor_entry_s *nbtab, unsigned int nentries)

  Retrieve the IPv6 neighbor table (similar to ARP for IPv4).

  :param nbtab: Pointer to an array of ``neighbor_entry_s`` structures to
                receive the neighbor table entries.
  :param nentries: Maximum number of entries in the array.

  :return: Number of entries returned on success; -1 on error with ``errno``
           set appropriately.

Connection Tracking (Netfilter)
=================================

  - :c:func:`netlib_get_conntrack`
  - :c:func:`netlib_parse_conntrack`

.. c:function:: int netlib_get_conntrack(sa_family_t family, netlib_conntrack_cb_t cb)

  Retrieve connection tracking entries from the kernel's connection tracking
  table.

  :param family: Address family filter (``AF_INET``, ``AF_INET6``, or
                 ``AF_UNSPEC`` for all).
  :param cb: Callback function to invoke for each connection. The callback
             receives a pointer to a ``netlib_conntrack_s`` structure.

  :return: 0 on success; -1 on error with ``errno`` set appropriately.

.. c:function:: int netlib_parse_conntrack(const struct nlmsghdr *nlh, size_t len, struct netlib_conntrack_s *ct)

  Parse a Netlink message containing connection tracking information.

  :param nlh: Pointer to the Netlink message header.
  :param len: Length of the message.
  :param ct: Pointer to a ``netlib_conntrack_s`` structure to receive the
             parsed connection information.

  :return: 0 on success; -1 on error.
