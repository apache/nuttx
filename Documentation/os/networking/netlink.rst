=====================
Netlink Route support
=====================

Netlink Route (:c:macro:`NETLINK_ROUTE`) allow notifying msg when the network 
changes. And then apps can obtain these changes by monitoring to netlink socket
messages.

NuttX supports Netlink Route groups

- RTMGRP_IPV4_ROUTE | RTMGRP_IPV6_ROUTE
  - Notify when IPV4|IPV6 routing table changes.

- RTMGRP_NEIGH
  - Notify when ARP (IPV4) or neighbors (IPV6) table changes.

- RTNLGRP_IPV6_PREFIX
  - Notify when IPV6 prefix changes.

Messages content
================

1. RTMGRP_IPV4_ROUTE | RTMGRP_IPV6_ROUTE

  ``RTM_NEWROUTE``, ``RTM_DELROUTE``, ``RTM_GETROUTE``:
  Create, remove or receive information about a network route. These
  messages contain an rtmsg structure with 3 optional sequence of
  rtattr structures following.

  .. code-block:: c

    struct getroute_recvfrom_ipv4addr_s
    {
      struct rtattr attr;
      in_addr_t     addr;
    };

    struct getroute_recvfrom_ipv4response_s
    {
      struct nlmsghdr hdr;
      struct rtmsg    rte;
      struct getroute_recvfrom_ipv4addr_s dst;
      struct getroute_recvfrom_ipv4addr_s genmask;
      struct getroute_recvfrom_ipv4addr_s gateway;
    };

    struct getroute_recvfrom_ipv6addr_s
    {
      struct rtattr  attr;
      net_ipv6addr_t addr;
    };

    struct getroute_recvfrom_ipv6response_s
    {
      struct nlmsghdr hdr;
      struct rtmsg    rte;
      struct getroute_recvfrom_ipv6addr_s dst;
      struct getroute_recvfrom_ipv6addr_s genmask;
      struct getroute_recvfrom_ipv6addr_s gateway;
    };

2. RTMGRP_NEIGH

  ``RTM_NEWNEIGH``, ``RTM_DELNEIGH``, ``RTM_GETNEIGH``:
  Add, remove or receive information about a neighbor table entry (e.g.,
  an ARP entry). The message contains an ndmsg structure and optional 
  sequence of rtattr structures following. And the date will be ``struct arpreq``
  in ``include/netinet/arp.h``or ``struct neighbor_entry_s`` in ``include/net/neighbor.h``

  .. code-block:: c

    struct getneigh_recvfrom_response_s
    {
      struct nlmsghdr hdr;
      struct ndmsg    msg;
      struct rtattr   attr;
      uint8_t         data[1];
    };

3. RTNLGRP_IPV6_PREFIX

  ``RTM_NEWPREFIX``: 
  Receive information about IPV6 prefix. The message contains an prefixmsg structure
  and two optional sequence of rtattr structures following. And the ``addr`` and  
  ``prefix_cacheinfo`` are parsed from the RA message.

  .. code-block:: c

    struct getprefix_recvfrom_addr_s
    {
      struct rtattr  attr;
      net_ipv6addr_t addr;
    };

    struct getprefix_recvfrom_cache_s
    {
      struct rtattr           attr;
      struct prefix_cacheinfo pci;
    };

    struct getprefix_recvfrom_response_s
    {
      struct nlmsghdr  hdr;
      struct prefixmsg pmsg;
      struct getprefix_recvfrom_addr_s  prefix;
      struct getprefix_recvfrom_cache_s pci;
    };

Usage
=====

.. code-block:: c

  struct sockaddr_nl addr;
  struct nlmsghdr *hdr;
  uint8_t buffer[BUFSIZE];
  int sd = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE); 

  addr.nl_family  = AF_NETLINK;
  addr.nl_groups  = RTMGRP_IPV4_ROUTE | RTMGRP_IPV6_ROUTE |
                    RTMGRP_NEIGH | RTMGRP_IPV6_PREFIX;

  bind(sd, (FAR struct sockaddr *)&addr, sizeof(addr)); /* Bind to device */
  while (1)
    {
      recv(sd, buf, BUFSIZE, 0);
      for (hdr = buf; NLMSG_OK(hdr, ret); hdr = NLMSG_NEXT(hdr, ret))
        {
          if (hdr->nlmsg_type == RTM_...)
            {
              /* Func parsed netlink msg*/
              ...
            }
        }
    }

  close(sd); /* Close the socket */