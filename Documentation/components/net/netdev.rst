===============
Network Devices
===============

-  ``include/nuttx/net/netdev.h``. All structures and APIs
   needed to work with network drivers are provided in this
   header file. The structure ``struct net_driver_s`` defines the
   interface and is passed to the network via
   ``netdev_register()``.

-  ``include/nuttx/net/netdev_lowerhalf.h``. (Recommended for new
   drivers, see :ref:`Network Drivers <netdriver>`)
   This header file defines the interface between the network device
   and the network stack. The network device is a lower-half driver
   that provides the network stack with the ability to send and receive
   packets.

IP Addresses
============

The structure ``struct net_driver_s`` now supports one IPv4 address and
multiple IPv6 addresses. Multiple IPv6 addresses is common in modern
network devices. For example, a network device may have a link-local
address and a global address. The link-local address is used for
neighbor discovery protocol and the global address is used for
communication with the Internet.

Configuration Options
---------------------

``CONFIG_NETDEV_MULTIPLE_IPv6``
  Enable support for multiple IPv6 addresses per network device.
  Depends on ``CONFIG_NET_IPv6``.
``CONFIG_NETDEV_MAX_IPv6_ADDR``
  Maximum number of IPv6 addresses that can be assigned to a single
  network device. Normally a link-local address and a global address
  are needed.

IPv4 Interfaces
---------------

Now we only support one IPv4 address per network device, and directly
use the :c:member:`d_ipaddr`, :c:member:`d_draddr` and :c:member:`d_netmask`
in :c:struct:`net_driver_s`.

.. c:struct:: net_driver_s

  .. code-block:: c

    struct net_driver_s
    {
    #ifdef CONFIG_NET_IPv4
      in_addr_t      d_ipaddr;      /* Host IPv4 address assigned to the network interface */
      in_addr_t      d_draddr;      /* Default router IP address */
      in_addr_t      d_netmask;     /* Network subnet mask */
    #endif
    };

IPv6 Interfaces
---------------

Now we support multiple IPv6 addresses per network device, and use
the :c:member:`d_ipv6` in :c:struct:`net_driver_s` to store the IPv6
addresses. For historical reason, we keep the old name :c:member:`d_ipv6addr`
and :c:member:`d_ipv6netmask` for backward compatibility. Please use
:c:member:`d_ipv6` for new drivers.

.. c:struct:: net_driver_s

  .. code-block:: c

    struct net_driver_s
    {
    #ifdef CONFIG_NET_IPv6
      struct netdev_ifaddr6_s d_ipv6[CONFIG_NETDEV_MAX_IPv6_ADDR];
    #endif
    };

Managing the IPv6 addresses by provided APIs would be more flexible:

  - :c:func:`netdev_ipv6_add()`
  - :c:func:`netdev_ipv6_del()`
  - :c:func:`netdev_ipv6_srcaddr()`
  - :c:func:`netdev_ipv6_lladdr()`
  - :c:func:`netdev_ipv6_lookup()`
  - :c:func:`netdev_ipv6_foreach()`

.. c:function:: int netdev_ipv6_add(FAR struct net_driver_s *dev, const net_ipv6addr_t addr, \
                                    unsigned int preflen);
.. c:function:: int netdev_ipv6_del(FAR struct net_driver_s *dev, const net_ipv6addr_t addr, \
                                    unsigned int preflen);

  Add or delete an IPv6 address on the network device

  :return: Zero is returned if the operation is successfully applied on
    the device; A negated errno value is returned if failed.

.. c:function:: FAR const uint16_t *netdev_ipv6_srcaddr(FAR struct net_driver_s *dev, \
                                                        const net_ipv6addr_t dst);

  Get the source IPv6 address (RFC6724).

  :return: A pointer to the IPv6 address is returned on success.  It will never be
    NULL, but can be an address containing g_ipv6_unspecaddr.

.. c:function:: FAR const uint16_t *netdev_ipv6_lladdr(FAR struct net_driver_s *dev);

  Get the link-local address of the network device.

  :return: A pointer to the link-local address is returned on success.
    NULL is returned if the address is not found on the device.

.. c:function:: FAR struct netdev_ifaddr6_s *netdev_ipv6_lookup(FAR struct net_driver_s *dev, \
                                                    const net_ipv6addr_t addr, bool maskcmp);

  Look up an IPv6 address in the network device's IPv6 addresses

  :return: A pointer to the matching IPv6 address entry is returned on success.
    NULL is returned if the IPv6 address is not found in the device.

.. c:function:: int netdev_ipv6_foreach(FAR struct net_driver_s *dev, \
                      devif_ipv6_callback_t callback, FAR void *arg);

  Enumerate each IPv6 address on a network device.  This function will
  terminate when either (1) all addresses have been enumerated or (2) when
  a callback returns any non-zero value.

  :return: Zero is returned if the enumeration is successfully completed;
    Non-zero value is returned if enumeration is terminated early by callback.

Ioctls for IP Addresses
-----------------------

  - :c:macro:`SIOCGIFADDR`
  - :c:macro:`SIOCSIFADDR`
  - :c:macro:`SIOCDIFADDR`
  - :c:macro:`SIOCGLIFADDR`
  - :c:macro:`SIOCSLIFADDR`
  - :c:macro:`SIOCGIFNETMASK`
  - :c:macro:`SIOCSIFNETMASK`
  - :c:macro:`SIOCGLIFNETMASK`
  - :c:macro:`SIOCSLIFNETMASK`

.. c:macro:: SIOCGIFADDR
.. c:macro:: SIOCSIFADDR
.. c:macro:: SIOCDIFADDR

  We just follow the Linux convention[1]:

    Get, set, or delete the address of the device using :c:member:`ifr_addr`,
    or :c:member:`ifr6_addr` with :c:member:`ifr6_prefixlen`.
    For compatibility, :c:macro:`SIOCGIFADDR` returns only :c:macro:`AF_INET`
    addresses, :c:macro:`SIOCSIFADDR` accepts :c:macro:`AF_INET` and
    :c:macro:`AF_INET6` addresses, and :c:macro:`SIOCDIFADDR` deletes
    only :c:macro:`AF_INET6` addresses.  A :c:macro:`AF_INET` address
    can be deleted by setting it to zero via :c:macro:`SIOCSIFADDR`.

  Note: Unlike Linux, the maximum number of IPv6 addresses is limited on
  NuttX.  If you add more IPv6 addresses when we have already reached the
  limit, the new addresses will replace addresses with same scope.

  [1]: https://man7.org/linux/man-pages/man7/netdevice.7.html

.. c:macro:: SIOCGLIFADDR
.. c:macro:: SIOCSLIFADDR

  Get or set the IPv6 address of the device using :c:member:`lifr_addr`.

  We follow the Linux convention[1] to allow interface name to be
  <eth>:<num>[2], to keep working with multiple IPv6 addresses.

  Note: Recommend to use :c:macro:`SIOCSIFADDR` and :c:macro:`SIOCDIFADDR`
  to manage IPv6 addresses, by which you don't need to care about the
  slot it stored.

  [1]: https://man7.org/linux/man-pages/man7/netdevice.7.html
  [2]: e.g. 'eth0:0' stands for the secondary address on eth0

.. c:macro:: SIOCGIFNETMASK
.. c:macro:: SIOCSIFNETMASK

  Get or set the IPv4 network mask for a device using :c:member:`ifr_netmask`.

.. c:macro:: SIOCGLIFNETMASK
.. c:macro:: SIOCSLIFNETMASK

  Get or set the IPv6 network mask for a device using :c:member:`lifr_netmask`.

  We follow the Linux convention to allow interface name to be <eth>:<num>,
  to keep working with multiple IPv6 addresses.

  Note: Recommend to use :c:macro:`SIOCSIFADDR` and :c:macro:`SIOCDIFADDR`
  to manage IPv6 addresses, by which you don't need to care about the
  slot it stored.
