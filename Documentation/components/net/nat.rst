=================================
Network Address Translation (NAT)
=================================

NuttX supports full cone or symmetric NAT logic, which currently supports

- TCP

- UDP

- ICMP

  - ECHO (REQUEST & REPLY)

  - Error Messages (DEST_UNREACHABLE & TIME_EXCEEDED & PARAMETER_PROBLEM)

Workflow
========

::

  Local Network (LAN)                          External Network (WAN)
                    |----------------|
       <local IP,   |                | <external IP,             <peer IP,
         -----------|                |-----------------------------
        local port> |                |  external port>            peer port>
                    |----------------|

- Outbound

  - **LAN** -> **Forward** -> **NAT** (only if targeting at WAN) -> **WAN**

  - All packets from **LAN** and targeting at **WAN** will be masqueraded
    with ``local ip:port`` changed to ``external ip:port``.

- Inbound

  - **WAN** -> **NAT** (only from WAN, change destination) -> **Forward** -> **LAN**

  - Packets from **WAN** will try to be changed back from
    ``external ip:port`` to ``local ip:port`` and send to **LAN**.

Configuration Options
=====================

``CONFIG_NET_NAT``
  Enable or disable Network Address Translation (NAT) function.
  Depends on ``CONFIG_NET_IPFORWARD``.
``CONFIG_NET_NAT44`` & ``CONFIG_NET_NAT66``
  Enable or disable NAT on IPv4 / IPv6.
  Depends on ``CONFIG_NET_NAT``.
``CONFIG_NET_NAT44_FULL_CONE`` & ``CONFIG_NET_NAT66_FULL_CONE``
  Enable Full Cone NAT logic. Full Cone NAT is easier to traverse than
  Symmetric NAT, and uses less resources than Symmetric NAT.
``CONFIG_NET_NAT44_SYMMETRIC`` & ``CONFIG_NET_NAT66_SYMMETRIC``
  Enable Symmetric NAT logic. Symmetric NAT will be safer than Full Cone NAT,
  be more difficult to traverse, and has more entries which may lead to heavier load.
``CONFIG_NET_NAT_HASH_BITS``
  The bits of the hashtable of NAT entries, hashtable has (1 << bits) buckets.
``CONFIG_NET_NAT_TCP_EXPIRE_SEC``
  The expiration time for idle TCP entry in NAT.
  The default value 86400 is suggested by RFC2663, Section 2.6,
  Page 5. But we may set it to shorter time like 240s for better
  performance.
``CONFIG_NET_NAT_UDP_EXPIRE_SEC``
  The expiration time for idle UDP entry in NAT.
``CONFIG_NET_NAT_ICMP_EXPIRE_SEC``
  The expiration time for idle ICMP entry in NAT.
``CONFIG_NET_NAT_ICMPv6_EXPIRE_SEC``
  The expiration time for idle ICMPv6 entry in NAT.
``CONFIG_NET_NAT_ENTRY_RECLAIM_SEC``
  The time to auto reclaim all expired NAT entries. A value of zero will
  disable auto reclaiming.
  Expired entries will be automatically reclaimed when matching
  inbound/outbound entries, so this config does not have significant
  impact when NAT is normally used, but very useful when the hashtable
  is big and there are only a few connections using NAT (which will
  only trigger reclaiming on a few chains in hashtable).

Usage
=====

  - :c:func:`nat_enable()`
  - :c:func:`nat_disable()`

.. c:function:: int nat_enable(FAR struct net_driver_s *dev);

  Enable NAT function on a network device, on which the outbound packets
  will be masqueraded.

  :return: Zero is returned if NAT function is successfully enabled on
    the device; A negated errno value is returned if failed.

.. c:function:: int nat_disable(FAR struct net_driver_s *dev);

  Disable NAT function on a network device.

  :return: Zero is returned if NAT function is successfully disabled on
    the device; A negated errno value is returned if failed.

Validation
==========

Validated on Ubuntu 22.04 x86_64 with NuttX SIM by following steps:

1. Configure NuttX with >=2 TAP devices (host route mode) and NAT enabled:

  ..  code-block:: Kconfig

      CONFIG_NET_IPFORWARD=y
      CONFIG_NET_NAT=y
      # CONFIG_SIM_NET_BRIDGE is not set
      CONFIG_SIM_NETDEV_NUMBER=2

2. Call ``nat_enable`` on one dev on startup, or manually enable NAT
   with ``iptables`` command (either may work).

  ..  code-block:: c

      /* arch/sim/src/sim/up_netdriver.c */
      int netdriver_init(void)
      {
        ...
        nat_enable(&g_sim_dev[0]);
        ...
      }

  ..  code-block:: shell

      iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE

3. Set IP Address for NuttX on startup

  ..  code-block:: shell

    ifconfig eth0 10.0.1.2
    ifup eth0
    ifconfig eth1 10.0.10.2
    ifup eth1

    # IPv6 if you need
    ifconfig eth0 inet6 add fc00:1::2/64 gw fc00:1::1
    ifconfig eth1 inet6 add fc00:10::2/64

4. Configure IP & namespace & route on host side (maybe need to be root, then try ``sudo -i``)

  ..  code-block:: bash

    IF_HOST="enp1s0"
    IF_0="tap0"
    IP_HOST_0="10.0.1.1"
    IF_1="tap1"
    IP_HOST_1="10.0.10.1"
    IP_NUTTX_1="10.0.10.2"

    # add net namespace LAN for $IF_1
    ip netns add LAN
    ip netns exec LAN sysctl -w net.ipv4.ip_forward=1
    ip link set $IF_1 netns LAN
    ip netns exec LAN ip link set $IF_1 up
    ip netns exec LAN ip link set lo up

    # add address and set default route
    ip addr add $IP_HOST_0/24 dev $IF_0
    ip netns exec LAN ip addr add $IP_HOST_1/24 dev $IF_1
    ip netns exec LAN ip route add default dev $IF_1 via $IP_NUTTX_1

    # nat to allow NuttX to access the internet
    iptables -t nat -A POSTROUTING -o $IF_HOST -j MASQUERADE
    iptables -A FORWARD -i $IF_HOST -o $IF_0 -j ACCEPT
    iptables -A FORWARD -i $IF_0 -o $IF_HOST -j ACCEPT
    sysctl -w net.ipv4.ip_forward=1

    # IPv6 if you need
    IP6_HOST_0="fc00:1::1"
    IP6_HOST_1="fc00:10::1"
    IP6_NUTTX_1="fc00:10::2"

    # add address and set default route
    ip -6 addr add $IP6_HOST_0/64 dev $IF_0
    ip netns exec LAN ip -6 addr add $IP6_HOST_1/64 dev $IF_1
    ip netns exec LAN ip -6 route add default dev $IF_1 via $IP6_NUTTX_1

    # nat to allow NuttX to access the internet
    ip6tables -t nat -A POSTROUTING -o $IF_HOST -j MASQUERADE
    ip6tables -A FORWARD -i $IF_HOST -o $IF_0 -j ACCEPT
    ip6tables -A FORWARD -i $IF_0 -o $IF_HOST -j ACCEPT
    sysctl -w net.ipv6.conf.all.forwarding=1

5. Do anything in the LAN namespace will go through NAT

  ..  code-block:: shell

    # Host side
    iperf -B 10.0.1.1 -s -i 1
    # LAN side
    sudo ip netns exec LAN iperf -B 10.0.10.1 -c 10.0.1.1 -i 1

  ..  code-block:: shell

    # Host side
    python3 -m http.server -b ::
    # LAN side
    for i in {1..20000}; do sudo ip netns exec LAN curl 'http://10.0.1.1:8000/' > /dev/null 2>1; done
    for i in {1..20000}; do sudo ip netns exec LAN curl 'http://[fc00:1::1]:8000/' > /dev/null 2>1; done

  ..  code-block:: shell

    # LAN side
    sudo ip netns exec LAN ping 8.8.8.8
    sudo ip netns exec LAN ping 2001:4860:4860::8888

  ..  code-block:: shell

    # LAN side
    sudo ip netns exec LAN traceroute -n 8.8.8.8     # ICMP error msg of UDP
    sudo ip netns exec LAN traceroute -n -T 8.8.8.8  # ICMP error msg of TCP
    sudo ip netns exec LAN traceroute -n -I 8.8.8.8  # ICMP error msg of ICMP
    sudo ip netns exec LAN traceroute -n 2001:4860:4860::8888
    sudo ip netns exec LAN traceroute -n -T 2001:4860:4860::8888
    sudo ip netns exec LAN traceroute -n -I 2001:4860:4860::8888

  ..  code-block:: shell

    # Host side
    tcpdump -nn -i tap0
    # LAN side
    sudo ip netns exec LAN tcpdump -nn -i tap1
