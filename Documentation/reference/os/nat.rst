=================================
Network Address Translation (NAT)
=================================

NuttX supports full cone NAT logic, which currently supports

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

  - :c:func:`ipv4_nat_enable()`
  - :c:func:`ipv4_nat_disable()`

.. c:function:: int ipv4_nat_enable(FAR struct net_driver_s *dev);

  Enable NAT function on a network device, on which the outbound packets
  will be masqueraded.

  Note that NAT is currently designed to be enabled on single device, it
  may work when enabled on multiple devices, but external ports will not
  be isolated between devices, so an external port used on one NAT device
  will also be used by same local ip:port on another NAT device.

  :return: Zero is returned if NAT function is successfully enabled on
    the device; A negated errno value is returned if failed.

.. c:function:: int ipv4_nat_disable(FAR struct net_driver_s *dev);

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

2. Call ``ipv4_nat_enable`` on one dev on startup

  ..  code-block:: c

      /* arch/sim/src/sim/up_netdriver.c */
      int netdriver_init(void)
      {
        ...
        ipv4_nat_enable(&g_sim_dev[0]);
        ...
      }

3. Set IP Address for NuttX on startup

  ..  code-block:: shell

    ifconfig eth0 10.0.1.2
    ifup eth0
    ifconfig eth1 10.0.10.2
    ifup eth1

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

5. Do anything in the LAN namespace will go through NAT

  ..  code-block:: shell

    # Host side
    iperf -B 10.0.1.1 -s -i 1
    # LAN side
    sudo ip netns exec LAN iperf -B 10.0.10.1 -c 10.0.1.1 -i 1

  ..  code-block:: shell

    # Host side
    python3 -m http.server
    # LAN side
    for i in {1..20000}; do sudo ip netns exec LAN curl 'http://10.0.1.1:8000/' > /dev/null 2>1; done

  ..  code-block:: shell

    # LAN side
    sudo ip netns exec LAN ping 8.8.8.8

  ..  code-block:: shell

    # LAN side
    sudo ip netns exec LAN traceroute -n 8.8.8.8     # ICMP error msg of UDP
    sudo ip netns exec LAN traceroute -n -T 8.8.8.8  # ICMP error msg of TCP

  ..  code-block:: shell

    # Host side
    tcpdump -nn -i tap0
    # LAN side
    sudo ip netns exec LAN tcpdump -nn -i tap1
