====
IPv6
====

.. warning:: 
    Migrated from: https://cwiki.apache.org/confluence/display/NUTTX/IPv6

NuttX has supported the Internet Protocol Version 4 (IPv4) for many years. 
There have been fragments of IPv6 in the code base for many years as well, 
but these fragments were not more than place markers and not functional.

But recently, post NuttX-7.6, I have focused some effort into completing the 
IPv6 implementation. This Wiki page contains notes from that integration 
effort and will, hopefully, evolve to provide full documentation for NuttX 
IPv6 support.

Current status: Basic functionality is complete and verified. That includes 
ICMPPv6 Neighbor Discover Protocol, IPCMPv6 Echo Request/Response 
(for ``ping6``), TCP/IPv6, and UDP/IPv6. It has also been proven that you 
can support a platform with `both` IPv4 and IPv6 enabled.

Ethernet Driver Requirements
============================

Basic Driver Requirements
-------------------------

In order to support IPv6, Ethernet drivers must do the following:

* They must recognize IPv6 packets and call ``ipv6_input`` in order to pass the 
  packets into the network stack. This is equivalent to calling ``ipv4_input`` 
  when an IPv4 pack is received.
* When sending an IPv6, the drivers must call ``neighbor_out()`` in order to add 
  the MAC address of the destination into the link layer header. IPv6's 
  `ICMPv6 Neighbor Discovery Protocol` is the moral equivalent of the 
  `Address Resolution Protocol` (ARP) used with IPv6. And the IPv6 
  ``neighbor_out()`` performs a similar function to the IPv4 ``arp_out()`` 
  function.
* Ethernet drivers must also support some additional address filtering. 
  For IPv4 support, most Ethernet drivers are configured to accept only 
  Ethernet packets with matching MAC addresses and broadcast packets (or 
  selected multicast packets if IGMP support is enabled). Additional 
  filtering support is needed to support IPv6.

All existing NuttX Ethernet drivers have already been modified to support 
the requirements of the first two bullets. However, additional logic must 
be added to most of the existing Ethernet drivers to support the final 
requirement.

Multicast Address Filtering
---------------------------

Each Ethernet device connects to the Ethernet wire via a PHY and so 
potentially has access to every packet that passes on the wire. In 
`promiscuous` mode, that is the behavior that is desired but normally 
it is not: The amount of traffic that appears on the wire would swamp 
most modest MCUs in promiscuous mode. So instead, the Ethernet MAC 
hardware will support address filtering. That is, the hardware will 
look at the Ethernet header at the beginning of each packet and will 
ignore packets that do not have the desired information in the Ethernet 
header. The software will see only those filtered packets that are desired.

Typically, the Ethernet MAC is set-up for `unicast` address filtering: The 
hardware is programmed so that that only packets whose destination Ethernet 
MAC address matches the MAC address programmed into the hardware are accepted. 
In addition, special `broadcast` Ethernet addresses will also be accepted. 
In this way, the volume of Ethernet data received by the MCU is greatly 
reduced.

`Multicast` addresses are a little different. Unlike broadcast addresses, 
there are many possible multicast addresses and so the Ethernet MAC hardware 
must support some special capability to match the destination Ethernet 
address in an incoming packet with a variety of multicast addresses. 
Usually this involves `hashing` the Ethernet address and performing a `hash 
table lookup` to check for an address match.

Each Ethernet driver uses a common interface that is defined in 
``nuttx/include/nuttx/net/netdev.h``. That interface defines, among other 
things, a set of calls into the Ethernet driver to perform a variety of 
functions. One of those functions is multicast address filtering:

.. code-block:: c

    #ifdef CONFIG_NET_IGMP
    int (*d_addmac)(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
    int (*d_rmmac)(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
    #endif

The ``d_addmac()`` interface adds a multicast address to the hash 
table; ``d_rmmac()`` removes a multicast address from the hash table.

These interface is only required if IGMP is supported, but the underlying 
ability to program multicast address filtered is required for full IPv6 
support. This interface exists in all Ethernet drivers but most are 
currently place holders and are `to-be-provided`. At present, only the 
STMicro STM32, the TI Tiva TM4C, and the Atmel SAM3/4 and SAMA5D3/4 
Ethernet drivers support multicast hash tables. This capability will 
have to be added to any additional Ethernet drivers that are modified 
to support IPv6.

ICMPv6 Neighbor Discovery Protocol
----------------------------------

The ICMPv6 Neighbor Discover protocol is the reason for this additional 
address filtering. The ICMPv6 Neighbor Discovery Protocol is the 
replacement for IPv4's ARP. It different from ARP in the it is 
implemented not at the Ethernet link layer, but within the IPv6 layer. 
In order to receive broadcast packets to ICMPv6, the IPv6 Multicast 
address of 33.33.ff.xx.xx.xx is used, where the xx.xx.xx part derives 
from the IPv6 address. The Ethernet driver filtering logic must be modified 
so that it accepts packets directed to the that MAC address.

At present, this additional support is only implemented for the TI Tiva 
TM4C129X Ethernet driver. Below is a snippet of code from that drier 
showing how this is implemented:

.. code-block:: c

    /* Set the MAC address */
    
    tiva_macaddress(priv);
    
    #ifdef CONFIG_NET_ICMPv6
    /* Set up the IPv6 multicast address */
    
    tiva_ipv6multicast(priv);
    #endif

Where `tiva_macaddress()` sets up the normal MAC address filtering and 
`tiva_ipv6multicast()` sets up the special filtering needed by IPv6:

.. code-block:: c 

    /****************************************************************************
   * Function: tiva_ipv6multicast
   *
   * Description:
   *   Configure the IPv6 multicast MAC address.
   *
   * Parameters:
   *   priv - A reference to the private driver state structure
   *
   * Returned Value:
   *   OK on success; Negated errno on failure.
   *
   * Assumptions:
   ***************************************************************************/
   
    #ifdef CONFIG_NET_ICMPv6
    static void tiva_ipv6multicast(FAR struct tiva_ethmac_s *priv)
    {
        struct net_driver_s *dev;
        uint16_t tmp16;
        uint8_t mac[6];
    
        /* For ICMPv6, we need to add the IPv6 multicast address
        * For IPv6 multicast addresses, the Ethernet MAC is derived by
        * the four low-order octets OR'ed with the MAC 33:33:00:00:00:00,
        * so for example the IPv6 address FF02:DEAD:BEEF::1:3 would map
        * to the Ethernet MAC address 33:33:00:01:00:03.
        * NOTES: This appears correct for the ICMPv6 Router Solicitation
        * Message, but the ICMPv6 Neighbor Solicitation message seems to
        * use 33:33:ff:01:00:03.
        */
    
        mac[0] = 0x33;
        mac[1] = 0x33;
    
        dev    = &priv->dev;
        tmp16  = dev->d_ipv6addr[6];
        mac[2] = 0xff;
        mac[3] = tmp16 >> 8;
    
        tmp16  = dev->d_ipv6addr[7];
        mac[4] = tmp16 & 0xff;
        mac[5] = tmp16 >> 8;
    
        nvdbg("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
        (void)tiva_addmac(dev, mac);
   
    #ifdef CONFIG_NET_ICMPv6_AUTOCONF
        /* Add the IPv6 all link-local nodes Ethernet address.  This is the
        * address that we expect to receive ICMPv6 Router Advertisement
        * packets.
        */
    
        (void)tiva_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);
   
    #endif /* CONFIG_NET_ICMPv6_AUTOCONF */
    #ifdef CONFIG_NET_ICMPv6_ROUTER
        /* Add the IPv6 all link-local routers Ethernet address.  This is the
        * address that we expect to receive ICMPv6 Router Solicitation
        * packets.
        */
    
        (void)tiva_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);
   
    #endif /* CONFIG_NET_ICMPv6_ROUTER */
    }
    #endif /* CONFIG_NET_ICMPv6 */


The following Ethernet drivers are complete and IPv6 ready. All others 
Ethernet drivers have all required IPv6 support `except` that they are 
missing (1) the required ICMPv6 addressing filtering described above 
and/or (2) support for multi-cast address filtering.

* STMicro STM32
* TI Tiva TM4C
* Atmel SAMA5D4
* NXP LPC17xx


Board Configurations
====================

At present, there are three board configuration that are pre-configured to 
use IPv6: ``nuttx/boards/arm/tiva/dk-tm4c129x/configs/ipv6``, 
``nuttx/boards/arm/stm32/stm32f4discovery/ipv6``, and 
``nuttx/boards/arm/tiva/tm4c1294-launchpad/configs/ipv6``. These default 
configurations have only IPv6 enabled. But the `README` files at in those 
board directories describes how to enable `both` IPv4 and IPv6 simultaneously.

Ping
====

Ping from Host PC
-----------------

Ping from Windows cmd Terminal
``````````````````````````````

.. code-block:: bash

    ping -6 fc00::2

Ping From Linux shell
`````````````````````

.. code-block:: bash

    ping6 fc00::2

Ping from the NuttShell (NSH)
-----------------------------

.. code-block:: bash

    nsh> ping6 fc00::2

NSH ifconfig
============

IPv4 Only
---------

``CONFIG_NET_IPv4=y`` and ``CONFIG_NET_IPv6=n``

.. code-block:: bash

    nsh> ifconfig
    eth0    Link encap: Ethernet HWaddr 00:1a:b6:02:81:14 at UP
            inet addr:10.0.0.2 DRaddr:10.0.0.1 Mask:255.255.255.0
    
                 IPv4   TCP   UDP  ICMP
    Received     003b  001c  0000  0004
    Dropped      001b  0000  0000  0000
      IPv4        VHL: 0000 Frg: 0000
      Checksum   0000  0000  0000  ----
      TCP         ACK: 0000 SYN: 0000
                  RST: 0000 0000
      Type       0000  ----  ----  0000
    Sent         0031  002d  0000  0004
      Rexmit     ----  ----  0000  ----

NOTE: The detailed packet statistics only appear if 
``CONFIG_NET_STATISTICS`` is enabled.

IPv6 Only
---------

``CONFIG_NET_IPv4=n`` and ``CONFIG_NET_IPv6=y``

.. code-block:: bash

    nsh> ifconfig
    eth0    Link encap: Ethernet HWaddr 00:1a:b6:02:81:14 at UP
            inet6 addr:fc00::2
            inet6 DRaddr:fc00::1
            inet6 Mask:ffff:ffff:ffff::ffff:ffff:ffff:ff80
    
                IPv6   TCP   UDP  ICMPv6
    Received     0007  0000  0000  0007
    Dropped      0000  0000  0000  0000
      IPv6        VHL: 0000
      Checksum   ----  0000  0000  ----
      TCP         ACK: 0000 SYN: 0000
                  RST: 0000 0000
      Type       0000  ----  ----  0000
    Sent         0011  0000  0000  0011
      Rexmit     ----  ----  0000  ----

Both IPv4 and IPv6
------------------

``CONFIG_NET_IPv4=y`` and ``CONFIG_NET_IPv6=y``

.. code-block:: bash

    nsh> ifconfig
    eth0    Link encap: Ethernet HWaddr 00:1a:b6:02:81:14 at UP
            inet addr:10.0.0.2 DRaddr:10.0.0.1 Mask:255.255.255.0
            inet6 addr:fc00::2
            inet6 DRaddr:fc00::1
            inet6 Mask:ffff:ffff:ffff::ffff:ffff:ffff:ff80
    
                 IPv4  IPv6   TCP   UDP  ICMP  ICMPv6
    Received     0047  000a  001c  0000  0004  000a
    Dropped      0027  0000  0000  0000  0000  0000
      IPv4        VHL: 0000 Frg: 0000
      IPv6        VHL: 0000
      Checksum   0000  ----  0000  0000  ----  ----
      TCP         ACK: 0000 SYN: 0000
                  RST: 0000 0000
      Type       0000  0000  ----  ----  0000  0000
    Sent         0033  000a  002f  0000  0004  000a
      Rexmit     ----  ----  ----  0000  ----  ----

Tests, Applications, and Network Utilities
==========================================

In addition to the core RTOS support IPv6, changes are also required to 
networking tests, to networking aware applications, and, of course, to all of 
the network utils (``netutils``).

* NuttShell (NSH): IPv6 support is partially available. NSH is capable of 
  initializing the IPv6 domain and some of the NSH commands have been adapted 
  to support IPv6. A ping6 command has been added. But there are many commands 
  that still require updating.
* Tests: There are several networking tests in ``apps/examples``. The 
  ``nettest`` test and the ``udp`` test have been adapted to work in the IPv6 
  domain, but none of the others have yet been adapted.
* Netutils: The network utilities in ``apps/netutils`` have been adapted to 
  work with IPv6: DHCP, FTP, TFTP, Telnet, etc. Support for managing IPv6 
  address have been included in the ``netlib``, but nothing else has yet been 
  updated.