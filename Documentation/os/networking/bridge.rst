===============
Ethernet Bridge
===============

The Ethernet bridge joins several Ethernet (or IEEE 802.11) network devices
into one layer 2 network, like a simple learning switch.  It follows the
IEEE 802.1D transparent bridge model without the spanning tree protocol.

A bridge is a virtual network device, ``br0`` for example.  Real devices are
added to it as *ports*.  Frames received on a port are forwarded to the other
ports, and frames for the local host are passed to the network stack through
the bridge device.  The IP configuration belongs to the bridge device: the
ports normally have no IP address.

::

        IP stack (sockets, DHCP server, ...)
                       |
                    [ br0 ]  <- IP address, MAC address
                    /     \
               [ eth0 ]  [ wlan0 ]  <- ports, no IP address
                  |          |
             wired LAN   Wi-Fi clients

Configuration Options
=====================

``CONFIG_NET_BRIDGE``
  Enable bridge support.  Requires ``CONFIG_NET_ETHERNET``,
  ``CONFIG_SCHED_LPWORK`` and ``CONFIG_IOB_NCHAINS`` > 0.
``CONFIG_NET_BRIDGE_MAX_PORTS``
  Maximum number of ports per bridge (default 4).
``CONFIG_NET_BRIDGE_FDB_SIZE``
  Number of MAC addresses that each bridge can learn (default 32).  When
  the table is full, the least recently seen address is replaced.
``CONFIG_NET_BRIDGE_AGEING_TIME``
  Seconds after which an address that sent no frame is forgotten
  (default 300).
``CONFIG_NET_BRIDGE_TXQ_LEN``
  Maximum number of frames waiting to be sent on each port (default 16).
  Frames are dropped when the queue is full.  Every queued frame holds at
  least one IOB, so ``CONFIG_IOB_NBUFFERS`` must leave room for the queues
  of all ports.

The TCP receive window is computed from the number of free IOBs, so a TCP
sender may fill all of them.  The bridge also needs IOBs that the window does
not account for: the frames waiting in the port queues and, for drivers that
use a flat buffer (``d_buf``), a copy of every received frame.  If none is
left, received frames are dropped and the TCP throughput collapses (the
sender waits for retransmission timeouts).  Set ``CONFIG_IOB_THROTTLE`` to
reserve IOBs for the bridge: they are not counted in the TCP receive window,
but the bridge can use them.  ``CONFIG_NET_BRIDGE_TXQ_LEN`` is a reasonable
value.

For example, on a PIC32MZ board (flat buffer driver, one bridge port,
``CONFIG_IOB_NBUFFERS=48``, ``CONFIG_IOB_BUFSIZE=1600``), TCP into the bridge
went from 2.4 Mbit/s to 45 Mbit/s with ``CONFIG_IOB_THROTTLE=16``.

Usage
=====

Bridges are configured with the Linux bridge ioctl commands, which the
``brctl`` command (``CONFIG_SYSTEM_BRCTL``) wraps:

``SIOCBRADDBR``
  Create a bridge.  The argument is the name of the new device.
``SIOCBRDELBR``
  Delete a bridge.  The bridge must be down.
``SIOCBRADDIF`` / ``SIOCBRDELIF``
  Add a port to a bridge or remove it.  The argument is a
  ``struct ifreq`` with the bridge name in ``ifr_name`` and the interface
  index of the port in ``ifr_ifindex``.

Example from NSH:

.. code-block:: console

  nsh> ifup eth0
  nsh> ifup eth1
  nsh> brctl addbr br0
  nsh> brctl addif br0 eth0
  nsh> brctl addif br0 eth1
  nsh> ifconfig br0 10.0.0.2 netmask 255.255.255.0
  nsh> ifup br0

The bridge takes the MAC address of its first port, unless one was set
before.  Its MTU is the smallest MTU of its ports.

Forwarding
==========

* The source address of every received frame is learned in the filtering
  database (FDB) of the bridge, together with the receiving port.
* Unicast frames for the address of the bridge or of one of its ports go to
  the local host.
* Unicast frames for a learned address are sent on the port of that address,
  or dropped if that is the receiving port.
* Unicast frames for an unknown address are sent on all other ports
  (flooding).
* Broadcast and multicast frames go to the local host and to all other
  ports, except the IEEE 802.1D reserved addresses ``01:80:C2:00:00:00`` to
  ``01:80:C2:00:00:0F``, which only go to the local host.
* Frames sent by the local host through the bridge device are forwarded the
  same way.

Packet sockets bound to a port see every frame received on it.  Packet
sockets bound to the bridge device see the frames for the local host and
the frames it sends.

Limitations
===========

* There is no spanning tree protocol.  The network must not contain loops
  through the bridge, or broadcast frames will circulate forever.
* To forward unicast frames, a port must receive frames for foreign MAC
  addresses: Ethernet devices need ``CONFIG_NET_PROMISCUOUS`` (if their
  driver supports it), and IEEE 802.11 devices must be in access point mode,
  since a station cannot send frames with foreign source addresses.
* Devices using the upper-half driver interface
  (``include/nuttx/net/netdev_lowerhalf.h``) pass every received frame to the
  bridge.  Other drivers only pass the frames that they give to
  ``ipv4_input()``, ``ipv6_input()`` or ``arp_input()``, so other protocols
  are not bridged on those ports.
* There is no VLAN filtering: tagged frames are forwarded unchanged.

Implementation
==============

The code is in ``net/bridge``.  A port is marked by the ``d_bridge`` field of
its ``struct net_driver_s``:

* **Receive**: the upper-half driver RX path and the ``ipv4_input()``,
  ``ipv6_input()`` and ``arp_input()`` entry points hand frames received on
  a port to ``bridge_input()`` instead of the network stack.
* **Transmit**: forwarded frames are queued on the egress port.  The port is
  notified with the ``BRIDGE_POLL`` poll type, and ``devif_poll()`` sends the
  queued frames as they are, without building a new link layer header.  This
  works for drivers using the upper-half interface and for drivers calling
  ``devif_poll()`` directly.
* **Locking**: the TX notification of the ports and the TX poll of the
  bridge device run on the low priority work queue, so that no network
  device is ever notified while another one is locked.  This avoids lock
  order inversions between two ports that forward to each other.

Testing on the Simulator
========================

The ``sim:bridge`` configuration has two TAP devices, ``eth0`` and ``eth1``.
On the Linux host, put each TAP device in its own network namespace (as
root), then bridge them in NuttX as shown above:

.. code-block:: console

  # ip netns add bra
  # ip link set tap0 netns bra
  # ip -n bra addr add 10.0.0.1/24 dev tap0
  # ip -n bra link set tap0 up
  # ip netns add brb
  # ip link set tap1 netns brb
  # ip -n brb addr add 10.0.0.3/24 dev tap1
  # ip -n brb link set tap1 up
  # ip netns exec bra ping 10.0.0.3

The ``nuttx`` program must run as root (or with ``CAP_NET_ADMIN``) to create
the TAP devices.
