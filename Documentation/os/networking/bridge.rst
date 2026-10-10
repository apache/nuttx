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
  Enable bridge support.  Requires ``CONFIG_NET_ETHERNET``.
``CONFIG_NET_BRIDGE_MAX_PORTS``
  Maximum number of ports per bridge (default 4).
``CONFIG_NET_BRIDGE_FDB_SIZE``
  Number of MAC addresses that each bridge can learn (default 32).  When
  the table is full, the least recently seen address is replaced.
``CONFIG_NET_BRIDGE_AGEING_TIME``
  Seconds after which an address that sent no frame is forgotten
  (default 300).

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
* Only devices that use the upper-half driver interface
  (``include/nuttx/net/netdev_lowerhalf.h``) can be ports.  Adding another
  device fails with ``EOPNOTSUPP``.
* To forward unicast frames, a port must receive frames for foreign MAC
  addresses: Ethernet devices need ``CONFIG_NET_PROMISCUOUS`` (if their
  driver supports it), and IEEE 802.11 devices must be in access point mode,
  since a station cannot send frames with foreign source addresses.
* There is no VLAN filtering: tagged frames are forwarded unchanged.
* A frame for a port that has no room is dropped, as there is no queue
  between the bridge and the ports.  Frames sent by the local host through
  the bridge device are not dropped for that reason: the network stack
  waits until the ports have room.

Implementation
==============

The code is in ``drivers/net/bridge.c``.  The bridge device is a network
device of the upper-half driver interface, like a VLAN device, and the ports
are ordinary upper-half devices:

* **Receive**: the upper half of a port gives every received frame to the
  bridge instead of the network stack.  The bridge learns the source
  address, sends the frame to the other ports with the ``transmit``
  operation of the port, and passes it to the network stack with
  ``netdev_lower_input()`` when the frame is for the local host.
* **Transmit**: the ``transmit`` operation of the bridge device forwards a
  frame sent by the local host in the same way.  The TX quota of the bridge
  device is the room of its ports with the least room, so the network
  stack only polls the bridge device when the ports can take a frame, and
  polls again when a port reports a completed transmission.
* **Locking**: the bridge lock serializes the filtering database and the
  transmission to the ports, and it is never held while a network device
  lock is taken, so the receive path of a port, which holds the port lock,
  can forward to another port without a lock order inversion.

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
