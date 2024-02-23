===============
Network Support
===============

.. toctree::
  :maxdepth: 1

  sixlowpan.rst
  socketcan.rst
  pkt.rst
  nat.rst
  netdev.rst
  netguardsize.rst
  slip.rst
  wqueuedeadlocks.rst

``net`` Directory Structure ::

  nuttx/
   |
   `- net/
       |
       +- arp        - Address resolution protocol (IPv4)
       +- bluetooth  - PF_BLUETOOTH socket interface
       +- can        - SocketCAN
       +- devif      - Stack/device interface layer
       +- icmp       - Internet Control Message Protocol (IPv4)
       +- icmpv6     - Internet Control Message Protocol (IPv6)
       +- ieee802154 - PF_IEEE802154 socket interface
       +- igmp       - IGMPv2 client
       +- inet       - PF_INET/PF_INET6 socket interface
       +- ipforward  - IP forwarding logic
       +- ipfrag     - Fragmentation and reassembly
       +- local      - Unix domain (local) sockets
       +- mld        - Multicast Listener Discovery (MLD)
       +- nat        - Network Address Translation (NAT)
       +- neighbor   - Neighbor Discovery Protocol (IPv6)
       +- netdev     - Socket network device interface
       +- netfilter  - Iptables Interface
       +- netlink    - Netlink IPC socket interface
       +- pkt        - "Raw" packet socket support
       +- procfs     - net devices PROCFS support
       +- route      - Routing table support
       +- rpmsg      - Rpmsg domain (remote) sockets
       +- sixlowpan  - 6LoWPAN implementation
       +- socket     - BSD socket interface
       +- tcp        - Transmission Control Protocol
       +- udp        - User Datagram Protocol
       +- usrsock    - User socket API for user-space networking stack
       `- utils      - Miscellaneous utility functions

    +-------------------------------------------------------------------++------------------------+
    |                     Application layer                             || usrsock daemon         |
    +-------------------------------------------------------------------++------------------------+
    +-------------------------------------------------------------------++----------------+ +-----+
    |                   Socket layer (socket/)                          || /dev/usrsock   | |     |
    +-------------------------------------------------------------------++----------------+ |     |
    +------------++--------------------------------------------------++-------------------+ |     |
    |  Network   || Protocol stacks (arp, ipv6, icmp, pkt, tcp, udp) || usrsock/          | |     |
    |   Device   |+--------------------------------------------------++-------------------+ |     |
    | Interface  |+------------------------------------++---------------------------------+ |     |
    | (netdev/)  ||  Network Device Interface (devif/) || Utilities                       | |     |
    +------------++------------------------------------++---------------------------------+ |     |
    +----------------------------------------------------------------+                      |     |
    |                    Network Device Drivers                      |                      | HAL |
    +----------------------------------------------------------------+                      +-----+
    +----------------------------------------------------------------+ +--------------------------+
    |                    Networking Hardware                         | |  Hardware TCP/IP Stack   |
    +----------------------------------------------------------------+ +--------------------------+
