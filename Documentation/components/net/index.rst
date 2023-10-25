===============
Network Support
===============

.. toctree::
  :maxdepth: 1

  sixlowpan.rst

``net`` Directory Structure ::

  nuttx/
   |
   `- net/
       |
       +- arp        - Address resolution protocol (IPv4)
       +- bluetooth  - PF_BLUETOOTH socket interface
       +- devif      - Stack/device interface layer
       +- ipfrag     - Fragmentation and reassembly
       +- icmp       - Internet Control Message Protocol (IPv4)
       +- icmpv6     - Internet Control Message Protocol (IPv6)
       +- ieee802154 - PF_IEEE802154 socket interface
       +- inet       - PF_INET/PF_INET6 socket interface
       +- ipforward  - IP forwarding logic
       +- local      - Unix domain (local) sockets
       +- mld        - Multicast Listener Discovery (MLD)
       +- neighbor   - Neighbor Discovery Protocol (IPv6)
       +- netdev     - Socket network device interface
       +- netlink    - Netlink IPC socket interface
       +- pkt        - "Raw" packet socket support
       +- sixlowpan  - 6LoWPAN implementation
       +- socket     - BSD socket interface
       +- route      - Routing table support
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
