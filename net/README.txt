README
======

Directory Structure
===================

  nuttx/
   |
   `- net/
       |
       +- arp      - Address resolution protocol (IPv4)
       +- devif    - Stack/device interface layer
       +- icmp     - Internet Control Message Protocol (IPv4)
       +- icmpv6   - Internet Control Message Protocol (IPv6)
       +- iob      - I/O buffering logic
       +- neighbor - Neighbor Discovery Protocol (IPv6)
       +- netdev   - Socket network device interface
       +- pkt      - "Raw" packet socket support
       +- socket   - BSD socket interface
       +- route    - Routing table support
       +- tcp      - Transmission Control Protocol
       +- udp      - User Datagram Protocol
       `- utils    - Miscellaneous utility functions


    +----------------------------------------------------------------+
    |                     Application layer                          |
    +----------------------------------------------------------------+
    +----------------------------------------------------------------+
    |                   Socket layer (socket/)                       |
    +----------------------------------------------------------------+
    +------------++--------------------------------------------------+
    |  Network   || Protocol stacks (arp, ipv6, icmp, pkt, tcp, udp) |
    |   Device   |+--------------------------------------------------+
    | Interface  |+------------------------------------++------------+
    | (netdev/)  ||  Network Device Interface (devif/) || Utilities  |
    +------------++------------------------------------++------------+
    +----------------------------------------------------------------+
    |                    Network Device Drivers                      |
    +----------------------------------------------------------------+
