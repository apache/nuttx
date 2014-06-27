README
======

Directory Structure
===================

  nuttx/
   |
   `- net/
       |
       +- arp      - Address resolution protocol
       +- devif    - Stack/device interface layer
       +- icmp     - Internet Control Message Protocol
       +- iob      - I/O buffering logic
       +- netdev   - Socket network device interface
       +- pkt      - "Raw" packet socket support
       +- socket   - BSD socket interface
       +- route    - Routing table support
       +- tcp      - Transmission Control Protocol
       +- udp      - User Datagram Protocol
       `- utils    - Miscellaneous utility functions


    +-------------------------------------------------------------+
    |                   Application layer                         |
    +-------------------------------------------------------------+
    +-------------------------------------------------------------+
    |                 Socket layer (socket/)                      |
    +-------------------------------------------------------------+
    +------------++-----------------------------------------------+
    |  Network   ||  Protocol stacks (arp, icmp, pkt, tcp, udp)   |
    |   Device   |+-----------------------------------------------+
    | Interface  |+---------------------------------++------------+
    | (netdev/)  ||Network Device Interface (devif/)|| Utilities  |
    +------------++---------------------------------++------------+
    +-------------------------------------------------------------+
    |                  Network Device Drivers                     |
    +-------------------------------------------------------------+
