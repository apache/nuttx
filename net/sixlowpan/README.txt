6LoWPAN Contents
----------------

  o 6LoWPAN Addressing
  o IPv6 Neighbor Discovery
  o Optimal 6LoWPAN Configuration
  o Star Configuration

6LoWPAN Addressing
------------------

The current 6LoWPAN implementation uses only link local, MAC-based
addressing addressing (as discussed in more detail below).  Thus if you know
the node addressing, then you know the IPv6 address (and vice-versa).

As a configuration option, the 6LoWPAN implementation will use either the
node's 2-byte short address or 8-byte extended address as the MAC address
that the IPv6 address is based on.  This is determined by the configuration
setting CONFIG_NET_6LOWPAN_EXTENDEDADDR.  By default, the 2-byte short
address is used for the IEEE802.15.4 MAC device's link layer address.  If
this option is selected, then an 8-byte extended address will be used,
instead.

All nodes operating on a network have unique, 8-byte extended address,
that was assigned before the network is configured.  6LoWPAN will use
either the extended address for direct communication within the PAN or
the short 2-byte address.  The short 2-byte address, however, is allocated
by the PAN coordinator when the device associated.  If short addresses are
used, the network cannot be brought up until is is associated.

Node addressing is modified through IOCTL calls from application logic.
The network must be in the DOWN state when ever the node addressing is
modified.  The modified node addresses will have no effect on the reported
IPv6 address until the network is brought to the UP state.  The new IPv6
MAC-based addresses are only instantiated when the network transitions
from the DOWN to UP state.

IPv6 Neighbor Discovery
-----------------------

IPv6 Neighbor Discovery is not supported.  The current ICMPv6 and neighbor-
related logic only works with Ethernet MAC.  For 6LoWPAN, a new more
conservative IPv6 neigbor discovery is provided by RFC 6775 which is not
currently supported.  With IPv6 neighbor discovery, any IPv6 address may be
associated with any short or extended address.  In fact, that is the whole
purpose of the neighbor discover logic:  It plays the same role as ARP in
IPv4; it ultimately just manages a neighbor table that, like the arp table,
provides the mapping between IP addresses and node addresses.

The NuttX, Contiki-based 6LoWPAN implementation circumvents  the need for
the neighbor discovery logic by using only MAC-based addressing, i.e., the
lower two or eight bytes of the IP address are the node address.

Most of the 6LoWPAN compression algorithms exploit this kind of addressing
to compress the IPv6 address to nothing but a single bit indicating that the
IP address derives from the node address.  In this use case, IPv6 neighbor
discover is not useful:  If we want to use IPv6 neighbor discovery, we could
dispense with the all MAC  based addressing.  But if we want to retain the
more compact MAC-based addressing, then we don't need IPv6 neighbor discovery.

However, it would still be nice to have enough in place to support ping6.
Full neighbor support would be necessary if we wanted to route 6LoWPAN frames
outside of the WPAN.

Optimal 6LoWPAN Configuration
-----------------------------

1. Link local IP addresses:

   128  112  96   80    64   48   32   16
   fe80 0000 0000 0000  xxxx xxxx xxxx xxxx

2. MAC-based IP addresses:

   128  112  96   80    64   48   32   16
   ---- ---- ---- ----  ---- ---- ---- ----
   AAAA xxxx xxxx xxxx  xxxx 00ff fe00 MMMM 2-byte short address IEEE 48-bit MAC
   AAAA 0000 0000 0000  NNNN NNNN NNNN NNNN 8-byte extended address IEEE EUI-64

   Where MMM is the 2-byte short address XORed 0x0200.  For example, the MAC
   address of 0xabcd would be 0xa9cd.  And NNNN NNNN NNNN NNNN is the 8-byte
   extended address address XOR 02000 0000 0000 0000.

   For link-local address, AAAA is 0xfe80

3. MAC based link-local addresses

   128  112  96   80    64   48   32   16
   ---- ---- ---- ----  ---- ---- ---- ----
   fe80 0000 0000 0000  0000 00ff fe00 MMMM 2-byte short address IEEE 48-bit MAC
   fe80 0000 0000 0000  NNNN NNNN NNNN NNNN 8-byte extended address IEEE EUI-64

4. To be compressable, port numbers must be in the range 0xf0b0-0xf0bf,
   hexadecimal.  That is 61616-61631 decimal.

5. IOBs: Must be big enough to hold one IEEE802.15.4 frame (typically 127).
   There must be enough IOBs to decompose the largest IPv6 packet
   (CONFIG_NET_6LOWPAN_PKTSIZE, default 1294, plus per frame overhead).

Fragmentation Headers
---------------------
A fragment header is placed at the beginning of the outgoing packet just
after the MAC header when the payload is too large to fit in a single IEEE
802.15.4 frame. The fragment header contains three fields: Datagram size,
datagram tag and datagram offset.

1. Datagram size describes the total (un-fragmented) payload.
2. Datagram tag identifies the set of fragments and is used to match
   fragments of the same payload.
3. Datagram offset identifies the fragment’s offset within the un-
   fragmented payload (in units of 8 bytes).

The length of the fragment header length is four bytes for the first header
(FRAG1) and five bytes for all subsequent headers (FRAGN).  For example,
this is a HC1 compressed first frame of a packet

  41 88 2a cefa 3412 cdab                       ### 9-byte MAC header
  c50e 000b                                     ### 4-byte FRAG1 header
  42                                            ### SIXLOWPAN_DISPATCH_HC1
    fb                                          ### SIXLOWPAN_HC1_HC_UDP_HC1_ENCODING
    e0                                          ### SIXLOWPAN_HC1_HC_UDP_UDP_ENCODING
    00                                          ### SIXLOWPAN_HC1_HC_UDP_TTL
    10                                          ### SIXLOWPAN_HC1_HC_UDP_PORTS
    0000                                        ### SIXLOWPAN_HC1_HC_UDP_CHKSUM

  104 byte Payload follows:
  4f4e452064617920 48656e6e792d7065 6e6e792077617320 7069636b696e6720
  757020636f726e20 696e207468652063 6f726e7961726420 7768656e2d2d7768
  61636b212d2d736f 6d657468696e6720 6869742068657220 75706f6e20746865
  20686561642e2027

This is the second frame of the same transfer:

  41 88 2b cefa 3412 cdab                       ### 9-byte MAC header
  e50e 000b 0d                                  ### 5 byte FRAGN header
  42                                            ### SIXLOWPAN_DISPATCH_HC1
    fb                                          ### SIXLOWPAN_HC1_HC_UDP_HC1_ENCODING
    e0                                          ### SIXLOWPAN_HC1_HC_UDP_UDP_ENCODING
    00                                          ### SIXLOWPAN_HC1_HC_UDP_TTL
    10                                          ### SIXLOWPAN_HC1_HC_UDP_PORTS
    0000                                        ### SIXLOWPAN_HC1_HC_UDP_CHKSUM

  104 byte Payload follows:
  476f6f646e657373 2067726163696f75 73206d6521272073 6169642048656e6e
  792d70656e6e793b 202774686520736b 79277320612d676f 696e6720746f2066
  616c6c3b2049206d 75737420676f2061 6e642074656c6c20 746865206b696e67
  2e270a0a536f2073

The payload length is encoded in the LS 11-bits of the first 16-bit value:
In this example the payload size is 0x050e or 1,294.  The tag is 0x000b.  In
the second frame, the fifth byte contains the offset 0x0d which is 13 << 3 =
104 bytes, the size of the payload on the first packet.

Star Configuration
------------------

The 6LoWPAN stack can be specially configured as member in a star topology;
either as a endpoint on the star os the star hub.  The endpoint is
created with the following settings in the configuration file:

  CONFIG_NET_STAR=y
  CONFIG_NET_STARPOINT=y

The CONFIG_NET_STARPOINT selection informs the endpoint 6LoWPAN stack that
it must send all frames to the hub of the star, rather than directly to the
recipient.  The star hub is assumed to be the cooordinator.

The star hub configuration, on the other hand, uses these setting:

  CONFIG_NET_STAR=y
  CONFIG_NET_STARHUB=y
  CONFIG_NET_IPFORWARD=y

The CONFIG_NET_IPFORWARD selection informs the hub that if it receives any
packets that are not destined for the hub, it should forward those packets
appropriately.  This affects the behavior of IPv6 packet reception logic but
does not change the behavior of the 6LoWPAN stack.
