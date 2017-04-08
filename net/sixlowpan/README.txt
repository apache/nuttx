Optimal 6loWPAN Configuration
-----------------------------

1. Link local IP addresses:

   128  112  96   80    64   48   32   16
   fe80 0000 0000 0000  xxxx xxxx xxxx xxxx

2. MAC-based IP addresses:

   128  112  96   80    64   48   32   16
   ---- ---- ---- ----  ---- ---- ---- ----
   xxxx xxxx xxxx xxxx  xxxx 00ff fe00 MMMM 2-byte Rime address IEEE 48-bit MAC
   fe80 0000 0000 0000  NNNN NNNN NNNN NNNN 8-byte Rime address IEEE EUI-64

   Where MMM is the 2-byte rime address XOR 0x0200.  For example, the MAC
   address of 0xabcd would be 0xa9cd.  And NNNN NNNN NNNN NNNN is the 8-byte
   rime address address XOR 02000 0000 0000 0000

3. MAC based link-local addresses

   128  112  96   80    64   48   32   16
   ---- ---- ---- ----  ---- ---- ---- ----
   fe80 0000 0000 0000  0000 00ff fe00 MMMM 2-byte Rime address IEEE 48-bit MAC
   fe80 0000 0000 0000  NNNN NNNN NNNN NNNN 8-byte Rime address IEEE EUI-64

4. Compressable port numbers in the rangs 0xf0b0-0xf0bf

5. IOBs: Must be big enough to hold one IEEE802.15.4 frame (CONFIG_NET_6LOWPAN_FRAMELEN,
   typically 127).  There must be enough IOBs to decompose the largest IPv6
   packet (CONFIG_NET_6LOWPAN_MTU, default 1294, plus per frame overhead).

Fragmentation Headers
---------------------
A fragment header is placed at the beginning of the outgoing packet when the
payload is too large to fit in a single IEEE 802.15.4 frame. The fragment
header contains three fields: Datagram size, datagram tag and datagram
offset.

1. Datagram size describes the total (un-fragmented) payload.
2. Datagram tag identifies the set of fragments and is used to match
   fragments of the same payload.
3. Datagram offset identifies the fragment’s offset within the un-
   fragmented payload (in units of 8 bytes).

The length of the fragment header length is four bytes for the first header
(FRAG1) and five bytes for all subsequent headers (FRAGN).  For example,
this is a HC1 compressed first frame of a packet

  c50e 000b                                                            ### 4-byte FRAG1 header
  01 08 01 0000 3412                                                   ### 7-byte FCF header
  42                                                                   ### SIXLOWPAN_DISPATCH_HC1
    fb                                                                 ### RIME_HC1_HC_UDP_HC1_ENCODING
    e0                                                                 ### RIME_HC1_HC_UDP_UDP_ENCODING
    00                                                                 ### RIME_HC1_HC_UDP_TTL
    10                                                                 ### RIME_HC1_HC_UDP_PORTS
    0000                                                               ### RIME_HC1_HC_UDP_CHKSUM
  4f4e452064617920 48656e6e792d7065 6e6e792077617320 7069636b696e6720  ### 80 byte payload
  757020636f726e20 696e207468652063 6f726e7961726420 7768656e2d2d7768
  61636b212d2d736f 6d657468696e6720                                                              g

This is the second frame of the same transfer:

  e50e 000b 0a                                                         ### 5 byte FRAGN header
  01 08 01 0000 3412                                                   ### 7-byte FCF header
  6869742068657220 75706f6e20746865 20686561642e2027 476f6f646e657373  ### 88 byte payload
  2067726163696f75 73206d6521272073 6169642048656e6e 792d70656e6e793b
  202774686520736b 79277320612d676f 696e6720746f2066

The payload length is encoded in the LS 11-bits of the first 16-bit value:
In this example the payload size is 0x050e or 1,294.  The tag is 0x000b.  In
the second frame, the fifth byte contains the offset 0x0a which is 10 << 3 =
80 bytes, the size of the payload on the first packet.
