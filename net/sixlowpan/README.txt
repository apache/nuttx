Optimal 6loWPAN Configuration:

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