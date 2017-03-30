/****************************************************************************
 * net/sixlowpan/sixlowpan_hc1.c
 *
 *   Copyright (C) 2017, Gregory Nutt, all rights reserved
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from Contiki:
 *
 *   Copyright (c) 2008, Swedish Institute of Computer Science.
 *   All rights reserved.
 *   Authors: Adam Dunkels <adam@sics.se>
 *            Nicolas Tsiftes <nvt@sics.se>
 *            Niclas Finne <nfi@sics.se>
 *            Mathilde Durvy <mdurvy@cisco.com>
 *            Julien Abeille <jabeille@cisco.com>
 *            Joakim Eriksson <joakime@sics.se>
 *            Joel Hoglund <joel@sics.se>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/net/netdev.h>
#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC1

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_compresshdr_hc1
 *
 * Description:
 *   Compress IP/UDP header using HC1 and HC_UDP
 *
 *   This function is called by the 6lowpan code to create a compressed
 *   6lowpan packet in the packetbuf buffer from a full IPv6 packet in the
 *   uip_buf buffer.
 *
 *   If we can compress everything, we use HC1 dispatch, if not we use
 *   IPv6 dispatch.  We can compress everything if:
 *
 *     - IP version is
 *     - Flow label and traffic class are 0
 *     - Both src and dest ip addresses are link local
 *     - Both src and dest interface ID are recoverable from lower layer
 *       header
 *     - Next header is either ICMP, UDP or TCP
 *
 *   Moreover, if next header is UDP, we try to compress it using HC_UDP.
 *   This is feasible is both ports are between F0B0 and F0B0 + 15\n\n
 *
 *   Resulting header structure:
 *   - For ICMP, TCP, non compressed UDP\n
 *     HC1 encoding = 11111010 (UDP) 11111110 (TCP) 11111100 (ICMP)\n
 *                      1                   2                   3
 *   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | LoWPAN HC1 Dsp | HC1 encoding  | IPv6 Hop limit| L4 hdr + data|
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | ...
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 *   - For compressed UDP
 *     HC1 encoding = 11111011, HC_UDP encoding = 11100000\n
 *                      1                   2                   3
 *   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | LoWPAN HC1 Dsp| HC1 encoding  |  HC_UDP encod.| IPv6 Hop limit|
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | src p.| dst p.| UDP checksum                  | L4 data...
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * Input Parmeters:
 *   dev      - A reference to the IEE802.15.4 network device state
 *   destaddr - L2 destination address, needed to compress the IP
 *              destination field
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sixlowpan_compresshdr_hc1(FAR struct net_driver_s *dev,
                               FAR struct rimeaddr_s *destaddr)
{
/* REVISIT: To be provided */
}

/****************************************************************************
 * Name: sixlowpan_uncompresshdr_hc1
 *
 * Description:
 *   Uncompress HC1 (and HC_UDP) headers and put them in sixlowpan_buf
 *
 *   This function is called by the input function when the dispatch is
 *   HC1.  It processes the packet in the rime buffer, uncompresses the
 *   header fields, and copies the result in the sixlowpan buffer.  At the
 *   end of the decompression, g_rime_hdrlen and uncompressed_hdr_len
 *   are set to the appropriate values
 *
 * Input Parameters:
 *   dev   - A reference to the IEE802.15.4 network device state
 *   iplen - Equal to 0 if the packet is not a fragment (IP length is then
 *           inferred from the L2 length), non 0 if the packet is a 1st
 *           fragment.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sixlowpan_uncompresshdr_hc1(FAR struct net_driver_s *dev,
                                 uint16_t iplen)
{
/* REVISIT: To be provided */
}

#endif /* CONFIG_NET_6LOWPAN_COMPRESSION_HC1 */
