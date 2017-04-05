/****************************************************************************
 * net/sixlowpan/sixlowpan_utils.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright (C) 2017, Gregory Nutt, all rights reserved
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from logic in Contiki:
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

#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/net/sixlowpan.h>

#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_ipfromrime
 *
 * Description:
 *   Create a link local IPv6 address from a rime address:
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte Rime address
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte Rime address
 *
 ****************************************************************************/

void sixlowpan_ipfromrime(FAR const struct rimeaddr_s *rime,
                          net_ipv6addr_t ipaddr)
{
  /* We consider only links with IEEE EUI-64 identifier or IEEE 48-bit MAC
   * addresses.
   */

  memset(ipaddr, 0, sizeof(net_ipv6addr_t));
  ipaddr[0] = HTONS(0xfe80);

#if CONFIG_NET_6LOWPAN_RIMEADDR_SIZE == 2
  ipaddr[5]  = HTONS(0x00ff);
  ipaddr[6]  = HTONS(0xfe00);
  memcpy(&ipaddr[7], rime, CONFIG_NET_6LOWPAN_RIMEADDR_SIZE);
  ipaddr[7] ^= HTONS(0x0200);
#else
  memcpy(&ipaddr[4], rime, CONFIG_NET_6LOWPAN_RIMEADDR_SIZE);
  ipaddr[4] ^= HTONS(0x0200);
#endif
}

/****************************************************************************
 * Name: sixlowpan_rimefromip
 *
 * Description:
 *   Extract the rime address from a link local IPv6 address:
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte Rime address
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte Rime address
 *
 ****************************************************************************/

void sixlowpan_rimefromip(const net_ipv6addr_t ipaddr,
                          FAR struct rimeaddr_s *rime)
{
  /* REVISIT: See notes about 2 byte addresses in sixlowpan_ipfromrime() */

  DEBUGASSERT(ipaddr[0] == HTONS(0xfe80));

#if CONFIG_NET_6LOWPAN_RIMEADDR_SIZE == 2
  memcpy(rime, &ipaddr[7], CONFIG_NET_6LOWPAN_RIMEADDR_SIZE);
#else
  memcpy(rime, &ipaddr[4], CONFIG_NET_6LOWPAN_RIMEADDR_SIZE);
#endif
  rime->u8[0] ^= 0x02;
}

/****************************************************************************
 * Name: sixlowpan_ismacbased
 *
 * Description:
 *   Check if the MAC address is encoded in the IP address:
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte Rime address
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte Rime address
 *
 ****************************************************************************/

bool sixlowpan_ismacbased(const net_ipv6addr_t ipaddr,
                          FAR const struct rimeaddr_s *rime)
{
  FAR const uint8_t *rimeptr = rime->u8;

#if CONFIG_NET_6LOWPAN_RIMEADDR_SIZE == 2
  return ((ipaddr[4] == htons((GETINT16(rimeptr, 0) ^ 0x0200))) &&
           ipaddr[5] == 0 && ipaddr[6] == 0 && ipaddr[7] == 0);
#else /* CONFIG_NET_6LOWPAN_RIMEADDR_SIZE == 8 */
  return ((ipaddr[4] == htons((GETINT16(rimeptr, 0) ^ 0x0200))) &&
           ipaddr[5] == GETINT16(rimeptr, 2) &&
           ipaddr[6] == GETINT16(rimeptr, 4) &&
           ipaddr[7] == GETINT16(rimeptr, 6));
#endif
}

#endif /* CONFIG_NET_6LOWPAN */
