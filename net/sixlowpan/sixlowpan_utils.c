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
#include <debug.h>

#include <nuttx/net/sixlowpan.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_addrfromip
 *
 * Description:
 *   Extract the IEEE 802.15.4 address from a link local IPv6 address:
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
 *
 ****************************************************************************/

void sixlowpan_addrfromip(const net_ipv6addr_t ipaddr,
                          FAR struct sixlowpan_addr_s *addr)
{
  DEBUGASSERT(ipaddr[0] == HTONS(0xfe80));

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
  memcpy(addr, &ipaddr[4], NET_6LOWPAN_ADDRSIZE);
#else
  memcpy(addr, &ipaddr[7], NET_6LOWPAN_ADDRSIZE);
#endif
  addr->u8[0] ^= 0x02;
}

/****************************************************************************
 * Name: sixlowpan_ismacbased
 *
 * Description:
 *   Check if the MAC address is encoded in the IP address:
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
 *
 ****************************************************************************/

bool sixlowpan_ismacbased(const net_ipv6addr_t ipaddr,
                          FAR const struct sixlowpan_addr_s *addr)
{
  FAR const uint8_t *byteptr = addr->u8;

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
  return (ipaddr[4] == htons((GETINT16(byteptr, 0) ^ 0x0200)) &&
          ipaddr[5] == GETINT16(byteptr, 2) &&
          ipaddr[6] == GETINT16(byteptr, 4) &&
          ipaddr[7] == GETINT16(byteptr, 6));
#else
  return (ipaddr[5] == HTONS(0x00ff) && ipaddr[6] == HTONS(0xfe00) &&
          ipaddr[7] == htons((GETINT16(byteptr, 0) ^ 0x0200)));
#endif
}

/****************************************************************************
 * Name: sixlowpan_src_panid
 *
 * Description:
 *   Get the source PAN ID from the IEEE802.15.4 radio.
 *
 * Input parameters:
 *   ieee  - A reference IEEE802.15.4 MAC network device structure.
 *   panid - The location in which to return the PAN ID.  0xfff may be
 *           returned if the device is not associated.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sixlowpan_src_panid(FAR struct ieee802154_driver_s *ieee,
                        FAR uint16_t *panid)
{
  FAR struct net_driver_s *dev = &ieee->i_dev;
  struct ieee802154_netradio_s arg;
  int ret;

  memcpy(arg.ifr_name, ieee->i_dev.d_ifname, IFNAMSIZ);
  ret = dev->d_ioctl(dev, PHY802154IOC_GET_PANID, (unsigned long)((uintptr_t)&arg));
  if (ret < 0)
    {
      wlerr("ERROR: PHY802154IOC_GET_PANID failed: %d\n", ret);
      return ret;
    }

  *panid = arg.u.panid;
  return OK;
}

#endif /* CONFIG_NET_6LOWPAN */
