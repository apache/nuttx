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
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_[s|e]addrfromip
 *
 * Description:
 *   sixlowpan_[s|e]addrfromip(): Extract the IEEE 802.15.14 address from a
 *   MAC-based IPv6 address.  sixlowpan_saddrfromip() and
 *   sixlowpan_eaddrfromip() handle short and extended addresses,
 *   respectively.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    xxxx 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    xxxx 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
 *
 ****************************************************************************/

#ifndef CONFIG_NET_STARPOINT
static void sixlowpan_saddrfromip(const net_ipv6addr_t ipaddr,
                                  FAR struct sixlowpan_saddr_s *saddr)
{
  DEBUGASSERT(ipaddr[0] == HTONS(0xfe80));

  /* Big-endian uint16_t to byte order */

  saddr->u8[0]  = ipaddr[7] >> 8;
  saddr->u8[1]  = ipaddr[7] & 0xff;
  saddr->u8[0] ^= 0x02;
}

static void sixlowpan_eaddrfromip(const net_ipv6addr_t ipaddr,
                                  FAR struct sixlowpan_eaddr_s *eaddr)
{
  FAR uint8_t *eptr = eaddr->u8;
  int i;

  DEBUGASSERT(ipaddr[0] == HTONS(0xfe80));

  for (i = 4; i < 8; i++)
    {
      /* Big-endian uint16_t to byte order */

      *eptr++ = ipaddr[i] >> 8;
      *eptr++ = ipaddr[i] & 0xff;
    }

  eaddr->u8[0] ^= 0x02;
}
#endif /* !CONFIG_NET_STARPOINT */

/****************************************************************************
 * Name: sixlowpan_coord_eaddr
 *
 * Description:
 *   Get the extended address of the PAN coordinator.
 *
 * Input parameters:
 *   ieee  - A reference IEEE802.15.4 MAC network device structure.
 *   eaddr - The location in which to return the extended address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_STARPOINT) && defined(CONFIG_NET_6LOWPAN_EXTENDEDADDR)
static int sixlowpan_coord_eaddr(FAR struct ieee802154_driver_s *ieee,
                                 FAR struct sixlowpan_eaddr_s *eaddr)
{
  FAR struct net_driver_s *dev = &ieee->i_dev;
  struct ieee802154_netmac_s arg;
  int ret;

  memcpy(arg.ifr_name, ieee->i_dev.d_ifname, IFNAMSIZ);
  arg.u.getreq.attr = IEEE802154_ATTR_MAC_COORD_EADDR ;
  ret = dev->d_ioctl(dev, MAC802154IOC_MLME_GET_REQUEST,
                     (unsigned long)((uintptr_t)&arg));
  if (ret < 0)
    {
      nerr("ERROR: MAC802154IOC_MLME_GET_REQUEST failed: %d\n", ret);
      return ret;
    }

  IEEE802154_EADDRCOPY(eaddr->u8, arg.u.getreq.attrval.mac.eaddr);
  return OK;
}
#endif

/****************************************************************************
 * Name: sixlowpan_coord_saddr
 *
 * Description:
 *   Get the short address of the PAN coordinator.
 *
 * Input parameters:
 *   ieee  - A reference IEEE802.15.4 MAC network device structure.
 *   saddr - The location in which to return the short address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_STARPOINT) && !defined(CONFIG_NET_6LOWPAN_EXTENDEDADDR)
static int sixlowpan_coord_saddr(FAR struct ieee802154_driver_s *ieee,
                                 FAR struct sixlowpan_saddr_s *saddr)
{
  FAR struct net_driver_s *dev = &ieee->i_dev;
  struct ieee802154_netmac_s arg;
  int ret;

  memcpy(arg.ifr_name, ieee->i_dev.d_ifname, IFNAMSIZ);
  arg.u.getreq.attr = IEEE802154_ATTR_MAC_COORD_SADDR ;
  ret = dev->d_ioctl(dev, MAC802154IOC_MLME_GET_REQUEST,
                     (unsigned long)((uintptr_t)&arg));
  if (ret < 0)
    {
      nerr("ERROR: MAC802154IOC_MLME_GET_REQUEST failed: %d\n", ret);
      return ret;
    }

  IEEE802154_SADDRCOPY(saddr->u8, arg.u.getreq.attrval.mac.saddr);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_destaddrfromip
 *
 * Description:
 *   sixlowpan_destaddrfromip(): Extract the IEEE 802.15.14 destination
 *   address from a MAC-based destination IPv6 address.  This function
 *   handles a tagged address union which may either a short or and
 *   extended destination address.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    xxxx 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    xxxx 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
 *
 *   In the case there the IEEE 802.15.4 node functions as an endpoint in a
 *   start topology, the destination address will, instead, be the address
 *   of the star hub (which is assumed to be the address of the cooordinator).
 *
 ****************************************************************************/

int sixlowpan_destaddrfromip(FAR struct ieee802154_driver_s *ieee,
                             const net_ipv6addr_t ipaddr,
                             FAR struct sixlowpan_tagaddr_s *destaddr)
{
#ifdef CONFIG_NET_STARPOINT
  int ret;

  /* If this node is a "point" in a star topology, then the destination
   * MAC address is the address of the hub/PAN coordinator.
   */

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
  ret = sixlowpan_coord_eaddr(ieee, &destaddr->u.eaddr);
  destaddr->extended = true;
#else
  memset(destaddr, 0, sizeof(struct sixlowpan_tagaddr_s));
  ret = sixlowpan_coord_saddr(ieee, &destaddr->u.saddr);
#endif

  return ret;

#else
  DEBUGASSERT(ipaddr[0] == HTONS(0xfe80));

  /* Otherwise, the destination MAC address is encoded in the IP address */

  if (SIXLOWPAN_IS_IID_16BIT_COMPRESSABLE(ipaddr))
    {
      memset(destaddr, 0, sizeof(struct sixlowpan_tagaddr_s));
      sixlowpan_saddrfromip(ipaddr, &destaddr->u.saddr);
    }
  else
    {
      sixlowpan_eaddrfromip(ipaddr, &destaddr->u.eaddr);
      destaddr->extended = true;
    }

  return OK;
#endif
}

/****************************************************************************
 * Name: sixlowpan_ipfrom[s|e]addr
 *
 * Description:
 *   sixlowpan_ipfrom[s|e]addr():  Create a link-local, MAC-based IPv6
 *   address from an IEEE802.15.4 short address (saddr) or extended address
 *   (eaddr).
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
 *
 ****************************************************************************/

void sixlowpan_ipfromsaddr(FAR const uint8_t *saddr,
                           FAR net_ipv6addr_t ipaddr)
{
  ipaddr[0]  = HTONS(0xfe80);
  ipaddr[1]  = 0;
  ipaddr[2]  = 0;
  ipaddr[3]  = 0;
  ipaddr[4]  = 0;
  ipaddr[5]  = HTONS(0x00ff);
  ipaddr[6]  = HTONS(0xfe00);
  ipaddr[7]  = (uint16_t)saddr[0] << 8 |  (uint16_t)saddr[1];
  ipaddr[7] ^= 0x200;
}

void sixlowpan_ipfromeaddr(FAR const uint8_t *eaddr,
                           FAR net_ipv6addr_t ipaddr)
{
  ipaddr[0]  = HTONS(0xfe80);
  ipaddr[1]  = 0;
  ipaddr[2]  = 0;
  ipaddr[3]  = 0;
  ipaddr[4]  = (uint16_t)eaddr[0] << 8 | (uint16_t)eaddr[1];
  ipaddr[5]  = (uint16_t)eaddr[2] << 8 | (uint16_t)eaddr[3];
  ipaddr[6]  = (uint16_t)eaddr[4] << 8 | (uint16_t)eaddr[5];
  ipaddr[7]  = (uint16_t)eaddr[6] << 8 | (uint16_t)eaddr[7];
  ipaddr[4] ^= 0x200;
}

/****************************************************************************
 * Name: sixlowpan_ismacbased
 *
 * Description:
 *   sixlowpan_ismacbased() will return true for IP addresses formed from
 *   IEEE802.15.4 MAC addresses.  sixlowpan_destaddrfromip() is intended to
 *   handle a tagged address or any size.  Local addresses are of a fixed
 *   but configurable size and sixlowpan_isaddrbased() is for use with such
 *   local addresses.
 *
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
 *
 ****************************************************************************/

bool sixlowpan_issaddrbased(const net_ipv6addr_t ipaddr,
                            FAR const struct sixlowpan_saddr_s *saddr)
{
  FAR const uint8_t *byteptr = saddr->u8;

  return (ipaddr[5] == HTONS(0x00ff) &&
          ipaddr[6] == HTONS(0xfe00) &&
          ipaddr[7] == (GETUINT16(byteptr, 0) ^ 0x0200));
}

bool sixlowpan_iseaddrbased(const net_ipv6addr_t ipaddr,
                            FAR const struct sixlowpan_eaddr_s *eaddr)
{
  FAR const uint8_t *byteptr = eaddr->u8;

  return (ipaddr[4] == (GETUINT16(byteptr, 0) ^ 0x0200) &&
          ipaddr[5] == GETUINT16(byteptr, 2) &&
          ipaddr[6] == GETUINT16(byteptr, 4) &&
          ipaddr[7] == GETUINT16(byteptr, 6));
}

bool sixlowpan_ismacbased(const net_ipv6addr_t ipaddr,
                          FAR const struct sixlowpan_tagaddr_s *addr)
{
  if (addr->extended)
    {
      return sixlowpan_iseaddrbased(ipaddr, &addr->u.eaddr);
    }
   else
    {
      return sixlowpan_issaddrbased(ipaddr, &addr->u.saddr);
    }
}

/****************************************************************************
 * Name: sixlowpan_src_panid
 *
 * Description:
 *   Get the source PAN ID from the IEEE802.15.4 MAC layer.
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
                        FAR uint8_t *panid)
{
  FAR struct net_driver_s *dev = &ieee->i_dev;
  struct ieee802154_netmac_s arg;
  int ret;

  memcpy(arg.ifr_name, ieee->i_dev.d_ifname, IFNAMSIZ);
  arg.u.getreq.attr = IEEE802154_ATTR_MAC_PANID;
  ret = dev->d_ioctl(dev, MAC802154IOC_MLME_GET_REQUEST,
                     (unsigned long)((uintptr_t)&arg));
  if (ret < 0)
    {
      nerr("ERROR: MAC802154IOC_MLME_GET_REQUEST failed: %d\n", ret);
      return ret;
    }

  IEEE802154_PANIDCOPY(panid, arg.u.getreq.attrval.mac.panid);
  return OK;
}

#endif /* CONFIG_NET_6LOWPAN */
