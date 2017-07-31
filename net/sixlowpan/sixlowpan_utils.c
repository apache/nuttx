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
#include <nuttx/wireless/pktradio.h>
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
 *    fe80 0000 0000 0000  0000 00ff fe00 xx00 1-byte short address IEEE 48-bit MAC
 *    xxxx 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    xxxx 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
 *
 ****************************************************************************/

#ifndef CONFIG_NET_STARPOINT
static void sixlowpan_saddrfromip(const net_ipv6addr_t ipaddr, FAR uint8_t *saddr)
{
  DEBUGASSERT(ipaddr[0] == HTONS(0xfe80));

  /* Big-endian uint16_t to byte order */

  saddr[0]  = ipaddr[7] >> 8;
  saddr[1]  = ipaddr[7] & 0xff;
  saddr[0] ^= 0x02;
}

static void sixlowpan_eaddrfromip(const net_ipv6addr_t ipaddr, FAR uint8_t *eaddr)
{
  FAR uint8_t *eptr = eaddr;
  int i;

  DEBUGASSERT(ipaddr[0] == HTONS(0xfe80));

  for (i = 4; i < 8; i++)
    {
      /* Big-endian uint16_t to byte order */

      *eptr++ = ipaddr[i] >> 8;
      *eptr++ = ipaddr[i] & 0xff;
    }

  eaddr[0] ^= 0x02;
}
#endif /* !CONFIG_NET_STARPOINT */

/****************************************************************************
 * Name: sixlowpan_coord_eaddr
 *
 * Description:
 *   Get the extended address of the PAN coordinator.
 *
 * Input parameters:
 *   radio - Reference to a radio network driver state instance.
 *   eaddr - The location in which to return the extended address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_STARPOINT) && defined(CONFIG_NET_6LOWPAN_EXTENDEDADDR)
static int sixlowpan_coord_eaddr(FAR struct sixlowpan_driver_s *radio,
                                 FAR struct netdev_varaddr_s *eaddr)
{
  FAR struct net_driver_s *dev = &radio->r_dev;
  struct ieee802154_netmac_s arg;
  int ret;

  memcpy(arg.ifr_name, radio->r_dev.d_ifname, IFNAMSIZ);
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
 *   radio - Reference to a radio network driver state instance.
 *   saddr - The location in which to return the short address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_STARPOINT) && !defined(CONFIG_NET_6LOWPAN_EXTENDEDADDR)
static int sixlowpan_coord_saddr(FAR struct sixlowpan_driver_s *radio,
                                 FAR struct netdev_varaddr_s *saddr)
{
  FAR struct net_driver_s *dev = &radio->r_dev;
  struct ieee802154_netmac_s arg;
  int ret;

  memcpy(arg.ifr_name, radio->r_dev.d_ifname, IFNAMSIZ);
  arg.u.getreq.attr = IEEE802154_ATTR_MAC_COORD_SADDR ;
  ret = dev->d_ioctl(dev, MAC802154IOC_MLME_GET_REQUEST,
                     (unsigned long)((uintptr_t)&arg));
  if (ret < 0)
    {
      nerr("ERROR: MAC802154IOC_MLME_GET_REQUEST failed: %d\n", ret);
      return ret;
    }

  IEEE802154_SADDRCOPY(saddr->nv_addr, arg.u.getreq.attrval.mac.saddr);
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
 *    fe80 0000 0000 0000  0000 00ff fe00 xx00 1-byte short address IEEE 48-bit MAC
 *    xxxx 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    xxxx 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
 *
 *   In the case there the IEEE 802.15.4 node functions as an endpoint in a
 *   start topology, the destination address will, instead, be the address
 *   of the star hub (which is assumed to be the address of the cooordinator).
 *
 ****************************************************************************/

int sixlowpan_destaddrfromip(FAR struct sixlowpan_driver_s *radio,
                             const net_ipv6addr_t ipaddr,
                             FAR struct netdev_varaddr_s *destaddr)
{
#ifdef CONFIG_NET_STARPOINT
  int ret;

  /* If this node is a "point" in a star topology, then the destination
   * MAC address is the address of the hub/PAN coordinator.
   */

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
  ret = sixlowpan_coord_eaddr(radio, &destaddr->nv_addr);
  destaddr->nv_addrlen = NET_6LOWPAN_EADDRSIZE;
#else
  memset(destaddr, 0, sizeof(struct netdev_varaddr_s));
  ret = sixlowpan_coord_saddr(radio, &destaddr->nv_addr);
  destaddr->nv_addrlen = NET_6LOWPAN_SADDRSIZE;
#endif

  return ret;

#else
  DEBUGASSERT(ipaddr[0] == HTONS(0xfe80));

  /* Otherwise, the destination MAC address is encoded in the IP address */

  if (SIXLOWPAN_IS_IID_16BIT_COMPRESSABLE(ipaddr))
    {
      memset(destaddr, 0, sizeof(struct netdev_varaddr_s));
      sixlowpan_saddrfromip(ipaddr, destaddr->nv_addr);
      destaddr->nv_addrlen = NET_6LOWPAN_SADDRSIZE;
    }
  else
    {
      sixlowpan_eaddrfromip(ipaddr, destaddr->nv_addr);
      destaddr->nv_addrlen = NET_6LOWPAN_EADDRSIZE;
    }

  return OK;
#endif
}

/****************************************************************************
 * Name: sixlowpan_ipfromaddr (plus helpers)
 *
 * Description:
 *   sixlowpan_ipfrom[s|e]addr():  Create a link-local, MAC-based IPv6
 *   address from an IEEE802.15.4 short address (saddr), extended address
 *   (eaddr), or other variable length radio addresses.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xx00 1-byte short address IEEE 48-bit MAC
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
 *
 ****************************************************************************/

#ifdef CONFIG_WIRELESS_PKTRADIO
static inline void sixlowpan_ipfrombyte(FAR const uint8_t *byte,
                                        FAR net_ipv6addr_t ipaddr)
{
  ipaddr[0]  = HTONS(0xfe80);
  ipaddr[1]  = 0;
  ipaddr[2]  = 0;
  ipaddr[3]  = 0;
  ipaddr[4]  = 0;
  ipaddr[5]  = HTONS(0x00ff);
  ipaddr[6]  = HTONS(0xfe00);
  ipaddr[7]  = (uint16_t)byte[0] << 8 ^ 0x0200;
}
#endif

static inline void sixlowpan_ipfromsaddr(FAR const uint8_t *saddr,
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
  ipaddr[7] ^= 0x0200;
}

static inline void sixlowpan_ipfromeaddr(FAR const uint8_t *eaddr,
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
  ipaddr[4] ^= 0x0200;
}

void sixlowpan_ipfromaddr(FAR const struct netdev_varaddr_s *addr,
                          FAR net_ipv6addr_t ipaddr)
{
  switch (addr->nv_addrlen)
    {
#ifdef CONFIG_WIRELESS_PKTRADIO
      case 1:
        sixlowpan_ipfrombyte(addr->nv_addr, ipaddr);
        break;
#endif

      case NET_6LOWPAN_SADDRSIZE:
        sixlowpan_ipfromsaddr(addr->nv_addr, ipaddr);
        break;

      case NET_6LOWPAN_EADDRSIZE:
        sixlowpan_ipfromeaddr(addr->nv_addr, ipaddr);
        break;

      default:
        nerr("ERROR: Unsupported address length: %u\n", addr->nv_addrlen);
        break;
    }
}

/****************************************************************************
 * Name: sixlowpan_ismacbased (and helpers)
 *
 * Description:
 *   sixlowpan_ismacbased() will return true for IP addresses formed from
 *   IEEE802.15.4 MAC addresses.  sixlowpan_destaddrfromip() is intended to
 *   handle a tagged address or any size.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xx00 1-byte short address IEEE 48-bit MAC
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
 *
 ****************************************************************************/

#ifdef CONFIG_WIRELESS_PKTRADIO
static inline bool sixlowpan_isbytebased(const net_ipv6addr_t ipaddr,
                                         uint8_t byte)
{
  return (ipaddr[5] == HTONS(0x00ff) &&
          ipaddr[6] == HTONS(0xfe00) &&
          ipaddr[7] == (((uint16_t)byte << 8) ^ 0x0200));
}
#endif

static inline bool sixlowpan_issaddrbased(const net_ipv6addr_t ipaddr,
                                          FAR const uint8_t *saddr)
{
  return (ipaddr[5] == HTONS(0x00ff) &&
          ipaddr[6] == HTONS(0xfe00) &&
          ipaddr[7] == (GETUINT16(saddr, 0) ^ 0x0200));
}

static inline bool sixlowpan_iseaddrbased(const net_ipv6addr_t ipaddr,
                                          FAR const uint8_t *eaddr)
{
  return (ipaddr[4] == (GETUINT16(eaddr, 0) ^ 0x0200) &&
          ipaddr[5] ==  GETUINT16(eaddr, 2) &&
          ipaddr[6] ==  GETUINT16(eaddr, 4) &&
          ipaddr[7] ==  GETUINT16(eaddr, 6));
}

bool sixlowpan_ismacbased(const net_ipv6addr_t ipaddr,
                          FAR const struct netdev_varaddr_s *addr)
{
  switch (addr->nv_addrlen)
    {
#ifdef CONFIG_WIRELESS_PKTRADIO
      case 1:
        return sixlowpan_isbytebased(ipaddr, addr->nv_addr[0]);
#endif

      case NET_6LOWPAN_SADDRSIZE:
        return sixlowpan_issaddrbased(ipaddr, addr->nv_addr);

      case NET_6LOWPAN_EADDRSIZE:
        return sixlowpan_iseaddrbased(ipaddr, addr->nv_addr);

      default:
        nerr("ERROR: Unsupported address length: %u\n", addr->nv_addrlen);
        return false;
    }
}

/****************************************************************************
 * Name: sixlowpan_src_panid
 *
 * Description:
 *   Get the source PAN ID from the IEEE802.15.4 MAC layer.
 *
 * Input parameters:
 *   radio - Reference to a radio network driver state instance.
 *   panid - The location in which to return the PAN ID.  0xfff may be
 *           returned if the device is not associated.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_WIRELESS_IEEE802154
int sixlowpan_src_panid(FAR struct sixlowpan_driver_s *radio,
                        FAR uint8_t *panid)
{
  FAR struct net_driver_s *dev = &radio->r_dev;
  struct ieee802154_netmac_s arg;
  int ret;

  memcpy(arg.ifr_name, radio->r_dev.d_ifname, IFNAMSIZ);
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
#endif

/****************************************************************************
 * Name: sixlowpan_extract_srcaddr
 *
 * Description:
 *   Extract the source MAC address from the radio-specific RX metadata, and
 *   return the source address in a radio-agnostic form.
 *
 * Input parameters:
 *   radio    - Reference to a radio network driver state instance.
 *   metadata - Opaque reference to the radio-specific RX metadata.
 *   srcaddr  - The location in which to return the source MAC address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sixlowpan_extract_srcaddr(FAR struct sixlowpan_driver_s *radio,
                              FAR const void *metadata,
                              FAR struct netdev_varaddr_s *srcaddr)
{
  DEBUGASSERT(radio != NULL && metadata != NULL && srcaddr != NULL);

#ifdef CONFIG_WIRELESS_IEEE802154
#ifdef CONFIG_WIRELESS_PKTRADIO
  if (radio->r_dev.d_lltype == NET_LL_IEEE802154)
#endif
    {
      FAR const struct ieee802154_data_ind_s *ind =
        (FAR const struct ieee802154_data_ind_s *)metadata;

      if (ind->src.mode == IEEE802154_ADDRMODE_SHORT)
        {
          srcaddr->nv_addrlen = NET_6LOWPAN_SADDRSIZE;
          memcpy(srcaddr->nv_addr, ind->src.saddr, NET_6LOWPAN_SADDRSIZE);
        }
      else
        {
          srcaddr->nv_addrlen = NET_6LOWPAN_EADDRSIZE;
          memcpy(srcaddr->nv_addr, ind->src.eaddr, NET_6LOWPAN_EADDRSIZE);
        }

      return OK;
    }
#endif

#ifdef CONFIG_WIRELESS_PKTRADIO
#ifdef CONFIG_WIRELESS_IEEE802154
  else
#endif
    {
      FAR const struct pktradio_metadata_s *pktmeta =
        (FAR const struct pktradio_metadata_s *)metadata;

      DEBUGASSERT(pktmeta->pm_src.pa_addrlen <= CONFIG_PKTRADIO_ADDRLEN);

      srcaddr->nv_addrlen = pktmeta->pm_src.pa_addrlen;
      memcpy(srcaddr->nv_addr,  pktmeta->pm_src.pa_addr,
             pktmeta->pm_src.pa_addrlen);

      return OK;
    }
#endif

  return -EINVAL; /* Shouldn't get here */
}

/****************************************************************************
 * Name: sixlowpan_extract_destaddr
 *
 * Description:
 *   Extract the destination MAC address from the radio-specific RX metadata,
 *   and return the destination address in a radio-agnostic form.
 *
 * Input parameters:
 *   radio    - Reference to a radio network driver state instance.
 *   metadata - Opaque reference to the radio-specific RX metadata.
 *   destaddr - The location in which to return the destination MAC address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sixlowpan_extract_destaddr(FAR struct sixlowpan_driver_s *radio,
                               FAR const void *metadata,
                               FAR struct netdev_varaddr_s *destaddr)
{
  DEBUGASSERT(radio != NULL && metadata != NULL && destaddr != NULL);

#ifdef CONFIG_WIRELESS_IEEE802154
#ifdef CONFIG_WIRELESS_PKTRADIO
  if (radio->r_dev.d_lltype == NET_LL_IEEE802154)
#endif
    {
      FAR const struct ieee802154_data_ind_s *ind =
        (FAR const struct ieee802154_data_ind_s *)metadata;

      if (ind->dest.mode == IEEE802154_ADDRMODE_SHORT)
        {
          destaddr->nv_addrlen = NET_6LOWPAN_SADDRSIZE;
          memcpy(destaddr->nv_addr, ind->dest.saddr, NET_6LOWPAN_SADDRSIZE);
        }
      else
        {
          destaddr->nv_addrlen = NET_6LOWPAN_EADDRSIZE;
          memcpy(destaddr->nv_addr, ind->dest.eaddr, NET_6LOWPAN_EADDRSIZE);
        }

      return OK;
    }
#endif

#ifdef CONFIG_WIRELESS_PKTRADIO
#ifdef CONFIG_WIRELESS_IEEE802154
  else
#endif
    {
      FAR const struct pktradio_metadata_s *pktmeta =
        (FAR const struct pktradio_metadata_s *)metadata;

      DEBUGASSERT(pktmeta->pm_dest.pa_addrlen <= CONFIG_PKTRADIO_ADDRLEN);

      destaddr->nv_addrlen = pktmeta->pm_dest.pa_addrlen;
      memcpy(destaddr->nv_addr, pktmeta->pm_dest.pa_addr,
             pktmeta->pm_dest.pa_addrlen);

      return OK;
    }
#endif

  return -EINVAL; /* Shouldn't get here */
}

#endif /* CONFIG_NET_6LOWPAN */
