/****************************************************************************
 * net/sixlowpan/sixlowpan_utils.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <nuttx/net/ip.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/sixlowpan.h>
#include <nuttx/wireless/pktradio.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "route/route.h"
#include "inet/inet.h"
#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* REVISIT: The setting CONFIG_PKTRADIO_ADDRLEN should be the *maximum*
 * address length.  If there is only a single packet radio then it should be
 * the exact address length of that radio.  If there are multiple packet
 * radios with different address lengths, then it will be inexact; it will
 * be the size of the longest address.
 */

#undef HAVE_BYTEADDR
#undef HAVE_SADDR
#undef HAVE_EADDR

#ifdef CONFIG_WIRELESS_IEEE802154
#  define HAVE_SADDR 1
#  define HAVE_EADDR 1
#endif

#ifdef CONFIG_WIRELESS_PKTRADIO
#  if CONFIG_PKTRADIO_ADDRLEN == 1
#    define HAVE_BYTEADDR 1
#  elif CONFIG_PKTRADIO_ADDRLEN == 2
#    define HAVE_BYTEADDR 1
#    define HAVE_SADDR 1
#  elif CONFIG_PKTRADIO_ADDRLEN == 8
#    define HAVE_BYTEADDR 1
#    define HAVE_SADDR 1
#    define HAVE_EADDR 1
#  else
#    error Unsupported value for CONFIG_PKTRADIO_ADDRLEN
#  endif
#endif

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
#ifdef HAVE_BYTEADDR
static void sixlowpan_baddrfromip(const net_ipv6addr_t ipaddr, FAR uint8_t *baddr)
{
  /* Big-endian uint16_t to byte order */

  baddr[0] = ipaddr[7] >> 8 ^ 0x02;
}
#endif

#ifdef HAVE_SADDR
static void sixlowpan_saddrfromip(const net_ipv6addr_t ipaddr, FAR uint8_t *saddr)
{
  /* Big-endian uint16_t to byte order */

  saddr[0]  = ipaddr[7] >> 8;
  saddr[1]  = ipaddr[7] & 0xff;
  saddr[0] ^= 0x02;
}
#endif

#ifdef HAVE_EADDR
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
#endif
#endif /* !CONFIG_NET_STARPOINT */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_nexthopaddr
 *
 * Description:
 *   sixlowpan_nexthopaddr(): If the destination is on-link, extract the
 *   IEEE 802.15.14 destination address from the destination IP address. If the
 *   destination is not reachable directly, use the routing table (if available)
 *   or fall back to the default router IP address and use the router IP address
 *   to derive the IEEE 802.15.4 MAC address.
 *
 ****************************************************************************/

int sixlowpan_nexthopaddr(FAR struct radio_driver_s *radio,
                          FAR const net_ipv6addr_t ipaddr,
                          FAR struct netdev_varaddr_s *destaddr)
{
  FAR net_ipv6addr_t router;
  int ret;

  /* Try to get the IEEE 802.15.4 MAC address of the destination.  This
   * assumes an encoding of the MAC address in the IPv6 address.
   */

  ret = sixlowpan_destaddrfromip(radio, ipaddr, destaddr);
  if (ret < 0)
    {
      /* Destination address is not on the local network */

#ifdef CONFIG_NET_ROUTE
      /* We have a routing table.. find the correct router to use in
       * this case (or, as a fall-back, use the device's default router
       * address).  We will use the router IPv6 address instead of the
       * destination address when determining the MAC address.
       */

      netdev_ipv6_router(&radio->r_dev, ipaddr, router);
#else
      /* Use the device's default router IPv6 address instead of the
       * destination address when determining the MAC address.
       */

      net_ipv6addr_copy(router, radio->r_dev.d_ipv6draddr);
#endif
      /* Get the IEEE 802.15.4 MAC address of the router.  This
       * assumes an encoding of the MAC address in the IPv6 address.
       */

      ret = sixlowpan_destaddrfromip(radio, router, destaddr);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

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
 *    ffxx xxxx xxxx xxxx  xxxx xxxx xxxx xxxx Multicast address (RFC 3513)
 *    ff02 0000 0000 0000  0000 0000 0000 0001 All nodes multicast group
 *    xxxx 0000 0000 0000  0000 00ff fe00 xx00 1-byte short address IEEE 48-bit MAC
 *    xxxx 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    xxxx 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
 *
 *   In the case there the IEEE 802.15.4 node functions as an endpoint in a
 *   start topology, the destination address will, instead, be the address
 *   of the star hub (which is assumed to be the address of the cooordinator).
 *
 ****************************************************************************/

int sixlowpan_destaddrfromip(FAR struct radio_driver_s *radio,
                             const net_ipv6addr_t ipaddr,
                             FAR struct netdev_varaddr_s *destaddr)
{
  struct radiodev_properties_s properties;
  int ret;

#ifdef  CONFIG_NET_STARPOINT
  /* Only the radio driver knows the correct address of the hub.  For IEEE
   * 802.15.4 this will be the address of the PAN coordinator.  For other
   * radios, this may be some configured, "well-known" address.
   */

  DEBUGASSERT(radio->r_properties != NULL);
  ret = radio->r_properties(radio, &properties);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(destaddr, &properties.sp_hubnode, sizeof(struct netdev_varaddr_s));
  return OK;

#else /* CONFIG_NET_STARPOINT */

   /* Check for a multicast address */

   if (net_is_addr_mcast(ipaddr))
     {
        DEBUGASSERT(radio->r_properties != NULL);
        ret = radio->r_properties(radio, &properties);
        if (ret < 0)
          {
            return ret;
          }

        /* Check for the broadcast IP address
         *
         * IPv6 does not implement the method of broadcast, and therefore
         * does not define broadcast addresses. Instead, IPv6 uses multicast
         * addressing to the all-nodes multicast group: ff02:0:0:0:0:0:0:1.
         *
         * However, the use of the all-nodes group is not common, and most
         * IPv6 protocols use a dedicated link-local multicast group to avoid
         * disturbing every interface in the network.
         */

        if (net_ipv6addr_cmp(ipaddr, g_ipv6_allnodes))
          {
            memcpy(destaddr, &properties.sp_bcast,
                   sizeof(struct netdev_varaddr_s));
          }

        /* Some other RFC 3513 multicast address */

        else
          {
            memcpy(destaddr, &properties.sp_mcast,
                   sizeof(struct netdev_varaddr_s));
          }

          return OK;
     }

  /* Otherwise, the destination MAC address is encoded in the IP address */

  /* If the address is link-local, or matches the prefix of the local address,
   * the interface identifier can be extracted from the lower bits of the address.
   */

  if (!sixlowpan_islinklocal(ipaddr) &&
      !net_ipv6addr_maskcmp(radio->r_dev.d_ipv6addr, ipaddr,
                            radio->r_dev.d_ipv6netmask))
    {
      return -EADDRNOTAVAIL;
    }

#ifdef CONFIG_WIRELESS_PKTRADIO
  /* If this is a packet radio, then we cannot know the correct size of the
   * radio's MAC address without asking.  The setting CONFIG_PKTRADIO_ADDRLEN
   * is inexact if there are multiple packet radios with different address
   * lengths; it that case it will be the size of the longest address.
   *
   * NOTE: This logic assumes that the packet radio's address length is a
   * constant.
   */

#ifdef CONFIG_WIRELESS_IEEE802154
  if (radio->r_dev.d_lltype == NET_LL_PKTRADIO)
#endif
    {
      DEBUGASSERT(radio->r_properties != NULL);
      ret = radio->r_properties(radio, &properties);
      if (ret < 0)
        {
          return ret;
        }

#ifdef HAVE_BYTEADDR
      if (properties.sp_addrlen == 1 &&
          SIXLOWPAN_IS_IID_8BIT_COMPRESSABLE(ipaddr))
        {
          memset(destaddr, 0, sizeof(struct netdev_varaddr_s));
          sixlowpan_baddrfromip(ipaddr, destaddr->nv_addr);
          destaddr->nv_addrlen = 1;
          return OK;
        }
      else
#endif
#ifdef HAVE_SADDR
      if (properties.sp_addrlen == 2 &&
          SIXLOWPAN_IS_IID_16BIT_COMPRESSABLE(ipaddr))
        {
          memset(destaddr, 0, sizeof(struct netdev_varaddr_s));
          sixlowpan_saddrfromip(ipaddr, destaddr->nv_addr);
          destaddr->nv_addrlen = 2;
          return OK;
        }
      else
#endif
#ifdef HAVE_EADDR
      if (properties.sp_addrlen == 8)
        {
          sixlowpan_eaddrfromip(ipaddr, destaddr->nv_addr);
          destaddr->nv_addrlen = 8;
          return OK;
        }
      else
#endif
        {
          /* Just to satisfy the last dangling 'else' */
        }

      return -EADDRNOTAVAIL;
    }

#endif /* CONFIG_WIRELESS_PKTRADIO */

#ifdef CONFIG_WIRELESS_IEEE802154
#ifdef CONFIG_WIRELESS_PKTRADIO
  else
#endif
    {
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
    }

#endif /* CONFIG_WIRELESS_IEEE802154 */
#endif /* CONFIG_NET_STARPOINT */
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

#ifdef HAVE_BYTEADDR
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

#ifdef HAVE_SADDR
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
#endif

#ifdef HAVE_EADDR
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
#endif

void sixlowpan_ipfromaddr(FAR const struct netdev_varaddr_s *addr,
                          FAR net_ipv6addr_t ipaddr)
{
  switch (addr->nv_addrlen)
    {
#ifdef HAVE_BYTEADDR
      case 1:
        sixlowpan_ipfrombyte(addr->nv_addr, ipaddr);
        break;
#endif

#ifdef HAVE_SADDR
      case NET_6LOWPAN_SADDRSIZE:
        sixlowpan_ipfromsaddr(addr->nv_addr, ipaddr);
        break;
#endif

#ifdef HAVE_EADDR
      case NET_6LOWPAN_EADDRSIZE:
        sixlowpan_ipfromeaddr(addr->nv_addr, ipaddr);
        break;
#endif

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

#ifdef HAVE_BYTEADDR
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
#ifdef HAVE_BYTEADDR
      case 1:
        return sixlowpan_isbytebased(ipaddr, addr->nv_addr[0]);
#endif

#ifdef HAVE_SADDR
      case NET_6LOWPAN_SADDRSIZE:
        return sixlowpan_issaddrbased(ipaddr, addr->nv_addr);
#endif

#ifdef HAVE_EADDR
      case NET_6LOWPAN_EADDRSIZE:
        return sixlowpan_iseaddrbased(ipaddr, addr->nv_addr);
#endif

      default:
        nerr("ERROR: Unsupported address length: %u\n", addr->nv_addrlen);
        return false;
    }
}

/****************************************************************************
 * Name: sixlowpan_radio_framelen
 *
 * Description:
 *   Get the maximum frame length supported by radio network drvier.
 *
 * Input Parameters:
 *   radio - Reference to a radio network driver state instance.
 *
 * Returned Value:
 *   A non-negative, maximum frame lengthis returned on success;  A negated
 *   errno valueis returned on any failure.
 *
 ****************************************************************************/

int sixlowpan_radio_framelen(FAR struct radio_driver_s *radio)
{
  struct radiodev_properties_s properties;
  int ret;

  /* Only the radio driver knows the correct max frame length supported by
   * the radio.
   */

  DEBUGASSERT(radio->r_properties != NULL);
  ret = radio->r_properties(radio, &properties);
  if (ret < 0)
    {
      return ret;
    }

  return (int)properties.sp_framelen;
}

/****************************************************************************
 * Name: sixlowpan_src_panid
 *
 * Description:
 *   Get the source PAN ID from the IEEE802.15.4 MAC layer.
 *
 * Input Parameters:
 *   radio - Reference to a radio network driver state instance.
 *   panid - The location in which to return the PAN ID.  0xfff may be
 *           returned if the device is not associated.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_WIRELESS_IEEE802154
int sixlowpan_src_panid(FAR struct radio_driver_s *radio,
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
 * Input Parameters:
 *   radio    - Reference to a radio network driver state instance.
 *   metadata - Opaque reference to the radio-specific RX metadata.
 *   srcaddr  - The location in which to return the source MAC address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sixlowpan_extract_srcaddr(FAR struct radio_driver_s *radio,
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
 * Input Parameters:
 *   radio    - Reference to a radio network driver state instance.
 *   metadata - Opaque reference to the radio-specific RX metadata.
 *   destaddr - The location in which to return the destination MAC address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sixlowpan_extract_destaddr(FAR struct radio_driver_s *radio,
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
