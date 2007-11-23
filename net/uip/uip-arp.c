/****************************************************************************
 * net/uip/uip-arp.c
 * Implementation of the ARP Address Resolution Protocol.
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Based on uIP which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* The Address Resolution Protocol ARP is used for mapping between IP
 * addresses and link level addresses such as the Ethernet MAC
 * addresses. ARP uses broadcast queries to ask for the link level
 * address of a known IP address and the host which is configured with
 * the IP address for which the query was meant, will respond with its
 * link level address.
 *
 * Note: This ARP implementation only supports Ethernet.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <sys/types.h>
#include <sys/ioctl.h>

#include <string.h>
#include <debug.h>

#include <netinet/in.h>
#include <net/uip/uip-arch.h>
#include <net/uip/uip-arp.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define ARP_REQUEST 1
#define ARP_REPLY   2

#define ARP_HWTYPE_ETH 1

#define ETHBUF ((struct uip_eth_hdr *)&dev->d_buf[0])
#define ARPBUF ((struct arp_hdr *)&dev->d_buf[UIP_LLH_LEN])
#define IPBUF  ((struct ethip_hdr *)&dev->d_buf[UIP_LLH_LEN])

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ARP header -- Size 28 bytes */

struct arp_hdr
{
  uint16 ah_hwtype;        /* 16-bit Hardware type (Ethernet=0x001) */
  uint16 ah_protocol;      /* 16-bit Protocoal type (IP=0x0800 */
  uint8  ah_hwlen;         /*  8-bit Hardware address size (6) */
  uint8  ah_protolen;      /*  8-bit Procotol address size (4) */
  uint16 ah_opcode;        /* 16-bit Operation */
  uint8  ah_shwaddr[6];    /* 48-bit Sender hardware address */   
  uint16 ah_sipaddr[2];    /* 32-bit Sender IP adress */
  uint8  ah_dhwaddr[6];    /* 48-bit Target hardware address */
  uint16 ah_dipaddr[2];    /* 32-bit Target IP address */
};

/* IP header -- Size 20 or 24 bytes */

struct ethip_hdr
{
  uint8  eh_vhl;           /*  8-bit Version (4) and header length (5 or 6) */
  uint8  eh_tos;           /*  8-bit Type of service (e.g., 6=TCP) */
  uint8  eh_len[2];        /* 16-bit Total length */
  uint8  eh_ipid[2];       /* 16-bit Identification */
  uint8  eh_ipoffset[2];   /* 16-bit IP flags + fragment offset */
  uint8  eh_ttl;           /*  8-bit Time to Live */
  uint8  eh_proto;         /*  8-bit Protocol */
  uint16 eh_ipchksum;      /* 16-bit Header checksum */
  uint16 eh_srcipaddr[2];  /* 32-bit Source IP address */
  uint16 eh_destipaddr[2]; /* 32-bit Destination IP address */
  uint16 eh_ipoption[2];   /* (optional) */
};

struct arp_entry
{
  in_addr_t           at_ipaddr;
  struct uip_eth_addr at_ethaddr;
  uint8               at_time;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uip_eth_addr broadcast_ethaddr =
  {{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}};
static const uint16 broadcast_ipaddr[2] = {0xffff, 0xffff};

static struct arp_entry arp_table[CONFIG_NET_ARPTAB_SIZE];
static uint8 g_arptime;


/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_NET_DUMPARP) && defined(CONFIG_DEBUG)
static void uip_arp_dump(struct arp_hdr *arp)
{
  ndbg("  HW type: %04x Protocol: %04x\n",
       arp->ah_hwtype, arp->ah_protocol);\
  ndbg("  HW len: %02x Proto len: %02x Operation: %04x\n",
        arp->ah_hwlen, arp->ah_protolen, arp->ah_opcode);
  ndbg("  Sender MAC: %02x:%02x:%02x:%02x:%02x:%02x IP: %d.%d.%d.%d\n",
       arp->ah_shwaddr[0], arp->ah_shwaddr[1], arp->ah_shwaddr[2],
       arp->ah_shwaddr[3], arp->ah_shwaddr[4], arp->ah_shwaddr[5],
       arp->ah_sipaddr[0] & 0xff, arp->ah_sipaddr[0] >> 8,
       arp->ah_sipaddr[1] & 0xff, arp->ah_sipaddr[1] >> 8);
  ndbg("  Dest MAC:   %02x:%02x:%02x:%02x:%02x:%02x IP: %d.%d.%d.%d\n",
       arp->ah_dhwaddr[0], arp->ah_dhwaddr[1], arp->ah_dhwaddr[2],
       arp->ah_dhwaddr[3], arp->ah_dhwaddr[4], arp->ah_dhwaddr[5],
       arp->ah_dipaddr[0] & 0xff, arp->ah_dipaddr[0] >> 8,
       arp->ah_dipaddr[1] & 0xff, arp->ah_dipaddr[1] >> 8);
}
#else
# define uip_arp_dump(arp)
#endif

static void uip_arp_update(uint16 *pipaddr, uint8 *ethaddr)
{
  struct arp_entry *tabptr;
  in_addr_t         ipaddr = uip_ip4addr_conv(pipaddr);
  int               i;

  /* Walk through the ARP mapping table and try to find an entry to
   * update. If none is found, the IP -> MAC address mapping is
   * inserted in the ARP table.
   */

  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      tabptr = &arp_table[i];

      /* Only check those entries that are actually in use. */

      if (tabptr->at_ipaddr != 0)
        {
          /* Check if the source IP address of the incoming packet matches
           * the IP address in this ARP table entry.
           */

          if (uip_ipaddr_cmp(ipaddr, tabptr->at_ipaddr))
            {
              /* An old entry found, update this and return. */
              memcpy(tabptr->at_ethaddr.addr, ethaddr, IFHWADDRLEN);
              tabptr->at_time = g_arptime;

              return;
            }
        }
    }

  /* If we get here, no existing ARP table entry was found, so we
     create one. */

  /* First, we try to find an unused entry in the ARP table. */

  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      tabptr = &arp_table[i];
      if (tabptr->at_ipaddr == 0)
        {
          break;
        }
    }

  /* If no unused entry is found, we try to find the oldest entry and
   * throw it away.
   */

  if (i == CONFIG_NET_ARPTAB_SIZE)
    {
      uint8 tmpage = 0;
      int   j      = 0;
      for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
        {
          tabptr = &arp_table[i];
          if (g_arptime - tabptr->at_time > tmpage)
            {
              tmpage = g_arptime - tabptr->at_time;
              j = i;
            }
        }
      i = j;
      tabptr = &arp_table[i];
    }

  /* Now, i is the ARP table entry which we will fill with the new
   * information.
   */

  tabptr->at_ipaddr = ipaddr;
  memcpy(tabptr->at_ethaddr.addr, ethaddr, IFHWADDRLEN);
  tabptr->at_time = g_arptime;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Initialize the ARP module. */

void uip_arp_init(void)
{
  int i;
  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      memset(&arp_table[i].at_ipaddr, 0, sizeof(in_addr_t));
    }
}

/* Periodic ARP processing function.
 *
 * This function performs periodic timer processing in the ARP module
 * and should be called at regular intervals. The recommended interval
 * is 10 seconds between the calls.
 */

void uip_arp_timer(void)
{
  struct arp_entry *tabptr;
  int i;

  ++g_arptime;
  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      tabptr = &arp_table[i];
      if (tabptr->at_ipaddr != 0 && g_arptime - tabptr->at_time >= UIP_ARP_MAXAGE)
        {
          tabptr->at_ipaddr = 0;
        }
    }
}

/* ARP processing for incoming IP packets
 *
 * This function should be called by the device driver when an IP
 * packet has been received. The function will check if the address is
 * in the ARP cache, and if so the ARP cache entry will be
 * refreshed. If no ARP cache entry was found, a new one is created.
 *
 * This function expects an IP packet with a prepended Ethernet header
 * in the d_buf[] buffer, and the length of the packet in the field
 * d_len.
 */

#if 0
void uip_arp_ipin(void)
{
  /* Only insert/update an entry if the source IP address of the
   * incoming IP packet comes from a host on the local network.
   */

  if ((IPBUF->eh_srcipaddr & dev->d_netmask) != (dev->d_ipaddr & dev->d_netmask))
    {
      return;
    }

  uip_arp_update(IPBUF->eh_srcipaddr, ETHBUF->eh_ethhdr.src);
}
#endif /* 0 */

/* ARP processing for incoming ARP packets.
 *
 * This function should be called by the device driver when an ARP
 * packet has been received. The function will act differently
 * depending on the ARP packet type: if it is a reply for a request
 * that we previously sent out, the ARP cache will be filled in with
 * the values from the ARP reply. If the incoming ARP packet is an ARP
 * request for our IP address, an ARP reply packet is created and put
 * into the d_buf[] buffer.
 *
 * When the function returns, the value of the field d_len
 * indicates whether the device driver should send out a packet or
 * not. If d_len is zero, no packet should be sent. If d_len is
 * non-zero, it contains the length of the outbound packet that is
 * present in the d_buf[] buffer.
 *
 * This function expects an ARP packet with a prepended Ethernet
 * header in the d_buf[] buffer, and the length of the packet in the
 * global variable d_len.
 */

void uip_arp_arpin(struct uip_driver_s *dev)
{
  in_addr_t ipaddr;

  if (dev->d_len < (sizeof(struct arp_hdr) + UIP_LLH_LEN))
    {
      dev->d_len = 0;
      return;
    }
  dev->d_len = 0;

  ipaddr = uip_ip4addr_conv(ARPBUF->ah_dipaddr);
  switch(ARPBUF->ah_opcode)
    {
      case HTONS(ARP_REQUEST):
        /* ARP request. If it asked for our address, we send out a reply. */

        if (uip_ipaddr_cmp(ipaddr, dev->d_ipaddr))
          {
            /* First, we register the one who made the request in our ARP
             * table, since it is likely that we will do more communication
             * with this host in the future.
             */

            uip_arp_update(ARPBUF->ah_sipaddr, ARPBUF->ah_shwaddr);

            /* The reply opcode is 2. */

            ARPBUF->ah_opcode = HTONS(2);

            memcpy(ARPBUF->ah_dhwaddr, ARPBUF->ah_shwaddr, IFHWADDRLEN);
            memcpy(ARPBUF->ah_shwaddr, dev->d_mac.addr, IFHWADDRLEN);
            memcpy(ETHBUF->src, dev->d_mac.addr, IFHWADDRLEN);
            memcpy(ETHBUF->dest, ARPBUF->ah_dhwaddr, IFHWADDRLEN);

            ARPBUF->ah_dipaddr[0] = ARPBUF->ah_sipaddr[0];
            ARPBUF->ah_dipaddr[1] = ARPBUF->ah_sipaddr[1];
            uiphdr_ipaddr_copy(ARPBUF->ah_sipaddr, &dev->d_ipaddr);
            uip_arp_dump(ARPBUF);

            ETHBUF->type          = HTONS(UIP_ETHTYPE_ARP);
            dev->d_len            = sizeof(struct arp_hdr) + UIP_LLH_LEN;
          }
        break;

      case HTONS(ARP_REPLY):
        /* ARP reply. We insert or update the ARP table if it was meant
         * for us.
         */

        if (uip_ipaddr_cmp(ipaddr, dev->d_ipaddr))
          {
            uip_arp_update(ARPBUF->ah_sipaddr, ARPBUF->ah_shwaddr);
          }
        break;
    }
}

/* Prepend Ethernet header to an outbound IP packet and see if we need
 * to send out an ARP request.
 *
 * This function should be called before sending out an IP packet. The
 * function checks the destination IP address of the IP packet to see
 * what Ethernet MAC address that should be used as a destination MAC
 * address on the Ethernet.
 *
 * If the destination IP address is in the local network (determined
 * by logical ANDing of netmask and our IP address), the function
 * checks the ARP cache to see if an entry for the destination IP
 * address is found. If so, an Ethernet header is prepended and the
 * function returns. If no ARP cache entry is found for the
 * destination IP address, the packet in the d_buf[] is replaced by
 * an ARP request packet for the IP address. The IP packet is dropped
 * and it is assumed that they higher level protocols (e.g., TCP)
 * eventually will retransmit the dropped packet.
 *
 * If the destination IP address is not on the local network, the IP
 * address of the default router is used instead.
 *
 * When the function returns, a packet is present in the d_buf[]
 * buffer, and the length of the packet is in the field d_len.
 */

void uip_arp_out(struct uip_driver_s *dev)
{
  struct arp_entry *tabptr;
  in_addr_t         ipaddr;
  in_addr_t         destipaddr;
  int               i;

  /* Find the destination IP address in the ARP table and construct
   * the Ethernet header. If the destination IP addres isn't on the
   * local network, we use the default router's IP address instead.
   *
   * If not ARP table entry is found, we overwrite the original IP
   * packet with an ARP request for the IP address.
   */

  /* First check if destination is a local broadcast. */

  if (uiphdr_ipaddr_cmp(IPBUF->eh_destipaddr, broadcast_ipaddr))
    {
      memcpy(ETHBUF->dest, broadcast_ethaddr.addr, IFHWADDRLEN);
    }
  else
    {
      /* Check if the destination address is on the local network. */

      destipaddr = uip_ip4addr_conv(IPBUF->eh_destipaddr);
      if (!uip_ipaddr_maskcmp(destipaddr, dev->d_ipaddr, dev->d_netmask))
        {
          /* Destination address was not on the local network, so we need to
           * use the default router's IP address instead of the destination
           * address when determining the MAC address.
           */

          uip_ipaddr_copy(ipaddr, dev->d_draddr);
        }
      else
        {
          /* Else, we use the destination IP address. */

          uip_ipaddr_copy(ipaddr, destipaddr);
        }

      /* Check if we already have this destination address in the ARP table */

      for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
        {
          tabptr = &arp_table[i];
          if (uip_ipaddr_cmp(ipaddr, tabptr->at_ipaddr))
            {
              break;
            }
        }

      if (i == CONFIG_NET_ARPTAB_SIZE)
        {
          /* The destination address was not in our ARP table, so we
           * overwrite the IP packet with an ARP request.
           */

          memset(ETHBUF->dest, 0xff, IFHWADDRLEN);
          memset(ARPBUF->ah_dhwaddr, 0x00, IFHWADDRLEN);
          memcpy(ETHBUF->src, dev->d_mac.addr, IFHWADDRLEN);
          memcpy(ARPBUF->ah_shwaddr, dev->d_mac.addr, IFHWADDRLEN);

          uiphdr_ipaddr_copy(ARPBUF->ah_dipaddr, &ipaddr);
          uiphdr_ipaddr_copy(ARPBUF->ah_sipaddr, &dev->d_ipaddr);

          ARPBUF->ah_opcode   = HTONS(ARP_REQUEST);
          ARPBUF->ah_hwtype   = HTONS(ARP_HWTYPE_ETH);
          ARPBUF->ah_protocol = HTONS(UIP_ETHTYPE_IP);
          ARPBUF->ah_hwlen    = IFHWADDRLEN;
          ARPBUF->ah_protolen = 4;
          uip_arp_dump(ARPBUF);

          ETHBUF->type        = HTONS(UIP_ETHTYPE_ARP);
          dev->d_len          = sizeof(struct arp_hdr) + UIP_LLH_LEN;
          return;
        }

      /* Build an ethernet header. */

      memcpy(ETHBUF->dest, tabptr->at_ethaddr.addr, IFHWADDRLEN);
    }

  /* Finish populating the ethernet header */

  memcpy(ETHBUF->src, dev->d_mac.addr, IFHWADDRLEN);
  ETHBUF->type = HTONS(UIP_ETHTYPE_IP);
  dev->d_len  += UIP_LLH_LEN;
}
#endif /* CONFIG_NET */
