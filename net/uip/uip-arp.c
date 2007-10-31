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

#include <sys/types.h>
#include <sys/ioctl.h>
#include <string.h>
#include <netinet/in.h>
#include <net/uip/uip-arch.h>
#include <net/uip/uip-arp.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define ARP_REQUEST 1
#define ARP_REPLY   2

#define ARP_HWTYPE_ETH 1

#define BUF   ((struct arp_hdr *)&dev->d_buf[0])
#define IPBUF ((struct ethip_hdr *)&dev->d_buf[0])

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct arp_hdr
{
  struct uip_eth_hdr  ah_ethhdr;
  uint16              ah_hwtype;
  uint16              ah_protocol;
  uint8               ah_hwlen;
  uint8               ah_protolen;
  uint16              ah_opcode;
  struct uip_eth_addr ah_shwaddr;
  uint16              ah_sipaddr[2];
  struct uip_eth_addr ah_dhwaddr;
  uint16              ah_dipaddr[2];
};

struct ethip_hdr
{
  struct uip_eth_hdr  eh_ethhdr;

  /* IP header. */

  uint8               eh_vhl;
  uint8               eh_tos;
  uint8               eh_len[2];
  uint8               eh_ipid[2];
  uint8               eh_ipoffset[2];
  uint8               eh_ttl;
  uint8               eh_proto;
  uint16              eh_ipchksum;
  uint16              eh_srcipaddr[2];
  uint16              eh_destipaddr[2];
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

static struct arp_entry arp_table[UIP_ARPTAB_SIZE];
static uint8 g_arptime;


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void uip_arp_update(uint16 *pipaddr, struct uip_eth_addr *ethaddr)
{
  struct arp_entry *tabptr;
  in_addr_t         ipaddr = uip_ip4addr_conv(pipaddr);
  int               i;

  /* Walk through the ARP mapping table and try to find an entry to
   * update. If none is found, the IP -> MAC address mapping is
   * inserted in the ARP table.
   */

  for (i = 0; i < UIP_ARPTAB_SIZE; ++i)
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
              memcpy(tabptr->at_ethaddr.addr, ethaddr->addr, IFHWADDRLEN);
              tabptr->at_time = g_arptime;

              return;
            }
        }
    }

  /* If we get here, no existing ARP table entry was found, so we
     create one. */

  /* First, we try to find an unused entry in the ARP table. */

  for (i = 0; i < UIP_ARPTAB_SIZE; ++i)
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

  if (i == UIP_ARPTAB_SIZE)
    {
      uint8 tmpage = 0;
      int   j      = 0;
      for (i = 0; i < UIP_ARPTAB_SIZE; ++i)
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
  memcpy(tabptr->at_ethaddr.addr, ethaddr->addr, IFHWADDRLEN);
  tabptr->at_time = g_arptime;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Initialize the ARP module. */

void uip_arp_init(void)
{
  int i;
  for (i = 0; i < UIP_ARPTAB_SIZE; ++i)
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
  for (i = 0; i < UIP_ARPTAB_SIZE; ++i)
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
  dev->d_len -= sizeof(struct uip_eth_hdr);

  /* Only insert/update an entry if the source IP address of the
     incoming IP packet comes from a host on the local network. */
  if ((IPBUF->eh_srcipaddr & dev->d_netmask) != (dev->d_ipaddr & dev->d_netmask))
    {
      return;
    }

  uip_arp_update(IPBUF->eh_srcipaddr, &(IPBUF->eh_ethhdr.src));
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
  if (dev->d_len < sizeof(struct arp_hdr))
    {
      dev->d_len = 0;
      return;
    }
  dev->d_len = 0;

  ipaddr = uip_ip4addr_conv(BUF->ah_dipaddr);
  switch(BUF->ah_opcode)
    {
      case HTONS(ARP_REQUEST):
        /* ARP request. If it asked for our address, we send out a reply. */

        if (uip_ipaddr_cmp(ipaddr, dev->d_ipaddr))
          {
            /* First, we register the one who made the request in our ARP
             * table, since it is likely that we will do more communication
             * with this host in the future.
             */

            uip_arp_update(BUF->ah_sipaddr, &BUF->ah_shwaddr);

            /* The reply opcode is 2. */

            BUF->ah_opcode = HTONS(2);

            memcpy(BUF->ah_dhwaddr.addr, BUF->ah_shwaddr.addr, IFHWADDRLEN);
            memcpy(BUF->ah_shwaddr.addr, dev->d_mac.addr, IFHWADDRLEN);
            memcpy(BUF->ah_ethhdr.src.addr, dev->d_mac.addr, IFHWADDRLEN);
            memcpy(BUF->ah_ethhdr.dest.addr, BUF->ah_dhwaddr.addr, IFHWADDRLEN);

            BUF->ah_dipaddr[0] = BUF->ah_sipaddr[0];
            BUF->ah_dipaddr[1] = BUF->ah_sipaddr[1];
            BUF->ah_sipaddr[0] = dev->d_ipaddr >> 16;
            BUF->ah_sipaddr[1] = dev->d_ipaddr & 0xffff;

            BUF->ah_ethhdr.type = HTONS(UIP_ETHTYPE_ARP);
            dev->d_len = sizeof(struct arp_hdr);
          }
        break;

      case HTONS(ARP_REPLY):
        /* ARP reply. We insert or update the ARP table if it was meant
         * for us.
         */

        if (uip_ipaddr_cmp(ipaddr, dev->d_ipaddr))
          {
            uip_arp_update(BUF->ah_sipaddr, &BUF->ah_shwaddr);
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
     the Ethernet header. If the destination IP addres isn't on the
     local network, we use the default router's IP address instead.

     If not ARP table entry is found, we overwrite the original IP
     packet with an ARP request for the IP address. */

  /* First check if destination is a local broadcast. */

  if (uiphdr_ipaddr_cmp(IPBUF->eh_destipaddr, broadcast_ipaddr))
    {
      memcpy(IPBUF->eh_ethhdr.dest.addr, broadcast_ethaddr.addr, IFHWADDRLEN);
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

      for (i = 0; i < UIP_ARPTAB_SIZE; ++i)
        {
          tabptr = &arp_table[i];
          if (uip_ipaddr_cmp(ipaddr, tabptr->at_ipaddr))
            {
              break;
            }
        }

      if (i == UIP_ARPTAB_SIZE)
        {
          /* The destination address was not in our ARP table, so we
           * overwrite the IP packet with an ARP request.
           */

          memset(BUF->ah_ethhdr.dest.addr, 0xff, IFHWADDRLEN);
          memset(BUF->ah_dhwaddr.addr, 0x00, IFHWADDRLEN);
          memcpy(BUF->ah_ethhdr.src.addr, dev->d_mac.addr, IFHWADDRLEN);
          memcpy(BUF->ah_shwaddr.addr, dev->d_mac.addr, IFHWADDRLEN);

          uiphdr_ipaddr_copy(BUF->ah_dipaddr, &ipaddr);
          uiphdr_ipaddr_copy(BUF->ah_sipaddr, &dev->d_ipaddr);
          BUF->ah_opcode      = HTONS(ARP_REQUEST); /* ARP request. */
          BUF->ah_hwtype      = HTONS(ARP_HWTYPE_ETH);
          BUF->ah_protocol    = HTONS(UIP_ETHTYPE_IP);
          BUF->ah_hwlen       = IFHWADDRLEN;
          BUF->ah_protolen    = 4;
          BUF->ah_ethhdr.type = HTONS(UIP_ETHTYPE_ARP);

          dev->d_appdata = &dev->d_buf[UIP_TCPIP_HLEN + UIP_LLH_LEN];

          dev->d_len = sizeof(struct arp_hdr);
          return;
        }

      /* Build an ethernet header. */

      memcpy(IPBUF->eh_ethhdr.dest.addr, tabptr->at_ethaddr.addr, IFHWADDRLEN);
    }
  memcpy(IPBUF->eh_ethhdr.src.addr, dev->d_mac.addr, IFHWADDRLEN);

  IPBUF->eh_ethhdr.type = HTONS(UIP_ETHTYPE_IP);

  dev->d_len += sizeof(struct uip_eth_hdr);
}
