/* uip-arp.c
 * Implementation of the ARP Address Resolution Protocol.
 * Author: Adam Dunkels <adam@dunkels.com>
 *
 * The Address Resolution Protocol ARP is used for mapping between IP
 * addresses and link level addresses such as the Ethernet MAC
 * addresses. ARP uses broadcast queries to ask for the link level
 * address of a known IP address and the host which is configured with
 * the IP address for which the query was meant, will respond with its
 * link level address.
 *
 * Note: This ARP implementation only supports Ethernet.
 *
 * Copyright (c) 2001-2003, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <string.h>
#include <netinet/in.h>
#include <net/uip/uip-arch.h>
#include <net/uip/uip-arp.h>

struct arp_hdr
{
  struct uip_eth_hdr ethhdr;
  uint16 hwtype;
  uint16 protocol;
  uint8 hwlen;
  uint8 protolen;
  uint16 opcode;
  struct uip_eth_addr shwaddr;
  in_addr_t sipaddr;
  struct uip_eth_addr dhwaddr;
  in_addr_t dipaddr;
};

struct ethip_hdr
{
  struct uip_eth_hdr ethhdr;

  /* IP header. */

  uint8  vhl;
  uint8  tos;
  uint8  len[2];
  uint8  ipid[2];
  uint8  ipoffset[2];
  uint8  ttl;
  uint8  proto;
  uint16 ipchksum;
  uint16 srcipaddr[2];
  uint16 destipaddr[2];
};

#define ARP_REQUEST 1
#define ARP_REPLY   2

#define ARP_HWTYPE_ETH 1

struct arp_entry
{
  in_addr_t ipaddr;
  struct uip_eth_addr ethaddr;
  uint8 time;
};

static const struct uip_eth_addr broadcast_ethaddr =
  {{0xff,0xff,0xff,0xff,0xff,0xff}};
static const uint16 broadcast_ipaddr[2] = {0xffff,0xffff};

static struct arp_entry arp_table[UIP_ARPTAB_SIZE];
static in_addr_t ipaddr;
static uint8 i, c;

static uint8 arptime;
static uint8 tmpage;

#define BUF   ((struct arp_hdr *)&dev->d_buf[0])
#define IPBUF ((struct ethip_hdr *)&dev->d_buf[0])

/* Initialize the ARP module. */

void uip_arp_init(void)
{
  for (i = 0; i < UIP_ARPTAB_SIZE; ++i)
    {
      memset(&arp_table[i].ipaddr, 0, sizeof(in_addr_t));
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

  ++arptime;
  for (i = 0; i < UIP_ARPTAB_SIZE; ++i)
    {
      tabptr = &arp_table[i];
      if (tabptr->ipaddr != 0 && arptime - tabptr->time >= UIP_ARP_MAXAGE)
        {
          tabptr->ipaddr = 0;
        }
    }
}

static void uip_arp_update(in_addr_t pipaddr, struct uip_eth_addr *ethaddr)
{
  struct arp_entry *tabptr;

  /* Walk through the ARP mapping table and try to find an entry to
   * update. If none is found, the IP -> MAC address mapping is
   * inserted in the ARP table.
   */

  for (i = 0; i < UIP_ARPTAB_SIZE; ++i)
    {
      tabptr = &arp_table[i];

      /* Only check those entries that are actually in use. */

      if (tabptr->ipaddr != 0)
        {
          /* Check if the source IP address of the incoming packet matches
           * the IP address in this ARP table entry.
           */

          if (uip_ipaddr_cmp(pipaddr, tabptr->ipaddr))
            {
              /* An old entry found, update this and return. */
              memcpy(tabptr->ethaddr.addr, ethaddr->addr, IFHWADDRLEN);
              tabptr->time = arptime;

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
      if (tabptr->ipaddr == 0)
        {
          break;
        }
    }

  /* If no unused entry is found, we try to find the oldest entry and
   * throw it away.
   */

  if (i == UIP_ARPTAB_SIZE)
    {
      tmpage = 0;
      c = 0;
      for (i = 0; i < UIP_ARPTAB_SIZE; ++i)
        {
          tabptr = &arp_table[i];
          if (arptime - tabptr->time > tmpage)
            {
              tmpage = arptime - tabptr->time;
              c = i;
            }
        }
      i = c;
      tabptr = &arp_table[i];
    }

  /* Now, i is the ARP table entry which we will fill with the new
   * information.
   */

  tabptr->ipaddr = pipaddr;
  memcpy(tabptr->ethaddr.addr, ethaddr->addr, IFHWADDRLEN);
  tabptr->time = arptime;
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
  if ((IPBUF->srcipaddr & dev->d_netmask) != (dev->d_ipaddr & dev->d_netmask))
    {
      return;
    }

  uip_arp_update(IPBUF->srcipaddr, &(IPBUF->ethhdr.src));
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
  if (dev->d_len < sizeof(struct arp_hdr))
    {
      dev->d_len = 0;
      return;
    }
  dev->d_len = 0;

  switch(BUF->opcode)
    {
      case HTONS(ARP_REQUEST):
        /* ARP request. If it asked for our address, we send out a reply. */

        if (uip_ipaddr_cmp(BUF->dipaddr, dev->d_ipaddr))
          {
            /* First, we register the one who made the request in our ARP
             * table, since it is likely that we will do more communication
             * with this host in the future.
             */

            uip_arp_update(BUF->sipaddr, &BUF->shwaddr);

            /* The reply opcode is 2. */

            BUF->opcode = HTONS(2);

            memcpy(BUF->dhwaddr.addr, BUF->shwaddr.addr, IFHWADDRLEN);
            memcpy(BUF->shwaddr.addr, dev->d_mac.addr, IFHWADDRLEN);
            memcpy(BUF->ethhdr.src.addr, dev->d_mac.addr, IFHWADDRLEN);
            memcpy(BUF->ethhdr.dest.addr, BUF->dhwaddr.addr, IFHWADDRLEN);

            BUF->dipaddr = BUF->sipaddr;
            BUF->sipaddr = dev->d_ipaddr;

            BUF->ethhdr.type = HTONS(UIP_ETHTYPE_ARP);
            dev->d_len = sizeof(struct arp_hdr);
          }
        break;

      case HTONS(ARP_REPLY):
        /* ARP reply. We insert or update the ARP table if it was meant
         * for us.
         */

        if (uip_ipaddr_cmp(BUF->dipaddr, dev->d_ipaddr))
          {
            uip_arp_update(BUF->sipaddr, &BUF->shwaddr);
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

  /* Find the destination IP address in the ARP table and construct
     the Ethernet header. If the destination IP addres isn't on the
     local network, we use the default router's IP address instead.

     If not ARP table entry is found, we overwrite the original IP
     packet with an ARP request for the IP address. */

  /* First check if destination is a local broadcast. */

  if (uiphdr_ipaddr_cmp(IPBUF->destipaddr, broadcast_ipaddr))
    {
      memcpy(IPBUF->ethhdr.dest.addr, broadcast_ethaddr.addr, IFHWADDRLEN);
    }
  else
    {
      /* Check if the destination address is on the local network. */

      if (!uip_ipaddr_maskcmp(IPBUF->destipaddr, dev->d_ipaddr, dev->d_netmask))
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

          uip_ipaddr_copy(ipaddr, IPBUF->destipaddr);
        }

      for (i = 0; i < UIP_ARPTAB_SIZE; ++i)
        {
          tabptr = &arp_table[i];
          if (uip_ipaddr_cmp(ipaddr, tabptr->ipaddr))
            {
              break;
            }
        }

      if (i == UIP_ARPTAB_SIZE)
        {
          /* The destination address was not in our ARP table, so we
           * overwrite the IP packet with an ARP request.
           */

          memset(BUF->ethhdr.dest.addr, 0xff, IFHWADDRLEN);
          memset(BUF->dhwaddr.addr, 0x00, IFHWADDRLEN);
          memcpy(BUF->ethhdr.src.addr, dev->d_mac.addr, IFHWADDRLEN);
          memcpy(BUF->shwaddr.addr, dev->d_mac.addr, IFHWADDRLEN);

          uip_ipaddr_copy(BUF->dipaddr, ipaddr);
          uip_ipaddr_copy(BUF->sipaddr, dev->d_ipaddr);
          BUF->opcode = HTONS(ARP_REQUEST); /* ARP request. */
          BUF->hwtype = HTONS(ARP_HWTYPE_ETH);
          BUF->protocol = HTONS(UIP_ETHTYPE_IP);
          BUF->hwlen = IFHWADDRLEN;
          BUF->protolen = 4;
          BUF->ethhdr.type = HTONS(UIP_ETHTYPE_ARP);

          dev->d_appdata = &dev->d_buf[UIP_TCPIP_HLEN + UIP_LLH_LEN];

          dev->d_len = sizeof(struct arp_hdr);
          return;
        }

      /* Build an ethernet header. */

      memcpy(IPBUF->ethhdr.dest.addr, tabptr->ethaddr.addr, IFHWADDRLEN);
    }
  memcpy(IPBUF->ethhdr.src.addr, dev->d_mac.addr, IFHWADDRLEN);

  IPBUF->ethhdr.type = HTONS(UIP_ETHTYPE_IP);

  dev->d_len += sizeof(struct uip_eth_hdr);
}
