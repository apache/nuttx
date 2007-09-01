/****************************************************************************
 * dhcpc.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Based heavily on portions of uIP:
 *
 *   Copyright (c) 2005, Swedish Institute of Computer Science
 *   All rights reserved.
 *
 * Copyright (c) 2005, Swedish Institute of Computer Science
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

#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <debug.h>

#include <net/uip/uip.h>
#include <net/uip/dhcpc.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* CLK_TCK is the frequency of the system clock (typically 100Hz) */

#define CLOCK_SECOND            CLK_TCK

#define STATE_INITIAL           0
#define STATE_SENDING           1
#define STATE_OFFER_RECEIVED    2
#define STATE_CONFIG_RECEIVED   3

#define BOOTP_BROADCAST         0x8000

#define DHCP_REQUEST            1
#define DHCP_REPLY              2
#define DHCP_HTYPE_ETHERNET     1
#define DHCP_HLEN_ETHERNET      6
#define DHCP_MSG_LEN            236

#define DHCPC_SERVER_PORT       67
#define DHCPC_CLIENT_PORT       68

#define DHCPDISCOVER            1
#define DHCPOFFER               2
#define DHCPREQUEST             3
#define DHCPDECLINE             4
#define DHCPACK                 5
#define DHCPNAK                 6
#define DHCPRELEASE             7

#define DHCP_OPTION_SUBNET_MASK 1
#define DHCP_OPTION_ROUTER      3
#define DHCP_OPTION_DNS_SERVER  6
#define DHCP_OPTION_REQ_IPADDR  50
#define DHCP_OPTION_LEASE_TIME  51
#define DHCP_OPTION_MSG_TYPE    53
#define DHCP_OPTION_SERVER_ID   54
#define DHCP_OPTION_REQ_LIST    55
#define DHCP_OPTION_END         255

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dhcpc_state_internal
{
  char state;
  sem_t sem;
  struct uip_udp_conn *conn;
  uint16 ticks;
  const void *mac_addr;
  int mac_len;
  struct dhcpc_state *result;
};

struct dhcp_msg
{
  uint8 op, htype, hlen, hops;
  uint8 xid[4];
  uint16 secs, flags;
  uint8 ciaddr[4];
  uint8 yiaddr[4];
  uint8 siaddr[4];
  uint8 giaddr[4];
  uint8 chaddr[16];
#ifndef CONFIG_NET_DHCP_LIGHT
  uint8 sname[64];
  uint8 file[128];
#endif
  uint8 options[312];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8 xid[4] = {0xad, 0xde, 0x12, 0x23};
static const uint8 magic_cookie[4] = {99, 130, 83, 99};
static volatile struct dhcpc_state_internal *gpdhcpc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8 *add_msg_type(uint8 *optptr, uint8 type)
{
  *optptr++ = DHCP_OPTION_MSG_TYPE;
  *optptr++ = 1;
  *optptr++ = type;
  return optptr;
}

static uint8 *add_server_id(struct dhcpc_state *presult, uint8 *optptr)
{
  *optptr++ = DHCP_OPTION_SERVER_ID;
  *optptr++ = 4;
  memcpy(optptr, presult->serverid, 4);
  return optptr + 4;
}

static uint8 *add_req_ipaddr(struct dhcpc_state *presult, uint8 *optptr)
{
  *optptr++ = DHCP_OPTION_REQ_IPADDR;
  *optptr++ = 4;
  memcpy(optptr, presult->ipaddr, 4);
  return optptr + 4;
}

static uint8 *add_req_options(uint8 *optptr)
{
  *optptr++ = DHCP_OPTION_REQ_LIST;
  *optptr++ = 3;
  *optptr++ = DHCP_OPTION_SUBNET_MASK;
  *optptr++ = DHCP_OPTION_ROUTER;
  *optptr++ = DHCP_OPTION_DNS_SERVER;
  return optptr;
}

static uint8 *add_end(uint8 *optptr)
{
  *optptr++ = DHCP_OPTION_END;
  return optptr;
}

static void create_msg(struct dhcpc_state_internal *pdhcpc, struct dhcp_msg *m)
{
  m->op = DHCP_REQUEST;
  m->htype = DHCP_HTYPE_ETHERNET;
  m->hlen = pdhcpc->mac_len;
  m->hops = 0;
  memcpy(m->xid, xid, sizeof(m->xid));
  m->secs = 0;
  m->flags = HTONS(BOOTP_BROADCAST); /*  Broadcast bit. */
  /*  uip_ipaddr_copy(m->ciaddr, uip_hostaddr);*/
  memcpy(m->ciaddr, uip_hostaddr, sizeof(m->ciaddr));
  memset(m->yiaddr, 0, sizeof(m->yiaddr));
  memset(m->siaddr, 0, sizeof(m->siaddr));
  memset(m->giaddr, 0, sizeof(m->giaddr));
  memcpy(m->chaddr, pdhcpc->mac_addr, pdhcpc->mac_len);
  memset(&m->chaddr[pdhcpc->mac_len], 0, sizeof(m->chaddr) - pdhcpc->mac_len);
#ifndef CONFIG_NET_DHCP_LIGHT
  memset(m->sname, 0, sizeof(m->sname));
  memset(m->file, 0, sizeof(m->file));
#endif

  memcpy(m->options, magic_cookie, sizeof(magic_cookie));
}

static void send_discover(struct dhcpc_state_internal *pdhcpc)
{
  uint8 *end;
  struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

  create_msg(pdhcpc, m);

  end = add_msg_type(&m->options[4], DHCPDISCOVER);
  end = add_req_options(end);
  end = add_end(end);

  uip_send(uip_appdata, end - (uint8 *)uip_appdata);
}

static void send_request(struct dhcpc_state_internal *pdhcpc)
{
  uint8 *end;
  struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

  create_msg(pdhcpc, m);

  end = add_msg_type(&m->options[4], DHCPREQUEST);
  end = add_server_id(pdhcpc->result, end);
  end = add_req_ipaddr(pdhcpc->result, end);
  end = add_end(end);

  uip_send(uip_appdata, end - (uint8 *)uip_appdata);
}

static uint8 parse_options(struct dhcpc_state *presult, uint8 *optptr, int len)
{
  uint8 *end = optptr + len;
  uint8 type = 0;

  while (optptr < end)
    {
      switch(*optptr)
        {
          case DHCP_OPTION_SUBNET_MASK:
            memcpy(presult->netmask, optptr + 2, 4);
            break;
          case DHCP_OPTION_ROUTER:
            memcpy(presult->default_router, optptr + 2, 4);
            break;
          case DHCP_OPTION_DNS_SERVER:
            memcpy(presult->dnsaddr, optptr + 2, 4);
            break;
          case DHCP_OPTION_MSG_TYPE:
            type = *(optptr + 2);
            break;
          case DHCP_OPTION_SERVER_ID:
            memcpy(presult->serverid, optptr + 2, 4);
            break;
          case DHCP_OPTION_LEASE_TIME:
            memcpy(presult->lease_time, optptr + 2, 4);
            break;
          case DHCP_OPTION_END:
            return type;
        }

      optptr += optptr[1] + 2;
    }
  return type;
}

static uint8 parse_msg(struct dhcpc_state_internal *pdhcpc)
{
  struct dhcpc_state *presult = pdhcpc->result;
  struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

  if (m->op == DHCP_REPLY &&
      memcmp(m->xid, xid, sizeof(xid)) == 0 &&
      memcmp(m->chaddr, pdhcpc->mac_addr, pdhcpc->mac_len) == 0)
    {
      memcpy(presult->ipaddr, m->yiaddr, 4);
      return parse_options(presult, &m->options[4], uip_datalen());
    }
  return 0;
}

static void handle_dhcp(struct dhcpc_state_internal *pdhcpc)
{
  struct dhcpc_state *presult = pdhcpc->result;

restart:
  pdhcpc->state = STATE_SENDING;
  pdhcpc->ticks = CLOCK_SECOND;

  do
    {
      /* Send the command */

      send_discover(pdhcpc);

      /* Wait for the response */

      uip_event_timedwait(UIP_NEWDATA, CLOCK_SECOND);

      if (uip_newdata() && parse_msg(pdhcpc) == DHCPOFFER)
        {
          pdhcpc->state = STATE_OFFER_RECEIVED;
          break;
        }

      if (pdhcpc->ticks < CLOCK_SECOND * 60)
        {
          pdhcpc->ticks *= 2;
        }
    }
  while(pdhcpc->state != STATE_OFFER_RECEIVED);

  pdhcpc->ticks = CLOCK_SECOND;

  do
    {
      /* Send the request */

      send_request(pdhcpc);

      /* Then wait to received the response */

      uip_event_timedwait(UIP_NEWDATA, CLOCK_SECOND);

      if (uip_newdata() && parse_msg(pdhcpc) == DHCPACK)
        {
          pdhcpc->state = STATE_CONFIG_RECEIVED;
          break;
        }

      if (pdhcpc->ticks <= CLOCK_SECOND * 10)
        {
          pdhcpc->ticks += CLOCK_SECOND;
        }
      else
        {
          goto restart;
        }
    }
  while(pdhcpc->state != STATE_CONFIG_RECEIVED);

  dbg("Got IP address %d.%d.%d.%d\n",
      uip_ipaddr1(presult->ipaddr), uip_ipaddr2(presult->ipaddr),
      uip_ipaddr3(presult->ipaddr), uip_ipaddr4(presult->ipaddr));
  dbg("Got netmask %d.%d.%d.%d\n",
      uip_ipaddr1(presult->netmask), uip_ipaddr2(presult->netmask),
      uip_ipaddr3(presult->netmask), uip_ipaddr4(presult->netmask));
  dbg("Got DNS server %d.%d.%d.%d\n",
      uip_ipaddr1(presult->dnsaddr), uip_ipaddr2(presult->dnsaddr),
      uip_ipaddr3(presult->dnsaddr), uip_ipaddr4(presult->dnsaddr));
  dbg("Got default router %d.%d.%d.%d\n",
      uip_ipaddr1(presult->default_router), uip_ipaddr2(presult->default_router),
      uip_ipaddr3(presult->default_router), uip_ipaddr4(presult->default_router));
  dbg("Lease expires in %ld seconds\n",
      ntohs(presult->lease_time[0])*65536ul + ntohs(presult->lease_time[1]));
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

void *dhcpc_open(const void *mac_addr, int mac_len)
{
  uip_ipaddr_t addr;
  struct dhcpc_state_internal *pdhcpc;

  pdhcpc = (struct dhcpc_state_internal *)malloc(sizeof(struct dhcpc_state_internal));
  if (pdhcpc)
    {
      memset(pdhcpc, 0, sizeof(struct dhcpc_state_internal));
      pdhcpc->mac_addr = mac_addr;
      pdhcpc->mac_len  = mac_len;
      pdhcpc->state    = STATE_INITIAL;
      sem_init(&pdhcpc->sem, 0, 0);

      uip_ipaddr(addr, 255,255,255,255);
      pdhcpc->conn = uip_udp_new(&addr, HTONS(DHCPC_SERVER_PORT));
      if (pdhcpc->conn != NULL)
        {
          uip_udp_bind(pdhcpc->conn, HTONS(DHCPC_CLIENT_PORT));
        }
    }
  return (void*)pdhcpc;
}

void dhcpc_close(void *handle)
{
  struct dchcpc_state_internal *pdhcpc = (struct dchcpc_state_internal *)handle;
  if (pdhcpc)
    {
      free(pdhcpc);
    }
}

/* This function is called by the UIP interrupt handling logic whenevent an
 * event of interest occurs.
 */

void uip_interrupt_udp_event(void)
{
  if (gpdhcpc)
    {
      sem_post(&gpdhcpc->sem);
    }
}

int dhcpc_request(void *handle, struct dhcpc_state *ds)
{
  struct dhcpc_state_internal *pdhcpc = (struct dhcpc_state_internal *)handle;
  uint16 ipaddr[2];

  if (pdhcpc->state == STATE_INITIAL)
    {
      uip_ipaddr(ipaddr, 0,0,0,0);
      uip_sethostaddr(ipaddr);
    }

  pdhcpc->result = ds;
  gpdhcpc = pdhcpc;
  sem_wait(&pdhcpc->sem);
  gpdhcpc = NULL;
  handle_dhcp(pdhcpc);
  return OK;
}
