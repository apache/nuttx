/************************************************************
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
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <debug.h>

#include <net/uip/uip.h>

#include "dhcpc.h"

/************************************************************
 * Definitions
 ************************************************************/

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

/************************************************************
 * Private Types
 ************************************************************/

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
#ifndef CONFIG_UIP_DHCP_LIGHT
  uint8 sname[64];
  uint8 file[128];
#endif
  uint8 options[312];
};

/************************************************************
 * Private Data
 ************************************************************/

static struct dhcpc_state s;

static const uint8 xid[4] = {0xad, 0xde, 0x12, 0x23};
static const uint8 magic_cookie[4] = {99, 130, 83, 99};

/************************************************************
 * Private Functions
 ************************************************************/

static uint8 *add_msg_type(uint8 *optptr, uint8 type)
{
  *optptr++ = DHCP_OPTION_MSG_TYPE;
  *optptr++ = 1;
  *optptr++ = type;
  return optptr;
}

static uint8 *add_server_id(uint8 *optptr)
{
  *optptr++ = DHCP_OPTION_SERVER_ID;
  *optptr++ = 4;
  memcpy(optptr, s.serverid, 4);
  return optptr + 4;
}

static uint8 *add_req_ipaddr(uint8 *optptr)
{
  *optptr++ = DHCP_OPTION_REQ_IPADDR;
  *optptr++ = 4;
  memcpy(optptr, s.ipaddr, 4);
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

static void create_msg(register struct dhcp_msg *m)
{
  m->op = DHCP_REQUEST;
  m->htype = DHCP_HTYPE_ETHERNET;
  m->hlen = s.mac_len;
  m->hops = 0;
  memcpy(m->xid, xid, sizeof(m->xid));
  m->secs = 0;
  m->flags = HTONS(BOOTP_BROADCAST); /*  Broadcast bit. */
  /*  uip_ipaddr_copy(m->ciaddr, uip_hostaddr);*/
  memcpy(m->ciaddr, uip_hostaddr, sizeof(m->ciaddr));
  memset(m->yiaddr, 0, sizeof(m->yiaddr));
  memset(m->siaddr, 0, sizeof(m->siaddr));
  memset(m->giaddr, 0, sizeof(m->giaddr));
  memcpy(m->chaddr, s.mac_addr, s.mac_len);
  memset(&m->chaddr[s.mac_len], 0, sizeof(m->chaddr) - s.mac_len);
#ifndef CONFIG_UIP_DHCP_LIGHT
  memset(m->sname, 0, sizeof(m->sname));
  memset(m->file, 0, sizeof(m->file));
#endif

  memcpy(m->options, magic_cookie, sizeof(magic_cookie));
}

static void send_discover(void)
{
  uint8 *end;
  struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

  create_msg(m);

  end = add_msg_type(&m->options[4], DHCPDISCOVER);
  end = add_req_options(end);
  end = add_end(end);

  uip_send(uip_appdata, end - (uint8 *)uip_appdata);
}

static void send_request(void)
{
  uint8 *end;
  struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

  create_msg(m);

  end = add_msg_type(&m->options[4], DHCPREQUEST);
  end = add_server_id(end);
  end = add_req_ipaddr(end);
  end = add_end(end);

  uip_send(uip_appdata, end - (uint8 *)uip_appdata);
}

static uint8 parse_options(uint8 *optptr, int len)
{
  uint8 *end = optptr + len;
  uint8 type = 0;

  while (optptr < end)
    {
      switch(*optptr)
        {
          case DHCP_OPTION_SUBNET_MASK:
            memcpy(s.netmask, optptr + 2, 4);
            break;
          case DHCP_OPTION_ROUTER:
            memcpy(s.default_router, optptr + 2, 4);
            break;
          case DHCP_OPTION_DNS_SERVER:
            memcpy(s.dnsaddr, optptr + 2, 4);
            break;
          case DHCP_OPTION_MSG_TYPE:
            type = *(optptr + 2);
            break;
          case DHCP_OPTION_SERVER_ID:
            memcpy(s.serverid, optptr + 2, 4);
            break;
          case DHCP_OPTION_LEASE_TIME:
            memcpy(s.lease_time, optptr + 2, 4);
            break;
          case DHCP_OPTION_END:
            return type;
        }

      optptr += optptr[1] + 2;
    }
  return type;
}

static uint8 parse_msg(void)
{
  struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

  if (m->op == DHCP_REPLY &&
     memcmp(m->xid, xid, sizeof(xid)) == 0 &&
     memcmp(m->chaddr, s.mac_addr, s.mac_len) == 0)
    {
      memcpy(s.ipaddr, m->yiaddr, 4);
      return parse_options(&m->options[4], uip_datalen());
    }
  return 0;
}

static void handle_dhcp(void)
{
restart:
  s.state = STATE_SENDING;
  s.ticks = CLOCK_SECOND;

  do
    {
      /* Send the command */

      send_discover();

      /* Wait for the response */

      uip_event_timedwait(UIP_NEWDATA, CLOCK_SECOND);

      if (uip_newdata() && parse_msg() == DHCPOFFER)
        {
          s.state = STATE_OFFER_RECEIVED;
          break;
        }

      if (s.ticks < CLOCK_SECOND * 60)
        {
          s.ticks *= 2;
        }
    }
  while(s.state != STATE_OFFER_RECEIVED);

  s.ticks = CLOCK_SECOND;

  do
    {
      /* Send the request */

      send_request();

      /* Then wait to received the response */

      uip_event_timedwait(UIP_NEWDATA, CLOCK_SECOND);

      if (uip_newdata() && parse_msg() == DHCPACK)
        {
          s.state = STATE_CONFIG_RECEIVED;
          break;
        }

      if (s.ticks <= CLOCK_SECOND * 10)
        {
          s.ticks += CLOCK_SECOND;
        }
      else
        {
          goto restart;
        }
    }
  while(s.state != STATE_CONFIG_RECEIVED);

  dbg("Got IP address %d.%d.%d.%d\n",
      uip_ipaddr1(s.ipaddr), uip_ipaddr2(s.ipaddr),
      uip_ipaddr3(s.ipaddr), uip_ipaddr4(s.ipaddr));
  dbg("Got netmask %d.%d.%d.%d\n",
      uip_ipaddr1(s.netmask), uip_ipaddr2(s.netmask),
      uip_ipaddr3(s.netmask), uip_ipaddr4(s.netmask));
  dbg("Got DNS server %d.%d.%d.%d\n",
      uip_ipaddr1(s.dnsaddr), uip_ipaddr2(s.dnsaddr),
      uip_ipaddr3(s.dnsaddr), uip_ipaddr4(s.dnsaddr));
  dbg("Got default router %d.%d.%d.%d\n",
      uip_ipaddr1(s.default_router), uip_ipaddr2(s.default_router),
      uip_ipaddr3(s.default_router), uip_ipaddr4(s.default_router));
  dbg("Lease expires in %ld seconds\n",
      ntohs(s.lease_time[0])*65536ul + ntohs(s.lease_time[1]));

  dhcpc_configured(&s);

  pthread_exit(NULL);
}

/************************************************************
 * Global Functions
 ************************************************************/

void dhcpc_init(const void *mac_addr, int mac_len)
{
  uip_ipaddr_t addr;

  s.mac_addr = mac_addr;
  s.mac_len  = mac_len;

  s.state = STATE_INITIAL;
  uip_ipaddr(addr, 255,255,255,255);
  s.conn = uip_udp_new(&addr, HTONS(DHCPC_SERVER_PORT));
  if (s.conn != NULL)
    {
      uip_udp_bind(s.conn, HTONS(DHCPC_CLIENT_PORT));
    }
}

/* This function is called by the UIP interrupt handling logic whenevent an
 * event of interest occurs.
 */

void uip_interrupt_udp_event(void)
{
  handle_dhcp();
}

void dhcpc_request(void)
{
  uint16 ipaddr[2];

  if (s.state == STATE_INITIAL)
    {
      uip_ipaddr(ipaddr, 0,0,0,0);
      uip_sethostaddr(ipaddr);
    }
}
