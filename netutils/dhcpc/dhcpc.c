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
#include <sys/socket.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <net/uip/uip.h>
#include <net/uip/dhcpc.h>
#include <net/uip/uip-lib.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

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

#define BUFFER_SIZE             256

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dhcpc_state_internal
{
  struct uip_udp_conn *conn;
  const void            *mac_addr;
  int                    mac_len;
  int                    sockfd;
  char                   buffer[256];
};

struct dhcp_msg
{
  uint8  op;
  uint8  htype;
  uint8  hlen;
  uint8  hops;
  uint8  xid[4];
  uint16 secs;
  uint16 flags;
  uint8  ciaddr[4];
  uint8  yiaddr[4];
  uint8  siaddr[4];
  uint8  giaddr[4];
  uint8  chaddr[16];
#ifndef CONFIG_NET_DHCP_LIGHT
  uint8  sname[64];
  uint8  file[128];
#endif
  uint8  options[312];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8 xid[4]          = {0xad, 0xde, 0x12, 0x23};
static const uint8 magic_cookie[4] = {99, 130, 83, 99};

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
  memcpy(optptr, &presult->ipaddr.s_addr, 4);
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

static void create_msg(struct dhcpc_state_internal *pdhcpc, struct dhcp_msg *pmsg)
{
  struct in_addr addr;

  pmsg->op    = DHCP_REQUEST;
  pmsg->htype = DHCP_HTYPE_ETHERNET;
  pmsg->hlen  = pdhcpc->mac_len;
  pmsg->hops  = 0;
  memcpy(pmsg->xid, xid, sizeof(pmsg->xid));
  pmsg->secs  = 0;
  pmsg->flags = HTONS(BOOTP_BROADCAST); /*  Broadcast bit. */

  uip_gethostaddr("eth0", &addr);
  memcpy(&pmsg->ciaddr, &addr.s_addr, sizeof(pmsg->ciaddr));
  memset(pmsg->yiaddr, 0, sizeof(pmsg->yiaddr));
  memset(pmsg->siaddr, 0, sizeof(pmsg->siaddr));
  memset(pmsg->giaddr, 0, sizeof(pmsg->giaddr));

  memcpy(pmsg->chaddr, pdhcpc->mac_addr, pdhcpc->mac_len);
  memset(&pmsg->chaddr[pdhcpc->mac_len], 0, sizeof(pmsg->chaddr) - pdhcpc->mac_len);
#ifndef CONFIG_NET_DHCP_LIGHT
  memset(pmsg->sname, 0, sizeof(pmsg->sname));
  memset(pmsg->file, 0, sizeof(pmsg->file));
#endif

  memcpy(pmsg->options, magic_cookie, sizeof(magic_cookie));
}

static int send_discover(struct dhcpc_state_internal *pdhcpc)
{
  struct dhcp_msg msg;
  struct sockaddr_in addr;
  uint8 *pend;
  int len;

  create_msg(pdhcpc, &msg);
  pend = add_msg_type(&msg.options[4], DHCPDISCOVER);
  pend = add_req_options(pend);
  pend = add_end(pend);
  len  = pend - (uint8*)&msg;

  addr.sin_family      = AF_INET;
  addr.sin_port        = HTONS(DHCPC_SERVER_PORT);
  addr.sin_addr.s_addr = INADDR_BROADCAST;

  return sendto(pdhcpc->sockfd, &msg, len, 0,
                (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
}

static int send_request(struct dhcpc_state_internal *pdhcpc, struct dhcpc_state *presult)
{
  struct dhcp_msg msg;
  struct sockaddr_in addr;
  uint8 *pend;
  int len;

  create_msg(pdhcpc, &msg);
  pend = add_msg_type(&msg.options[4], DHCPREQUEST);
  pend = add_server_id(presult, pend);
  pend = add_req_ipaddr(presult, pend);
  pend = add_end(pend);
  len  = pend - (uint8*)&msg;

  addr.sin_family      = AF_INET;
  addr.sin_port        = HTONS(DHCPC_SERVER_PORT);
  addr.sin_addr.s_addr = INADDR_BROADCAST;

  return sendto(pdhcpc->sockfd, &msg, len, 0,
                (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
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
            memcpy(&presult->netmask.s_addr, optptr + 2, 4);
            break;
          case DHCP_OPTION_ROUTER:
            memcpy(&presult->default_router.s_addr, optptr + 2, 4);
            break;
          case DHCP_OPTION_DNS_SERVER:
            memcpy(&presult->dnsaddr.s_addr, optptr + 2, 4);
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

static uint8 parse_msg(struct dhcpc_state_internal *pdhcpc, int buflen, struct dhcpc_state *presult)
{
  struct dhcp_msg *pbuffer = (struct dhcp_msg *)pdhcpc->buffer;

  if (pbuffer->op == DHCP_REPLY &&
      memcmp(pbuffer->xid, xid, sizeof(xid)) == 0 &&
      memcmp(pbuffer->chaddr, pdhcpc->mac_addr, pdhcpc->mac_len) == 0)
    {
      memcpy(&presult->ipaddr.s_addr, pbuffer->yiaddr, 4);
      return parse_options(presult, &pbuffer->options[4], buflen);
    }
  return 0;
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

void *dhcpc_open(const void *mac_addr, int mac_len)
{
  struct dhcpc_state_internal *pdhcpc;
  struct sockaddr_in addr;
  struct timeval tv;

  /* Allocate an internal DHCP structure */

  pdhcpc = (struct dhcpc_state_internal *)malloc(sizeof(struct dhcpc_state_internal));
  if (pdhcpc)
    {
      /* Initialize the allocated structure */

      memset(pdhcpc, 0, sizeof(struct dhcpc_state_internal));
      pdhcpc->mac_addr = mac_addr;
      pdhcpc->mac_len  = mac_len;

      /* Create a UDP socket */

      pdhcpc->sockfd    = socket(PF_INET, SOCK_DGRAM, 0);
      if (pdhcpc->sockfd < 0)
        {
          free(pdhcpc);
          return NULL;
        }

      /* bind the socket */

      addr.sin_family      = AF_INET;
      addr.sin_port        = HTONS(DHCPC_CLIENT_PORT);
      addr.sin_addr.s_addr = INADDR_ANY;

      if (bind(pdhcpc->sockfd, (struct sockaddr*)&addr, sizeof(struct sockaddr_in)) < 0)
        {
          close(pdhcpc->sockfd);
          free(pdhcpc);
          return NULL;
        }

      /* Configure for read timeouts */

      tv.tv_sec  = 30;
      tv.tv_usec = 0;
      if (setsockopt(pdhcpc->sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval)) < 0)
        {
          close(pdhcpc->sockfd);
          free(pdhcpc);
          return NULL;
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

int dhcpc_request(void *handle, struct dhcpc_state *presult)
{
  struct dhcpc_state_internal *pdhcpc = (struct dhcpc_state_internal *)handle;
  ssize_t result;
  int state;

  /* Loop until we receive the offer */

  do
    {
      state = STATE_SENDING;

      do
        {
          /* Send the command */

          if (send_discover(pdhcpc) < 0)
            {
              return ERROR;
            }

          /* Get the response */

          result = recv(pdhcpc->sockfd, pdhcpc->buffer, BUFFER_SIZE, 0);
          if (result >= 0)
            {
              if (parse_msg(pdhcpc, result, presult) == DHCPOFFER)
                {
                  state = STATE_OFFER_RECEIVED;
                }
            }
          else if (*get_errno_ptr() != EAGAIN)
            {
              /* An error other than a timeout was received */

              return ERROR;
            }
        }
      while (state != STATE_OFFER_RECEIVED);

      do
        {
          /* Send the request */

          if (send_request(pdhcpc, presult) < 0)
            {
              return ERROR;
            }

          /* Get the response */

          result = recv(pdhcpc->sockfd, pdhcpc->buffer, BUFFER_SIZE, 0);
          if (result >= 0)
            {
              if (parse_msg(pdhcpc, result, presult) == DHCPACK)
                {
                  state = STATE_CONFIG_RECEIVED;
                }
            }
          else if (*get_errno_ptr() != EAGAIN)
            {
              /* An error other than a timeout was received */

              return ERROR;
            }
        }
      while (state != STATE_CONFIG_RECEIVED);
    }
  while(state != STATE_CONFIG_RECEIVED);

  dbg("Got IP address %d.%d.%d.%d\n",
      (presult->ipaddr.s_addr >> 24 ) & 0xff,
      (presult->ipaddr.s_addr >> 16 ) & 0xff,
      (presult->ipaddr.s_addr >> 8  ) & 0xff,
      (presult->ipaddr.s_addr       ) & 0xff);
  dbg("Got netmask %d.%d.%d.%d\n",
      (presult->netmask.s_addr >> 24 ) & 0xff,
      (presult->netmask.s_addr >> 16 ) & 0xff,
      (presult->netmask.s_addr >> 8  ) & 0xff,
      (presult->netmask.s_addr       ) & 0xff);
  dbg("Got DNS server %d.%d.%d.%d\n",
      (presult->dnsaddr.s_addr >> 24 ) & 0xff,
      (presult->dnsaddr.s_addr >> 16 ) & 0xff,
      (presult->dnsaddr.s_addr >> 8  ) & 0xff,
      (presult->dnsaddr.s_addr       ) & 0xff);
  dbg("Got default router %d.%d.%d.%d\n",
      (presult->default_router.s_addr >> 24 ) & 0xff,
      (presult->default_router.s_addr >> 16 ) & 0xff,
      (presult->default_router.s_addr >> 8  ) & 0xff,
      (presult->default_router.s_addr       ) & 0xff);
  dbg("Lease expires in %ld seconds\n",
      ntohs(presult->lease_time[0])*65536ul + ntohs(presult->lease_time[1]));
  return OK;
}
