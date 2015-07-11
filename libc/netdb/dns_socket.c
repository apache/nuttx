/****************************************************************************
 * libc/netdb/dns_socket.c
 * DNS host name to IP address resolver.
 *
 * The uIP DNS resolver functions are used to lookup a hostname and
 * map it to a numerical IP address. It maintains a list of resolved
 * hostnames that can be queried. New hostnames can be resolved using the
 * dns_whois() function.
 *
 *   Copyright (C) 2007, 2009, 2012, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based heavily on portions of uIP:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2002-2003, Adam Dunkels.
 *   All rights reserved.
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
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/socket.h>
#include <sys/time.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <nuttx/net/dnsclient.h>
#include <apps/netutils/netlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NETDB_DNSCLIENT_ENTRIES
#  define RESOLV_ENTRIES 4
#else /* CONFIG_NETDB_DNSCLIENT_ENTRIES */
#  define RESOLV_ENTRIES CONFIG_NETDB_DNSCLIENT_ENTRIES
#endif /* CONFIG_NETDB_DNSCLIENT_ENTRIES */

#ifndef NULL
#  define NULL (void *)0
#endif /* NULL */

/* The maximum number of retries when asking for a name */

#define MAX_RETRIES 8

#define DNS_FLAG1_RESPONSE        0x80
#define DNS_FLAG1_OPCODE_STATUS   0x10
#define DNS_FLAG1_OPCODE_INVERSE  0x08
#define DNS_FLAG1_OPCODE_STANDARD 0x00
#define DNS_FLAG1_AUTHORATIVE     0x04
#define DNS_FLAG1_TRUNC           0x02
#define DNS_FLAG1_RD              0x01
#define DNS_FLAG2_RA              0x80
#define DNS_FLAG2_ERR_MASK        0x0f
#define DNS_FLAG2_ERR_NONE        0x00
#define DNS_FLAG2_ERR_NAME        0x03

#define SEND_BUFFER_SIZE 64

#ifdef CONFIG_NETDB_DNSCLIENT_MAXRESPONSE
#  define RECV_BUFFER_SIZE CONFIG_NETDB_DNSCLIENT_MAXRESPONSE
#else
#  define RECV_BUFFER_SIZE 96
#endif

#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
#  define ADDRLEN sizeof(struct sockaddr_in6)
#else
#  define ADDRLEN sizeof(struct sockaddr_in)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The DNS message header */

struct dns_hdr
{
  uint16_t id;
  uint8_t  flags1;
  uint8_t  flags2;
  uint16_t numquestions;
  uint16_t numanswers;
  uint16_t numauthrr;
  uint16_t numextrarr;
};

/* The DNS answer message structure */

struct dns_answer
{
  /* DNS answer record starts with either a domain name or a pointer
   * to a name already present somewhere in the packet.
   */

  uint16_t type;
  uint16_t class;
  uint16_t ttl[2];
  uint16_t len;
#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
  struct in6_addr ipaddr;
#else
  struct in_addr ipaddr;
#endif
};

struct namemap
{
  uint8_t state;
  uint8_t tmr;
  uint8_t retries;
  uint8_t seqno;
  uint8_t err;
  char name[32];
#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
  struct in6_addr ipaddr;
#else
  struct in_addr ipaddr;
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_seqno;
#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
static struct sockaddr_in6 g_dnsserver;
#else
static struct sockaddr_in g_dnsserver;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_parse_name
 *
 * Description:
 *   Walk through a compact encoded DNS name and return the end of it.
 *
 ****************************************************************************/

static FAR unsigned char *dns_parse_name(FAR unsigned char *query)
{
  unsigned char n;

  do
    {
      n = *query++;

      while (n > 0)
        {
          ++query;
          --n;
        }
    }
  while (*query != 0);

  return query + 1;
}

/****************************************************************************
 * Name: dns_send_query
 *
 * Description:
 *   Runs through the list of names to see if there are any that have
 *   not yet been queried and, if so, sends out a query.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
static int dns_send_query(int sockfd, FAR const char *name,
                          FAR struct sockaddr_in6 *addr)
#else
static int dns_send_query(int sockfd, FAR const char *name,
                          FAR struct sockaddr_in *addr)
#endif
{
  register FAR struct dns_hdr *hdr;
  FAR char *query;
  FAR char *nptr;
  FAR const char *nameptr;
  uint8_t seqno = g_seqno++;
  static unsigned char endquery[] = {0, 0, 1, 0, 1};
  char buffer[SEND_BUFFER_SIZE];
  int n;

  hdr               = (FAR struct dns_hdr *)buffer;
  memset(hdr, 0, sizeof(struct dns_hdr));
  hdr->id           = htons(seqno);
  hdr->flags1       = DNS_FLAG1_RD;
  hdr->numquestions = HTONS(1);
  query             = buffer + 12;

  /* Convert hostname into suitable query format. */

  nameptr = name - 1;
  do
   {
     nameptr++;
     nptr = query++;
     for (n = 0; *nameptr != '.' && *nameptr != 0; nameptr++)
       {
         *query++ = *nameptr;
         n++;
       }

     *nptr = n;
   }
  while (*nameptr != 0);

  memcpy(query, endquery, 5);

  return sendto(sockfd, buffer, query + 5 - buffer,
                0, (struct sockaddr*)addr, ADDRLEN);
}

/****************************************************************************
 * Name: dns_recv_response
 *
 * Description:
 *   Called when new UDP data arrives
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
#  error "Not implemented"
#else
static int dns_recv_response(int sockfd, FAR struct sockaddr_in *addr)
#endif
{
  FAR unsigned char *nameptr;
  char buffer[RECV_BUFFER_SIZE];
  FAR struct dns_answer *ans;
  FAR struct dns_hdr *hdr;
#if 0 /* Not used */
  uint8_t nquestions;
#endif
  uint8_t nanswers;
  int ret;

  /* Receive the response */

  ret = recv(sockfd, buffer, RECV_BUFFER_SIZE, 0);
  if (ret < 0)
    {
      return ret;
    }

  hdr = (FAR struct dns_hdr *)buffer;

  ndbg("ID %d\n", htons(hdr->id));
  ndbg("Query %d\n", hdr->flags1 & DNS_FLAG1_RESPONSE);
  ndbg("Error %d\n", hdr->flags2 & DNS_FLAG2_ERR_MASK);
  ndbg("Num questions %d, answers %d, authrr %d, extrarr %d\n",
       htons(hdr->numquestions), htons(hdr->numanswers),
       htons(hdr->numauthrr), htons(hdr->numextrarr));

  /* Check for error. If so, call callback to inform */

  if ((hdr->flags2 & DNS_FLAG2_ERR_MASK) != 0)
    {
      return ERROR;
    }

  /* We only care about the question(s) and the answers. The authrr
   * and the extrarr are simply discarded.
   */

#if 0 /* Not used */
  nquestions = htons(hdr->numquestions);
#endif
  nanswers   = htons(hdr->numanswers);

  /* Skip the name in the question. TODO: This should really be
   * checked against the name in the question, to be sure that they
   * match.
   */

#ifdef CONFIG_DEBUG_NET
  {
    int d = 64;
    nameptr = dns_parse_name((unsigned char *)buffer + 12) + 4;

    for (;;)
      {
        ndbg("%02X %02X %02X %02X %02X %02X %02X %02X \n",
             nameptr[0],nameptr[1],nameptr[2],nameptr[3],
             nameptr[4],nameptr[5],nameptr[6],nameptr[7]);

        nameptr += 8;
        d -= 8;
        if (d < 0)
          {
            break;
          }
      }
  }
#endif

  nameptr = dns_parse_name((unsigned char *)buffer + 12) + 4;

  for (; nanswers > 0; nanswers--)
    {
      /* The first byte in the answer resource record determines if it
       * is a compressed record or a normal one.
       */

      if (*nameptr & 0xc0)
        {
          /* Compressed name. */

          nameptr += 2;
          ndbg("Compressed answer\n");
        }
      else
        {
          /* Not compressed name. */

          nameptr = dns_parse_name(nameptr);
        }

      ans = (struct dns_answer *)nameptr;
      ndbg("Answer: type %x, class %x, ttl %x, length %x \n", /* 0x%08X\n", */
           htons(ans->type), htons(ans->class),
           (htons(ans->ttl[0]) << 16) | htons(ans->ttl[1]),
           htons(ans->len) /* , ans->ipaddr.s_addr */);

      /* Check for IP address type and Internet class. Others are discarded. */

      if (ans->type  == HTONS(1) &&
          ans->class == HTONS(1) &&
          ans->len   == HTONS(4))
        {
          ans->ipaddr.s_addr = *(FAR uint32_t *)(nameptr + 10);

          ndbg("IP address %d.%d.%d.%d\n",
               (ans->ipaddr.s_addr       ) & 0xff,
               (ans->ipaddr.s_addr >> 8  ) & 0xff,
               (ans->ipaddr.s_addr >> 16 ) & 0xff,
               (ans->ipaddr.s_addr >> 24 ) & 0xff);

          /* TODO: we should really check that this IP address is the one
           * we want.
           */

          addr->sin_addr.s_addr = ans->ipaddr.s_addr;
          return OK;
        }
      else
        {
          nameptr = nameptr + 10 + htons(ans->len);
        }
    }

  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_bind
 *
 * Description:
 *   Initialize the DNS resolver using the caller provided socket.
 *
 ****************************************************************************/

int dns_bind(FAR int *sockfd)
{
  struct timeval tv;
  int ret;

  /* If the socket is already open, then close it now */

  if (*sockfd >= 0)
    {
      dns_free(sockfd);
    }

  /* Create a new socket */

  *sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  if (*sockfd < 0)
    {
      ndbg("ERROR: socket() failed: %d\n", errno);
      return ERROR;
    }

  /* Set up a receive timeout */

  tv.tv_sec  = 30;
  tv.tv_usec = 0;

  ret = setsockopt(*sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv,
                   sizeof(struct timeval));

  if (ret < 0)
    {
      ndbg("ERROR: setsockopt() failed: %d\n", errno);
      close(*sockfd);
      *sockfd = -1;
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: dns_free
 *
 * Description:
 *   Release the DNS resolver by closing the socket.
 *
 ****************************************************************************/

int dns_free(FAR int *sockfd)
{
  if (*sockfd >= 0)
    {
      close(*sockfd);
      *sockfd = -1;
    }

  return OK;
}

/****************************************************************************
 * Name: dns_query
 *
 * Description:
 *   Using the DNS resolver socket (sockfd), look up the the 'hostname', and
 *   return its IP address in 'ipaddr'
 *
 * Returned Value:
 *   Returns zero (OK) if the query was successful.
 *
 ****************************************************************************/

int dns_query(int sockfd, FAR const char *hostname, FAR in_addr_t *ipaddr)
{
#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
  struct sockaddr_in6 addr;
#else
  struct sockaddr_in addr;
#endif

  /* First check if the host is an IP address. */

  if (!netlib_ipaddrconv(hostname, (uint8_t*)ipaddr))
    {
      /* 'host' does not point to a valid address string.  Try to resolve
       *  the host name to an IP address.
       */

      if (dns_whois(sockfd, hostname, &addr) < 0)
        {
          /* Needs to set the errno here */

          return ERROR;
        }

      /* Save the host address -- Needs fixed for IPv6 */

      *ipaddr = addr.sin_addr.s_addr;
  }

  return OK;
}

/****************************************************************************
 * Name: dns_setserver
 *
 * Description:
 *   Configure which DNS server to use for queries
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
void dns_setserver(FAR const struct in6_addr *dnsserver)
#else
void dns_setserver(FAR const struct in_addr *dnsserver)
#endif
{
#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
  g_dnsserver.sin6_family = AF_INET6;
  g_dnsserver.sin6_port   = HTONS(53);

  memcpy(&g_dnsserver.sin6_addr, dnsserver, ADDRLEN);
#else
  g_dnsserver.sin_family  = AF_INET;
  g_dnsserver.sin_port    = HTONS(53);

  g_dnsserver.sin_addr.s_addr = dnsserver->s_addr;
#endif
}

/****************************************************************************
 * Name: dns_getserver
 *
 * Description:
 *   Obtain the currently configured DNS server.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
void dns_getserver(FAR struct in6_addr *dnsserver)
#else
void dns_getserver(FAR struct in_addr *dnsserver)
#endif
{
#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
  memcpy(dnsserver, &g_dnsserver.sin6_addr, ADDRLEN);
#else
  dnsserver->s_addr = g_dnsserver.sin_addr.s_addr;
#endif
}

/****************************************************************************
 * Name: dns_whois
 *
 * Description:
 *   Get the binding for 'name' using the DNS server accessed via 'sockfd'
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
int dns_whois(int sockfd, FAR const char *name,
                     FAR struct sockaddr_in6 *addr)
#else
int dns_whois(int sockfd, FAR const char *name,
                     FAR struct sockaddr_in *addr)
#endif
{
  int retries;
  int ret;

  /* Loop while receive timeout errors occur and there are remaining retries */

  for (retries = 0; retries < 3; retries++)
    {
      ret = dns_send_query(sockfd, name, &g_dnsserver);
      if (ret < 0)
        {
          return ERROR;
        }

      ret = dns_recv_response(sockfd, addr);
      if (ret >= 0)
        {
          /* Response received successfully */

          return OK;
        }

      else if (errno != EAGAIN)
        {
          /* Some failure other than receive timeout occurred */

          return ERROR;
        }
    }

  return ERROR;
}
