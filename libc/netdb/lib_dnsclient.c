/****************************************************************************
 * libc/netdb/lib_dnsclient.c
 * DNS host name to IP address resolver.
 *
 * The uIP DNS resolver functions are used to lookup a hostname and
 * map it to a numerical IP address.
 *
 *   Copyright (C) 2007, 2009, 2012, 2014-2015 Gregory Nutt. All rights reserved.
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

#include <nuttx/net/dns.h>

#include "netdb/lib_dns.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The maximum number of retries when asking for a name */

#define MAX_RETRIES      8

/* Buffer sizes */

#define SEND_BUFFER_SIZE 64
#define RECV_BUFFER_SIZE CONFIG_NETDB_DNSCLIENT_MAXRESPONSE

/****************************************************************************
 * Private Types
 ****************************************************************************/

union dns_server_u
{
  struct sockaddr     addr;
#ifdef CONFIG_NET_IPv4
  struct sockaddr_in  ipv4;
#endif
#ifdef CONFIG_NET_IPv6
  struct sockaddr_in6 ipv6;
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_seqno;
static union dns_server_u g_dns_server;

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

static FAR uint8_t *dns_parse_name(FAR uint8_t *query)
{
  uint8_t n;

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

static int dns_send_query(int sd, FAR const char *name,
                          FAR union dns_server_u *uaddr, uint16_t rectype)
{
  register FAR struct dns_header_s *hdr;
  FAR uint8_t *dest;
  FAR uint8_t *nptr;
  FAR const char *src;
  uint8_t seqno = g_seqno++;
  uint8_t buffer[SEND_BUFFER_SIZE];
  socklen_t addrlen;
  int errcode;
  int ret;
  int n;

  hdr               = (FAR struct dns_header_s *)buffer;
  memset(hdr, 0, sizeof(struct dns_header_s));
  hdr->id           = htons(seqno);
  hdr->flags1       = DNS_FLAG1_RD;
  hdr->numquestions = HTONS(1);
  dest              = buffer + 12;

  /* Convert hostname into suitable query format. */

  src = name - 1;
  do
   {
      /* Copy the name string */

      src++;
      nptr = dest++;
      for (n = 0; *src != '.' && *src != 0; src++)
        {
          *dest++ = *(uint8_t *)src;
          n++;
        }

      /* Pre-pend the name length */

      *nptr = n;
    }
  while (*src != '\0');

  /* Add NUL termination, DNS record type, and DNS class */

  *dest++ = '\0';                  /* NUL termination */
  *dest++ = (rectype >> 8);        /* DNS record type (big endian) */
  *dest++ = (rectype & 0xff);
  *dest++ = (DNS_CLASS_IN >> 8);   /* DNS record class (big endian) */
  *dest++ = (DNS_CLASS_IN & 0xff);

  /* Send the request */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (uaddr->addr.sa_family == AF_INET)
#endif
    {
      addrlen = sizeof(struct sockaddr_in);
    }
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      addrlen = sizeof(struct sockaddr_in6);
    }
#endif

  ret = sendto(sd, buffer, dest - buffer, 0, &uaddr->addr, addrlen);

  /* Return the negated errno value on sendto failure */

  if (ret < 0)
    {
      errcode = get_errno();
      ndbg("ERROR: sendto failed: %d\n", errcode);
      return -errcode;
    }

  return OK;
}

/****************************************************************************
 * Name: dns_recv_response
 *
 * Description:
 *   Called when new UDP data arrives
 *
 ****************************************************************************/

static int dns_recv_response(int sd, FAR struct sockaddr *addr,
                             FAR socklen_t *addrlen)
{
  FAR uint8_t *nameptr;
  char buffer[RECV_BUFFER_SIZE];
  FAR struct dns_answer_s *ans;
  FAR struct dns_header_s *hdr;
#if 0 /* Not used */
  uint8_t nquestions;
#endif
  uint8_t nanswers;
  int errcode;
  int ret;

  /* Receive the response */

  ret = recv(sd, buffer, RECV_BUFFER_SIZE, 0);
  if (ret < 0)
    {
      errcode = get_errno();
      ndbg("ERROR: recv failed: %d\n", errcode);
      return -errcode;
    }

  hdr = (FAR struct dns_header_s *)buffer;

  nvdbg("ID %d\n", htons(hdr->id));
  nvdbg("Query %d\n", hdr->flags1 & DNS_FLAG1_RESPONSE);
  nvdbg("Error %d\n", hdr->flags2 & DNS_FLAG2_ERR_MASK);
  nvdbg("Num questions %d, answers %d, authrr %d, extrarr %d\n",
        htons(hdr->numquestions), htons(hdr->numanswers),
        htons(hdr->numauthrr), htons(hdr->numextrarr));

  /* Check for error */

  if ((hdr->flags2 & DNS_FLAG2_ERR_MASK) != 0)
    {
      ndbg("ERROR: DNS reported error: flags2=%02x\n", hdr->flags2);
      return -EPROTO;
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
    nameptr = dns_parse_name((uint8_t *)buffer + 12) + 4;

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

  nameptr = dns_parse_name((uint8_t *)buffer + 12) + 4;

  for (; nanswers > 0; nanswers--)
    {
      /* The first byte in the answer resource record determines if it
       * is a compressed record or a normal one.
       */

      if (*nameptr & 0xc0)
        {
          /* Compressed name. */

          nameptr += 2;
          nvdbg("Compressed answer\n");
        }
      else
        {
          /* Not compressed name. */

          nameptr = dns_parse_name(nameptr);
        }

      ans = (FAR struct dns_answer_s *)nameptr;

      nvdbg("Answer: type=%04x, class=%04x, ttl=%06x, length=%04x \n",
            htons(ans->type), htons(ans->class),
            (htons(ans->ttl[0]) << 16) | htons(ans->ttl[1]),
            htons(ans->len));

      /* Check for IPv4/6 address type and Internet class. Others are discarded. */

#ifdef CONFIG_NET_IPv4
      if (ans->type  == HTONS(DNS_RECTYPE_A) &&
          ans->class == HTONS(DNS_CLASS_IN) &&
          ans->len   == HTONS(4))
        {
          ans->u.ipv4.s_addr = *(FAR uint32_t *)(nameptr + 10);

          nvdbg("IPv4 address: %d.%d.%d.%d\n",
                (ans->u.ipv4.s_addr       ) & 0xff,
                (ans->u.ipv4.s_addr >> 8  ) & 0xff,
                (ans->u.ipv4.s_addr >> 16 ) & 0xff,
                (ans->u.ipv4.s_addr >> 24 ) & 0xff);

          if (*addrlen >= sizeof(struct sockaddr_in))
            {
              FAR struct sockaddr_in *inaddr;

              inaddr                  = (FAR struct sockaddr_in *)addr;
              inaddr->sin_family      = AF_INET;
              inaddr->sin_port        = 0;
              inaddr->sin_addr.s_addr = ans->u.ipv4.s_addr;

              *addrlen = sizeof(struct sockaddr_in);
              return OK;
            }
          else
            {
              return -ERANGE;
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (ans->type  == HTONS(DNS_RECTYPE_AAAA) &&
          ans->class == HTONS(DNS_CLASS_IN) &&
          ans->len   == HTONS(16))
        {
          memcpy(&ans->u.ipv6.s6_addr, nameptr + 10, 16);

          nvdbg("IPv6 address: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
                htons(ans->u.ipv6.s6_addr[7]),  htons(ans->u.ipv6.s6_addr[6]),
                htons(ans->u.ipv6.s6_addr[5]),  htons(ans->u.ipv6.s6_addr[4]),
                htons(ans->u.ipv6.s6_addr[3]),  htons(ans->u.ipv6.s6_addr[2]),
                htons(ans->u.ipv6.s6_addr[1]),  htons(ans->u.ipv6.s6_addr[0]));

          if (*addrlen >= sizeof(struct sockaddr_in6))
            {
              FAR struct sockaddr_in6 *inaddr;

              inaddr                  = (FAR struct sockaddr_in6 *)addr;
              inaddr->sin6_family      = AF_INET;
              inaddr->sin6_port        = 0;
              memcpy(inaddr->sin6_addr.s6_addr, ans->u.ipv6.s6_addr, 16);

              *addrlen = sizeof(struct sockaddr_in6);
              return OK;
            }
          else
            {
              return -ERANGE;
            }
        }
      else
#endif
        {
          nameptr = nameptr + 10 + htons(ans->len);
        }
    }

  return -EADDRNOTAVAIL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_bind
 *
 * Description:
 *   Initialize the DNS resolver and return a socket bound to the DNS name
 *   server.  The name server was previously selected via dns_server().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, the bound, non-negative socket descriptor is returned.  A
 *   negated errno value is returned on any failure.
 *
 ****************************************************************************/

int dns_bind(void)
{
  struct timeval tv;
  int errcode;
  int sd;
  int ret;

  /* Create a new socket */

  sd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sd < 0)
    {
      errcode = get_errno();
      ndbg("ERROR: socket() failed: %d\n", errcode);
      return -errcode;
    }

  /* Set up a receive timeout */

  tv.tv_sec  = 30;
  tv.tv_usec = 0;

  ret = setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval));
  if (ret < 0)
    {
      errcode = get_errno();
      ndbg("ERROR: setsockopt() failed: %d\n", errcode);
      close(sd);
      return -errcode;
    }

  return sd;
}

/****************************************************************************
 * Name: dns_query
 *
 * Description:
 *   Using the DNS resolver socket (sd), look up the the 'hostname', and
 *   return its IP address in 'ipaddr'
 *
 * Returned Value:
 *   Returns zero (OK) if the query was successful.
 *
 ****************************************************************************/

int dns_query(int sd, FAR const char *hostname, FAR struct sockaddr *addr,
              FAR socklen_t *addrlen)
{
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv4)
  int noipv4 = false;
  int noipv6 = false;
#endif
  int retries;
  int ret;

  /* Loop while receive timeout errors occur and there are remaining retries */

  for (retries = 0; retries < 3; retries++)
    {
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      /* If we know that the IPv4 address is not available, then don't try
       * again.
       */

      if (!noipv4)
#endif
        {
          /* Send the IPv4 query */

          ret = dns_send_query(sd, hostname, &g_dns_server, DNS_RECTYPE_A);
          if (ret < 0)
            {
              ndbg("ERROR: IPv4 dns_send_query failed: %d\n", ret);
              return ret;
            }

          /* Obtain the IPv4 response */

          ret = dns_recv_response(sd, addr, addrlen);
          if (ret >= 0)
            {
              /* IPv4 response received successfully */

              return OK;
            }

          /* Handle errors */

          ndbg("ERROR: IPv4 dns_recv_response failed: %d\n", ret);

#ifdef CONFIG_NET_IPv6
          if (ret != -EADDRNOTAVAIL)
            {
              /* The IPv4 address is not available. */

              noipv4 = true;
              if (noipv6)
                {
                  /* Neither address is available */

                  return ret;
                }
            }
          else
#endif
          if (ret != -EAGAIN)
            {
              /* Some failure other than receive timeout occurred */

              return ret;
            }
        }
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      /* If we know that the IPv4 address is not available, then don't try
       * again.
       */

      if (!noipv6)
#endif
        {
          /* Send the IPv6 query */

          ret = dns_send_query(sd, hostname, &g_dns_server,
                               DNS_RECTYPE_AAAA);
          if (ret < 0)
            {
              ndbg("ERROR: IPv6 dns_send_query failed: %d\n", ret);
              return ret;
            }

          /* Obtain the IPv6 response */

          ret = dns_recv_response(sd, addr, addrlen);
          if (ret >= 0)
            {
              /* IPv6 response received successfully */

              return OK;
            }

          /* Handle errors */

          ndbg("ERROR: IPv6 dns_recv_response failed: %d\n", ret);

#ifdef CONFIG_NET_IPv4
          if (ret != -EADDRNOTAVAIL)
            {
              /* The IPv6 address is not available. */

              noipv6 = true;
              if (noipv4)
                {
                  /* Neither address is available */

                  return ret;
                }
            }
          else
#endif
          if (ret != -EAGAIN)
            {
              /* Some failure other than receive timeout occurred */

              return ret;
            }
        }
#endif
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: dns_setserver
 *
 * Description:
 *   Configure which DNS server to use for queries
 *
 ****************************************************************************/

int dns_setserver(FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR uint16_t *pport;
  size_t copylen;

  DEBUGASSERT(addr != NULL);

  /* Copy the new server IP address into our private global data structure */

#ifdef CONFIG_NET_IPv4
  /* Check for an IPv4 address */

  if (addr->sa_family == AF_INET)
    {
      /* Set up for the IPv4 address copy */

      copylen = sizeof(struct sockaddr_in);
      pport   = &g_dns_server.ipv4.sin_port;
    }
  else
#endif

#ifdef CONFIG_NET_IPv6
  /* Check for an IPv6 address */

  if (addr->sa_family == AF_INET6)
    {
      /* Set up for the IPv6 address copy */

      copylen = sizeof(struct sockaddr_in6);
      pport   = &g_dns_server.ipv6.sin6_port;
    }
  else
#endif
    {
      nvdbg("ERROR: Unsupported family: %d\n", addr->sa_family);
      return -ENOSYS;
    }

  /* Copy the IP address */

  if (addrlen < copylen)
    {
      nvdbg("ERROR: Invalid addrlen %ld for family %d\n",
            (long)addrlen, addr->sa_family);
      return -EINVAL;
    }

  memcpy(&g_dns_server.addr, addr, copylen);

  /* A port number of zero means to use the default DNS server port number */

  if (*pport == 0)
    {
      *pport = HTONS(53);
    }

  return OK;
}

/****************************************************************************
 * Name: dns_getserver
 *
 * Description:
 *   Obtain the currently configured DNS server.
 *
 ****************************************************************************/

int dns_getserver(FAR struct sockaddr *addr, FAR socklen_t *addrlen)
{
  socklen_t copylen;

  DEBUGASSERT(addr != NULL && addrlen != NULL);

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (g_dns_server.addr.sa_family == AF_INET)
#endif
    {
      copylen = sizeof(struct sockaddr_in);
    }
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      copylen = sizeof(struct sockaddr_in6);
    }
#endif

  /* Copy the DNS server address to the caller-provided buffer */

  if (copylen > *addrlen)
    {
      nvdbg("ERROR: addrlen %ld too small for address family %d\n",
            (long)addrlen, g_dns_server.addr.sa_family);
      return -EINVAL;
    }

  memcpy(addr, &g_dns_server.addr, copylen);
  *addrlen = copylen;
  return OK;
}
