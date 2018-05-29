/****************************************************************************
 * libs/libc/netdb/lib_dnsquery.c
 * DNS host name to IP address resolver.
 *
 * The DNS resolver functions are used to lookup a hostname and map it to a
 * numerical IP address.
 *
 *   Copyright (C) 2007, 2009, 2012, 2014-2018 Gregory Nutt. All rights
 *     reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/net/net.h>
#include <nuttx/net/dns.h>

#include "netdb/lib_dns.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The maximum number of retries when asking for a name */

#define MAX_RETRIES      8

/* Buffer sizes
 *
 * The SEND_BUFFER_SIZE depends the configured DNS name size,
 * sizeof(DNS query0 = Header (12 bytes) + DNS Name (Variable) +
 *                     Query type (2 bytes) + Query Class (2 bytes)
 */

#define SEND_BUFFER_SIZE (32 + CONFIG_NETDB_DNSCLIENT_NAMESIZE)
#define RECV_BUFFER_SIZE CONFIG_NETDB_DNSCLIENT_MAXRESPONSE

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dns_query_s
{
  int sd;                         /* DNS server socket */
  int result;                     /* Explanation of the failure */
  FAR const char *hostname;       /* Hostname to lookup */
  FAR struct sockaddr *addr;      /* Location to return host address */
  FAR socklen_t *addrlen;         /* Length of the address */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_seqno;           /* Sequence number of the next request */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_parse_name
 *
 * Description:
 *   Walk through a compact encoded DNS name and return the end of it.
 *
 * Input Parameters:
 *
 *   query    - A pointer to the starting byte of the name entry in the DNS
 *              response.
 *   queryend - A pointer to the byte after the last byte of the DNS response.
 *
 * Returned Value:
 *   Pointer to the first byte after the parsed name, or the value of
 *   `queryend` if the name did not fit into provided DNS response.
 *
 ****************************************************************************/

static FAR uint8_t *dns_parse_name(FAR uint8_t *query, FAR uint8_t *queryend)
{
  uint8_t n;

  while (query < queryend)
    {
      n = *query++;

      /* Check for a leading or trailing pointer.*/

      if (n & 0xC0)
        {
          /* Eat second pointer byte and terminate search */

          ninfo("Compressed answer\n");
          query++;
          break;
        }

      /* Check for final label with zero-length */

      if (!n)
        {
          break;
        }

      /* Eat non-empty label */

      query += n;
    }

  if (query >= queryend)
    {
      /* Always return `queryend` in case of errors */

      nerr("ERROR: DNS response is too short\n");
      query = queryend;
    }

  return query;
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
  uint8_t buffer[SEND_BUFFER_SIZE];
  uint8_t seqno;
  socklen_t addrlen;
  int errcode;
  int ret;
  int n;

  /* Increment the sequence number */

  dns_semtake();
  seqno = g_seqno++;
  dns_semgive();

  /* Initialize the request header */

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
  if (uaddr->ipv4.sin_family == AF_INET)
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
      nerr("ERROR: sendto failed: %d\n", errcode);
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
  FAR uint8_t *endofbuffer;
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

  ret = _NX_RECV(sd, buffer, RECV_BUFFER_SIZE, 0);
  if (ret < 0)
    {
      errcode = -_NX_GETERRNO(ret);
      nerr("ERROR: recv failed: %d\n", errcode);
      return errcode;
    }

  if (ret < 12)
    {
      /* DNS header can't fit in received data */

      nerr("ERROR: DNS response is too short\n");
      return -EILSEQ;
    }

  hdr         = (FAR struct dns_header_s *)buffer;
  endofbuffer = (FAR uint8_t*)buffer + ret;

  ninfo("ID %d\n", htons(hdr->id));
  ninfo("Query %d\n", hdr->flags1 & DNS_FLAG1_RESPONSE);
  ninfo("Error %d\n", hdr->flags2 & DNS_FLAG2_ERR_MASK);
  ninfo("Num questions %d, answers %d, authrr %d, extrarr %d\n",
        htons(hdr->numquestions), htons(hdr->numanswers),
        htons(hdr->numauthrr), htons(hdr->numextrarr));

  /* Check for error */

  if ((hdr->flags2 & DNS_FLAG2_ERR_MASK) != 0)
    {
      nerr("ERROR: DNS reported error: flags2=%02x\n", hdr->flags2);
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

#if defined(CONFIG_DEBUG_NET) && defined(CONFIG_DEBUG_INFO)
  {
    int d = 64;

    nameptr = dns_parse_name((uint8_t *)buffer + 12, endofbuffer);
    if (nameptr == endofbuffer)
      {
        return -EILSEQ;
      }

    nameptr += 4;

    for (; ; )
      {
        ninfo("%02X %02X %02X %02X %02X %02X %02X %02X \n",
              nameptr[0], nameptr[1], nameptr[2], nameptr[3],
              nameptr[4], nameptr[5], nameptr[6], nameptr[7]);

        nameptr += 8;
        d -= 8;
        if (d < 0)
          {
            break;
          }
      }
  }
#endif

  nameptr = dns_parse_name((uint8_t *)buffer + 12, endofbuffer);
  if (nameptr == endofbuffer)
    {
      return -EILSEQ;
    }

  nameptr += 4;

  for (; nanswers > 0; nanswers--)
    {
      /* Each answer starts with a name */

      nameptr = dns_parse_name(nameptr, endofbuffer);
      if (nameptr == endofbuffer)
        {
          return -EILSEQ;
        }

      ans = (FAR struct dns_answer_s *)nameptr;

      ninfo("Answer: type=%04x, class=%04x, ttl=%06x, length=%04x \n",
            htons(ans->type), htons(ans->class),
            (htons(ans->ttl[0]) << 16) | htons(ans->ttl[1]),
            htons(ans->len));

      /* Check for IPv4/6 address type and Internet class. Others are discarded. */

#ifdef CONFIG_NET_IPv4
      if (ans->type  == HTONS(DNS_RECTYPE_A) &&
          ans->class == HTONS(DNS_CLASS_IN) &&
          ans->len   == HTONS(4) &&
          nameptr + 10 + 4 <= endofbuffer)
        {
          ans->u.ipv4.s_addr = *(FAR uint32_t *)(nameptr + 10);

          ninfo("IPv4 address: %d.%d.%d.%d\n",
                (ans->u.ipv4.s_addr      ) & 0xff,
                (ans->u.ipv4.s_addr >> 8 ) & 0xff,
                (ans->u.ipv4.s_addr >> 16) & 0xff,
                (ans->u.ipv4.s_addr >> 24) & 0xff);

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
          ans->len   == HTONS(16) &&
          nameptr + 10 + 16 <= endofbuffer)
        {
          memcpy(&ans->u.ipv6.s6_addr, nameptr + 10, 16);

          ninfo("IPv6 address: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
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
 * Name: dns_query_callback
 *
 * Description:
 *   Using the DNS information and this DNS server address, look up the
 *   hostname.
 *
 * Input Parameters:
 *   arg      - Query arguements
 *   addr     - DNS name server address
 *   addrlen  - Length of the DNS name server address.
 *
 * Returned Value:
 *   Returns one (1) if the query was successful.  Zero is returned in all
 *   other cases.  The result field of the query structure is set to a
 *   negated errno value indicate the reason for the last failure (only).
 *
 ****************************************************************************/

static int dns_query_callback(FAR void *arg, FAR struct sockaddr *addr,
                              FAR socklen_t addrlen)
{
  FAR struct dns_query_s *query = (FAR struct dns_query_s *)arg;
  int retries;
  int ret;

  /* Loop while receive timeout errors occur and there are remaining retries */

  for (retries = 0; retries < 3; retries++)
    {
#ifdef CONFIG_NET_IPv4
      /* Is this an IPv4 address? */

      if (addr->sa_family == AF_INET)
        {
          /* Yes.. verify the address size */

          if (addrlen < sizeof(struct sockaddr_in))
            {
              /* Return zero to skip this address and try the next
               * namserver address in resolv.conf.
               */

              nerr("ERROR: Invalid IPv4 address size: %d\n", addrlen);
              query->result = -EINVAL;
              return 0;
            }

          /* Send the IPv4 query */

          ret = dns_send_query(query->sd, query->hostname,
                               (FAR union dns_server_u *)addr,
                               DNS_RECTYPE_A);
          if (ret < 0)
            {
              /* Return zero to skip this address and try the next
               * namserver address in resolv.conf.
               */

              nerr("ERROR: IPv4 dns_send_query failed: %d\n", ret);
              query->result = ret;
              return 0;
            }

          /* Obtain the IPv4 response */

          ret = dns_recv_response(query->sd, query->addr, query->addrlen);
          if (ret >= 0)
            {
              /* IPv4 response received successfully */

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
              /* Save the answer in the DNS cache */

              dns_save_answer(query->hostname, query->addr, *query->addrlen);
#endif
              /* Return 1 to indicate to (1) stop the traversal, and (2)
               * indicate that the address was found.
               */

              return 1;
            }

          /* Handle errors */

          nerr("ERROR: IPv4 dns_recv_response failed: %d\n", ret);

          if (ret == -EADDRNOTAVAIL)
            {
              /* The IPv4 address is not available.  Return zero to
               * continue the tranversal with the next nameserver
               * address in resolv.conf.
               */

              query->result = -EADDRNOTAVAIL;
              return 0;
            }
          else if (ret != -EAGAIN)
            {
              /* Some failure other than receive timeout occurred.  Return
               * zero to skip this address and try the next namserver
               * address in resolv.conf.
               */

              query->result = ret;
              return 0;
            }
        }
      else
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
      /* Is this an IPv4 address? */

      if (query->addr->sa_family == AF_INET6)
        {
          /* Yes.. verify the address size */

          if (addrlen < sizeof(struct sockaddr_in6))
            {
              /* Return zero to skip this address and try the next
               * namserver address in resolv.conf.
               */

              nerr("ERROR: Invalid IPv6 address size: %d\n", addrlen);
              query->result = -EINVAL;
              return 0;
            }

          /* Send the IPv6 query */

          ret = dns_send_query(query->sd, query->hostname,
                              (FAR union dns_server_u *)addr,
                               DNS_RECTYPE_AAAA);
          if (ret < 0)
            {
              /* Return zero to skip this address and try the next
               * namserver address in resolv.conf.
               */

              nerr("ERROR: IPv6 dns_send_query failed: %d\n", ret);
              query->result = ret;
              return 0;
            }

          /* Obtain the IPv6 response */

          ret = dns_recv_response(query->sd, query->addr, query->addrlen);
          if (ret >= 0)
            {
              /* IPv6 response received successfully */

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
              /* Save the answer in the DNS cache */

              dns_save_answer(query->hostname, query->addr, *query->addrlen);
#endif
              /* Return 1 to indicate to (1) stop the traversal, and (2)
               * indicate that the address was found.
               */

              return 1;
            }

          /* Handle errors */

          nerr("ERROR: IPv6 dns_recv_response failed: %d\n", ret);

          if (ret == -EADDRNOTAVAIL)
            {
              /* The IPv6 address is not available.  Return zero to
               * continue the tranversal with the next nameserver
               * address in resolv.conf.
               */

              query->result = -EADDRNOTAVAIL;
              return 0;
            }
          else if (ret != -EAGAIN)
            {
              /* Some failure other than receive timeout occurred.  Return
               * zero to skip this address and try the next namserver
               * address in resolv.conf.
               */

              query->result = ret;
              return 0;
            }
        }
      else
#endif
        {
           /* Unsupported address family. Return zero to continue the
            * tranversal with the next nameserver address in resolv.conf.
            */

           return 0;
        }
    }

  /* We tried three times and could not communicate with this nameserver.
   * Perhaps it is down?  Return zero to continue with the next address
   * in the resolv.conf file.
   */

  query->result = -ETIMEDOUT;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_query
 *
 * Description:
 *   Using the DNS resolver socket (sd), look up the 'hostname', and
 *   return its IP address in 'ipaddr'
 *
 * Input Parameters:
 *   sd       - The socket descriptor previously initialized by dsn_bind().
 *   hostname - The hostname string to be resolved.
 *   addr     - The location to return the IP address associated with the
 *     hostname
 *   addrlen  - On entry, the size of the buffer backing up the 'addr'
 *     pointer.  On return, this location will hold the actual size of
 *     the returned address.
 *
 * Returned Value:
 *   Returns zero (OK) if the query was successful.
 *
 ****************************************************************************/

int dns_query(int sd, FAR const char *hostname, FAR struct sockaddr *addr,
              FAR socklen_t *addrlen)
{
  FAR struct dns_query_s query;
  int ret;

  /* Set up the query info structure */

  query.sd       = sd;
  query.result   = -EADDRNOTAVAIL;
  query.hostname = hostname;
  query.addr     = addr;
  query.addrlen  = addrlen;

  /* Perform the query. dns_foreach_nameserver() will return:
   *
   *  1 - The query was successful.
   *  0 - Look up failed
   * <0 - Some other failure (?, shouldn't happen)
   */

  ret = dns_foreach_nameserver(dns_query_callback, &query);
  if (ret > 0)
    {
      /* The lookup was successful */

      ret = OK;
    }
  else if (ret == 0)
    {
      ret = query.result;
    }

  return ret;
}
