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
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <semaphore.h>
#include <time.h>
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

/* Use clock monotonic, if possible */

#ifdef CONFIG_CLOCK_MONOTONIC
#  define DNS_CLOCK CLOCK_MONOTONIC
#else
#  define DNS_CLOCK CLOCK_REALTIME
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This describes either an IPv4 or IPv6 address.  It is essentially a named
 * alternative to sockaddr_storage.
 */

union dns_server_u
{
  struct sockaddr     addr;        /* Common address representation */
#ifdef CONFIG_NET_IPv4
  struct sockaddr_in  ipv4;        /* IPv4 address */
#endif
#ifdef CONFIG_NET_IPv6
  struct sockaddr_in6 ipv6;        /* IPv6 address */
#endif
};

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
/* This described one entry in the cache of resolved hostnames */

struct dns_cache_s
{
#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  time_t              ctime;      /* Creation time */
#endif
  char                name[CONFIG_NETDB_DNSCLIENT_NAMESIZE];
  union dns_server_u  addr;       /* Resolved address */
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t   g_dns_sem;         /* Protects g_seqno and DNS cache */
static bool    g_dns_initialized; /* DNS data structures initialized */
static bool    g_dns_address;     /* We have the address of the DNS server */
#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
static uint8_t g_dns_head;        /* Head of the circular, DNS resolver cache */
static uint8_t g_dns_tail;        /* Tail of the circular, DNS resolver cache */
#endif
static uint8_t g_seqno;           /* Sequence number of the next request */

/* The DNS server address */

static union dns_server_u g_dns_server;

#ifdef CONFIG_NETDB_DNSSERVER_IPv6
/* This is the default IPv6 DNS server address */

static const uint16_t g_ipv6_hostaddr[8] =
{
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_1),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_2),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_3),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_4),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_5),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_6),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_7),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_8)
};
#endif

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
/* This is the DNS resolver cache */

static struct dns_cache_s g_dns_cache[CONFIG_NETDB_DNSCLIENT_ENTRIES];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_semtake
 *
 * Description:
 *   Take the DNS semaphore, ignoring errors do to the receipt of signals.
 *
 ****************************************************************************/

static void dns_semtake(void)
{
  int errcode = 0;
  int ret;

  do
    {
       ret = sem_wait(&g_dns_sem);
       if (ret < 0)
         {
           errcode = get_errno();
           DEBUGASSERT(errcode == EINTR);
         }
    }
  while (ret < 0 && errcode == EINTR);
}

/****************************************************************************
 * Name: dns_semgive
 *
 * Description:
 *   Release the DNS semaphore
 *
 ****************************************************************************/

#define dns_semgive() sem_post(&g_dns_sem)

/****************************************************************************
 * Name: dns_initialize
 *
 * Description:
 *   Make sure that the DNS client has been properly initialized for use.
 *
 ****************************************************************************/

static bool dns_initialize(void)
{
  /* Have DNS data structures been initialized? */

  if (!g_dns_initialized)
    {
      sem_init(&g_dns_sem, 0, 1);
      g_dns_initialized = true;
    }

  /* Has the DNS server IP address been assigned? */

  if (!g_dns_address)
    {
#if defined(CONFIG_NETDB_DNSSERVER_IPv4)
       struct sockaddr_in addr4;
       int ret;

       /* No, configure the default IPv4 DNS server address */

       addr4.sin_family      = AF_INET;
       addr4.sin_port        = DNS_DEFAULT_PORT;
       addr4.sin_addr.s_addr = HTONL(CONFIG_NETDB_DNSSERVER_IPv4ADDR);

       ret = dns_setserver((FAR struct sockaddr *)&addr4,
                           sizeof(struct sockaddr_in));
       if (ret < 0)
         {
           return false;
         }

#elif defined(CONFIG_NETDB_DNSSERVER_IPv6)
       struct sockaddr_in6 addr6;
       int ret;

       /* No, configure the default IPv6 DNS server address */

       addr6.sin6_family = AF_INET6;
       addr6.sin6_port   = DNS_DEFAULT_PORT;
       memcpy(addr6.sin6_addr.s6_addr, g_ipv6_hostaddr, 16);

       ret = dns_setserver((FAR struct sockaddr *)&addr6,
                           sizeof(struct sockaddr_in6));
       if (ret < 0)
         {
           return false;
         }

#else
       /* No, then we are not ready to perform DNS queries */

       return false;
#endif
    }

  return true;
}

/****************************************************************************
 * Name: dns_save_answer
 *
 * Description:
 *   Same the last resolved hostname in the DNS cache
 *
 * Input Parameters:
 *   hostname - The hostname string to be cached.
 *   addr     - The IP address associated with the hostname
 *   addrlen  - The size of the of the IP address.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
static void dns_save_answer(FAR const char *hostname,
                            FAR const struct sockaddr *addr,
                            socklen_t addrlen)
{
  FAR struct dns_cache_s *entry;
#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  struct timespec now;
#endif
  int next;
  int ndx;

  /* Get exclusive access to the DNS cache */

  dns_semtake();

  /* Get the index to the new head of the list */

  ndx  = g_dns_head;
  next = ndx + 1;
  if (next >= CONFIG_NETDB_DNSCLIENT_ENTRIES)
    {
      next = 0;
    }

  /* If the next head pointer would match the tail index, then increment
   * the tail index, discarding the oldest mapping in the cache.
   */

  if (next == g_dns_tail)
    {
      int tmp = g_dns_tail + 1;
      if (tmp >= CONFIG_NETDB_DNSCLIENT_ENTRIES)
        {
          tmp = 0;
        }

      g_dns_tail = tmp;
    }

  /* Save the answer in the cache */

  entry = &g_dns_cache[ndx];

#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  /* Get the current time, using CLOCK_MONOTONIC if possible */

  (void)clock_settime(DNS_CLOCK, &now);
  entry->ctime = (time_t)now.tv_sec;
#endif

  strncpy(entry->name, hostname, CONFIG_NETDB_DNSCLIENT_NAMESIZE);
  memcpy(&entry->addr.addr, addr, addrlen);

  /* Save the updated head index */

  g_dns_head = next;
  dns_semgive();
}
#endif

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

    for (; ; )
      {
        ndbg("%02X %02X %02X %02X %02X %02X %02X %02X \n",
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

  /* Has the DNS client been properly initialized? */

  if (!dns_initialize())
    {
      ndbg("ERROR: DNS client has not been initialized\n");
      return -EDESTADDRREQ;
    }

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
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
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

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
              /* Save the answer in the DNS cache */

              dns_save_answer(hostname, addr, *addrlen);
#endif
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

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
              /* Save the answer in the DNS cache */

              dns_save_answer(hostname, addr, *addrlen);
#endif
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
      *pport = HTONS(DNS_DEFAULT_PORT);
    }

  /* We now have a valid DNS address */

  g_dns_address = true;
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

/****************************************************************************
 * Name: dns_find_answer
 *
 * Description:
 *   Check if we already have the resolved hostname address in the cache.
 *
 * Input Parameters:
 *   hostname - The hostname string to be resolved.
 *   addr     - The location to return the IP address associated with the
 *     hostname
 *   addrlen  - On entry, the size of the buffer backing up the 'addr'
 *     pointer.  On return, this location will hold the actual size of
 *     the returned address.
 *
 * Returned Value:
 *   If the host name was successfully found in the DNS name resolution
 *   cache, zero (OK) will be returned.  Otherwise, some negated errno
 *   value will be returned, typically -ENOENT meaning that the hostname
 *   was not found in the cache.
 *
 ****************************************************************************/

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
int dns_find_answer(FAR const char *hostname, FAR struct sockaddr *addr,
                    FAR socklen_t *addrlen)
{
  FAR struct dns_cache_s *entry;
#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  struct timespec now;
  uint32_t elapsed;
  int ret;
#endif
  int next;
  int ndx;

  /* If DNS not initialized, no need to proceed */

  if (!g_dns_initialized)
    {
      ndbg("ERROR: DNS not initialized yet\n");
      return -EAGAIN;
    }

  /* Get exclusive access to the DNS cache */

  dns_semtake();

#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  /* Get the current time, using CLOCK_MONOTONIC if possible */

  ret = clock_settime(DNS_CLOCK, &now);
#endif

  /* REVISIT: This is not thread safe */

  for (ndx = g_dns_tail; ndx != g_dns_head; ndx = next)
    {
      entry = &g_dns_cache[ndx];

      /* Advance the index for the next time through the loop, handling
       * wrapping to the beginning of the circular buffer.
       */

      next = ndx + 1;
      if (next >= CONFIG_NETDB_DNSCLIENT_ENTRIES)
        {
          next = 0;
        }

#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
      /* Check if this entry has expired
       * REVISIT: Does not this calculation assume that the sizeof(time_t)
       * is equal to the sizeof(uint32_t)?
       */

      elapsed = (uint32_t)now.tv_sec - (uint32_t)entry->ctime;
      if (ret >= 0 && elapsed > CONFIG_NETDB_DNSCLIENT_LIFESEC)
        {
          /* This entry has expired.  Increment the tail index to exclude
           * this entry on future traversals.
           */

          g_dns_tail = next;
        }
      else
#endif
        {
          /* The entry has not expired, check for a name match.  Notice that
           * because the names are truncated to CONFIG_NETDB_DNSCLIENT_NAMESIZE,
           * this has the possibility of aliasing two names and returning
           * the wrong entry from the cache.
           */

          if (strncmp(hostname, entry->name, CONFIG_NETDB_DNSCLIENT_NAMESIZE) == 0)
            {
              socklen_t inlen;

              /* We have a match.  Return the resolved host address */

#ifdef CONFIG_NET_IPv4
              if (entry->addr.addr.sa_family == AF_INET)
#ifdef CONFIG_NET_IPv6
#endif
                {
                   inlen = sizeof(struct sockaddr_in);
                }
#endif

#ifdef CONFIG_NET_IPv6
              else
#ifdef CONFIG_NET_IPv4
#endif
                {
                   inlen = sizeof(struct sockaddr_in6);
                }
#endif
              /* Make sure that the address will fit in the caller-provided
               * buffer.
               */

              if (*addrlen < inlen)
                {
                  ret = -ERANGE;
                  goto errout_with_sem;
                }

              /* Return the address information */

              memcpy(addr, &entry->addr.addr, inlen);
              *addrlen = inlen;

              dns_semgive();
              return OK;
            }
        }
    }

  ret = -ENOENT;

errout_with_sem:
  dns_semgive();
  return ret;
}
#endif
