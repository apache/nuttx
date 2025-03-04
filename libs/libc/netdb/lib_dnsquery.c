/****************************************************************************
 * libs/libc/netdb/lib_dnsquery.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-License-Identifier: 2007, 2009, 2012, 2014-2018 Gregory Nutt.
 * SPDX-License-Identifier:  2002-2003, Adam Dunkels. All rights reserved.
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
 * SPDX-FileContributor: Adam Dunkels <adam@dunkels.com>
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
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <arpa/inet.h>

#include <nuttx/lib/lib.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/dns.h>

#include "netdb/lib_dns.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Buffer sizes
 *
 * The SEND_BUFFER_SIZE depends the configured DNS name size:
 *
 * sizeof(DNS query0 = Header (12 bytes) + DNS Name (Variable) +
 *                     Query type (2 bytes) + Query Class (2 bytes)
 *
 * sizeof(DNS Name) = Encoded length (1 byte) + Maximum length of name +
 *                    NUL-terminator (1 byte)
 */

#define SEND_BUFFER_SIZE  (16 + CONFIG_NETDB_DNSCLIENT_NAMESIZE + 2)
#define RECV_BUFFER_SIZE  CONFIG_NETDB_DNSCLIENT_MAXRESPONSE
#define QUERY_BUFFER_SIZE MAX(SEND_BUFFER_SIZE, RECV_BUFFER_SIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dns_query_s
{
  int result;                     /* Explanation of the failure */
  FAR const char *hostname;       /* Hostname to lookup */
  FAR union dns_addr_u *addr;     /* Location to return host address */
  FAR int *naddr;                 /* Number of returned addresses */
  uint32_t ttl;                   /* Time to Live, unit:s */
};

/* Query info to check response against. */

struct dns_query_info_s
{
  uint16_t id;                                     /* Query ID */
  uint16_t rectype;                                /* Queried record type */
  uint16_t qnamelen;                               /* Queried hostname length */
  char qname[CONFIG_NETDB_DNSCLIENT_NAMESIZE + 2]; /* Queried hostname in
                                                    * encoded format + NUL */
};

struct dns_query_data_s
{
  struct dns_query_s query;
  struct dns_query_info_s qinfo;
  uint8_t buffer[QUERY_BUFFER_SIZE]; /* Buffer to hold request & response */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stream_send
 *
 * Description:
 *   A wrapper of send() to deal with short results for SOCK_STREAM socket.
 *
 * Input Parameters:
 *   Same as send().
 *
 * Returned Value:
 *   Same as send().
 *
 ****************************************************************************/

static ssize_t stream_send(int fd, FAR const void *buf, size_t len)
{
  ssize_t total = 0;

  while (len > 0)
    {
      ssize_t ret = send(fd, buf, len, 0);
      if (ret == -1)
        {
          if (total == 0)
            {
              total = ret;
            }
          break;
        }

      buf = (FAR const uint8_t *)buf + len;
      len -= ret;
      total += ret;
    }

  return total;
}

/****************************************************************************
 * Name: stream_recv
 *
 * Description:
 *   A wrapper of recv() to deal with short results for SOCK_STREAM socket.
 *
 * Input Parameters:
 *   Same as recv().
 *
 * Returned Value:
 *   Same as recv().
 *
 ****************************************************************************/

static ssize_t stream_recv(int fd, FAR void *buf, size_t len)
{
  ssize_t total = 0;

  while (len > 0)
    {
      ssize_t ret = recv(fd, buf, len, 0);
      if (ret == 0)
        {
          /* the peer closed the connection */

          set_errno(EMSGSIZE);
          ret = -1;
        }

      if (ret == -1)
        {
          if (total == 0)
            {
              total = ret;
            }
          break;
        }

      buf = (FAR uint8_t *)buf + len;
      len -= ret;
      total += ret;
    }

  return total;
}

/****************************************************************************
 * Name: stream_send_record
 *
 * Description:
 *   Send a DNS message over SOCK_STREAM socket.
 *
 * Input Parameters:
 *   Same as send().
 *
 * Returned Value:
 *   Same as send().
 *
 ****************************************************************************/

static ssize_t stream_send_record(int fd, FAR const void *buf, size_t len)
{
  ssize_t ret;
  uint8_t reclen[2];

  /* RFC 1035
   * 4.2.2. TCP usage
   *
   * > The message is prefixed with a two byte length field which
   * > gives the message length, excluding the two byte length field.
   */

  reclen[0] = (uint8_t)(len >> 8);
  reclen[1] = (uint8_t)len;
  ret = stream_send(fd, reclen, sizeof(reclen));
  if (ret < sizeof(reclen))
    {
      return -1;
    }

  return stream_send(fd, buf, len);
}

/****************************************************************************
 * Name: stream_recv_record
 *
 * Description:
 *   Receive a DNS message over SOCK_STREAM socket.
 *
 * Input Parameters:
 *   Same as recv().
 *
 * Returned Value:
 *   Same as recv().
 *
 ****************************************************************************/

static ssize_t stream_recv_record(int fd, FAR void *buf, size_t len)
{
  size_t rlen;
  ssize_t ret;
  uint8_t reclen[2];

  /* RFC 1035
   * 4.2.2. TCP usage
   *
   * > The message is prefixed with a two byte length field which
   * > gives the message length, excluding the two byte length field.
   */

  ret = stream_recv(fd, reclen, sizeof(reclen));
  if (ret < sizeof(reclen))
    {
      if (ret >= 0)
        {
          set_errno(EMSGSIZE);
        }

      return -1;
    }

  rlen = ((uint16_t)reclen[0] << 8) + reclen[1];
  if (rlen > len)
    {
      nerr("ERROR: DNS response (%zu bytes) didn't fit "
           "the buffer. (%zu bytes) You may need to bump "
           "CONFIG_NETDB_DNSCLIENT_MAXRESPONSE\n", rlen, len);
      set_errno(EMSGSIZE);
      return -1;
    }

  ret = stream_recv(fd, buf, rlen);
  if (ret != rlen)
    {
      if (ret >= 0)
        {
          set_errno(EMSGSIZE);
        }

      return -1;
    }

  return ret;
}

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
 *   queryend - A pointer to the byte after the last byte of the response.
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

      /* Check for a leading or trailing pointer */

      if ((n & 0xc0) != 0)
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
 * Name: dns_alloc_id
 *
 * Description:
 *   Gets a new ID for query. Function manufactures a reasonably unique ID,
 *   but we accept rare collisions, since they are harmless when resolver
 *   clients retry failed DNS few times.
 *
 ****************************************************************************/

static inline uint16_t dns_alloc_id(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)ts.tv_nsec + ((uint32_t)ts.tv_nsec >> 16);
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
                          FAR union dns_addr_u *uaddr, uint16_t rectype,
                          FAR struct dns_query_info_s *qinfo,
                          FAR uint8_t *buffer,
                          bool stream)
{
  FAR struct dns_header_s *hdr;
  FAR uint8_t *dest;
  FAR uint8_t *nptr;
  FAR char *qname;
  FAR char *qptr;
  FAR const char *src;
  uint16_t id;
  socklen_t addrlen;
  int ret;
  int len;
  int n;

  /* Get a new ID for query */

  id = dns_alloc_id();

  /* Initialize the request header */

  hdr               = (FAR struct dns_header_s *)buffer;
  memset(hdr, 0, sizeof(*hdr));
  hdr->id           = HTONS(id);
  hdr->flags1       = DNS_FLAG1_RD;
  hdr->numquestions = HTONS(1);

  /* Convert hostname into suitable query format.
   *
   * There is space for CONFIG_NETDB_DNSCLIENT_NAMESIZE
   * plus one pre-pended name length and NUL-terminator
   * (other pre-pended name lengths replace dots).
   */

  src   = name - 1;
  dest  = buffer + sizeof(*hdr);
  qname = qinfo->qname;
  len   = 0;
  do
    {
      /* Copy the name string to both query and saved info. */

      src++;
      nptr = dest++;
      qptr = qname++;
      len++;

      for (n = 0;
           *src != '.' && *src != '\0' &&
           len <= CONFIG_NETDB_DNSCLIENT_NAMESIZE;
           src++)
        {
          *dest++  = *(FAR uint8_t *)src;
          *qname++ = *(FAR uint8_t *)src;
          n++;
          len++;
        }

      /* Pre-pend the name length */

      *nptr = n;
      *qptr = n;
    }
  while (*src != '\0' && len <= CONFIG_NETDB_DNSCLIENT_NAMESIZE);

  /* Add NUL termination */

  *dest++  = '\0';
  *qname++ = '\0';

  /* Store name length to saved info */

  DEBUGASSERT(len <= CONFIG_NETDB_DNSCLIENT_NAMESIZE + 1);
  DEBUGASSERT(qname - qinfo->qname == len + 1);
  qinfo->qnamelen = len;

  /* Add DNS record type, and DNS class */

  *dest++ = (rectype >> 8);        /* DNS record type (big endian) */
  *dest++ = (rectype & 0xff);
  *dest++ = (DNS_CLASS_IN >> 8);   /* DNS record class (big endian) */
  *dest++ = (DNS_CLASS_IN & 0xff);

  qinfo->rectype = HTONS(rectype);
  qinfo->id      = hdr->id;

  /* Send the request */

  if (uaddr->addr.sa_family == AF_INET)
    {
      addrlen = sizeof(struct sockaddr_in);
    }
  else
    {
      addrlen = sizeof(struct sockaddr_in6);
    }

  ret = connect(sd, &uaddr->addr, addrlen);
  if (ret < 0)
    {
      ret = -get_errno();
      nerr("ERROR: connect failed: %d\n", ret);
      return ret;
    }

  if (stream)
    {
      ret = stream_send_record(sd, buffer, dest - buffer);
    }
  else
    {
      ret = send(sd, buffer, dest - buffer, 0);
    }

  if (ret < 0)
    {
      ret = -get_errno();
      nerr("ERROR: sendto failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: dns_recv_response
 *
 * Description:
 *   Called when new UDP data arrives
 *
 * Returned Value:
 *   Returns number of valid IP address responses.  Negated errno value is
 *   returned in all other cases.
 *
 ****************************************************************************/

static int dns_recv_response(int sd, FAR union dns_addr_u *addr, int naddr,
                             FAR struct dns_query_info_s *qinfo,
                             FAR uint32_t *ttl, FAR uint8_t *buffer,
                             bool stream, bool *should_try_stream)
{
  FAR uint8_t *nameptr;
  FAR uint8_t *namestart;
  FAR uint8_t *endofbuffer;
  FAR struct dns_answer_s *ans;
  FAR struct dns_header_s *hdr;
  FAR struct dns_question_s *que;
  uint16_t nquestions;
  uint16_t nanswers;
  uint16_t temp;
  int naddr_read;
  int ret;

  if (naddr <= 0)
    {
      return -ERANGE;
    }

  /* Receive the response */

  if (stream)
    {
      ret = stream_recv_record(sd, buffer, RECV_BUFFER_SIZE);
    }
  else
    {
      ret = recv(sd, buffer, RECV_BUFFER_SIZE, 0);
    }

  if (ret < 0)
    {
      ret = -get_errno();
      nerr("ERROR: recv failed: %d\n", ret);
      return ret;
    }

  if (ret < sizeof(*hdr))
    {
      /* DNS header can't fit in received data */

      nerr("ERROR: DNS response is too short\n");
      return -EILSEQ;
    }

  hdr         = (FAR struct dns_header_s *)buffer;
  endofbuffer = buffer + ret;

  ninfo("ID %d\n", NTOHS(hdr->id));
  ninfo("Query %d\n", hdr->flags1 & DNS_FLAG1_RESPONSE);
  ninfo("Error %d\n", hdr->flags2 & DNS_FLAG2_ERR_MASK);
  ninfo("Num questions %d, answers %d, authrr %d, extrarr %d\n",
        NTOHS(hdr->numquestions), NTOHS(hdr->numanswers),
        NTOHS(hdr->numauthrr), NTOHS(hdr->numextrarr));

  /* Check for error */

  if ((hdr->flags1 & DNS_FLAG1_TRUNC) != 0)
    {
      /* RFC 2181
       * 9. The TC (truncated) header bit
       *
       * > When a DNS client receives a reply with TC set,
       * > it should ignore that response, and query again,
       * > using a mechanism, such as a TCP connection,
       * > that will permit larger replies.
       */

      if (stream)
        {
          nerr("ERROR: DNS response truncated on stream socket.\n");
          return -EPROTO;
        }

      ninfo("ERROR: DNS response truncated. "
            "Falling back to stream socket.\n");
      *should_try_stream = true;
      return -EAGAIN;
    }

  if ((hdr->flags2 & DNS_FLAG2_ERR_MASK) != 0)
    {
      nerr("ERROR: DNS reported error: flags2=%02x\n", hdr->flags2);
      return -EPROTO;
    }

  /* Check for matching ID. */

  if (hdr->id != qinfo->id)
    {
      nerr("ERROR: DNS wrong response ID (expected %d, got %d)\n",
           NTOHS(qinfo->id), NTOHS(hdr->id));
      return -EBADMSG;
    }

  /* We only care about the question(s) and the answers. The authrr
   * and the extrarr are simply discarded.
   */

  nquestions = NTOHS(hdr->numquestions);
  nanswers   = NTOHS(hdr->numanswers);

  /* We only ever send queries with one question. */

  if (nquestions != 1)
    {
      nerr("ERROR: DNS wrong number of questions %d\n", nquestions);
      return -EBADMSG;
    }

  /* Skip the name in the answer, but do check that it
   * matches against the name in the question.
   */

  namestart = buffer + sizeof(*hdr);
  nameptr   = dns_parse_name(namestart, endofbuffer);
  if (nameptr == endofbuffer)
    {
      return -EILSEQ;
    }

  /* Since dns_parse_name() skips any pointer bytes,
   * we cannot compare for equality here.
   */

  if (nameptr - namestart < qinfo->qnamelen)
    {
      nerr("ERROR: DNS response name wrong length\n");
      return -EBADMSG;
    }

  /* qname is NUL-terminated and we must include NUL to the comparison. */

  if (memcmp(namestart, qinfo->qname, qinfo->qnamelen + 1) != 0)
    {
      nerr("ERROR: DNS response with wrong name\n");
      return -EBADMSG;
    }

  /* Validate query type and class */

  que = (FAR struct dns_question_s *)nameptr;

  /* N.B. Unaligned access may occur here */

  temp = HTONS(DNS_CLASS_IN);
  if (memcmp(&que->type, &qinfo->rectype, sizeof(uint16_t)) != 0 ||
      memcmp(&que->class, &temp, sizeof(uint16_t)) != 0)
    {
      nerr("ERROR: DNS response with wrong question\n");
      return -EBADMSG;
    }

  /* Skip over question */

  nameptr += sizeof(struct dns_question_s);

  ret = OK;
  naddr_read = 0;

  for (; nanswers > 0; nanswers--)
    {
      /* Each answer starts with a name */

      nameptr = dns_parse_name(nameptr, endofbuffer);
      if (nameptr == endofbuffer)
        {
          ret = -EILSEQ;
          nwarn("Further parse returned %d\n", ret);
          break;
        }

      ans = (FAR struct dns_answer_s *)nameptr;

      ninfo("Answer: type=%04x, class=%04x, ttl=%06x, length=%04x\n",
            NTOHS(ans->type), NTOHS(ans->class),
            (NTOHS(ans->ttl[0]) << 16) | NTOHS(ans->ttl[1]),
            NTOHS(ans->len));
      if (ttl)
        {
          *ttl = (NTOHS(ans->ttl[0]) << 16) | NTOHS(ans->ttl[1]);
        }

      /* Check for IPv4/6 address type and Internet class. Others are
       * discarded.
       */

#ifdef CONFIG_NET_IPv4
      if (ans->type  == HTONS(DNS_RECTYPE_A) &&
          ans->class == HTONS(DNS_CLASS_IN) &&
          ans->len   == HTONS(4) &&
          nameptr + 10 + 4 <= endofbuffer)
        {
          FAR struct sockaddr_in *inaddr;

          nameptr += 10 + 4;

          ninfo("IPv4 address: %u.%u.%u.%u\n",
                ip4_addr1(ans->u.ipv4.s_addr),
                ip4_addr2(ans->u.ipv4.s_addr),
                ip4_addr3(ans->u.ipv4.s_addr),
                ip4_addr4(ans->u.ipv4.s_addr));

          inaddr                  = &addr[naddr_read].ipv4;
          inaddr->sin_family      = AF_INET;
          inaddr->sin_port        = 0;
          inaddr->sin_addr.s_addr = ans->u.ipv4.s_addr;

          if (++naddr_read >= naddr)
            {
              ret = -ERANGE;
              break;
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
          FAR struct sockaddr_in6 *inaddr;

          nameptr += 10 + 16;

          ninfo("IPv6 address: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
                NTOHS(ans->u.ipv6.s6_addr16[0]),
                NTOHS(ans->u.ipv6.s6_addr16[1]),
                NTOHS(ans->u.ipv6.s6_addr16[2]),
                NTOHS(ans->u.ipv6.s6_addr16[3]),
                NTOHS(ans->u.ipv6.s6_addr16[4]),
                NTOHS(ans->u.ipv6.s6_addr16[5]),
                NTOHS(ans->u.ipv6.s6_addr16[6]),
                NTOHS(ans->u.ipv6.s6_addr16[7]));

          inaddr                  = &addr[naddr_read].ipv6;
          inaddr->sin6_family     = AF_INET6;
          inaddr->sin6_port       = 0;
          memcpy(inaddr->sin6_addr.s6_addr, ans->u.ipv6.s6_addr, 16);

          if (++naddr_read >= naddr)
            {
              ret = -ERANGE;
              break;
            }
        }
      else
#endif
        {
          nameptr = nameptr + 10 + NTOHS(ans->len);
        }
    }

  if (naddr_read == 0 && ret == OK)
    {
      ret = -EADDRNOTAVAIL;
    }

  return naddr_read > 0 ? naddr_read : ret;
}

/****************************************************************************
 * Name: dns_query_error
 *
 * Description:
 *   Displays information about dns query errors
 *
 * Input Parameters:
 *   prompt  - Error description.
 *   ret     - Error code.
 *   uaddr   - DNS name server address.
 *
 ****************************************************************************/

static void dns_query_error(FAR const char *prompt, int ret,
                            FAR union dns_addr_u *uaddr)
{
  char addrstr[INET6_ADDRSTRLEN];

#ifdef CONFIG_NET_IPv4
  if (uaddr->addr.sa_family == AF_INET)
    {
      inet_ntop(AF_INET, &uaddr->ipv4.sin_addr, addrstr, INET6_ADDRSTRLEN);
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (uaddr->addr.sa_family == AF_INET6)
    {
      inet_ntop(AF_INET6, &uaddr->ipv6.sin6_addr, addrstr, INET6_ADDRSTRLEN);
    }
  else
#endif
    {
      strlcpy(addrstr, "Unknown address", sizeof(addrstr));
    }

  nerr("%s: %d, server address: %s\n", prompt, ret, addrstr);
}

/****************************************************************************
 * Name: dns_query_callback
 *
 * Description:
 *   Using the DNS information and this DNS server address, look up the
 *   hostname.
 *
 * Input Parameters:
 *   arg      - Query arguments
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
  FAR struct dns_query_data_s *qdata = arg;
  FAR struct dns_query_s      *query = &qdata->query;
  int next = 0;
  int retries;
  int ret;
  int sd;
  bool stream = false;

  /* Loop while receive timeout errors occur and there are remaining
   * retries.
   */

  for (retries = 0; retries < CONFIG_NETDB_DNSCLIENT_RETRIES; retries++)
    {
      bool should_try_stream;

try_stream:
#ifdef CONFIG_NET_IPv6
      if (dns_is_queryfamily(AF_INET6))
        {
          /* Send the IPv6 query */

          sd = dns_bind(addr->sa_family, stream);
          if (sd < 0)
            {
              query->result = sd;
              return 0;
            }

          ret = dns_send_query(sd, query->hostname,
                               (FAR union dns_addr_u *)addr,
                               DNS_RECTYPE_AAAA, &qdata->qinfo,
                               qdata->buffer, stream);
          if (ret < 0)
            {
              dns_query_error("ERROR: IPv6 dns_send_query failed",
                              ret, (FAR union dns_addr_u *)addr);
              query->result = ret;
            }
          else
            {
              /* Obtain the IPv6 response */

              should_try_stream = false;
              ret = dns_recv_response(sd, &query->addr[next],
                                      CONFIG_NETDB_MAX_IPv6ADDR,
                                      &qdata->qinfo,
                                      &query->ttl, qdata->buffer,
                                      stream, &should_try_stream);
              if (ret >= 0)
                {
                  next += ret;
                }
              else
                {
                  if (!stream && should_try_stream)
                    {
                      stream = true;
                      goto try_stream; /* Don't consume retry count */
                    }

                  dns_query_error("ERROR: IPv6 dns_recv_response failed",
                                  ret, (FAR union dns_addr_u *)addr);
                  query->result = ret;
                }
            }

          close(sd);
        }
#endif

#ifdef CONFIG_NET_IPv4
      if (dns_is_queryfamily(AF_INET))
        {
          /* Send the IPv4 query */

          sd = dns_bind(addr->sa_family, stream);
          if (sd < 0)
            {
              query->result = sd;
              return 0;
            }

          ret = dns_send_query(sd, query->hostname,
                               (FAR union dns_addr_u *)addr,
                               DNS_RECTYPE_A, &qdata->qinfo, qdata->buffer,
                               stream);
          if (ret < 0)
            {
              dns_query_error("ERROR: IPv4 dns_send_query failed",
                              ret, (FAR union dns_addr_u *)addr);
              query->result = ret;
            }
          else
            {
              /* Obtain the IPv4 response */

              if (next >= *query->naddr)
                {
                  next = *query->naddr / 2;
                }

              should_try_stream = false;
              ret = dns_recv_response(sd, &query->addr[next],
                                      CONFIG_NETDB_MAX_IPv4ADDR,
                                      &qdata->qinfo,
                                      &query->ttl, qdata->buffer,
                                      stream, &should_try_stream);
              if (ret >= 0)
                {
                  next += ret;
                }
              else
                {
                  if (!stream && should_try_stream)
                    {
                      stream = true;
                      goto try_stream; /* Don't consume retry count */
                    }

                  dns_query_error("ERROR: IPv4 dns_recv_response failed",
                                  ret, (FAR union dns_addr_u *)addr);
                  query->result = ret;
                }
            }

          close(sd);
        }
#endif /* CONFIG_NET_IPv4 */

      if (next > 0)
        {
#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
          /* Save the answer in the DNS cache */

          dns_save_answer(query->hostname, query->addr, next, query->ttl);
#endif
          /* Return 1 to indicate to (1) stop the traversal, and (2)
           * indicate that the address was found.
           */

          *query->naddr = next;
          return 1;
        }
      else if (query->result != -EAGAIN)
        {
          break;
        }
    }

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
 *   hostname - The hostname string to be resolved.
 *   addr     - The location to return the IP addresses associated with the
 *     hostname.
 *   naddr    - On entry, the count of addresses backing up the 'addr'
 *     pointer.  On return, this location will hold the actual count of
 *     the returned addresses.
 *
 * Returned Value:
 *   Returns zero (OK) if the query was successful.
 *
 ****************************************************************************/

int dns_query(FAR const char *hostname, FAR union dns_addr_u *addr,
              FAR int *naddr)
{
  FAR struct dns_query_data_s *qdata = lib_malloc(sizeof(*qdata));
  int ret;

  if (qdata == NULL)
    {
      return -ENOMEM;
    }

  /* Set up the query info structure */

  qdata->query.result   = -EADDRNOTAVAIL;
  qdata->query.hostname = hostname;
  qdata->query.addr     = addr;
  qdata->query.naddr    = naddr;

  /* Perform the query. dns_foreach_nameserver() will return:
   *
   *  1 - The query was successful.
   *  0 - Look up failed
   * <0 - Some other failure (?, shouldn't happen)
   */

  ret = dns_foreach_nameserver(dns_query_callback, qdata);
  if (ret > 0)
    {
      /* The lookup was successful */

      ret = OK;
    }
  else if (ret == 0)
    {
      ret = qdata->query.result;
    }

  /* Free the query data */

  lib_free(qdata);

  return ret;
}
