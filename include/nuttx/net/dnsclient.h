/****************************************************************************
 * include/nuttx/net/dnsclient.h
 * DNS resolver code header file.
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Inspired by/based on uIP logic by Adam Dunkels:
 *
 *   Copyright (c) 2002-2003, Adam Dunkels. All rights reserved.
 *   Author Adam Dunkels <adam@dunkels.com>
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

#ifndef __INCLUDE_NUTTX_NET_DNSCLIENT_H
#define __INCLUDE_NUTTX_NET_DNSCLIENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <netinet/in.h>

#include <nuttx/net/netconfig.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 /* If both IPv4 and IPv6 are enabled, the DNS client can support only one or
 * the other.
 */

#if !defined(CONFIG_NETDB_DNSCLIENT_IPv4) && \
    !defined(CONFIG_NETDB_DNSCLIENT_IPv6)
#  ifdef CONFIG_NET_IPv6
#     define CONFIG_NETDB_DNSCLIENT_IPv6 1
#  else
#     define CONFIG_NETDB_DNSCLIENT_IPv4 1
#  endif
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: dns_bind
 *
 * Description:
 *   Initialize the DNS resolver using the caller provided socket.
 *
 ****************************************************************************/

int dns_bind(FAR int *sockfd);

/****************************************************************************
 * Name: dns_free
 *
 * Description:
 *   Release the DNS resolver by closing the socket.
 *
 ****************************************************************************/

int dns_free(FAR int *sockfd);

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

int dns_query(int sockfd, FAR const char *hostname, FAR in_addr_t *ipaddr);

/****************************************************************************
 * Name: dns_setserver
 *
 * Description:
 *   Configure which DNS server to use for queries
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
void dns_setserver(FAR const struct in6_addr *dnsserver);
#else
void dns_setserver(FAR const struct in_addr *dnsserver);
#endif

/****************************************************************************
 * Name: dns_getserver
 *
 * Description:
 *   Obtain the currently configured DNS server.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
void dns_getserver(FAR struct in6_addr *dnsserver);
#else
void dns_getserver(FAR struct in_addr *dnsserver);
#endif

/****************************************************************************
 * Name: dns_whois
 *
 * Description:
 *   Get the binding for 'name' using the DNS server accessed via 'sockfd'
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
int  dns_whois(int sockfd, FAR const char *name,
               FAR struct sockaddr_in6 *addr);
#else
int  dns_whois(int sockfd, FAR const char *name,
               FAR struct sockaddr_in *addr);
#endif

/****************************************************************************
 * Name: dns_gethostip
 *
 * Descriptions:
 *   Combines the operations of dns_bind(), dns_query(), and
 *   dns_free() to obtain the the IP address ('ipaddr') associated with
 *   the 'hostname' in one operation.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
int dns_gethostip(FAR const char *hostname, FAR struct in6_addr *ipaddr);
#else
int dns_gethostip(FAR const char *hostname, FAR in_addr_t *ipaddr);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NET_DNSCLIENT_H */
