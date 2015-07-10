/****************************************************************************
 * libc/netdb/dns_resolver.c
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

#include <netinet/in.h>

#include <nuttx/net/dnsclient.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_sockfd = -1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_bind
 *
 * Description:
 *   Initialize the DNS resolver using an internal, share-able socket.
 *
 ****************************************************************************/

int dns_bind(void)
{
  return dns_bind_sock(&g_sockfd);
}

/****************************************************************************
 * Name: dns_query
 *
 * Description:
 *   Using the internal DNS resolver socket, look up the the 'hostname', and
 *   return its IP address in 'ipaddr'
 *
 * Returned Value:
 *   Returns zero (OK) if the query was successful.
 *
 ****************************************************************************/

int dns_query(FAR const char *hostname, FAR in_addr_t *ipaddr)
{
  return dns_query_sock(g_sockfd, hostname, ipaddr);
}

/****************************************************************************
 * Name: dns_whois
 *
 * Description:
 *   Get the binding for 'name' using the DNS server accessed via the DNS
 *   resolvers internal socket.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT_IPv6
int dns_whois(FAR const char *name, FAR struct sockaddr_in6 *addr)
#else
int dns_whois(FAR const char *name, FAR struct sockaddr_in *addr)
#endif
{
  return dns_whois_socket(g_sockfd, name, addr);
}
