/****************************************************************************
 * libc/netdb/lib_dns.h
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

#ifndef __LIBC_NETDB_LIB_DNS_H
#define __LIBC_NETDB_LIB_DNS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/socket.h>
#include <netinet/in.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/dns.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* DNS client configuration **************************************************/

#ifndef CONFIG_NETDB_DNSCLIENT_ENTRIES
#  define CONFIG_NETDB_DNSCLIENT_ENTRIES 4
#endif

#define RESOLV_ENTRIES CONFIG_NETDB_DNSCLIENT_ENTRIES

#ifndef CONFIG_NETDB_DNSCLIENT_MAXRESPONSE
#  define CONFIG_NETDB_DNSCLIENT_MAXRESPONSE 96
#endif

#ifndef CONFIG_NETDB_DNSCLIENT_NAMESIZE
#  define CONFIG_NETDB_DNSCLIENT_NAMESIZE 32
#endif

#ifndef CONFIG_NETDB_DNSCLIENT_LIFESEC
#  define CONFIG_NETDB_DNSCLIENT_LIFESEC 3600
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

int dns_bind(void);

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
              FAR socklen_t *addrlen);

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
                    FAR socklen_t *addrlen);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __LIBC_NETDB_LIB_DNS_H */
