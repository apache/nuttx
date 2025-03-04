/****************************************************************************
 * libs/libc/netdb/lib_dns.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __LIBS_LIBC_NETDB_LIB_DNS_H
#define __LIBS_LIBC_NETDB_LIB_DNS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <sys/socket.h>
#include <netinet/in.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/dns.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DNS client configuration *************************************************/

#ifndef CONFIG_NETDB_DNSCLIENT_ENTRIES
#  define CONFIG_NETDB_DNSCLIENT_ENTRIES 4
#endif

#ifndef CONFIG_NETDB_DNSCLIENT_MAXRESPONSE
#  define CONFIG_NETDB_DNSCLIENT_MAXRESPONSE 96
#endif

#ifndef CONFIG_NETDB_DNSCLIENT_NAMESIZE
#  define CONFIG_NETDB_DNSCLIENT_NAMESIZE 32
#endif

#ifndef CONFIG_NETDB_DNSCLIENT_LIFESEC
#  define CONFIG_NETDB_DNSCLIENT_LIFESEC 3600
#endif

#ifndef CONFIG_NETDB_RESOLVCONF_PATH
#  define CONFIG_NETDB_RESOLVCONF_PATH "/etc/resolv.conf"
#endif

#ifndef CONFIG_NETDB_DNSSERVER_NAMESERVERS
#  define CONFIG_NETDB_DNSSERVER_NAMESERVERS 1
#endif

#define DNS_MAX_ADDRSTR   48
#define DNS_MAX_LINE      64
#define NETDB_DNS_KEYWORD "nameserver"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes either an IPv4 or IPv6 address.  It is essentially a named
 * alternative to sockaddr_storage.
 */

union dns_addr_u
{
  struct sockaddr     addr;        /* Common address representation */
#ifdef CONFIG_NET_IPv4
  struct sockaddr_in  ipv4;        /* IPv4 address */
#endif
#ifdef CONFIG_NET_IPv6
  struct sockaddr_in6 ipv6;        /* IPv6 address */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifndef CONFIG_NETDB_RESOLVCONF
/* The DNS server addresses */

EXTERN union dns_addr_u g_dns_servers[];
EXTERN uint8_t g_dns_nservers;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: dns_lock
 *
 * Description:
 *   Take the DNS mutex, ignoring errors due to the receipt of signals.
 *
 ****************************************************************************/

void dns_lock(void);

/****************************************************************************
 * Name: dns_unlock
 *
 * Description:
 *   Release the DNS mutex
 *
 ****************************************************************************/

void dns_unlock(void);

/****************************************************************************
 * Name: dns_breaklock
 *
 * Description:
 *   Break the DNS lock
 *
 ****************************************************************************/

void dns_breaklock(FAR unsigned int *count);

/****************************************************************************
 * Name: dns_restorelock
 *
 * Description:
 *   Restore the DNS lock
 *
 ****************************************************************************/

void dns_restorelock(unsigned int count);

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

int dns_bind(sa_family_t family, bool stream);

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
              FAR int *naddr);

/****************************************************************************
 * Name: dns_is_queryfamily
 *
 * Description:
 *   Determine if the specified address family is available for DNS query.
 *
 * Input Parameters:
 *   family - The address family. AF_INET or AF_INET6 is specified.
 *
 * Returned Value:
 *   Returns true if the address family specified in the family argument
 *   is available.
 *
 ****************************************************************************/

bool dns_is_queryfamily(sa_family_t family);

/****************************************************************************
 * Name: dns_save_answer
 *
 * Description:
 *   Save the last resolved hostname in the DNS cache
 *
 * Input Parameters:
 *   hostname - The hostname string to be cached.
 *   addr     - The IP addresses associated with the hostname.
 *   naddr    - The count of the IP addresses.
 *   ttl      - The TTL of the IP addresses.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
void dns_save_answer(FAR const char *hostname,
                     FAR const union dns_addr_u *addr, int naddr,
                     uint32_t ttl);
#endif

/****************************************************************************
 * Name: dns_clear_answer
 *
 * Description:
 *   Clear the resolved hostname in the DNS cache
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
void dns_clear_answer(void);
#endif

/****************************************************************************
 * Name: dns_find_answer
 *
 * Description:
 *   Check if we already have the resolved hostname address in the cache.
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
 *   If the host name was successfully found in the DNS name resolution
 *   cache, zero (OK) will be returned.  Otherwise, some negated errno
 *   value will be returned, typically -ENOENT meaning that the hostname
 *   was not found in the cache.
 *
 ****************************************************************************/

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
int dns_find_answer(FAR const char *hostname, FAR union dns_addr_u *addr,
                    FAR int *naddr);
#endif

/****************************************************************************
 * Name: dns_notify_nameserver
 ****************************************************************************/

void dns_notify_nameserver(FAR const struct sockaddr *addr,
                           socklen_t addrlen);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __LIBS_LIBC_NETDB_LIB_DNS_H */
