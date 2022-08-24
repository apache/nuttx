/****************************************************************************
 * libs/libc/netdb/lib_gethostbyaddrr.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>
#include <nuttx/net/loopback.h>

#include "netdb/lib_netdb.h"

#ifdef CONFIG_LIBC_NETDB

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This is the layout of the caller provided memory area */

struct hostent_info_s
{
  int       hi_addrtypes[CONFIG_NETDB_MAX_IPADDR];
  int       hi_lengths[CONFIG_NETDB_MAX_IPADDR];
  FAR char *hi_addrlist[CONFIG_NETDB_MAX_IPADDR + 1];
  char      hi_data[1];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_lo_ipv4match
 *
 * Description:
 *   Check if the address is the reserved IPv4 address for the local
 *   loopback device.
 *
 * Input Parameters:
 *   addr - The address of the host to find.
 *   len - The length of the address
 *   type - The type of the address
 *
 * Returned Value:
 *  True if the address is the IPv4 local loopback address.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_LOOPBACK) && defined(CONFIG_NET_IPv4)
static bool lib_lo_ipv4match(FAR const void *addr, socklen_t len, int type)
{
  FAR struct in_addr *ipv4addr;

  if (type == AF_INET && len >= sizeof(struct in_addr))
    {
      ipv4addr = (FAR struct in_addr *)addr;
      return net_ipv4addr_maskcmp(ipv4addr->s_addr,
                                  g_lo_ipv4addr,
                                  g_lo_ipv4mask);
    }

  return false;
}
#endif

/****************************************************************************
 * Name: lib_lo_ipv6match
 *
 * Description:
 *   Check if the address is the reserved IPv6 address for the local
 *   loopback device.
 *
 * Input Parameters:
 *   addr - The address of the host to find.
 *   len - The length of the address
 *   type - The type of the address
 *
 * Returned Value:
 *  True if the address is the IPv4 local loopback address.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_LOOPBACK) && defined(CONFIG_NET_IPv6)
static bool lib_lo_ipv6match(FAR const void *addr, socklen_t len, int type)
{
  FAR struct in6_addr *ipv6addr;

  if (type == AF_INET6 && len >= sizeof(struct in6_addr))
    {
      ipv6addr = (FAR struct in6_addr *)addr;
      return net_ipv6addr_maskcmp(ipv6addr->s6_addr16,
                                  g_lo_ipv6addr,
                                  g_lo_ipv6mask);
    }

  return false;
}
#endif

/****************************************************************************
 * Name: lib_localhost
 *
 * Description:
 *   Check if the address is the reserved address for the local loopback
 *   device.
 *
 * Input Parameters:
 *   addr - The address of the host to find.
 *   len - The length of the address
 *   type - The type of the address
 *   host - Caller provided location to return the host data.
 *   buf - Caller provided buffer to hold string data associated with the
 *     host data.
 *   buflen - The size of the caller-provided buffer
 *
 * Returned Value:
 *   Zero (OK) is returned on success, -1 (ERROR) is returned on a failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOOPBACK
static int lib_localhost(FAR const void *addr, socklen_t len, int type,
                         FAR struct hostent_s *host, FAR char *buf,
                         size_t buflen)
{
  FAR struct hostent_info_s *info;
  FAR char *dest;
  int namelen;

  /* Make sure that space remains to hold the hostent structure */

  if (buflen <= sizeof(struct hostent_info_s))
    {
      return -ERANGE;
    }

  info    = (FAR struct hostent_info_s *)buf;
  dest    = info->hi_data;
  buflen -= (sizeof(struct hostent_info_s) - 1);

  memset(host, 0, sizeof(struct hostent_s));
  memset(info, 0, sizeof(struct hostent_info_s));

  host->h_addrtypes = info->hi_addrtypes;
  host->h_lengths   = info->hi_lengths;
  host->h_addr_list = info->hi_addrlist;

#ifdef CONFIG_NET_IPv4
  if (lib_lo_ipv4match(addr, len, type))
    {
      /* Save the IPv4 address */

      host->h_lengths[0]   = sizeof(struct in_addr);
      host->h_addr_list[0] = (FAR char *)&g_lo_ipv4addr;
      host->h_addrtypes[0] = AF_INET;
      goto out_copyname;
    }
#endif

#ifdef CONFIG_NET_IPv6
  if (lib_lo_ipv6match(addr, len, type))
    {
      /* Save the IPv6 address */

      host->h_lengths[0]   = sizeof(struct in6_addr);
      host->h_addr_list[0] = (FAR char *)&g_lo_ipv6addr;
      host->h_addrtypes[0] = AF_INET6;
      goto out_copyname;
    }
#endif

  /* Return 1 meaning that we have no errors but no match either */

  return 1;

out_copyname:

  /* And copy localhost host name */

  namelen = strlen(g_lo_hostname);
  if (namelen + 1 > buflen)
    {
      return -ERANGE;
    }

  strlcpy(dest, g_lo_hostname, buflen);
  host->h_name = dest;

  return 0;
}
#endif

/****************************************************************************
 * Name: lib_hostfile_lookup
 *
 * Description:
 *   Try to look-up the host name from the network host file
 *
 * Input Parameters:
 *   addr - The address of the host to find.
 *   len - The length of the address
 *   type - The type of the address
 *   host - Caller provided location to return the host data.
 *   buf - Caller provided buffer to hold string data associated with the
 *     host data.
 *   buflen - The size of the caller-provided buffer
 *   h_errnop - There h_errno value returned in the event of a failure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, -1 (ERROR) is returned on a failure
 *   with the returned h_errno value provided the reason for the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_HOSTFILE
int lib_hostfile_lookup(FAR const void *addr, socklen_t len, int type,
                        FAR struct hostent_s *host, FAR char *buf,
                        size_t buflen, FAR int *h_errnop)
{
  FAR FILE *stream;
  int herrnocode;
  int nread;

  /* Search the hosts file for a match */

  stream = fopen(CONFIG_NETDB_HOSTCONF_PATH, "r");
  if (stream == NULL)
    {
      int errcode = -get_errno();

      nerr("ERROR:  Failed to open the hosts file %s: %d\n",
           CONFIG_NETDB_HOSTCONF_PATH, errcode);
      UNUSED(errcode);

      herrnocode = NO_RECOVERY;
      goto errorout_with_herrnocode;
    }

  /* Loop reading entries from the hosts file until a match is found or
   * until we hit the end-of-file.
   */

  do
    {
      /* Read the next entry from the hosts file */

      nread = parse_hostfile(stream, host, buf, buflen);
      if (nread < 0)
        {
          /* Possible errors:
           *     ERANGE - Buffer not big enough
           *     ESPIPE - End of file (or possibly a read error).
           *     EAGAIN - Error parsing the line (E.g., missing hostname)
           */

          if (nread == -ESPIPE)
            {
              nread = 0;
            }
          else if (nread != -EAGAIN)
            {
              herrnocode = NO_RECOVERY;
              goto errorout_with_stream;
            }
        }
      else if (len == host->h_lengths[0] && type == host->h_addrtypes[0])
        {
          /* We successfully read the entry and the type and size of the
           * address is good.  Now compare the addresses:
           */

          FAR char *hostaddr = host->h_addr_list[0];
          if (hostaddr != NULL)
            {
              ninfo("Comparing addresses...\n");
              if (memcmp(addr, hostaddr, len) == 0)
                {
                  /* We have a match */

                  fclose(stream);
                  return OK;
                }
            }
        }
    }
  while (nread != 0);

  /* We get here when the end of the hosts file is encountered without
   * finding the hostname.
   */

  herrnocode = HOST_NOT_FOUND;

errorout_with_stream:
  fclose(stream);

errorout_with_herrnocode:
  if (h_errnop)
    {
      *h_errnop = herrnocode;
    }

  return ERROR;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gethostbyaddr_r
 *
 * Description:
 *   The gethostbyaddr_r() function returns a structure of type hostent for
 *   the given host address addr of length len and address type type. Valid
 *   address types are AF_INET and AF_INET6. The host address argument is a
 *   pointer to a struct of a type depending on the address type, for example
 *   a struct in_addr *  for address type AF_INET.
 *
 *   gethostbyaddr_r() is *not* POSIX but is similar to a Glibc extension and
 *   is used internally by NuttX to implement the POSIX gethostbyaddr().
 *
 * Input Parameters:
 *   addr - The address of the host to find.
 *   len - The length of the address
 *   type - The type of the address
 *   host - Caller provided location to return the host data.
 *   buf - Caller provided buffer to hold string data associated with the
 *     host data.
 *   buflen - The size of the caller-provided buffer
 *   result - There host entry returned in the event of a success.
 *   h_errnop - There h_errno value returned in the event of a failure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, -1 (ERROR) is returned on a failure
 *   with the returned h_errno value provided the reason for the failure.
 *
 ****************************************************************************/

int gethostbyaddr_r(FAR const void *addr, socklen_t len, int type,
                    FAR struct hostent *host, FAR char *buf,
                    size_t buflen, FAR struct hostent **result,
                    FAR int *h_errnop)
{
#if defined(CONFIG_NET_LOOPBACK) || defined(CONFIG_NETDB_HOSTFILE)
  struct hostent_s tmp;
#endif
  int ret;

  DEBUGASSERT(addr != NULL && host != NULL && buf != NULL);
  DEBUGASSERT(type == AF_INET || type == AF_INET6);

  /* Linux man page says result must be NULL in case of failure. */

  *result = NULL;

  /* Make sure that the h_errno has a non-error code */

  if (h_errnop)
    {
      *h_errnop = 0;
    }

#ifdef CONFIG_NET_LOOPBACK
  /* Check for the local loopback address */

  ret = lib_localhost(addr, len, type, &tmp, buf, buflen);
  if (ret == OK)
    {
      /* Yes.. we are done */

      convert_hostent(&tmp, AF_UNSPEC, host);
      *result = host;
      return OK;
    }
#endif

  /* TODO:
   *
   * 1. Look in the DNS cache to see if we have the address mapping already
   *    in place.  If not,
   * 2. Perform a reverse DNS lookup.  And if that fails as well, then
   *    finally
   * 3. Search the hosts file for a match.
   */

#ifdef CONFIG_NETDB_HOSTFILE
  /* Search the hosts file for a match */

  ret = lib_hostfile_lookup(addr, len, type, &tmp, buf, buflen, h_errnop);
  if (ret == OK)
    {
      convert_hostent(&tmp, AF_UNSPEC, host);
      *result = host;
      return OK;
    }
#else
  /* The host file file is not supported.  The host address mapping was not
   * found from any lookup heuristic
   */

  if (h_errnop)
    {
      *h_errnop = HOST_NOT_FOUND;
    }

  ret = ERROR;
#endif

  return ret;
}

#endif /* CONFIG_LIBC_NETDB */
