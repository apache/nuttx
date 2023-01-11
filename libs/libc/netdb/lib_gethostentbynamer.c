/****************************************************************************
 * libs/libc/netdb/lib_gethostentbynamer.c
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
#include <sys/socket.h>

#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/net/dns.h>
#include <nuttx/net/loopback.h>

#include "netdb/lib_dns.h"
#include "netdb/lib_netdb.h"

#ifdef CONFIG_LIBC_NETDB

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This is the layout of the caller provided memory area */

struct hostent_info_s
{
  int       hi_addrtypes[CONFIG_NETDB_MAX_IPADDR + 1];
  int       hi_lengths[CONFIG_NETDB_MAX_IPADDR + 1];
  FAR char *hi_addrlist[CONFIG_NETDB_MAX_IPADDR + 1];
  char      hi_data[1];
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_numeric_address
 *
 * Description:
 *   Check if the name is a numeric IP address. In this case, simply copy
 *   name into the h_name field and its struct in_addr equivalent into the
 *   h_addr_list[0] field of the returned hostent structure.
 *
 * Input Parameters:
 *   name - The name of the host to find.
 *   host - Caller provided location to return the host data.
 *   buf - Caller provided buffer to hold string data associated with the
 *     host data.
 *   buflen - The size of the caller-provided buffer
 *
 * Returned Value:
 *   Zero (0) is returned if the name is an numeric IP address.
 *
 ****************************************************************************/

static int lib_numeric_address(FAR const char *name,
                               FAR struct hostent_s *host,
                               FAR char *buf, size_t buflen)
{
  FAR struct hostent_info_s *info;
  FAR char *ptr;
  socklen_t addrlen;
  int namelen;
  int ret;

  /* Verify that we have a buffer big enough to get started (it still may not
   * be big enough).
   */

  if (buflen <= sizeof(struct hostent_info_s))
    {
      return -ERANGE;
    }

  info    = (FAR struct hostent_info_s *)buf;
  ptr     = info->hi_data;
  buflen -= (sizeof(struct hostent_info_s) - 1);

  memset(host, 0, sizeof(struct hostent_s));
  memset(info, 0, sizeof(struct hostent_info_s));

  host->h_addrtypes = info->hi_addrtypes;
  host->h_lengths   = info->hi_lengths;
  host->h_addr_list = info->hi_addrlist;

  /* If the address contains a colon, then it might be a numeric IPv6
   * address
   */

  if (strchr(name, ':') != NULL)
    {
      /* Make sure that space remains to hold the IPv6 address */

      addrlen = sizeof(struct in6_addr);
      if (buflen < addrlen)
        {
          return -ERANGE;
        }

      ret = inet_pton(AF_INET6, name, ptr);

      /* The inet_pton() function returns 1 if the conversion succeeds. It
       * will return 0 if the input is not a valid IP address string, or -1
       * if the address family argument is unsupported.
       */

      if (ret < 1)
        {
          /* Conversion failed.  Must not be a IPv6 address */

          return 1;
        }

      host->h_addrtypes[0] = AF_INET6;
    }

  /* If the address contains a colon, then it might be a numeric IPv6
   * address.
   */

  else if (strchr(name, '.') != NULL)
    {
      /* Make sure that space remains to hold the IPv4 address */

      addrlen = sizeof(struct in_addr);
      if (buflen < addrlen)
        {
          return -ERANGE;
        }

      ret = inet_pton(AF_INET, name, ptr);

      /* The inet_pton() function returns 1 if the conversion succeeds. It
       * will return 0 if the input is not a valid IP address string, or -1
       * if the address family argument is unsupported.
       */

      if (ret < 1)
        {
          /* Conversion failed.  Must not be an IPv4 address */

          return 1;
        }

      host->h_addrtypes[0] = AF_INET;
    }

  /* No colon?  No period?  Can't be a numeric address */

  else
    {
      return 1;
    }

  host->h_addr_list[0] = ptr;
  host->h_lengths[0]   = addrlen;

  ptr    += addrlen;
  buflen -= addrlen;

  /* And copy name */

  namelen = strlen(name);
  if ((namelen + 1) > buflen)
    {
      return -ERANGE;
    }

  strlcpy(ptr, name, buflen);

  /* Set the address to h_name */

  host->h_name = ptr;
  return 0;
}

/****************************************************************************
 * Name: lib_localhost
 *
 * Description:
 *   Check if the name is the reserved name for the local loopback device.
 *
 * Input Parameters:
 *   name - The name of the host to find.
 *   host - Caller provided location to return the host data.
 *   buf - Caller provided buffer to hold string data associated with the
 *     host data.
 *   buflen - The size of the caller-provided buffer
 *
 * Returned Value:
 *   Zero (0) is returned if the name is the loopback device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOOPBACK
static int lib_localhost(FAR const char *name, FAR struct hostent_s *host,
                         FAR char *buf, size_t buflen)
{
  FAR struct hostent_info_s *info;
  FAR char *dest;
  int namelen;
  int i = 0;

  if (strcmp(name, g_lo_hostname) == 0)
    {
      /* Yes.. it is the localhost */

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
      /* Save the IPv4 address */

      info->hi_addrtypes[i] = AF_INET;
      info->hi_lengths[i]   = sizeof(struct in_addr);
      info->hi_addrlist[i]  = (FAR char *)&g_lo_ipv4addr;
      i++;
#endif

#ifdef CONFIG_NET_IPv6
      /* Save the IPv6 address */

      info->hi_addrtypes[i] = AF_INET6;
      info->hi_lengths[i]   = sizeof(struct in6_addr);
      info->hi_addrlist[i]  = (FAR char *)&g_lo_ipv6addr;
      i++;
#endif

      /* And copy name */

      namelen = strlen(name);
      if ((namelen + 1) > buflen)
        {
          return -ERANGE;
        }

      strlcpy(dest, name, buflen);

      /* Set the address to h_name */

      host->h_name = dest;
      return 0;
    }

  return 1;
}
#endif

/****************************************************************************
 * Name: lib_find_answer
 *
 * Description:
 *   Check if we previously resolved this hostname and if that resolved
 *   address is already available in the DNS cache.
 *
 * Input Parameters:
 *   name - The name of the host to find.
 *   host - Caller provided location to return the host data.
 *   buf - Caller provided buffer to hold string data associated with the
 *     host data.
 *   buflen - The size of the caller-provided buffer
 *
 * Returned Value:
 *   Zero (0) is returned if the DNS lookup was successful.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT
#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
static int lib_find_answer(FAR const char *name, FAR struct hostent_s *host,
                           FAR char *buf, size_t buflen)
{
  FAR struct hostent_info_s *info;
  FAR char *ptr;
  FAR void *addrdata;
  socklen_t addrlen;
  int naddr;
  int addrtype;
  int namelen;
  int ret;
  int i;

  /* Verify that we have a buffer big enough to get started (it still may not
   * be big enough).
   */

  if (buflen <= sizeof(struct hostent_info_s))
    {
      return -ERANGE;
    }

  /* Initialize buffers */

  info    = (FAR struct hostent_info_s *)buf;
  ptr     = info->hi_data;
  buflen -= (sizeof(struct hostent_info_s) - 1);

  /* Verify again that there is space for at least one address. */

  if (buflen < sizeof(union dns_addr_u))
    {
      return -ERANGE;
    }

  memset(host, 0, sizeof(struct hostent_s));
  memset(info, 0, sizeof(struct hostent_info_s));

  host->h_addrtypes = info->hi_addrtypes;
  host->h_lengths   = info->hi_lengths;
  host->h_addr_list = info->hi_addrlist;

  /* Try to get the host address using the DNS name server */

  naddr = buflen / sizeof(union dns_addr_u);
  ret = dns_find_answer(name, (FAR union dns_addr_u *)ptr, &naddr);
  if (ret < 0)
    {
      /* No, nothing found in the cache */

      return ret;
    }

  DEBUGASSERT(naddr <= CONFIG_NETDB_MAX_IPADDR);

  /* Get the address type. */

  for (i = 0; i < naddr; i++)
    {
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (((FAR struct sockaddr_in *)ptr)->sin_family == AF_INET)
#endif
        {
          addrlen  = sizeof(struct in_addr);
          addrtype = AF_INET;
          addrdata = &((FAR struct sockaddr_in *)ptr)->sin_addr;
        }
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          addrlen  = sizeof(struct in6_addr);
          addrtype = AF_INET6;
          addrdata = &((FAR struct sockaddr_in6 *)ptr)->sin6_addr;
        }
#endif

      info->hi_addrtypes[i] = addrtype;
      info->hi_lengths[i]   = addrlen;
      info->hi_addrlist[i]  = addrdata;

      ptr    += sizeof(union dns_addr_u);
      buflen -= sizeof(union dns_addr_u);
    }

  /* And copy name */

  namelen = strlen(name);
  if ((namelen + 1) > buflen)
    {
      return -ERANGE;
    }

  strlcpy(ptr, name, buflen);

  /* Set the address to h_name */

  host->h_name = ptr;
  return OK;
}
#endif
#endif /* CONFIG_NETDB_DNSCLIENT */

/****************************************************************************
 * Name: lib_dns_query
 *
 * Description:
 *   Combines the operations of dns_bind(), dns_query(), and dns_free() to
 *   obtain the IP address ('ipaddr') associated with the 'hostname' in one
 *   operation.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT
static int lib_dns_query(FAR const char *hostname,
                         FAR union dns_addr_u *addr, int *naddr)
{
  /* Perform the query to get the IP address */

  return dns_query(hostname, addr, naddr);
}
#endif /* CONFIG_NETDB_DNSCLIENT */

/****************************************************************************
 * Name: lib_dns_lookup
 *
 * Description:
 *   Try to look-up the host name from the DNS server
 *
 * Input Parameters:
 *   name - The name of the host to find.
 *   host - Caller provided location to return the host data.
 *   buf - Caller provided buffer to hold string data associated with the
 *     host data.
 *   buflen - The size of the caller-provided buffer
 *
 * Returned Value:
 *   Zero (0) is returned if the DNS lookup was successful.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT
static int lib_dns_lookup(FAR const char *name, FAR struct hostent_s *host,
                          FAR char *buf, size_t buflen)
{
  FAR struct hostent_info_s *info;
  FAR char *ptr;
  FAR void *addrdata;
  socklen_t addrlen;
  int naddr;
  int addrtype;
  int namelen;
  int ret;
  int i;

  /* Verify that we have a buffer big enough to get started (it still may not
   * be big enough).
   * Verify that there is space for at least one address.
   */

  namelen = strlen(name);
  if (buflen < sizeof(struct hostent_info_s) - 1 + sizeof(union dns_addr_u) +
      namelen + 1)
    {
      return -ERANGE;
    }

  /* Initialize buffers */

  info    = (FAR struct hostent_info_s *)buf;
  ptr     = info->hi_data;
  buflen -= (sizeof(struct hostent_info_s) - 1);

  memset(host, 0, sizeof(struct hostent_s));
  memset(info, 0, sizeof(struct hostent_info_s));

  host->h_addrtypes = info->hi_addrtypes;
  host->h_lengths   = info->hi_lengths;
  host->h_addr_list = info->hi_addrlist;

  /* Try to get the host address using the DNS name server */

  naddr = (buflen - (namelen + 1)) / sizeof(union dns_addr_u);
  DEBUGASSERT(naddr >= 1);
  ret = lib_dns_query(name, (FAR union dns_addr_u *)ptr, &naddr);
  if (ret < 0)
    {
      return ret;
    }

  /* We can read more than maximum, limit here. */

  naddr = MIN(naddr, CONFIG_NETDB_MAX_IPADDR);

  for (i = 0; i < naddr; i++)
    {
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (((FAR struct sockaddr_in *)ptr)->sin_family == AF_INET)
#endif
        {
          addrlen  = sizeof(struct in_addr);
          addrtype = AF_INET;
          addrdata = &((FAR struct sockaddr_in *)ptr)->sin_addr;
        }
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          addrlen  = sizeof(struct in6_addr);
          addrtype = AF_INET6;
          addrdata = &((FAR struct sockaddr_in6 *)ptr)->sin6_addr;
        }
#endif

      info->hi_addrtypes[i] = addrtype;
      info->hi_lengths[i]   = addrlen;
      info->hi_addrlist[i]  = addrdata;

      DEBUGASSERT(buflen >= namelen + 1 + sizeof(union dns_addr_u));
      ptr    += sizeof(union dns_addr_u);
      buflen -= sizeof(union dns_addr_u);
    }

  /* And copy name */

  DEBUGASSERT(buflen >= namelen + 1);
  strlcpy(ptr, name, buflen);

  /* Set the address to h_name */

  host->h_name = ptr;

  return OK;
}
#endif /* CONFIG_NETDB_DNSCLIENT */

/****************************************************************************
 * Name: lib_hostfile_lookup
 *
 * Description:
 *   Try to look-up the host name from the network host file
 *
 * Input Parameters:
 *   name - The name of the host to find.
 *   host - Caller provided location to return the host data.
 *   buf - Caller provided buffer to hold string data associated with the
 *     host data.
 *   buflen - The size of the caller-provided buffer
 *
 * Returned Value:
 *   Zero (0) is returned if the host file lookup was successful.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_HOSTFILE
static int lib_hostfile_lookup(FAR const char *name,
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
      else if (nread > 0)
        {
          /* We successfully read the entry */

          ninfo("Comparing %s to %s\n", name, host->h_name);

          /* Check for a host name match */

          if (strcmp(name, host->h_name) == 0)
            {
               /* We have a match */

               fclose(stream);
               return OK;
            }

          /* For a match with any host alias */

          if (host->h_aliases != NULL)
            {
              FAR char **alias;

              for (alias = host->h_aliases; *alias != NULL; alias++)
                {
                  /* Check for a host alias match */

                  if (strcmp(name, *alias) == 0)
                    {
                      /* We have a match */

                      fclose(stream);
                      return OK;
                    }
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
#endif /* CONFIG_NETDB_HOSTFILE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gethostentbyname_r
 *
 * Description:
 *   The gethostentbyname_r() function returns a structure of type hostent_s
 *   for the given host name. Here name is either a hostname, or an IPv4
 *   address in standard dot notation (as for inet_addr(3)), or an IPv6
 *   address in colon (and possibly dot) notation.
 *
 *   If name is an IPv4 or IPv6 address, no lookup is performed and
 *   gethostentbyname_r() simply copies name into the h_name field
 *   and its struct in_addr equivalent into the h_addr_list[0] field of the
 *   returned hostent structure.
 *
 * Input Parameters:
 *   name - The name of the host to find.
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

int gethostentbyname_r(FAR const char *name,
                       FAR struct hostent_s *host, FAR char *buf,
                       size_t buflen, FAR int *h_errnop, int flags)
{
  DEBUGASSERT(name != NULL && host != NULL && buf != NULL);

  /* Make sure that the h_errno has a non-error code */

  if (h_errnop)
    {
      *h_errnop = 0;
    }

  /* Check for a numeric hostname */

  if (lib_numeric_address(name, host, buf, buflen) == 0)
    {
      /* Yes.. we are done */

      return OK;
    }
  else if ((flags & AI_NUMERICHOST) != 0)
    {
      *h_errnop = EAI_NONAME;

      return ERROR;
    }

#ifdef CONFIG_NET_LOOPBACK
  /* Check for the local loopback host name */

  if (lib_localhost(name, host, buf, buflen) == 0)
    {
      /* Yes.. we are done */

      return OK;
    }
#endif

  /* Try to find the name in the HOSTALIASES environment variable */

#ifdef CONFIG_NETDB_DNSCLIENT
#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
  /* Check if we already have this hostname mapping cached */

  if (lib_find_answer(name, host, buf, buflen) >= 0)
    {
      /* Found the address mapping in the cache */

      return OK;
    }
#endif

  /* Try to get the host address using the DNS name server */

  if (lib_dns_lookup(name, host, buf, buflen) >= 0)
    {
      /* Successful DNS lookup! */

      return OK;
    }
#endif /* CONFIG_NETDB_DNSCLIENT */

#ifdef CONFIG_NETDB_HOSTFILE
  /* Search the hosts file for a match */

  return lib_hostfile_lookup(name, host, buf, buflen, h_errnop);

#else
  /* The host file file is not supported.  The host name mapping was not
   * found from any lookup heuristic
   */

  if (h_errnop)
    {
      *h_errnop = HOST_NOT_FOUND;
    }

  return ERROR;
#endif
}

#endif /* CONFIG_LIBC_NETDB */
