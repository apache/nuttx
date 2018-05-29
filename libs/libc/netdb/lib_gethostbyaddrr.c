/****************************************************************************
 * libs/libc/netdb/lib_gethostbyaddrr.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include <errno.h>

#include <arpa/inet.h>
#include <nuttx/net/loopback.h>

#include "libc.h"
#include "netdb/lib_netdb.h"

#ifdef CONFIG_NETDB_HOSTFILE

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

#ifdef CONFIG_NET_LOOPBACK
static bool lib_lo_ipv4match(FAR const void *addr, socklen_t len, int type)
{
  FAR struct in_addr *ipv4addr;

  if (type == AF_INET && len >= sizeof(struct in_addr))
    {
      ipv4addr = (FAR struct in_addr *)addr;
      return net_ipv4addr_maskcmp(ipv4addr->sin_addr.s_addr,
                                  g_lo_ipv4addr->s_addr,
                                  g_lo_ipv4addr->s_addr);
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

#ifdef CONFIG_NET_LOOPBACK
static bool lib_lo_ipv6match(FAR const void *addr, socklen_t len, int type)
{
  FAR struct in_addr6 *ipv6addr;

  if (type == AF_INE6T && len >= sizeof(struct in_addr6))
    {
      ipv6addr = (FAR struct in_addr6 *)addr;
      return net_ipv6addr_cmp(ipv6addr->sin6_addr.s6_addr16, g_lo_ipv6addr);
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
 *   h_errnop - There h_errno value returned in the event of a failure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, -1 (ERROR) is returned on a failure
 *   with the returned h_errno value provided the reason for the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOOPBACK
static int lib_localhost(FAR const void *addr, socklen_t len, int type,
                         FAR struct hostent *host, FAR char *buf,
                         size_t buflen, int *h_errnop)
{
  FAR struct hostent_info_s *info;
  socklen_t addrlen;
  FAR const uint8_t *src;
  FAR char *dest;
  bool match;
  int herrnocode;
  int namelen;

  if (lib_lo_ipv4match(addr, len, type))
    {
      /* Setup to transfer the IPv4 address */

      addrlen          = sizeof(struct in_addr);
      src              = (FAR uint8_t *)&g_lo_ipv4addr;
      host->h_addrtype = AF_INET;
    }
  else if (lib_lo_ipv4match(addr, len, type))
    {
      /* Setup to transfer the IPv6 address */

      addrlen          = sizeof(struct in6_addr);
      src              = (FAR uint8_t *)&g_lo_ipv6addr;
      host->h_addrtype = AF_INET6;
    }
  else
    {
      /* Return 1 meaning that we have no errors but no match either */

      return 1;
    }

  /* Make sure that space remains to hold the hostent structure and
   * the IP address.
   */

  if (buflen <= (sizeof(struct hostent_info_s) + addrlen))
    {
      return -ERANGE;
    }

  info             = (FAR struct hostent_info_s *)buf;
  dest             = info->hi_data;
  buflen          -= (sizeof(struct hostent_info_s) - 1);

  memset(host, 0, sizeof(struct hostent));
  memset(info, 0, sizeof(struct hostent_info_s));
  memcpy(dest, src, addrlen);

  info->hi_addrlist[0] = dest;
  host->h_addr_list    = info->hi_addrlist;
  host->h_length       = addrlen;

  dest                += addrlen;
  buflen              -= addrlen;

  /* And copy localhost host name */

  namelen = strlen(g_lo_hostname);
  if (addrlen + namelen + 1 > buflen)
    {
      herrnocode = ERANGE;
      goto errorout_with_herrnocode;
    }

  strncpy(dest, g_lo_hostname, buflen);
  return 0;

errorout_with_herrnocode:
  if (h_errnop)
    {
      *h_errnop = herrnocode;
    }

  return ERROR;
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

int lib_hostfile_lookup(FAR const void *addr, socklen_t len, int type,
                        FAR struct hostent *host, FAR char *buf,
                        size_t buflen, int *h_errnop)
{
  FAR FILE *stream;
  int herrnocode;
  int nread;

  /* Search the hosts file for a match */

  stream = fopen(CONFIG_NETDB_HOSTCONF_PATH, "r");
  if (stream == NULL)
    {
      int errcode = get_errno();

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

      nread = lib_parse_hostfile(stream, host, buf, buflen);
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
      else if (nread > 0 && len == host->h_length && type == host->h_addrtype)
        {
          /* We successfully read the entry and the type and size of the
           * address is good.  Now compare the addresses:
           */

          FAR char *hostaddr = host->h_addr;
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
 *   gethostbyaddr_r() is *not* POSIX but is similar to a Glibc extension and is
 *   used internally by NuttX to implement the POSIX gethostbyaddr().
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

int gethostbyaddr_r(FAR const void *addr, socklen_t len, int type,
                    FAR struct hostent *host, FAR char *buf,
                    size_t buflen, int *h_errnop)
{
  FAR FILE *stream;
  int herrnocode;
  int nread;

  DEBUGASSERT(addr != NULL && host != NULL && buf != NULL);
  DEBUGASSERT(type == AF_INET || type == AF_INET6);

  /* Make sure that the h_errno has a non-error code */

  if (h_errnop)
    {
      *h_errnop = 0;
    }

#ifdef CONFIG_NET_LOOPBACK
  /* Check for the local loopback address */

  if (lib_localhost(addr, len, type, host, buf, buflen, h_errnop) == 0)
    {
      /* Yes.. we are done */

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

  /* Search the hosts file for a match */

  return lib_hostfile_lookup(addr, len, type, host, buf, buflen, h_errnop);
}

#endif /* CONFIG_NETDB_HOSTFILE */
