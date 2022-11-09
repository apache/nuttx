/****************************************************************************
 * libs/libc/netdb/lib_dnsforeach.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/net/dns.h>

#include "netdb/lib_dns.h"

#ifdef CONFIG_NETDB_DNSCLIENT

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_NETDB_RESOLVCONF
static FAR char *skip_spaces(FAR char *ptr)
{
  while (isspace(*ptr)) ptr++;
  return ptr;
}

static FAR char *find_spaces(FAR char *ptr)
{
  while (*ptr && !isspace(*ptr)) ptr++;
  return ptr;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_foreach_nameserver
 *
 * Description:
 *   Traverse each nameserver entry in the resolv.conf file and perform
 *   the provided callback.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_RESOLVCONF

int dns_foreach_nameserver(dns_callback_t callback, FAR void *arg)
{
  union dns_addr_u u;
  FAR FILE *stream;
  char line[DNS_MAX_LINE];
  FAR char *addrstr;
  FAR char *ptr;
  uint16_t port;
  int keylen;
  int ret;

  /* Open the resolver configuration file */

  stream = fopen(CONFIG_NETDB_RESOLVCONF_PATH, "r");
  if (stream == NULL)
    {
      ret = -get_errno();
      nerr("ERROR: Failed to open %s: %d\n",
        CONFIG_NETDB_RESOLVCONF_PATH, ret);
      DEBUGASSERT(ret < 0);
      return ret;
    }

  dns_lock();

  keylen = strlen(NETDB_DNS_KEYWORD);
  while (fgets(line, DNS_MAX_LINE, stream) != NULL)
    {
      ptr = skip_spaces(line);
      if (strncmp(ptr, NETDB_DNS_KEYWORD, keylen) == 0)
        {
          /* Skip over the 'nameserver' keyword */

          ptr = find_spaces(ptr);
          addrstr = skip_spaces(ptr);
          if (*addrstr == '\0')
            {
              nerr("ERROR: Missing address in %s record\n",
                   CONFIG_NETDB_RESOLVCONF_PATH);
              continue;
            }

          /* Make sure that the address string is NUL terminated and
           * not followed by garbage.
           */

          ptr = find_spaces(addrstr);
          *ptr = '\0';

          /* Convert the address string to a binary representation. */

          port = HTONS(DNS_DEFAULT_PORT);

#ifdef CONFIG_NETDB_RESOLVCONF_NONSTDPORT
          /* The OpenBSD version supports a [host]:port syntax.  When a
           * non-standard port is specified the host address must be
           * enclosed in square brackets.  For example:
           *
           *   nameserver [10.0.0.1]:5353
           *   nameserver [::1]:5353
           */

          if (*addrstr == '[')
            {
              /* Make sure that there is a right bracket */

              ptr = strchr(addrstr, ']');
              if (ptr == NULL)
                {
                  nerr("ERROR: Missing right bracket after %s\n", line);
                  continue;
                }

              /* Replace the right bracket with a NULL terminator */

              addrstr++;
              *ptr++ = '\0';

              /* Get the port number following the right bracket */

              if (*ptr++ == ':')
                {
                  FAR char *portstr;
                  int tmp;

                  /* Isolate the port string */

                  portstr = ptr;
                  ptr     = find_spaces(ptr);
                  *ptr    = '\0';

                  /* Get the port number */

                  tmp = atoi(portstr);
                  if (tmp != 0)
                    {
                      port = HTONS(tmp);
                    }
                }
            }
#endif /* CONFIG_NETDB_RESOLVCONF_NONSTDPORT */

#ifdef CONFIG_NET_IPv4
          /* Try to convert the IPv4 address */

          ret = inet_pton(AF_INET, addrstr, &u.ipv4.sin_addr);

          /* The inet_pton() function returns 1 if the conversion succeeds */

          if (ret == 1)
            {
              u.ipv4.sin_family = AF_INET;
              u.ipv4.sin_port   = port;
              ret = callback(arg, (FAR struct sockaddr *)&u.ipv4,
                             sizeof(struct sockaddr_in));
            }
          else
#endif
#ifdef CONFIG_NET_IPv6
            {
              /* Try to convert the IPv6 address */

              ret = inet_pton(AF_INET6, addrstr, &u.ipv6.sin6_addr);

              /* The inet_pton() function returns 1 if the conversion
               * succeeds.
               */

              if (ret == 1)
                {
                  u.ipv6.sin6_family = AF_INET6;
                  u.ipv6.sin6_port   = port;
                  ret = callback(arg, (FAR struct sockaddr *)&u.ipv6,
                                 sizeof(struct sockaddr_in6));
                }
              else
#endif
                {
                  nerr("ERROR: Unrecognized address: %s\n", addrstr);
                  ret = OK;
                }
#ifdef CONFIG_NET_IPv6
            }
#endif

          if (ret != OK)
            {
              break;
            }
        }
    }

  dns_unlock();
  fclose(stream);
  return ret;
}

#else /* CONFIG_NETDB_RESOLVCONF */

int dns_foreach_nameserver(dns_callback_t callback, FAR void *arg)
{
  FAR struct sockaddr *addr;
  int ret = OK;
  int i;

  dns_lock();
  for (i = 0; i < g_dns_nservers; i++)
    {
#ifdef CONFIG_NET_IPv4
      /* Check for an IPv4 address */

      if (g_dns_servers[i].addr.sa_family == AF_INET)
        {
          struct sockaddr_in copy;

          /* Operate on copy of server address, in case it changes. */

          memcpy(&copy, &g_dns_servers[i].ipv4, sizeof(struct sockaddr_in));
          addr = (FAR struct sockaddr *)&copy;

          /* Perform the callback */

          ret = callback(arg, addr, sizeof(struct sockaddr_in));
        }
      else
#endif

#ifdef CONFIG_NET_IPv6
      /* Check for an IPv6 address */

      if (g_dns_servers[i].addr.sa_family == AF_INET6)
        {
          struct sockaddr_in6 copy;

          /* Operate on copy of server address, in case it changes. */

          memcpy(&copy, &g_dns_servers[i].ipv6, sizeof(struct sockaddr_in6));
          addr = (FAR struct sockaddr *)&copy;

          /* Perform the callback */

          ret = callback(arg, addr, sizeof(struct sockaddr_in6));
        }
      else
#endif
        {
          nerr("ERROR: Unsupported family: %d\n",
                g_dns_servers[i].addr.sa_family);
          ret = -ENOSYS;
        }

      if (ret != OK)
        {
          break;
        }
    }

  dns_unlock();
  return ret;
}

#endif /* CONFIG_NETDB_RESOLVCONF */
#endif /* CONFIG_NETDB_DNSCLIENT */
