/****************************************************************************
 * libs/libc/netdb/lib_getservbynamer.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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

#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/net/ip.h>

#include <netdb.h>

#include "lib_netdb.h"

#ifdef CONFIG_LIBC_NETDB

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Declare your services here. TODO: read from /etc/services?
 * REVISIT: This is just an example, lets not add full list here.
 */

const struct services_db_s g_services_db[] =
{
  { "ntp", 123, IP_PROTO_TCP },
  { "ntp", 123, IP_PROTO_UDP },
  { NULL,  0,   0            }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getservbyname_r
 ****************************************************************************/

int getservbyname_r(FAR const char *name, FAR const char *proto,
                    FAR struct servent *result_buf, FAR char *buf,
                    size_t buflen, FAR struct servent **result)
{
  int protocol;
  int i;

  DEBUGASSERT(name != NULL);
  DEBUGASSERT(result_buf != NULL && result != NULL);

  /* Linux man page says result must be NULL in case of failure. */

  *result = NULL;

  if (proto == NULL)
    {
      protocol = 0;
    }
  else if (strcmp(proto, "tcp") == 0)
    {
      protocol = IPPROTO_TCP;
    }
  else if (strcmp(proto, "udp") == 0)
    {
      protocol = IPPROTO_UDP;
    }
  else
    {
      return EINVAL;
    }

  for (i = 0; g_services_db[i].s_name; i++)
    {
      if (strcmp(name, g_services_db[i].s_name) == 0 &&
          (protocol == 0 || protocol == g_services_db[i].s_protocol))
        {
          result_buf->s_name = (FAR char *)name;
          result_buf->s_aliases = NULL;
          result_buf->s_port = htons(g_services_db[i].s_port);

          if (g_services_db[i].s_protocol == IPPROTO_TCP)
            {
              result_buf->s_proto = "tcp";
            }
          else
            {
              result_buf->s_proto = "udp";
            }

          *result = result_buf;
          return 0;
        }
    }

  return ENOENT;
}

#endif /* CONFIG_LIBC_NETDB */
