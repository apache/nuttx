/****************************************************************************
 * libs/libc/netdb/lib_netdb.c
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

#include <netdb.h>

#include "netdb/lib_netdb.h"

#ifdef CONFIG_LIBC_NETDB

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct hostent g_hostent;
char g_hostbuffer[CONFIG_NETDB_BUFSIZE];

/* When the <netdb.h> header is included, h_errno shall be available as a
 * modifiable lvalue of type int. It is unspecified whether h_errno is a
 * macro or an identifier declared with external linkage.
 */

int h_errno;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool convert_hostent(const FAR struct hostent_s *in,
                     int type, FAR struct hostent *out)
{
  int i;
  int j;

  /* Initialize the output of hostent */

  out->h_name = in->h_name;
  out->h_aliases = in->h_aliases;
  if (type != AF_UNSPEC)
    {
      out->h_addrtype = type;
      if (type == AF_INET)
        {
          out->h_length = sizeof(struct in_addr);
        }
      else
        {
          out->h_length = sizeof(struct in6_addr);
        }
    }
  else
    {
      type = in->h_addrtypes[0];
      out->h_addrtype = in->h_addrtypes[0];
      out->h_length = in->h_lengths[0];
    }

  out->h_addr_list = in->h_addr_list;

  /* Remove different type from list */

  for (i = j = 0; in->h_addr_list[i]; i++)
    {
      if (type == in->h_addrtypes[i])
        {
          in->h_addr_list[j++] = in->h_addr_list[i];
        }
    }

  in->h_addr_list[j] = NULL;
  return j != 0;
}

#endif /* CONFIG_LIBC_NETDB */
