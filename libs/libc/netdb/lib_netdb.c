/****************************************************************************
 * libs/libc/netdb/lib_netdb.c
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

bool convert_hostent(FAR const struct hostent_s *in,
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
