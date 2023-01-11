/****************************************************************************
 * libs/libc/netdb/lib_gethostbyname2r.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <netdb.h>

#include "netdb/lib_netdb.h"

#ifdef CONFIG_LIBC_NETDB

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gethostbyname2_r
 *
 * Description:
 *   The gethostbyname2_r() function returns a structure of type hostent for
 *   the given host name. Here name is either a hostname, or an IPv4 address
 *   in standard dot notation (as for inet_addr(3)), or an IPv6 address in
 *   colon (and possibly dot) notation.
 *
 *   If name is an IPv4 or IPv6 address, no lookup is performed and
 *   gethostbyname2_r() simply copies name into the h_name field
 *   and its struct in_addr equivalent into the h_addr_list[0] field of the
 *   returned hostent structure.
 *
 *   gethostname2_r() is *not* POSIX but is similar to a Glibc extension and
 *   is used internally by NuttX to implement the POSIX gethostname().
 *
 * Input Parameters:
 *   name - The name of the host to find.
 *   type - The type of the address to find.
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

int gethostbyname2_r(FAR const char *name, int type,
                     FAR struct hostent *host, FAR char *buf,
                     size_t buflen, FAR struct hostent **result,
                     FAR int *h_errnop)
{
  struct hostent_s tmp;
  int ret;

  /* Linux man page says result must be NULL in case of failure. */

  *result = NULL;

  ret = gethostentbyname_r(name, &tmp, buf, buflen, h_errnop, 0);
  if (ret == OK)
    {
      if (convert_hostent(&tmp, type, host))
        {
          *result = host;
        }
      else
        {
          if (h_errnop)
            {
              *h_errnop = HOST_NOT_FOUND;
            }

          ret = ERROR;
        }
    }

  return ret;
}

#endif /* CONFIG_LIBC_NETDB */
