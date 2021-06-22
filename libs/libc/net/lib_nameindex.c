/****************************************************************************
 * libs/libc/net/lib_nameindex.c
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

#include <net/if.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: if_nameindex
 *
 * Description:
 *   The if_nameindex() function returns an array of if_nameindex structures,
 *   each containing information about one of the network interfaces on the
 *   local system. The if_nameindex structure contains at least the following
 *   entries:
 *         unsigned int if_index;
 *         FAR char     *if_name;
 *   The if_index field contains the interface index. The if_name field
 *   points to the null-terminated interface name.  The end of the array
 *   is indicated by entry with if_index set to zero and if_name set to NULL.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, if_nameindex() returns pointer to the array; on error, NULL
 *   is returned, and errno is set to indicate the error.
 *
 ****************************************************************************/

FAR struct if_nameindex *if_nameindex(void)
{
  FAR struct if_nameindex *ifn;
  FAR char *buf;
  int i = 0;
  int j;

  ifn = lib_malloc((sizeof(*ifn) + IF_NAMESIZE) * MAX_IFINDEX);
  if (ifn == NULL)
    {
      set_errno(ENOBUFS);
      return NULL;
    }

  buf = (FAR char *)(ifn + MAX_IFINDEX);
  for (j = 1; j <= MAX_IFINDEX; j++)
    {
      ifn[i].if_name = if_indextoname(j, buf);
      if (ifn[i].if_name)
        {
          ifn[i++].if_index = j;
          buf += IF_NAMESIZE;
        }
    }

  ifn[i].if_name = NULL;
  ifn[i].if_index = 0;

  return ifn;
}
