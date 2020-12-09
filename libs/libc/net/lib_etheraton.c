/****************************************************************************
 * libs/libc/net/lib_etheraton.c
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
#include <net/ethernet.h>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int xdigit(char c)
{
  unsigned d;

  d = (unsigned)(c - '0');
  if (d < 10)
    {
      return (int)d;
    }

  d = (unsigned)(c - 'a');
  if (d < 6)
    {
      return (int)(10 + d);
    }

  d = (unsigned)(c - 'A');
  if (d < 6)
    {
      return (int)(10 + d);
    }

  return -1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ether_aton_r
 *
 * Description:
 *   Convert Ethernet address in the standard hex-digits-and-colons to binary
 *   representation.
 *   Re-entrant version (GNU extensions)
 *
 ****************************************************************************/

FAR struct ether_addr *ether_aton_r(FAR const char *asc,
                                    FAR struct ether_addr *addr)
{
  int i, val0, val1;

  for (i = 0; i < ETHER_ADDR_LEN; ++i)
    {
      val0 = xdigit(*asc);
      asc++;
      if (val0 < 0)
        {
          return NULL;
        }

      val1 = xdigit(*asc);
      asc++;
      if (val1 < 0)
        {
          return NULL;
        }

      addr->ether_addr_octet[i] = (u_int8_t)((val0 << 4) + val1);

      if (i < ETHER_ADDR_LEN - 1)
        {
          if (*asc != ':')
            {
              return NULL;
            }

          asc++;
        }
    }

  if (*asc != '\0')
    {
      return NULL;
    }

  return addr;
}

/****************************************************************************
 * Name: ether_aton
 *
 * Description:
 *   Convert Ethernet address in the standard hex-digits-and-colons to binary
 *   representation.
 *
 ****************************************************************************/

FAR struct ether_addr *ether_aton(FAR const char *asc)
{
  static struct ether_addr addr;
  return ether_aton_r(asc, &addr);
}
