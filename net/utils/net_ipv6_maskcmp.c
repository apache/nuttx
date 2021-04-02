/****************************************************************************
 * net/utils/net_ipv6_maskcmp.c
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

#include <nuttx/net/ip.h>

#ifdef CONFIG_NET_IPv6

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_ipv6addr_maskcmp
 *
 * Description:
 *   Compare two IPv6 addresses under a netmask.  The mask is used to mask
 *   out the bits that are to be compared:  Buts within the mask much
 *   match exactly; bits outside if the mask are ignored.
 *
 * Input Parameters:
 *   addr1 - The first IP address.
 *   addr2 - The second IP address.
 *   mask  - The netmask.
 *
 * Returned Value:
 *   True if the address under the mask are equal
 *
 ****************************************************************************/

bool net_ipv6addr_maskcmp(const net_ipv6addr_t addr1,
                          const net_ipv6addr_t addr2,
                          const net_ipv6addr_t mask)
{
  int i;

  /* Start from the "bottom" where the addresses will most likely differ */

  for (i = 7; i >= 0; i--)
    {
      /* Same? */

      if ((addr1[i] & mask[i]) != (addr2[i] & mask[i]))
        {
          /* No.. the addresses are different */

          return false;
        }
    }

  /* The addresses are the same */

  return true;
}

#endif /* CONFIG_NET_IPv6 */
