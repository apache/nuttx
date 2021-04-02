/****************************************************************************
 * net/utils/net_ipv6_pref2mask.c
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

#include "utils/utils.h"

#ifdef CONFIG_NET_IPv6

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_ipv6_pref2mask
 *
 * Description:
 *   Convert a IPv6 prefix length to a network mask.  The prefix length
 *   specifies the number of MS bits under mask (0-128)
 *
 * Input Parameters:
 *   preflen  - Determines the width of the netmask (in bits).  Range 0-128
 *   mask  - The location to return the netmask.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void net_ipv6_pref2mask(uint8_t preflen, net_ipv6addr_t mask)
{
  unsigned int bit;
  unsigned int i;

  /* Set the network mask.  preflen is the number of MS bits under the mask.
   *
   * Eg. preflen = 38
   *     NETMASK: ffff ffff fc00 0000  0000 0000 0000 0000
   *     bit:                                       1 1..1
   *                 1 1..3 3..4 4..6  6..7 8..9 9..1 1..2
   *              0..5 6..1 2..7 8..3  4..9 0..5 6..1 2..7
   *     preflen:                                   1 1..1
   *                 1 1..3 3..4 4..6  6..8 8..9 9..1 1..2
   *              1..6 7..2 3..8 9..4  5..0 1..6 7..2 3..8
   */

  for (i = 0; i < 8; i++)
    {
      /* bit = {0, 16, 32, 48, 64, 80, 96, 112} */

      bit = i << 4;

      if (preflen > bit)
        {
          /* Eg. preflen = 38, bit = {0, 16, 32} */

          if (preflen >= (bit + 16))
            {
              /* Eg. preflen = 38, bit = {0, 16} */

              mask[i] = 0xffff;
            }
          else
            {
              /* Eg. preflen = 38, bit = {32}
               *     preflen - bit = 6
               *     mask = 0xffff << (16-6)
               *          = 0xfc00
               */

              mask[i] = 0xffff << (16 - (preflen - bit));
            }
        }
      else
        {
          /* Eg. preflen=38, bit= {48, 64, 80, 96, 112} */

          mask[i] = 0x0000;
        }
    }
}

#endif /* CONFIG_NET_IPv6 */
