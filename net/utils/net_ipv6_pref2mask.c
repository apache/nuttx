/****************************************************************************
 * net/utils/net_ipv6_pref2mask.c
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
