/****************************************************************************
 * net/utils/net_ipv6_maskcmp.c
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

