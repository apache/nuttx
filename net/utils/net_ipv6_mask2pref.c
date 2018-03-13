/****************************************************************************
 * net/utils/net_ipv6_mask2pref.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <arpa/inet.h>

#include "utils/utils.h"

#ifdef CONFIG_NET_IPv6

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_nibblemap[16] =
{
  0, 0, 0, 0, 0, 0, 0, 0, /* 0: No bits, 1-7: Should not happen */
  1, 1, 1, 1,             /* 8: 1 bit, 9-b: Should not happen */
  2, 2, 3, 4              /* c: 2 bits, d: Should not happen, e: 3 bits, f: 4 bits */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_msbits4
 *
 * Description:
 *   Count the number of leading '1' bits in an 4-bit nibble
 *
 ****************************************************************************/

static inline uint8_t net_msbits4(uint8_t nibble)
{
  /* Return the number of leading zeroes: 0-4) */

  return g_nibblemap[nibble];
}

/****************************************************************************
 * Name: net_msbits8
 *
 * Description:
 *   Count the number of leading '1' bits in an 8-bit byte
 *
 ****************************************************************************/

static uint8_t net_msbits8(uint8_t byval)
{
  uint8_t ones;

  /* Check the MS nibble */

  ones = net_msbits4(byval >> 4);
  if (ones == 4)
    {
      /* All ones, try the LS nibble */

      ones += net_msbits4(byval & 0x0f);
    }

  /* Return the number of leading ones (0-8) */

  return ones;
}

/****************************************************************************
 * Name: net_msbits16
 *
 * Description:
 *   Count the number of leading '1' bits in a 16-bit half-workd
 *
 ****************************************************************************/

static inline uint8_t net_msbits16(uint16_t hword)
{
   uint8_t ones;

  /* Look at the MS byte of the 16-bit value */

  ones = net_msbits8((uint8_t)(hword >> 8));
  if (ones == 8)
    {
      /* All '1's, try the LS byte */

      ones += net_msbits8((uint8_t)(hword & 0xff));
    }

  /* Return the number of leading ones (0-15) */

  return ones;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_ipv6_mask2pref
 *
 * Description:
 *   Convert a 128-bit netmask to a prefix length.  The Nuttx IPv6
 *   networking uses 128-bit network masks internally.  This function
 *   converts the IPv6 netmask to a prefix length.
 *
 *   The prefix length is the number of MS '1' bits on in the netmask.
 *   This, of course, assumes that all MS bits are '1' and all LS bits are
 *   '0' with no intermixed 1's and 0's.  This function searches from the MS
 *   bit until the first '0' is found (this does not necessary mean that
 *   there might not be additional '1' bits following the firs '0', but that
 *   will be a malformed netmask.
 *
 * Input Parameters:
 *   mask   Points to an IPv6 netmask in the form of uint16_t[8]
 *
 * Returned Value:
 *   The prefix length, range 0-128 on success;  This function will not
 *   fail.
 *
 ****************************************************************************/

uint8_t net_ipv6_mask2pref(FAR const uint16_t *mask)
{
  uint8_t preflen;
  int i;

  /* Count the leading all '1' 16-bit groups */

  for (i = 0, preflen = 0; i < 8 && mask[i] == 0xffff; i++, preflen += 16);

  /* Now i either, (1) indexes past the end of the mask, or (2) is the index
   * to the first half-word that is not equal to 0xffff.
   */

  if (i < 8)
    {
      preflen += net_msbits16(ntohs(mask[i]));
    }

  /* Return the prefix length */

  return preflen;
}

#endif /* CONFIG_NET_IPv6 */
