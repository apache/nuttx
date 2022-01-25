/****************************************************************************
 * net/utils/net_ipv6_mask2pref.c
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
  /* Return the number of leading ones: 0-4) */

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
 *   Convert a 128-bit netmask to a prefix length.  The NuttX IPv6
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
      preflen += net_msbits16(NTOHS(mask[i]));
    }

  /* Return the prefix length */

  return preflen;
}

#endif /* CONFIG_NET_IPv6 */
