/****************************************************************************
 * net/utils/net_chksum.c
 *
 *   Copyright (C) 2007-2010, 2012, 2014-2015 Gregory Nutt.  All rights
 *     reserved.
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
#ifdef CONFIG_NET

#include "utils/utils.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: chksum
 *
 * Description:
 *   Calculate the raw change some over the memory region described by
 *   data and len.
 *
 * Input Parameters:
 *   sum  - Partial calculations carried over from a previous call to
 *          chksum().  This should be zero on the first time that check
 *          sum is called.
 *   data - Beginning of the data to include in the checksum.
 *   len  - Length of the data to include in the checksum.
 *
 * Returned Value:
 *   The updated checksum value.
 *
 ****************************************************************************/

#ifndef CONFIG_NET_ARCH_CHKSUM
uint16_t chksum(uint16_t sum, FAR const uint8_t *data, uint16_t len)
{
  FAR const uint8_t *dataptr;
  FAR const uint8_t *last_byte;
  uint16_t t;

  dataptr = data;
  last_byte = data + len - 1;

  while (dataptr < last_byte)
    {
      /* At least two more bytes */

      t = ((uint16_t)dataptr[0] << 8) + dataptr[1];
      sum += t;
      if (sum < t)
        {
          sum++; /* carry */
        }

      dataptr += 2;
    }

  if (dataptr == last_byte)
    {
      t = (dataptr[0] << 8) + 0;
      sum += t;
      if (sum < t)
        {
          sum++; /* carry */
        }
    }

  /* Return sum in host byte order. */

  return sum;
}
#endif /* CONFIG_NET_ARCH_CHKSUM */

/****************************************************************************
 * Name: net_chksum
 *
 * Description:
 *   Calculate the Internet checksum over a buffer.
 *
 *   The Internet checksum is the one's complement of the one's complement
 *   sum of all 16-bit words in the buffer.
 *
 *   See RFC1071.
 *
 *   If CONFIG_NET_ARCH_CHKSUM is defined, then this function must be
 *   provided by architecture-specific logic.
 *
 * Input Parameters:
 *
 *   buf - A pointer to the buffer over which the checksum is to be computed.
 *
 *   len - The length of the buffer over which the checksum is to be computed.
 *
 * Returned Value:
 *   The Internet checksum of the buffer.
 *
 ****************************************************************************/

#ifndef CONFIG_NET_ARCH_CHKSUM
uint16_t net_chksum(FAR uint16_t *data, uint16_t len)
{
  return htons(chksum(0, (uint8_t *)data, len));
}
#endif /* CONFIG_NET_ARCH_CHKSUM */

#endif /* CONFIG_NET */
