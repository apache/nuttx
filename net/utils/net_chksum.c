/****************************************************************************
 * net/utils/net_chksum.c
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
#ifdef CONFIG_NET

#include "utils/utils.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: chksum
 *
 * Description:
 *   Calculate the raw change sum over the memory region described by
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
 *   len - The length of the buffer over which the checksum is to be
 *         computed.
 *
 * Returned Value:
 *   The Internet checksum of the buffer.
 *
 ****************************************************************************/

#ifndef CONFIG_NET_ARCH_CHKSUM
uint16_t net_chksum(FAR uint16_t *data, uint16_t len)
{
  return HTONS(chksum(0, (uint8_t *)data, len));
}
#endif /* CONFIG_NET_ARCH_CHKSUM */

#endif /* CONFIG_NET */
