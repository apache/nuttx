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
 * Name: chksum_iob
 *
 * Description:
 *   Calculate the Internet checksum over an iob chain buffer.
 *
 * Input Parameters:
 *   sum    - Partial calculations carried over from a previous call to
 *            chksum().  This should be zero on the first time that check
 *            sum is called.
 *   iob    - An iob chain buffer over which the checksum is to be computed.
 *   offset - Specifies the byte offset of the start of valid data.
 *
 * Returned Value:
 *   The updated checksum value.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_IOB
uint16_t chksum_iob(uint16_t sum, FAR struct iob_s *iob, uint16_t offset)
{
  /* Skip to the I/O buffer containing the data offset */

  while (iob != NULL && offset > iob->io_len)
    {
      offset -= iob->io_len;
      iob     = iob->io_flink;
    }

  /* If the link pointer is not empty, loop to walk through all I/O buffer
   * and accumulate the sum
   */

  while (iob != NULL)
    {
      sum = chksum(sum, iob->io_data + iob->io_offset + offset,
                   iob->io_len - offset);
      iob = iob->io_flink;
      offset = 0;
    }

  return sum;
}
#endif /* CONFIG_MM_IOB */

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

/****************************************************************************
 * Name: net_chksum_iob
 *
 * Description:
 *   Calculate the Internet checksum over an iob chain buffer.
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
 *   sum    - Partial calculations carried over from a previous call to
 *            chksum().  This should be zero on the first time that check
 *            sum is called.
 *   iob    - An iob chain buffer over which the checksum is to be computed.
 *   offset - Specifies the byte offset of the start of valid data.
 *
 * Returned Value:
 *   The Internet checksum of the given iob chain buffer.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_IOB
uint16_t net_chksum_iob(uint16_t sum, FAR struct iob_s *iob, uint16_t offset)
{
  return HTONS(chksum_iob(sum, iob, offset));
}
#endif /* CONFIG_MM_IOB */

/****************************************************************************
 * Name: net_chksum_adjust
 *
 * Description:
 *   Adjusts the checksum of a packet without having to completely
 *   recalculate it, as described in RFC 3022, Section 4.2, Page 9.
 *
 * Input Parameters:
 *   chksum - points to the chksum in the packet
 *   optr   - points to the old data in the packet
 *   olen   - length of old data
 *   nptr   - points to the new data in the packet
 *   nlen   - length of new data
 *
 * Limitations:
 *   The algorithm is applicable only for even offsets and even lengths.
 ****************************************************************************/

void net_chksum_adjust(FAR uint16_t *chksum,
                       FAR const uint16_t *optr, ssize_t olen,
                       FAR const uint16_t *nptr, ssize_t nlen)
{
#ifdef CONFIG_ENDIAN_BIG
#  warning "Not verified on big-endian yet."
#endif

  int32_t x;
  int32_t oldval;
  int32_t newval;

  x = NTOHS(*chksum);
  x = ~x & 0xffff;
  while (olen > 0)
    {
      oldval = NTOHS(*optr);

      x -= oldval & 0xffff;
      if (x <= 0)
        {
          x--;
          x &= 0xffff;
        }

      optr++;
      olen -= 2;
    }
  while (nlen > 0)
    {
      newval = NTOHS(*nptr);

      x += newval & 0xffff;
      if ((x & 0x10000) != 0)
        {
          x++;
          x &= 0xffff;
        }

      nptr++;
      nlen -= 2;
    }

  x = ~x & 0xffff;
  *chksum = HTONS(x);
}

#endif /* CONFIG_NET */
