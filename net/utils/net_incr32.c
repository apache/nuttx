/****************************************************************************
 * net/utils/net_incr32.c
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
#include <nuttx/net/netdev.h>

#ifndef CONFIG_NET_ARCH_INCR32

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_carry32
 *
 * Description:
 *   Calculate the Internet checksum over a buffer.
 *
 ****************************************************************************/

static inline void net_carry32(FAR uint8_t *sum, uint16_t op16)
{
  if (sum[2] < (op16 >> 8))
    {
      ++sum[1];
      if (sum[1] == 0)
        {
          ++sum[0];
        }
    }

  if (sum[3] < (op16 & 0xff))
    {
      ++sum[2];
      if (sum[2] == 0)
        {
          ++sum[1];
          if (sum[1] == 0)
            {
              ++sum[0];
            }
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_incr32
 *
 * Description:
 *
 *   Carry out a 32-bit addition.
 *
 *   By defining CONFIG_NET_ARCH_INCR32, the architecture can replace
 *   net_incr32 with hardware assisted solutions.
 *
 * Input Parameters:
 *   op32 - A pointer to a 4-byte array representing a 32-bit integer in
 *          network byte order (big endian).  This value may not be word
 *          aligned. The value pointed to by op32 is modified in place
 *
 *   op16 - A 16-bit integer in host byte order.
 *
 ****************************************************************************/

void net_incr32(FAR uint8_t *op32, uint16_t op16)
{
  op32[3] += (op16 & 0xff);
  op32[2] += (op16 >> 8);
  net_carry32(op32, op16);
}

#endif /* CONFIG_NET_ARCH_INCR32 */
