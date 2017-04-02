/****************************************************************************
 * net/utils/net_incr32.c
 *
 *   Copyright (C) 2007-2010, 2012, 2014-2015, 2017 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/icmp.h>

#include "utils/utils.h"

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
