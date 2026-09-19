/****************************************************************************
 * arch/arm/src/am67/am67_rat.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Region Address Translator (RAT) sliding window.
 *
 * The Cortex-R5F is a 32-bit master on a 36-bit SoC: DDR beyond the first
 * 2 GB lives at 0x8_8000_0000 and cannot be addressed by the core
 * directly.  The R5FSS integrates a per-core RAT (config space at
 * 0x2ffe_0000, 4 regions) that remaps a 32-bit input window onto any
 * 36-bit output address (TRM 7.2.2.6 / 7.3.2.6.4).
 *
 * This driver dedicates one region as a 16 MB sliding window: callers ask
 * for a pointer to an arbitrary 36-bit physical address and the window is
 * re-aimed at the containing 16 MB block.  The window range is mapped
 * Non-cacheable in the MPU (am67_mpuinit.c), so accesses through it are
 * also coherent with the A53 without cache maintenance.
 *
 * The single window is a shared resource: the returned pointer is only
 * valid until the next am67_rat_map() call, and callers must be
 * serialized (the sole current user is the vhost-net driver, whose
 * transmit/receive both run on the netdev work thread).
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stddef.h>

#include <arch/barriers.h>

#include "arm_internal.h"
#include "am67_rat.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AM67_RAT_BASE        (0x2ffe0000ul)
#define AM67_RAT_CTRL(n)     (AM67_RAT_BASE + 0x20 + 0x10 * (n))
#define AM67_RAT_ADDR(n)     (AM67_RAT_BASE + 0x24 + 0x10 * (n))
#define AM67_RAT_TRANS_L(n)  (AM67_RAT_BASE + 0x28 + 0x10 * (n))
#define AM67_RAT_TRANS_U(n)  (AM67_RAT_BASE + 0x2c + 0x10 * (n))

#define AM67_RAT_CTRL_EN     (1ul << 31)

#define AM67_RAT_REGION      (0)  /* RAT region index used for the window */
#define AM67_RAT_WIN_LOG2    (24) /* 16 MB window */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_rat_map
 *
 * Description:
 *   Return a CPU pointer to the given 36-bit physical address by sliding
 *   the RAT window over its 16 MB block.  If avail is non-NULL it
 *   receives the number of contiguous bytes reachable from the returned
 *   pointer before the window ends (callers accessing more must map
 *   again at the advanced address).
 *
 *   The pointer is valid only until the next call.  Not reentrant.
 *
 ****************************************************************************/

FAR void *am67_rat_map(uint64_t pa, FAR size_t *avail)
{
  static uint64_t cur_block = UINT64_MAX;
  uint64_t block = pa & ~((uint64_t)AM67_RAT_WIN_SIZE - 1);
  uint32_t offset = (uint32_t)(pa & (AM67_RAT_WIN_SIZE - 1));

  if (block != cur_block)
    {
      putreg32(0, AM67_RAT_CTRL(AM67_RAT_REGION));
      putreg32(AM67_RAT_WIN_BASE, AM67_RAT_ADDR(AM67_RAT_REGION));
      putreg32((uint32_t)block, AM67_RAT_TRANS_L(AM67_RAT_REGION));
      putreg32((uint32_t)(block >> 32), AM67_RAT_TRANS_U(AM67_RAT_REGION));
      putreg32(AM67_RAT_CTRL_EN | AM67_RAT_WIN_LOG2,
               AM67_RAT_CTRL(AM67_RAT_REGION));
      UP_DSB();
      cur_block = block;
    }

  if (avail != NULL)
    {
      *avail = AM67_RAT_WIN_SIZE - offset;
    }

  return (FAR void *)(AM67_RAT_WIN_BASE + offset);
}
