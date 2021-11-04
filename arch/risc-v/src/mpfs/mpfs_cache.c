/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_cache.c
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
#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/board/board_liberodefs.h>

#include "riscv_arch.h"
#include "hardware/mpfs_cache.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_SYSREG_L2_SHUTDOWN_CR    (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_L2_SHUTDOWN_CR_OFFSET)

#define mb()                          asm volatile ("fence" ::: "memory")

#define MPFS_L2LIM_ADDR               0x08200000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_enable_cache
 *
 * Description:
 *   Enables L2 and L1 caches and the cache ways.  The values are defined in
 *   the board_liberodefs.h -file.  Those values are generated via the Libero
 *   SoC Design Suite or utilized from the reference implementation.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpfs_enable_cache(void)
{
  /* Increasing the ways decreases the 2 MB l2lim area:
   *   - Way0:  0x081e0000 - 0x08200000
   *   - Way1:  0x081c0000 - 0x081e0000
   *   ...
   *   - Way15: 0x08000000 - 0x08020000
   *
   * For example, 7 + 1 ways eats up 1 MB of the l2lim whereas all 16 would
   * fill up the entire region.
   *
   * First, check that the l2lim is not overlapping with the cache.
   * MPFS_IDLESTACK_TOP may be also elsewhere, when configured into DDR
   * etc.  which makes the check pointless.
   */

  if ((MPFS_IDLESTACK_TOP & 0xff000000) == 0x08000000)
    {
      DEBUGASSERT((MPFS_L2LIM_ADDR - (LIBERO_SETTING_WAY_ENABLE + 1) *
                  0x20000) > MPFS_IDLESTACK_TOP);
    }

  putreg32(LIBERO_SETTING_WAY_ENABLE, MPFS_CACHE_WAY_ENABLE);

  putreg32(LIBERO_SETTING_L2_SHUTDOWN_CR, MPFS_SYSREG_L2_SHUTDOWN_CR);

  putreg32(LIBERO_SETTING_WAY_MASK_DMA, MPFS_CACHE_WAY_MASK_DMA);
  putreg32(LIBERO_SETTING_WAY_MASK_AXI4_PORT_0,
           MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_0);
  putreg32(LIBERO_SETTING_WAY_MASK_AXI4_PORT_1,
           MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_1);
  putreg32(LIBERO_SETTING_WAY_MASK_AXI4_PORT_2,
           MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_2);
  putreg32(LIBERO_SETTING_WAY_MASK_AXI4_PORT_3,
           MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_3);

  putreg32(LIBERO_SETTING_WAY_MASK_E51_ICACHE,
           MPFS_CACHE_WAY_MASK_E51_DCACHE);

  putreg32(LIBERO_SETTING_WAY_MASK_U54_1_DCACHE,
           MPFS_CACHE_WAY_MASK_U54_1_DCACHE);
  putreg32(LIBERO_SETTING_WAY_MASK_U54_1_ICACHE,
           MPFS_CACHE_WAY_MASK_U54_1_ICACHE);
  putreg32(LIBERO_SETTING_WAY_MASK_U54_2_DCACHE,
           MPFS_CACHE_WAY_MASK_U54_2_DCACHE);
  putreg32(LIBERO_SETTING_WAY_MASK_U54_2_ICACHE,
           MPFS_CACHE_WAY_MASK_U54_2_ICACHE);
  putreg32(LIBERO_SETTING_WAY_MASK_U54_3_DCACHE,
           MPFS_CACHE_WAY_MASK_U54_3_DCACHE);
  putreg32(LIBERO_SETTING_WAY_MASK_U54_3_ICACHE,
           MPFS_CACHE_WAY_MASK_U54_3_ICACHE);
  putreg32(LIBERO_SETTING_WAY_MASK_U54_4_DCACHE,
           MPFS_CACHE_WAY_MASK_U54_4_DCACHE);
  putreg32(LIBERO_SETTING_WAY_MASK_U54_4_ICACHE,
           MPFS_CACHE_WAY_MASK_U54_4_ICACHE);

  /* L2 scratchpad region needs to be configured right here.  Currently
   * we have no OpenSBI or other modules using the region so it isn't
   * configured.  This corresponds to LIBERO_SETTING_NUM_SCRATCH_PAD_WAYS
   * = 0.
   */

  putreg32(LIBERO_SETTING_WAY_MASK_E51_DCACHE,
           MPFS_CACHE_WAY_MASK_E51_DCACHE);

  mb();
}
