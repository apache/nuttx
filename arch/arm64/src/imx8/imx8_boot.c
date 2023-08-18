/****************************************************************************
 * arch/arm64/src/imx8/imx8_boot.c
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

#include <nuttx/cache.h>
#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#endif

#include <arch/chip/chip.h>
#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_mmu.h"
#include "imx8_boot.h"
#include "imx8_serial.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct arm_mmu_region g_mmu_regions[] =
{
    MMU_REGION_FLAT_ENTRY("DEVICE_REGION",
                          CONFIG_DEVICEIO_BASEADDR, MB(512),
                          MT_DEVICE_NGNRNE | MT_RW | MT_SECURE),

    MMU_REGION_FLAT_ENTRY("DRAM0_S0",
                          CONFIG_RAMBANK1_ADDR, MB(512),
                          MT_NORMAL | MT_RW | MT_SECURE),
};

const struct arm_mmu_config g_mmu_config =
{
  .num_regions = nitems(g_mmu_regions),
  .mmu_regions = g_mmu_regions,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_el_init
 *
 * Description:
 *   The function called from arm64_head.S at very early stage for these
 * platform, it's use to:
 *   - Handling special hardware initialize routine which is need to
 *     run at high ELs
 *   - Initialize system software such as hypervisor or security firmware
 *     which is need to run at high ELs
 *
 ****************************************************************************/

void arm64_el_init(void)
{
  write_sysreg(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC, cntfrq_el0);

  ARM64_ISB();
}

/****************************************************************************
 * Name: arm64_chip_boot
 *
 * Description:
 *   Complete boot operations started in arm64_head.S
 *
 ****************************************************************************/

void arm64_chip_boot(void)
{
  /* MAP IO and DRAM, enable MMU. */

  arm64_mmu_init(true);

  /* Perform board-specific device initialization. This would include
   * configuration of board specific resources such as GPIOs, LEDs, etc.
   */

  imx8_board_initialize();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

  arm64_earlyserialinit();
#endif
}
