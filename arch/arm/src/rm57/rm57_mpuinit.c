/****************************************************************************
 * arch/arm/src/rm57/rm57_mpuinit.c
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

#include <nuttx/config.h>

#include <stdint.h>

#include "rm57_mpuinit.h"
#include "mpu.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* End of the programmed flash image, padded up to a 64KB boundary by the
 * .flashpad section in the board linker script.
 */

extern uint8_t _eflash[];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_mpu_reset
 *
 * Description:
 *   Reset all MPU regions by disabling each region.
 *
 ****************************************************************************/

static void rm57_mpu_reset(void)
{
  int i;

  for (i = 0; i < CONFIG_ARM_MPU_NREGIONS; i++)
    {
      mpu_set_rgnr(i);
      mpu_set_drbar(0);
      mpu_set_drsr(0);
      mpu_set_dracr(0);
    }
}

/****************************************************************************
 * Name: rm57_mpu_init
 *
 * Description:
 *   Initialize the MPU by disabling it, resetting all regions, configuring
 *   the flash/SRAM/peripheral regions, and then re-enabling the MPU.
 *
 ****************************************************************************/

void rm57_mpu_init(void)
{
  mpu_control(false);

  rm57_mpu_reset();

  /* Fence off the whole flash first, then overlay just the programmed part
   * as cacheable and executable.  Order matters: overlapping MPU regions
   * resolve in favour of the highest-numbered region, and mpu_allocregion()
   * hands out increasing region numbers, so the guard must be configured
   * before the region that is meant to win.
   *
   * The programmed extent comes from _eflash rather than a constant so the
   * two stay in step as the image grows.  mpu_modify_region() rounds the
   * size up to a power of two and then trims it back with sub-regions, so a
   * 0x60000 image yields a 512KB region with sub-regions 0-5 enabled: an
   * exact 0x0-0x5ffff match, with no blank flash left inside it.
   */

  rm57_flash_guard_region(RM57_FLASH_BASE, RM57_PFLASH);
  rm57_flash_region(RM57_FLASH_BASE, (uintptr_t)_eflash - RM57_FLASH_BASE);
  rm57_sram_region(RM57_RAM_BASE, RM57_SRAM);
  rm57_periph_region(RM57_PERIPH_START_ADDR, RM57_PERIPH_SIZE);

  mpu_control(true);
}
