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

#include "rm57_mpuinit.h"
#include "mpu.h"

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

  rm57_flash_region(RM57_FLASH_BASE, RM57_PFLASH);
  rm57_sram_region(RM57_RAM_BASE, RM57_SRAM);
  rm57_periph_region(RM57_PERIPH_START_ADDR, RM57_PERIPH_SIZE);

  mpu_control(true);
}
