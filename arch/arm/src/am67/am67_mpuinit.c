/****************************************************************************
 * arch/arm/src/am67/am67_mpuinit.c
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
#include <nuttx/userspace.h>
#include <arch/barriers.h>
#include <assert.h>
#include <sys/param.h>

#include "am67_mpuinit.h"
#include "mpu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_mpu_reset
 *
 * Description:
 *   Reset all MPU regions by disabling each region.
 *
 ****************************************************************************/

void am67_mpu_reset(void)
{
  for (int i = 0; i < AM67_NUM_OF_MPU_REGION; i++)
    {
      mpu_set_region_zero(i);
    }
}

/****************************************************************************
 * Name: am67_mpu_init
 *
 * Description:
 *   Initialize the MPU by disabling it, resetting all regions, configuring
 *   specific memory regions, and then re-enabling the MPU.
 *
 ****************************************************************************/

void am67_mpu_init(void)
{
  mpu_control(false);

  am67_mpu_disable_br();

  am67_mpu_reset();

  am67_register_region(AM67_REGISTER_START_ADDR, AM67_REGISTER_SIZE);
  am67_tcma_region(AM67_TCMA_START_ADDR, AM67_TCMA_SIZE);
  am67_tcmb_region(AM67_TCMB_START_ADDR, AM67_TCMB_SIZE);
  am67_mcu_msram_region(AM67_MCU_MSRAM_START_ADDR, AM67_MCU_MSRAM_SIZE);
  am67_ddr_region(AM67_DDR_START_ADDR, AM67_DDR_SIZE);

  mpu_control(true);
}
