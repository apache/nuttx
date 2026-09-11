/****************************************************************************
 * arch/arm/src/stm32h5/stm32_mpuinit.c
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

#include <assert.h>
#include <sys/param.h>

#include "mpu.h"
#include "stm32_mpuinit.h"

#ifdef CONFIG_ARM_MPU

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mpuinitialize
 *
 * Description:
 *   Configure the MPU.
 *
 *   For FLAT build:
 *     - just reset and enable the MPU with the default background region.
 *
 ****************************************************************************/

void stm32_mpuinitialize(void)
{
  /* Show MPU information */

  mpu_showtype();

  /* Reset the MPU in case a bootloader left it configured */

  mpu_reset();

  /* Then enable the MPU */

  mpu_control(true, false, true);
}

#endif /* CONFIG_ARM_MPU */
