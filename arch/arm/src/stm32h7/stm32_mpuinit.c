/****************************************************************************
 * arch/arm/src/stm32h7/stm32_mpuinit.c
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

#ifdef CONFIG_BUILD_PROTECTED
#  include <nuttx/userspace.h>
#endif

#include "mpu.h"
#include "hardware/stm32_memorymap.h"
#include "stm32_mpuinit.h"

#ifdef CONFIG_ARM_MPU

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_RPTUN
#  ifdef CONFIG_STM32H7_SHMEM_SRAM3
#    define STM32_SHMEM_BASE STM32_SRAM3_BASE
#    define STM32_SHMEM_SIZE STM32H7_SRAM3_SIZE
#  else
#    error missing shmem MPU configuration
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mpuinitialize
 *
 * Description:
 *   Configure the MPU.
 *
 *   If PROTECTED build:
 *     - permit user-space access to only restricted STM32 resources.
 *
 *   If RPTUN:
 *     - configure shared memory as non-cacheable
 *
 ****************************************************************************/

void stm32_mpuinitialize(void)
{
#ifdef CONFIG_BUILD_PROTECTED
  uintptr_t datastart = MIN(USERSPACE->us_datastart, USERSPACE->us_bssstart);
  uintptr_t dataend   = MAX(USERSPACE->us_dataend,   USERSPACE->us_bssend);

  DEBUGASSERT(USERSPACE->us_textend >= USERSPACE->us_textstart &&
              dataend >= datastart);
#endif

  /* Show MPU information */

  mpu_showtype();

  /* Reset MPU if enabled */

  mpu_reset();

#ifdef CONFIG_BUILD_PROTECTED
  /* Configure user flash and SRAM space */

  mpu_user_flash(USERSPACE->us_textstart,
                 USERSPACE->us_textend - USERSPACE->us_textstart);

  mpu_user_intsram(datastart, dataend - datastart);
#endif

#ifdef CONFIG_RPTUN
  /* Configure shared memory as non-cacheable */

  mpu_priv_shmem((uintptr_t)STM32_SHMEM_BASE, STM32_SHMEM_SIZE);
#endif

  /* Then enable the MPU */

  mpu_control(true, false, true);
}

#ifdef CONFIG_BUILD_PROTECTED
/****************************************************************************
 * Name: stm32_mpu_uheap
 *
 * Description:
 *  Map the user-heap region.
 *
 *  This logic may need an extension to handle external SDRAM).
 *
 ****************************************************************************/

void stm32_mpu_uheap(uintptr_t start, size_t size)
{
  mpu_user_intsram(start, size);
}
#endif

#endif /* CONFIG_ARM_MPU */
