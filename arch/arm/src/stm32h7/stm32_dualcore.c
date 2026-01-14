/****************************************************************************
 * arch/arm/src/stm32h7/stm32_dualcore.c
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

#include "hardware/stm32_rcc.h"
#include "hardware/stm32_hsem.h"
#include "hardware/stm32_syscfg.h"

#include "arm_internal.h"

#include "stm32_dualcore.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check CM4 core configuration */

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM4
#  if defined(CONFIG_ARMV7M_DCACHE) || defined(CONFIG_ARMV7M_DTCM) || \
      defined(CONFIG_ARMV7M_ICACHE)
#    error Invalid configuration for CM4 core
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if (defined(CONFIG_ARCH_CHIP_STM32H7_CORTEXM7) &&  \
     defined(CONFIG_STM32H7_CORTEXM4_ENABLED)) || \
    defined(CONFIG_ARCH_CHIP_STM32H7_CORTEXM4)

/****************************************************************************
 * Name: stm32_cm4_boot
 *
 * Description:
 *   Return true if CM4 was started at boot
 *
 ****************************************************************************/

static bool stm32_cm4_boot(void)
{
  uint32_t regval = 0;

  /* Make sure that SYSCFG is enabled */

  regval = getreg32(STM32_RCC_APB4ENR);
  regval |= RCC_APB4ENR_SYSCFGEN;
  putreg32(regval, STM32_RCC_APB4ENR);

  return (bool)(getreg32(STM32_SYSCFG_UR1) & SYSCFG_UR1_BCM4);
}
#endif

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM4

/****************************************************************************
 * Name: stm32_cm4_busywait_lock_sem
 ****************************************************************************/

void stm32_cm4_busywait_lock_sem(uint8_t id)
{
  while ((getreg32(STM32_HSEM_RX(id)) & HSEM_SEMX_LOCK) == 0);
}

/****************************************************************************
 * Name: stm32_cpu2sem_wait
 ****************************************************************************/

static void stm32_cpu2sem_wait(void)
{
  /* CM4 started at boot - wait for CM7 initialization done */

  putreg32(RCC_AHB4ENR_HSEMEN, STM32_RCC_AHB4ENR);

  /* Wait for CPU1 */

  stm32_cm4_busywait_lock_sem(CPU2_HOLD_HSEM);
}
#endif

#if defined(CONFIG_ARCH_CHIP_STM32H7_CORTEXM7) && \
    defined(CONFIG_STM32H7_CORTEXM4_ENABLED)

/****************************************************************************
 * Name: stm32_cm7_take_sem
 ****************************************************************************/

static bool stm32_cm7_take_sem(uint8_t id)
{
  return (getreg32(STM32_HSEM_RLRX(id)) ==
          ((HSEM_COREID_CPU1 << HSEM_SEMX_COREID_SHIFT) | HSEM_SEMX_LOCK));
}

/****************************************************************************
 * Name: stm32_cpu2sem_take
 ****************************************************************************/

static void stm32_cpu2sem_take(void)
{
  /* Take semaphore */

  while (stm32_cm7_take_sem(CPU2_HOLD_HSEM) == 0);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_STM32H7_CORTEXM7) && \
    defined(CONFIG_STM32H7_CORTEXM4_ENABLED)

/****************************************************************************
 * Name: stm32h7_start_cm4
 *
 * Description:
 *   Start CM4 core
 *
 ****************************************************************************/

void stm32h7_start_cm4(void)
{
  uint32_t regval = 0;

  /* Get BCM4 bit */

  regval = getreg32(STM32_RCC_APB4ENR);
  regval |= RCC_APB4ENR_SYSCFGEN;
  putreg32(regval, STM32_RCC_APB4ENR);

  regval = getreg32(STM32_SYSCFG_UR1);

  if (stm32_cm4_boot() == true)
    {
      /* CM4 started at boot - signal that CM7 initialization done */

      stm32_cpu2sem_take();
    }
#ifdef CONFIG_STM32H7_CORTEXM7_BOOTM4
  else
    {
      /* CM4 not started at boot - force CM4 boot */

      getreg32(STM32_RCC_GCR);
      regval |= RCC_GCR_BOOT_C1;
      putreg32(regval, STM32_RCC_GCR);
    }
#endif
}
#endif

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM4
/****************************************************************************
 * Name: stm32h7_waitfor_cm7
 *
 * Description:
 *   Wait for CM7 core initialization
 *
 ****************************************************************************/

void stm32h7_waitfor_cm7(void)
{
  if (stm32_cm4_boot() == true)
    {
      /* Wait for CM7 initialization done if we started at boot */

      stm32_cpu2sem_wait();
    }
  else
    {
      /* CM4 boot forced by CM7 - initialization done */
    }
}
#endif
