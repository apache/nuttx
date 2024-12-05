/****************************************************************************
 * arch/arm/src/nrf91/nrf91_spu.c
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

#include "arm_internal.h"

#include "sau.h"

#include "hardware/nrf91_spu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_NRF91_SPU_NONSECURE)
/****************************************************************************
 * Name: nrf91_spu_mem_default
 ****************************************************************************/

static void nrf91_spu_mem_default(void)
{
  int i = 0;

  /* Security attribute for FLASH */

  for (i = CONFIG_NRF91_FLASH_NS_START; i < SPU_FLASH_REGIONS; i++)
    {
      modifyreg32(NRF91_SPU_FLASHREGIONPERM(i),
                  SPU_FLASHREGION_PERM_SECATTR, 0);
    }

  /* Security attribute for RAM */

  for (i = CONFIG_NRF91_RAM_NS_START; i < SPU_RAM_REGIONS; i++)
    {
      modifyreg32(NRF91_SPU_RAMREGIONPERM(i),
                  SPU_RAMREGION_PERM_SECATTR, 0);
    }
}

/****************************************************************************
 * Name: nrf91_spu_periph
 ****************************************************************************/

static void nrf91_spu_periph(void)
{
#ifdef CONFIG_NRF91_REGULATORS_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_REGULATORS_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_POWERCLOCK_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_POWER_CLOCK_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_GPIO0_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_GPIO0_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_NVMC_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_NVMC_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_SERIAL0_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_SERIAL0_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_SERIAL1_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_SERIAL1_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_SERIAL2_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_SERIAL2_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_SERIAL3_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_SERIAL3_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_TIMER0_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_TIMER0_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_TIMER1_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_TIMER1_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_TIMER2_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_TIMER2_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_RTC0_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_RTC0_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_RTC1_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_RTC1_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_WDT0_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_WDT0_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_WDT1_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_WDT1_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_PWM0_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_PWM0_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_PWM1_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_PWM1_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_PWM2_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_PWM2_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

#ifdef CONFIG_NRF91_IPC_NS
  modifyreg32(NRF91_SPU_PERIPHIDPERM(NRF91_IPC_ID),
              SPU_PERIPHID_PERM_SECATTR, 0);
#endif

  /* Make all GPIO non-secure */

  modifyreg32(NRF91_SPU_GPIOPORTPERM(0), 0xffffffff, 0);
}
#endif /* NRF91_CONFIG_NONSECURE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_spu_configure
 ****************************************************************************/

void nrf91_spu_configure(void)
{
  /* Allow the security attribution to be set by the Nordic SPU */

  sau_control(false, true);

#if defined(CONFIG_NRF91_SPU_NONSECURE)
  /* Make all interrupts non-secure */

  up_secure_irq_all(false);

  /* Peripheral configuration */

  nrf91_spu_periph();

  /* Memory SPU configuration */

  nrf91_spu_mem_default();
#elif defined(CONFIG_NRF91_SPU_CUSTOM)
  /* User-specific SPU configuration */

  board_spu_configure();
#endif
}
