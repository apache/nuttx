/****************************************************************************
 * arch/arm/src/common/stm32/stm32_lsi.c
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
#include "stm32_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The LSI enable/disable sequence is identical on every STM32, regardless of
 * the CPU core.  Only the register/bits that carry LSION/LSIRDY differ:
 *
 *   - most families : RCC_CSR  (LSION/LSIRDY)
 *   - STM32C0        : RCC_CSR2 (LSION/LSIRDY)
 *   - STM32WB        : RCC_CSR  (LSI1ON/LSI1RDY)
 *   - STM32H5/U5     : RCC_BDCR (LSION/LSIRDY)
 *
 * A single driver therefore serves all of them, selecting the register with
 * the preprocessor below.
 */

#if defined(CONFIG_ARCH_CHIP_STM32H5) || defined(CONFIG_ARCH_CHIP_STM32U5)
#  define STM32_RCC_LSI_REG STM32_RCC_BDCR
#  define RCC_LSI_LSION      RCC_BDCR_LSION
#  define RCC_LSI_LSIRDY     RCC_BDCR_LSIRDY
#elif defined(CONFIG_ARCH_CHIP_STM32C0)
#  define STM32_RCC_LSI_REG STM32_RCC_CSR2
#  define RCC_LSI_LSION      RCC_CSR2_LSION
#  define RCC_LSI_LSIRDY     RCC_CSR2_LSIRDY
#elif defined(CONFIG_ARCH_CHIP_STM32WB)
#  define STM32_RCC_LSI_REG STM32_RCC_CSR
#  define RCC_LSI_LSION      RCC_CSR_LSI1ON
#  define RCC_LSI_LSIRDY     RCC_CSR_LSI1RDY
#else
#  define STM32_RCC_LSI_REG STM32_RCC_CSR
#  define RCC_LSI_LSION      RCC_CSR_LSION
#  define RCC_LSI_LSIRDY     RCC_CSR_LSIRDY
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rcc_enablelsi
 *
 * Description:
 *   Enable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32_rcc_enablelsi(void)
{
  /* Enable the Internal Low-Speed (LSI) RC Oscillator by setting the LSION
   * bit in the controlling RCC register.
   */

  modifyreg32(STM32_RCC_LSI_REG, 0, RCC_LSI_LSION);

  /* Wait for the internal LSI oscillator to be stable. */

  while ((getreg32(STM32_RCC_LSI_REG) & RCC_LSI_LSIRDY) == 0);
}

/****************************************************************************
 * Name: stm32_rcc_disablelsi
 *
 * Description:
 *   Disable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32_rcc_disablelsi(void)
{
  /* Disable the Internal Low-Speed (LSI) RC Oscillator by resetting the
   * LSION bit in the controlling RCC register.
   */

  modifyreg32(STM32_RCC_LSI_REG, RCC_LSI_LSION, 0);

  /* LSIRDY should go low after 3 LSI clock cycles */
}
