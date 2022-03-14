/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_lsi.c
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
#include "stm32l4_rcc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_rcc_enablelsi
 *
 * Description:
 *   Enable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32l4_rcc_enablelsi(void)
{
  /* Enable the Internal Low-Speed (LSI) RC Oscillator by setting the LSION
   * bit the RCC CSR register.
   */

  modifyreg32(STM32L4_RCC_CSR, 0, RCC_CSR_LSION);

  /* Wait for the internal LSI oscillator to be stable. */

  while ((getreg32(STM32L4_RCC_CSR) & RCC_CSR_LSIRDY) == 0);
}

/****************************************************************************
 * Name: stm32l4_rcc_disablelsi
 *
 * Description:
 *   Disable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32l4_rcc_disablelsi(void)
{
  /* Enable the Internal Low-Speed (LSI) RC Oscillator by setting the LSION
   * bit the RCC CSR register.
   */

  modifyreg32(STM32L4_RCC_CSR, RCC_CSR_LSION, 0);

  /* LSIRDY should go low after 3 LSI clock cycles */
}
