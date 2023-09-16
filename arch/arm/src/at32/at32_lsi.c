/****************************************************************************
 * arch/arm/src/at32/at32_lsi.c
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
#include "at32_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_rcc_enablelsi
 *
 * Description:
 *   Enable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void at32_rcc_enablelsi(void)
{
  /* Enable the Internal Low-Speed (LSI) RC Oscillator by setting the LSION
   * bit in the CRM CTRLSTS register.
   */

  modifyreg32(AT32_CRM_CTRLSTS, 0, CRM_CTRLSTS_LICKEN);

  /* Wait for the internal RC 40 kHz oscillator to be stable. */

  while ((getreg32(AT32_CRM_CTRLSTS) & CRM_CTRLSTS_LICKSTBL) == 0);
}

/****************************************************************************
 * Name: at32_rcc_disablelsi
 *
 * Description:
 *   Disable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void at32_rcc_disablelsi(void)
{
  /* Enable the Internal Low-Speed (LSI) RC Oscillator by setting the LSION
   * bit in the CRM CTRLSTS register.
   */

  modifyreg32(AT32_CRM_CTRLSTS, CRM_CTRLSTS_LICKEN, 0);

  /* LSIRDY should go low after 3 LSI clock cycles */
}
