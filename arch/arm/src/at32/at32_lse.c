/****************************************************************************
 * arch/arm/src/at32/at32_lse.c
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
#include "at32_pwr.h"
#include "at32_rcc.h"
#include "at32_waste.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_rcc_enablelse
 *
 * Description:
 *   Enable the External Low-Speed (LSE) oscillator.
 *
 * Todo:
 *   Check for LSE good timeout and return with -1,
 *
 ****************************************************************************/

void at32_rcc_enablelse(void)
{
  /* The LSE is in the RTC domain and write access is denied to this domain
   * after reset, you have to enable write access using DBP bit in the PWR CR
   * register before to configuring the LSE.
   */

  at32_pwr_enablebkp(true);

  /* Enable the External Low-Speed (LSE) oscillator by setting the LSEON bit
   * the RCC BDCR register.
   */

  modifyreg16(AT32_CRM_BPDC, 0, CRM_BPDC_LEXTEN);

  /* Wait for the LSE clock to be ready */

  while ((getreg16(AT32_CRM_BPDC) & CRM_BPDC_LEXTSTBL) == 0)
    {
      at32_waste();
    }

  /* Disable backup domain access if it was disabled on entry */

  at32_pwr_enablebkp(false);
}
