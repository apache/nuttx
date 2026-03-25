/****************************************************************************
 * boards/arm/ht32f491x3/esk32/src/ht32_boot.c
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

#include <stdbool.h>
#include <stdint.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "hardware/ht32f491x3_crm.h"
#include "hardware/ht32f491x3_flash.h"
#include "hardware/ht32f491x3_gpio.h"
#include "hardware/ht32f491x3_memorymap.h"
#include "hardware/ht32f491x3_pwc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HT32_BOARD_HEXT_FREQUENCY      8000000u
#define HT32_BOARD_PLL_150_NS          75u
#define HT32_BOARD_PLL_150_MS          1u

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void ht32f491x3_clockconfig(void)
{
  /* The ESK32 board has an 8 MHz crystal, so follow Holtek's 150 MHz
   * HEXT->PLL startup sequence for this board.
   */

  modifyreg32(HT32_CRM_CTRL, 0, HT32_CRM_CTRL_HICKEN);
  while ((getreg32(HT32_CRM_CTRL) & HT32_CRM_CTRL_HICKSTBL) == 0)
    {
    }

  modifyreg32(HT32_CRM_CFG, HT32_CRM_CFG_SCLKSEL_MASK,
              HT32_CRM_CFG_SEL_HICK);
  while ((getreg32(HT32_CRM_CFG) & HT32_CRM_CFG_SCLKSTS_MASK) !=
         HT32_CRM_CFG_STS_HICK)
    {
    }

  modifyreg32(HT32_CRM_CFG,
              HT32_CRM_CFG_AHBDIV_MASK |
              HT32_CRM_CFG_APB1DIV_MASK |
              HT32_CRM_CFG_APB2DIV_MASK,
              HT32_CRM_CFG_AHBDIV_NONE |
              HT32_CRM_CFG_APB1DIV_2 |
              HT32_CRM_CFG_APB2DIV_1);

  putreg32(HT32_FLASH_PSR_PROGRAM(HT32_FLASH_WAIT_CYCLE_4), HT32_FLASH_PSR);

  modifyreg32(HT32_CRM_APB1EN, 0, HT32_CRM_APB1EN_PWCEN);
  modifyreg32(HT32_PWC_LDOOV, HT32_PWC_LDOOVSEL_MASK,
              HT32_PWC_LDO_OUTPUT_1V3);

  modifyreg32(HT32_CRM_CTRL, HT32_CRM_CTRL_HEXTBYPS, HT32_CRM_CTRL_HEXTEN);
  while ((getreg32(HT32_CRM_CTRL) & HT32_CRM_CTRL_HEXTSTBL) == 0)
    {
    }

  putreg32((HT32_BOARD_PLL_150_MS << HT32_CRM_PLLCFG_PLLMS_SHIFT) |
           (HT32_BOARD_PLL_150_NS << HT32_CRM_PLLCFG_PLLNS_SHIFT) |
           HT32_CRM_PLLCFG_FR_2 |
           HT32_CRM_PLLCFG_SOURCE_HEXT,
           HT32_CRM_PLLCFG);

  modifyreg32(HT32_CRM_CTRL, 0, HT32_CRM_CTRL_PLLEN);
  while ((getreg32(HT32_CRM_CTRL) & HT32_CRM_CTRL_PLLSTBL) == 0)
    {
    }

  modifyreg32(HT32_CRM_MISC2, HT32_CRM_MISC2_AUTOSTEP_MASK,
              HT32_CRM_MISC2_AUTOSTEP_ENABLE);
  modifyreg32(HT32_CRM_CFG, HT32_CRM_CFG_SCLKSEL_MASK,
              HT32_CRM_CFG_SEL_PLL);
  while ((getreg32(HT32_CRM_CFG) & HT32_CRM_CFG_SCLKSTS_MASK) !=
         HT32_CRM_CFG_STS_PLL)
    {
    }

  modifyreg32(HT32_CRM_MISC2, HT32_CRM_MISC2_AUTOSTEP_MASK, 0);
}

void ht32f491x3_boardinitialize(void)
{
}
