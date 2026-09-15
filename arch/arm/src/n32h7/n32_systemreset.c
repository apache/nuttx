/****************************************************************************
 * arch/arm/src/n32h7/n32_systemreset.c
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

#include <stdint.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/n32h7/chip.h>

#include "arm_internal.h"
#include "hardware/n32h7_rcc.h"
#include "n32_systemreset.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_get_reset_cause
 *
 * Description:
 *   Get cause of the last CPU reset. This is done by reading reset status
 *   registger.
 *
 * Returned Value:
 *   CPU reset cause in form of macros defined in sam_systemreset.h. This is
 *   to avoid passing boardctl dependent structure to architecture layer.
 *   Board level specific code should include sam_systemreset.h and set
 *   boardctl result according to that. -1 is returned in case of invalid
 *   value in status register.
 *
 ****************************************************************************/

int n32_get_reset_cause(void)
{
  int ret = -1;
  static uint32_t rstsr = 0;

  if (rstsr == 0)
    {
      rstsr = getreg32(N32_RCC_CTRLSTS);
      putreg32(RCC_CTRLSTS_RMRSTF, N32_RCC_CTRLSTS);
      putreg32(~RCC_CTRLSTS_RMRSTF, N32_RCC_CTRLSTS);
    }

  if (rstsr & RCC_CTRLSTS_PORRSTF)
    {
      ret = N32H7_RESET_PWRUP;
    }
  else if (rstsr & RCC_CTRLSTS_BORRSTF)
    {
      ret = N32H7_RESET_BOR;
    }
  else if (rstsr & (RCC_CTRLSTS_WWDG1RSTF | RCC_CTRLSTS_WWDG2RSTF |
                    RCC_CTRLSTS_IWDG1RSTF | RCC_CTRLSTS_IWDG2RSTF))
    {
      ret = N32H7_RESET_WDOG;
    }
  else if (rstsr & RCC_CTRLSTS_CM7SFTRSTF)
    {
      ret = N32H7_RESET_SWRST;
    }
  else if (rstsr & RCC_CTRLSTS_PINRSTF)
    {
      ret = N32H7_RESET_NRST;
    }

  return ret;
}
