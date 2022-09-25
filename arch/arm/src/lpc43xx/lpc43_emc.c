/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_emc.c
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

/* TODO: add #if defined(CONFIG_LPC43_EMC) */

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>

#include "arm_internal.h"
#include "chip.h"
#include "lpc43_pinconfig.h"
#include "lpc43_emc.h"
#include "hardware/lpc43_creg.h"
#include "hardware/lpc43_cgu.h"
#include "hardware/lpc43_ccu.h"
#include "lpc43_rgu.h"
#include "lpc43_gpio.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
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
 * Name: lpc43_emcinit
 *
 * Description:
 *   Initialize EMC controller. Start in full power
 *   mode.
 *
 ****************************************************************************/

void lpc43_emcinit(uint32_t enable,
                   uint32_t clock_ratio,
                   uint32_t endian_mode)
{
  uint32_t regval;

  /* Enable clock for EMC controller. */

  regval = getreg32(LPC43_CCU1_M4_EMC_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU1_M4_EMC_CFG);

  /* Configure endian mode and clock ratio. */

  regval = 0;
  if (endian_mode)
    regval |= EMC_CONFIG_EM;
  if (clock_ratio)
    regval |= EMC_CONFIG_CR;

  putreg32(regval, LPC43_EMC_CONFIG);

  /* Enable EMC 001 normal memory map, no low power mode. */

  putreg32(EMC_CONTROL_ENA, LPC43_EMC_CONTROL);
}

/****************************************************************************
 * Name: lpc43_lowpowermode
 *
 * Description:
 *   Set EMC lowpower mode.
 *
 ****************************************************************************/

void lpc43_lowpowermode(uint8_t enable)
{
  uint32_t regval;

  regval = getreg32(LPC43_EMC_CONTROL);
  if (enable)
    {
      regval |= EMC_CONTROL_LOWPOWER;
      putreg32(regval, LPC43_EMC_CONTROL);
    }
  else
    {
      regval &= ~EMC_CONTROL_LOWPOWER;
      putreg32(regval, LPC43_EMC_CONTROL);
    }
}

/****************************************************************************
 * Name: lpc43_emcenable
 *
 * Description:
 *   Enable or disable EMC controller.
 *
 ****************************************************************************/

void lpc43_emcenable(uint8_t enable)
{
  uint32_t regval;

  regval = getreg32(LPC43_EMC_CONTROL);
  if (enable)
    {
      regval |= EMC_CONTROL_ENA;
      putreg32(regval, LPC43_EMC_CONTROL);
    }
  else
    {
      regval &= ~EMC_CONTROL_ENA;
      putreg32(regval, LPC43_EMC_CONTROL);
    }
}
