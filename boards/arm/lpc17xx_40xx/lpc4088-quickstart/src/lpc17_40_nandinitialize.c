/****************************************************************************
 * boards/arm/lpc17xx_40xx/lpc4088-quickstart/src/lpc17_40_nandinitialize.c
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

#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "lpc4088-quickstart.h"

#if defined(CONFIG_LPC17_40_SPIFI)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc4088_quickstart_nand_initialize
 *
 * Description:
 *   Initialize NAND FLASH
 *
 ****************************************************************************/

void _lpc4088_quickstart_nand_initialize(void)
{
  uint32_t regval;

  /* Set the memory width and byte lanes */

  regval = getreg32(LPC17_40_EMC_STATICCONFIG1);
  regval &= ~EMC_STATICCONFIG_MW_MASK;
  regbal |= (EMC_STATICCONFIG_MW_8BIT | EMC_STATICCONFIG_PB);
  putreg32(regval, LPC17_40_EMC_STATICCONFIG1);

  /* Configure timing */

  putreg32(2, LPC17_40_EMC_STATICWAITWEN1);
  putreg32(2, LPC17_40_EMC_STATICWAITOEN1);
  putreg32(31, LPC17_40_EMC_STATICWAITRD1);
  putreg32(31, LPC17_40_EMC_STATICWAITPAGE1);
  putreg32(31, LPC17_40_EMC_STATICWAITWR1);
  putreg32(31, LPC17_40_EMC_STATICWAITTURN1);

  /* GPIO P2[21] connects to the Ready/Busy pin of the NAND part.
   * We need to reconfigure this pin as normal GPIO input.
   */

  lpc17_40_gpioconfig(GPIO_NAND_RB);
}

#endif /* CONFIG_LPC17_40_SPIFI */
