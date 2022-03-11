/****************************************************************************
 * boards/arm/lpc17xx_40xx/open1788/src/lpc17_40_norinitialize.c
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
#include "open1788.h"

#if defined(CONFIG_LPC17_40_EMC) && defined(CONFIG_LPC17_40_EXTNOR)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: open1788_nor_initialize
 *
 * Description:
 *   Initialize NOR FLASH
 *
 ****************************************************************************/

void open1788_nor_initialize(void)
{
  uint32_t regval;

  /* Set the memory width and byte lanes */

  regval = getreg32(LPC17_40_EMC_STATICCONFIG0);
  regval &= ~EMC_STATICCONFIG_MW_MASK;
  regbal |= (EMC_STATICCONFIG_MW_16BIT | EMC_STATICCONFIG_PB);
  putreg32(regval, LPC17_40_EMC_STATICCONFIG0);

  /* Configure timing */

  putreg32(2, LPC17_40_EMC_STATICWAITWEN0);
  putreg32(2, LPC17_40_EMC_STATICWAITOEN0);
  putreg32(31, LPC17_40_EMC_STATICWAITRD0);
  putreg32(31, LPC17_40_EMC_STATICWAITPAGE0);
  putreg32(31, LPC17_40_EMC_STATICWAITWR0);
  putreg32(31, LPC17_40_EMC_STATICWAITTURN0);

  up_mdelay(10);
}

#endif /* CONFIG_LPC17_40_EMC && CONFIG_LPC17_40_EXTNOR */
