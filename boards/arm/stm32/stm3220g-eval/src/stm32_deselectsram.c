/****************************************************************************
 * boards/arm/stm32/stm3220g-eval/src/stm32_deselectsram.c
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

#include "arm_internal.h"
#include "stm32.h"
#include "stm3220g-eval.h"

#ifdef CONFIG_STM32_FSMC

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
 * Name: stm32_deselectsram
 *
 * Description:
 *   Disable SRAM
 *
 ****************************************************************************/

void stm32_deselectsram(void)
{
  /* Restore registers to their power up settings */

  putreg32(FSMC_BCR_RSTVALUE, STM32_FSMC_BCR2);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_RSTVALUE, STM32_FSMC_BTR2);

  /* Disable AHB clocking to the FSMC */

  stm32_fsmc_disable();
}

#endif /* CONFIG_STM32_FSMC */
