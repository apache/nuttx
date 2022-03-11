/****************************************************************************
 * arch/arm/src/sama5/sam_systemreset.c
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
#include <arch/sama5/chip.h>

#include "arm_internal.h"
#include "hardware/sam_rstc.h"

#ifdef CONFIG_SAMA5_SYSTEMRESET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Internal reset logic.
 *
 ****************************************************************************/

void up_systemreset(void)
{
  uint32_t rstcr;
#if defined(CONFIG_SAMA5_EXTRESET_ERST) && CONFIG_SAMA5_EXTRESET_ERST != 0
  uint32_t rstmr;
#endif

  rstcr  = (RSTC_CR_PROCRST | RSTC_CR_KEY);

#if defined(CONFIG_SAMA5_EXTRESET_ERST) && CONFIG_SAMA5_EXTRESET_ERST != 0
  rstcr |= RSTC_CR_EXTRST;

  rstmr  = getreg32(SAM_RSTC_MR);
  rstmr &= ~RSTC_MR_ERSTL_MASK;
  rstmr &= RSTC_MR_ERSTL(CONFIG_SAMA5_EXTRESET_ERST - 1) | RSTC_MR_KEY;
  putreg32(rstmr, SAM_RSTC_MR);
#endif

  putreg32(rstcr, SAM_RSTC_CR);

  /* Wait for the reset */

  for (; ; );
}
#endif /* CONFIG_SAMA5_SYSTEMRESET */
