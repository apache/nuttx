/****************************************************************************
 * arch/arm/src/sama5/sam_sckc.c
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

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_sckc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sckc_enable
 *
 * Description:
 *   Enable or disable the slow clock oscillator driver by an external
 *   crystal.
 *
 * Input Parameters:
 *   enable - True: enable the slow clock, False: disable the slow clock
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_sckc_enable(bool enable)
{
  uint32_t regval;

#ifdef ATSAMA5D3
  /* REVISIT: Missing the logic that disables the external OSC32 */

  /* Enable external OSC 32 kHz */

  regval  = getreg32(SAM_SCKC_CR);
  regval |= SCKC_CR_OSC32EN;
  putreg32(regval, SAM_SCKC_CR);

  /* Wait for 32,768 XTAL start-up time */

  up_udelay(5 * USEC_PER_SEC / BOARD_SLOWCLK_FREQUENCY);

  /* Disable OSC 32 kHz bypass */

  regval &= ~SCKC_CR_OSC32BYP;
  putreg32(regval, SAM_SCKC_CR);

  /* Switch slow clock source to external OSC 32 kHz */

  regval |= SCKC_CR_OSCSEL;
  putreg32(regval, SAM_SCKC_CR);

  /* Wait 5 slow clock cycles for internal resynchronization */

  up_udelay(5 * USEC_PER_SEC / BOARD_SLOWCLK_FREQUENCY);

  /* Disable internal RC 32 kHz */

  regval &= ~SCKC_CR_RCEN;
  putreg32(regval, SAM_SCKC_CR);

#else
  /* Switch slow clock source to external OSC 32 kHz */

  regval = enable ? SCKC_CR_OSCSEL : 0;
  putreg32(regval, SAM_SCKC_CR);

  /* Wait 5 slow clock cycles for internal resynchronization */

  up_udelay(5 * USEC_PER_SEC / BOARD_SLOWCLK_FREQUENCY);
#endif
}
