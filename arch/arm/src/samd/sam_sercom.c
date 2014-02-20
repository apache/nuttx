/****************************************************************************
 * arch/arm/src/samd/sam_lowputc.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   1. "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *      Datasheet", 42129J–SAM–12/2013
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "up_arch.h"

#include "sam_config.h"

#include "chip/sam_pm.h"
#include "chip/sam_gclk.h"
#include "chip/sam_sercom.h"

#include "sam_sercom.h"

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
 * Name: sercom_coreclk_configure
 *
 * Description:
 *   Configure the SERCOM core source clock.
 *
 *   Two generic clocks are used by the SERCOM: GCLK_SERCOMx_CORE and
 *   GCLK_SERCOM_SLOW.  The core clock (GCLK_SERCOMx_CORE) is required to
 *   clock the SERCOM while operating as a master, while the slow clock
 *   (GCLK_SERCOM_SLOW) is only required for certain functions.  SERCOM
 *   modules must share the same slow GCLK channel ID.
 *
 *   The baud-rate generator runs off the GCLK_SERCOMx_CORE clock (or,
 *   optionally, external clock).
 *
 ****************************************************************************/

void sercom_coreclk_configure(int sercom, int gclkgen, bool wrlock)
{
  uint16_t regval;
  uint8_t glckcore;

  /* Set up the SERCOMn_GCLK_ID_CORE clock */

  glckcore = (uint8_t)SERCOM_GCLK_ID_CORE(sercom);
  regval   = ((uint16_t)glckcore << GCLK_CLKCTRL_ID_SHIFT);

  /* Select and disable the SERCOMn_GCLK_ID_CORE generic clock */

  putreg16(regval, SAM_GCLK_CLKCTRL);

  /* Wait for clock to become disabled */

  while ((getreg16(SAM_GCLK_CLKCTRL) & GCLK_CLKCTRL_CLKEN) != 0);

  /* Select the SERCOMn_GCLK_ID_CORE source clock generator */

  regval |= (uint16_t)gclkgen << GCLK_CLKCTRL_GEN_SHIFT;

  /* Write the new configuration */

  putreg16(regval, SAM_GCLK_CLKCTRL);

  /* Enable the SERCOMn_GCLK_ID_CORE generic clock, optionally locking
   * further writes to this GCLK.
   */

  regval |= GCLK_CLKCTRL_CLKEN;

  if (wrlock)
    {
      regval |= GCLK_CLKCTRL_WRTLOCK;
    }

  putreg16(regval, SAM_GCLK_CLKCTRL);
}

/****************************************************************************
 * Name: sercom_slowclk_configure
 *
 * Description:
 *   Configure the SERCOM slow source clock.
 *
 *   Two generic clocks are used by the SERCOM: GCLK_SERCOMx_CORE and
 *   GCLK_SERCOM_SLOW.  The core clock (GCLK_SERCOMx_CORE) is required to
 *   clock the SERCOM while operating as a master, while the slow clock
 *   (GCLK_SERCOM_SLOW) is only required for certain functions.  SERCOM
 *   modules must share the same slow GCLK channel ID.
 *
 ****************************************************************************/

void sercom_slowclk_configure(int gclkgen)
{
  static bool configured = false;
  uint16_t regval;

  /* Since GCLK_SERCOM_SLOW is shard amongst all SERCOM modules, it should
   * only be configured one time.
   */

  if (!configured)
    {
      /* Set up the SERCOM_GCLK_ID_SLOW clock */

      regval = (SERCOM_GCLK_ID_SLOW << GCLK_CLKCTRL_ID_SHIFT);

      /* Select and disable the SERCOM_GCLK_ID_SLOW generic clock */

      putreg16(regval, SAM_GCLK_CLKCTRL);

      /* Wait for clock to become disabled */

      while ((getreg16(SAM_GCLK_CLKCTRL) & GCLK_CLKCTRL_CLKEN) != 0);

      /* Select the SERCOM_GCLK_ID_SLOW clock source generator */

      regval |= (uint16_t)gclkgen << GCLK_CLKCTRL_GEN_SHIFT;

      /* Write the new configuration */

      putreg16(regval, SAM_GCLK_CLKCTRL);

      /* Enable the GCLK_SERCOM_SLOW generic clock and lock further
       * writes to this GCLK.  When this bit is written, it will lock
       * further writes to the generic clock pointed by the CLKCTRL.ID. The
       * generic clock generator pointed by CLKCTRL.GEN and the GENDIV.DIV
       * will also be locked.
       *
       * We lock the SERCOM slow clock because it is common to all SERCOM modules
       * and, once set, should not be changed again.
       */

      regval |= (/* GCLK_CLKCTRL_WRTLOCK | */ GCLK_CLKCTRL_CLKEN);
      putreg16(regval, SAM_GCLK_CLKCTRL);

      /* Now we are configured */

      configured = true;
    }
}
