/****************************************************************************
 * arch/arm/src/samd2l2/sam_sercom.c
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

/* References:
 *   1. "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *      Datasheet", 42129J-SAM-12/2013
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "arm_internal.h"
#include "sam_config.h"
#include "sam_pm.h"
#include "sam_gclk.h"
#include "sam_sercom.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_SERCOM0_4
#if defined(CONFIG_SAMD2L2_SERCOM0) || defined(CONFIG_SAMD2L2_SERCOM1) || \
    defined(CONFIG_SAMD2L2_SERCOM2) || defined(CONFIG_SAMD2L2_SERCOM3) || \
    defined(CONFIG_SAMD2L2_SERCOM4)
#  define HAVE_SERCOM0_4
#endif

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

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)
void sercom_coreclk_configure(int sercom, int gclkgen, bool wrlock)
{
  uint16_t regval;
  uint8_t gclkcore;

  /* Set up the SERCOMN_GCLK_ID_CORE clock */

  gclkcore = (uint8_t)SERCOM_GCLK_ID_CORE(sercom);
  regval   = ((uint16_t)gclkcore << GCLK_CLKCTRL_ID_SHIFT);

  /* Select and disable the SERCOMN_GCLK_ID_CORE generic clock */

  putreg16(regval, SAM_GCLK_CLKCTRL);

  /* Wait for clock to become disabled */

  while ((getreg16(SAM_GCLK_CLKCTRL) & GCLK_CLKCTRL_CLKEN) != 0);

  /* Select the SERCOMN_GCLK_ID_CORE source clock generator */

  regval |= (uint16_t)gclkgen << GCLK_CLKCTRL_GEN_SHIFT;

  /* Write the new configuration */

  putreg16(regval, SAM_GCLK_CLKCTRL);

  /* Enable the SERCOMN_GCLK_ID_CORE generic clock, optionally locking
   * further writes to this GCLK.
   */

  regval |= GCLK_CLKCTRL_CLKEN;

  if (wrlock)
    {
      regval |= GCLK_CLKCTRL_WRTLOCK;
    }

  putreg16(regval, SAM_GCLK_CLKCTRL);
}
#endif

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

void sercom_slowclk_configure(int sercom, int gclkgen)
{
#if defined(CONFIG_ARCH_FAMILY_SAML21)
#ifdef HAVE_SERCOM0_4
  static bool configured  = false;
#endif
#ifdef CONFIG_SAMD2L2_SERCOM5
  static bool configured5 = false;
#endif
#ifdef CONFIG_DEBUG_ASSERTIONS
#ifdef HAVE_SERCOM0_4
  static uint8_t slowgen04  = 0xff;
#endif
#ifdef CONFIG_SAMD2L2_SERCOM5
  static uint8_t slowgen5 = 0xff;
#endif
#endif

  /* Setup the SERCOMN_GCLK channel.  SERCOM0-4 use a common channel, but
   * SERCOM5 uses a different channel.  Configuration should be done only
   * once.
   */

  switch (sercom)
    {
#ifdef HAVE_SERCOM0_4
#ifdef CONFIG_SAMD2L2_SERCOM0
    case 0:
#endif
#ifdef CONFIG_SAMD2L2_SERCOM1
    case 1:
#endif
#ifdef CONFIG_SAMD2L2_SERCOM2
    case 2:
#endif
#ifdef CONFIG_SAMD2L2_SERCOM3
    case 3:
#endif
#ifdef CONFIG_SAMD2L2_SERCOM4
    case 4:
#endif
      if (!configured)
        {
          /* Configure the common slow clock channel */

          sam_gclk_chan_enable(GCLK_CHAN_SERCOM0_SLOW, gclkgen);

          /* The slow clock is now configured and should not be configured
           * again.
           */

          configured = true;
#ifdef CONFIG_DEBUG_ASSERTIONS
          slowgen04  = (uint8_t)gclkgen;
#endif
        }

#ifdef CONFIG_DEBUG_ASSERTIONS
      /* Already configured.  This is okay provided that the same GCLK
       * generator is being used.  Otherwise, there is a problem.
       */

      else
        {
          DEBUGASSERT((int)slowgen04 == gclkgen);
        }
#endif
      break;
#endif /* HAVE_SERCOM0_4 */

#ifdef CONFIG_SAMD2L2_SERCOM5
    case 5:
      if (!configured5)
        {
          /* Configure the common slow clock channel */

          sam_gclk_chan_enable(GCLK_CHAN_SERCOM5_SLOW, gclkgen);

          /* The slow clock is now configured and should not be configured
           * again.
           */

          configured5 = true;
#ifdef CONFIG_DEBUG_ASSERTIONS
          slowgen5    = (uint8_t)gclkgen;
#endif
        }

#ifdef CONFIG_DEBUG_ASSERTIONS
      /* Already configured.  This is okay provided that the same GCLK
       * generator is being used.  Otherwise, there is a problem.
       */

      else
        {
          DEBUGASSERT((int)slowgen5 == gclkgen);
        }
#endif
      break;
#endif /* CONFIG_SAMD2L2_SERCOM5 */

    /* Unsupported or invalid SERCOM number provided */

    default:
      DEBUGPANIC();
      break;
    }

#elif defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)
  static bool configured = false;
  uint16_t regval;

  /* Since GCLK_SERCOM_SLOW is shared amongst all SERCOM modules, it should
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
       * We lock the SERCOM slow clock because it is common to all SERCOM
       * modules and, once set, should not be changed again.
       */

      regval |= (/* GCLK_CLKCTRL_WRTLOCK | */ GCLK_CLKCTRL_CLKEN);
      putreg16(regval, SAM_GCLK_CLKCTRL);

      /* Now we are configured */

      configured = true;
    }
#endif
}
