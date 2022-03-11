/****************************************************************************
 * arch/arm/src/samd5e5/sam_sercom.c
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
#include <stdbool.h>
#include <assert.h>

#include "arm_internal.h"
#include "sam_config.h"

#include "chip.h"
#include "hardware/sam_pm.h"
#include "sam_gclk.h"
#include "sam_sercom.h"

#include <arch/board/board.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_slowclk_configured = false;
#ifdef CONFIG_DEBUG_ASSERTIONS
static uint8_t g_slowgen = 0xff;
#endif

static const uint8_t g_corclk_channel[SAMD5E5_NSERCOM] =
{
  GCLK_CHAN_SERCOM0_CORE
#if SAMD5E5_NSERCOM > 1
  , GCLK_CHAN_SERCOM1_CORE
#endif
#if SAMD5E5_NSERCOM > 2
  , GCLK_CHAN_SERCOM2_CORE
#endif
#if SAMD5E5_NSERCOM > 3
  , GCLK_CHAN_SERCOM3_CORE
#endif
#if SAMD5E5_NSERCOM > 4
  , GCLK_CHAN_SERCOM4_CORE
#endif
#if SAMD5E5_NSERCOM > 5
  , GCLK_CHAN_SERCOM5_CORE
#endif
#if SAMD5E5_NSERCOM > 6
  , GCLK_CHAN_SERCOM6_CORE
#endif
#if SAMD5E5_NSERCOM > 7
  , GCLK_CHAN_SERCOM7_CORE
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sercom_enable
 *
 * Description:
 *   Enable clocking to a SERCOM module
 *
 * Assumptions/Limitation:
 *   This operation is global and atomic.  Interrupts will be masked.
 *
 ****************************************************************************/

void sercom_enable(int sercom)
{
  DEBUGASSERT((unsigned)sercom < SAMD5E5_NSERCOM);

  switch (sercom)
    {
      case 0:
        sam_apb_sercom0_enableperiph();
        break;

      case 1:
        sam_apb_sercom1_enableperiph();
        break;

      case 2:
        sam_apb_sercom2_enableperiph();
        break;

      case 3:
        sam_apb_sercom3_enableperiph();
        break;

      case 4:
        sam_apb_sercom4_enableperiph();
        break;

      case 5:
        sam_apb_sercom5_enableperiph();
        break;

      case 6:
        sam_apb_sercom6_enableperiph();
        break;

      case 7:
        sam_apb_sercom7_enableperiph();
        break;

      default:
        break;
    }
}

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

void sercom_coreclk_configure(int sercom, int coregen, bool wrlock)
{
  uint8_t corechan;

  DEBUGASSERT((unsigned)sercom < SAMD5E5_NSERCOM);

  /* Set up the SERCOMN_GCLK_ID_CORE clock */

  corechan = g_corclk_channel[sercom];
  sam_gclk_chan_enable(corechan, coregen, wrlock);
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

void sercom_slowclk_configure(int sercom, int slowgen)
{
  DEBUGASSERT((unsigned)sercom < SAMD5E5_NSERCOM);

  /* Setup the SERCOMN_GCLK channel. */

  if (!g_slowclk_configured)
    {
      /* Configure the slow clock channel.  The slow clock is shared for all
       * of SERCOM modules and, hence, only need to configured once.
       */

      sam_gclk_chan_enable(GCLK_CHAN_SERCOMN_SLOW, slowgen,
                           BOARD_SERCOM_SLOWLOCK);

      /* The slow clock is now configured and should not be re=configured
       * again.
       */

       g_slowclk_configured = true;
#ifdef CONFIG_DEBUG_ASSERTIONS
       g_slowgen = (uint8_t)slowgen;
#endif
    }

#ifdef CONFIG_DEBUG_ASSERTIONS
  /* Already g_slowclk_configured.  This is okay provided that the same GCLK
   * generator is being used.  Otherwise, there is a problem.
   */

  else
    {
      DEBUGASSERT((int)g_slowgen == slowgen);
    }
#endif
}
