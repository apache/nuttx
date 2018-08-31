/****************************************************************************
 * arch/arm/src/samd5e5/sam_sercom.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <assert.h>

#include "up_arch.h"

#include "sam_config.h"

#include "chip.h"
#include "chip/sam_pm.h"
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

  /* Set up the SERCOMn_GCLK_ID_CORE clock */

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

  /* Setup the SERCOMn_GCLK channel. */

  if (!g_slowclk_configured)
    {
      /* Configure the slow clock channel.  The slow clock is shared for all
       * of SERCOM modules and, hence, only need to configured once.
       */

      sam_gclk_chan_enable(GCLK_CHAN_SERCOMn_SLOW, slowgen,
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
