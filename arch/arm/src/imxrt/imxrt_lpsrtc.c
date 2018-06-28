/************************************************************************************
 * arch/arm/src/imxrt/imxrt_lpsrtc.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/rtc.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "chip/imxrt_snvs.h"
#include "imxrt_periphclks.h"
#include "imxrt_hprtc.h"
#include "imxrt_lpsrtc.h"

#ifdef CONFIG_IMXRT_SNVS_LPSRTC

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: imxrt_lpsrtc_enable
 *
 * Description:
 *   Enable/start the LPRTC time counter.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void imxrt_lpsrtc_enable(void)
{
  uint32_t regval;

  /* Enable the LPSRTC */

  regval  = getreg32(IMXRT_SNVS_LPCR);
  regval |= SNVS_LPCR_MCENV;
  putreg32(regval, IMXRT_SNVS_LPCR);

  while ((getreg32(IMXRT_SNVS_LPCR) & SNVS_LPCR_MCENV) == 0)
    {
    }
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: imxrt_lpsrtc_initialize
 *
 * Description:
 *   Initialize the LPSRTC per the selected configuration.  This function is called
 *   via up_rtc_initialize (see imxrt_hprtc.c).
 *
 *   NOTE that the LPSRTC is always configured synchronized with the HPRTC.  This
 *   means that the time is set via the LPSRTC but read via the HPRTC.  Also, only
 *   the alarms from the HPRTC are used.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int imxrt_lpsrtc_initialize(void)
{
#ifdef CONFIG_IMXRTC_LPSRTC_CALENABLE
  uint32_t regval;
#endif

  /* Initialize the HPRTC */

  imxrt_hprtc_initialize();

  /* Enable clocking to the the SNVS LP module.
   * Clock is on during all modes, except STOP mode.
   */

  imxrt_clockall_snvs_lp();

#ifdef CONFIG_IMXRTC_LPSRTC_CALENABLE
  /* Set the LPSRTC calibration value */

  regval  = getreg32(IMXRT_SNVS_LPCR);
  regval &= ~SNVS_LPCR_LPCALBVAL_MASK;
  regval |= SNVS_LPCR_LPCALBVAL(CONFIG_IMXRTC_LPSRTC_CALVALUE);
  regval |= SNVS_LPCR_LPCALBEN;
  putreg32(regval, IMXRT_SNVS_LPCR);
#endif

  /* Disable tamper pins.
   * TODO:  I don't think this applies the currently supported device.
   */

  /* Enable the LPSRTC */

  imxrt_lpsrtc_enable();

  /* Synchronize HPRTC time with the LPSRTC and enable HPRTC */

  imxrt_hprtc_synchronize();
  return OK;
}

/************************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution RTC/counter
 *   hardware implementation selected.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC is set but neither
 *   CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ************************************************************************************/

#ifndef CONFIG_RTC_HIRES
time_t up_rtc_time(void)
{
  /* Delegate to imxrt_hprtc_time() */

  return imxrt_hprtc_time();
}
#endif

/************************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able to
 *   set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_settime(FAR const struct timespec *ts)
{
  uint32_t regval;

  DEBUGASSERT(ts != NULL);

  /* Disable the LPSRTC */

  regval  = getreg32(IMXRT_SNVS_LPCR);
  regval &= ~SNVS_LPCR_MCENV;
  putreg32(regval, IMXRT_SNVS_LPCR);

  while ((getreg32(IMXRT_SNVS_LPCR) & SNVS_LPCR_MCENV) != 0)
    {
    }

  /* Set LPSRTC time in seconds.  We could do better by accounting for the
   * ts->tv_nsec unused residual.
   *
   * IMXRT_SNVS_LPSMCMR Bits 9-14 = 15-bit MSB of alarm setting.
   * IMXRT_SNVS_LPSMCLR 32-bit LSB of alarm setting.
   */

  putreg32((uint32_t)ts->tv_sec >> 17, IMXRT_SNVS_LPSMCMR);
  putreg32((uint32_t)ts->tv_sec << 15, IMXRT_SNVS_LPSMCLR);

  /* The time has been set */

  putreg32(CONFIG_IMXRT_RTC_MAGIC,
           IMXRT_SNVS_LPGPR(CONFIG_IMXRT_RTC_MAGIC_REG));

  /* Unconditionally re-enable the LPSRTC */

  imxrt_lpsrtc_enable();
  return OK;
}

/****************************************************************************
 * Name: imxrt_lpsrtc_havesettime
 *
 * Description:
 *   Check if the LPSRTC time has been set
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

bool imxrt_lpsrtc_havesettime(void)
{
  return (getreg32(IMXRT_SNVS_LPGPR(CONFIG_IMXRT_RTC_MAGIC_REG)) ==
          CONFIG_IMXRT_RTC_MAGIC);
}
#endif /* CONFIG_IMXRT_SNVS_LPSRTC */
