/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_rtc.c
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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/rtc.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/s32k1xx_rtc.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_rtc.h"

#ifdef CONFIG_S32K1XX_RTC

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Variable determines the state of the RTC module.
 *
 * After initialization value is set to 'true' if RTC starts successfully.
 * The value can be changed to false also during operation if RTC for
 * some reason fails.
 */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_rtc_enable
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
 ****************************************************************************/

static void s32k1xx_rtc_enable(void)
{
  uint32_t regval;

  /* Enable the rtc */

  regval  = getreg32(S32K1XX_RTC_SR);
  regval |= RTC_SR_TCE;
  putreg32(regval, S32K1XX_RTC_SR);
}

/****************************************************************************
 * Name: s32k1xx_rtc_disable
 *
 * Description:
 *   disable the LPRTC time counter.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void s32k1xx_rtc_disable(void)
{
  uint32_t regval;

  /* Enable the rtc */

  regval  = getreg32(S32K1XX_RTC_SR);
  regval &= ~RTC_SR_TCE;
  putreg32(regval, S32K1XX_RTC_SR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the rtc per the selected configuration.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  uint32_t regval;

  /* Disable the clock out pin */

  regval  = getreg32(S32K1XX_RTC_CR);

  regval &= ~RTC_CR_CPE;

  putreg32(regval, S32K1XX_RTC_CR);

  /* Increment on 32.768Khz clock */

  regval  = getreg32(S32K1XX_RTC_CR);

  regval &= ~RTC_CR_LPOS;

  putreg32(regval, S32K1XX_RTC_CR);

  /* Set Update mode */

  regval  = getreg32(S32K1XX_RTC_CR);

  regval &= ~(RTC_CR_UM);

  putreg32(regval, S32K1XX_RTC_CR);

  /* Set Non-Supervisor access mode */

  regval  = getreg32(S32K1XX_RTC_CR);

  regval &= ~(RTC_CR_SUP);

  putreg32(regval, S32K1XX_RTC_CR);

  regval  = getreg32(S32K1XX_RTC_SR);

  if (regval & RTC_SR_TIF)
    {
      regval &= ~RTC_SR_TCE;
      putreg32(regval, S32K1XX_RTC_SR);

      /* Write TSR register to clear invalid */

      putreg32(0x0, S32K1XX_RTC_TSR);
    }

  /* Enable the rtc */

  s32k1xx_rtc_enable();

  g_rtc_enabled = true;

  return OK;
}

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution
 *   RTC/counter hardware implementation selected.  It is only used by the
 *   RTOS during initialization to set up the system time when CONFIG_RTC is
 *   set but neither CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ****************************************************************************/
#ifndef CONFIG_RTC_HIRES
time_t up_rtc_time(void)
{
  uint32_t regval;

  regval  = getreg32(S32K1XX_RTC_TSR);
  regval &= RTC_TSR_MASK;

  return (uint32_t) (regval);
}
#endif

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC clock/counter.  This
 *   interface is only supported by the high-resolution RTC/counter hardware
 *   implementation. It is used to replace the system timer.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_HIRES
int up_rtc_gettime(struct timespec *tp)
{
  irqstate_t flags;
  uint32_t seconds;
  uint32_t prescaler;
  uint32_t prescaler2;

  /* Get prescaler and seconds register. this is in a loop which ensures that
   * registers will be re-read if during the reads the prescaler has
   * wrapped-around.
   */

  flags = enter_critical_section();
  do
    {
      prescaler = getreg32(S32K1XX_RTC_TPR);
      seconds = getreg32(S32K1XX_RTC_TSR);
      prescaler2 = getreg32(S32K1XX_RTC_TPR);
    }
  while (prescaler > prescaler2);

  leave_critical_section(flags);

  /* Build seconds + nanoseconds from seconds and prescaler register */

  tp->tv_sec = seconds;
  tp->tv_nsec = prescaler * (1000000000 / CONFIG_RTC_FREQUENCY);
  return OK;
}
#endif

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able
 *   to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(const struct timespec *ts)
{
  DEBUGASSERT(ts != NULL);

  irqstate_t flags;
  uint32_t seconds;
  uint32_t prescaler;

  seconds = ts->tv_sec;
#ifdef CONFIG_RTC_HIRES
  prescaler = ts->tv_nsec / (1000000000 / CONFIG_RTC_FREQUENCY);
#else
  prescaler = 0;
#endif

  flags = enter_critical_section();

  s32k1xx_rtc_disable();

  putreg32(prescaler, S32K1XX_RTC_TPR); /* Always write prescaler first */
  putreg32(seconds, S32K1XX_RTC_TSR);

  s32k1xx_rtc_enable();

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: s32k1xx_rtc_havesettime
 *
 * Description:
 *   Check if the rtc time has been set
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

bool s32k1xx_rtc_havesettime(void)
{
  return 1; /* TODO */
}
#endif /* CONFIG_s32k1xx_SNVS_rtc */
