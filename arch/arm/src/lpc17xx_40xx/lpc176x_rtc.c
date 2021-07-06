/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc176x_rtc.c
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

#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "chip.h"
#include "hardware/lpc17_40_syscon.h"

#include "lpc17_40_rtc.h"

#ifdef CONFIG_RTC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* This RTC implementation supports only date/time RTC hardware */

#ifndef CONFIG_RTC_DATETIME
#  error "CONFIG_RTC_DATETIME must be set to use this driver"
#endif

#ifdef CONFIG_RTC_HIRES
#  error "CONFIG_RTC_HIRES must NOT be set with this driver"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM
static alarmcb_t g_alarmcb;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_dumpregs
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumpregs(FAR const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("  DOM : %08x\n", (getreg32(LPC17_40_RTC_DOM) & RTC_DOM_MASK));
  rtcinfo("  DOW : %08x\n", (getreg32(LPC17_40_RTC_DOW) & RTC_DOW_MASK));
}
#else
#  define rtc_dumpregs(msg)
#endif

/****************************************************************************
 * Name: rtc_dumptime
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumptime(FAR struct tm *tp, FAR const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("  tm_sec: %08x\n", tp->tm_sec);
  rtcinfo("  tm_min: %08x\n", tp->tm_min);
  rtcinfo(" tm_hour: %08x\n", tp->tm_hour);
  rtcinfo(" tm_mday: %08x\n", tp->tm_mday);
  rtcinfo("  tm_mon: %08x\n", tp->tm_mon);
  rtcinfo(" tm_year: %08x\n", tp->tm_year);
}
#else
#  define rtc_dumptime(tp, msg)
#endif

/****************************************************************************
 * Name: rtc_setup
 *
 * Description:
 *   Performs first time configuration of the RTC.  A special value written
 *   into back-up register 0 will prevent this function from being called on
 *   sub-sequent resets or power up.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static int rtc_setup(void)
{
  uint32_t regval;

  /* Clear all register to be default */

  putreg32((uint32_t)0x00, LPC17_40_RTC_ILR);
  putreg32((uint32_t)0x00, LPC17_40_RTC_CCR);
  putreg32((uint32_t)0x00, LPC17_40_RTC_CIIR);
  putreg32((uint32_t)0xff, LPC17_40_RTC_AMR);
  putreg32((uint32_t)0x00, LPC17_40_RTC_CALIB);

  /* Enable power to the RTC module */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCRTC;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Enable counters */

  putreg32((uint32_t)0x01, LPC17_40_RTC_CCR);
  return OK;
}

/****************************************************************************
 * Name: rtc_interrupt
 *
 * Description:
 *    RTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int rtc_interrupt(int irq, void *context, FAR void *arg)
{
#warning "Missing logic"
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.
 *   This function is called once during the OS initialization sequence
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
  int ret;

  rtc_dumpregs("On reset");

  /* Attach the RTC interrupt handler */

#ifdef CONFIG_RTC_ALARM
  ret = irq_attach(LPC17_40_IRQ_RTC, rtc_interrupt, NULL);
  if (ret == OK)
    {
      up_enable_irq(LPC17_40_IRQ_RTC);
    }
#endif /* CONFIG_RTC_ALARM */

  /* Perform the one-time setup of the RTC */

  ret = rtc_setup();

  /* Configure RTC interrupt to catch alarm interrupts. All RTC interrupts
   * are connected to the EXTI controller.  To enable the RTC Alarm
   * interrupt, the following sequence is required:
   *
   * 1. Configure and enable the EXTI Line 17 in interrupt mode and select
   *    the rising edge sensitivity.
   * 2. Configure and enable the RTC_Alarm IRQ channel in the NVIC.
   * 3. Configure the RTC to generate RTC alarms (Alarm A or Alarm B).
   */

  g_rtc_enabled = true;
  rtc_dumpregs("After Initialization");
  return ret;
}

/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS
 *   during initialization to set up the system time when CONFIG_RTC and
 *   CONFIG_RTC_DATETIME are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.
 *   That sub-second accuracy is lost in this interface.  However, since the
 *   system time is reinitialized on each power-up/reset, there will be no
 *   timing inaccuracy in the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_getdatetime(FAR struct tm *tp)
{
  rtc_dumpregs("Reading Time");

  /* Convert the RTC time to fields in struct tm format.  All of the STM32
   * All of the ranges of values correspond between struct tm and the time
   * register.
   */

  tp->tm_sec  = ((getreg32(LPC17_40_RTC_SEC) & RTC_SEC_MASK));
  tp->tm_min  = ((getreg32(LPC17_40_RTC_MIN) & RTC_MIN_MASK));
  tp->tm_hour = ((getreg32(LPC17_40_RTC_HOUR) & RTC_HOUR_MASK));

  /* Now convert the RTC date to fields in struct tm format:
   * Days: 1-31 match in both cases.
   * Month: STM32 is 1-12, struct tm is 0-11.
   * Years: STM32 is 00-99, struct tm is years since 1900.
   *
   * Issue:  I am not sure what the STM32 years mean.  Are these the
   * years 2000-2099?  I'll assume so.
   */

  tp->tm_mday = ((getreg32(LPC17_40_RTC_DOM) & RTC_DOM_MASK));
  tp->tm_mon  = ((getreg32(LPC17_40_RTC_MONTH) & RTC_MONTH_MASK)) - 1;
  tp->tm_year = ((getreg32(LPC17_40_RTC_YEAR) & RTC_YEAR_MASK)-1900);

  rtc_dumptime(tp, "Returning");
  return OK;
}

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

int up_rtc_settime(FAR const struct timespec *tp)
{
  FAR struct tm newtime;

  /* Break out the time values
   * (not that the time is set only to units of seconds)
   */

  gmtime_r(&tp->tv_sec, &newtime);
  rtc_dumptime(&newtime, "Setting time");

  /* Then write the broken out values to the RTC */

  putreg32(((newtime.tm_sec) & RTC_SEC_MASK), LPC17_40_RTC_SEC);
  putreg32(((newtime.tm_min) & RTC_MIN_MASK), LPC17_40_RTC_MIN);
  putreg32(((newtime.tm_hour) & RTC_HOUR_MASK), LPC17_40_RTC_HOUR);
  putreg32(((newtime.tm_mday) & RTC_DOM_MASK), LPC17_40_RTC_DOM);
  putreg32((((newtime.tm_mon)+1) & RTC_MONTH_MASK), LPC17_40_RTC_MONTH);
  putreg32(((newtime.tm_year) & RTC_YEAR_MASK)+1900, LPC17_40_RTC_YEAR);

  return OK;
}

/****************************************************************************
 * Name: lpc17_40_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *   Up to two alarms can be supported (ALARM A and ALARM B).
 *
 * Input Parameters:
 *   tp - the time to set the alarm
 *   callback - the function to call when the alarm expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int lpc17_40_rtc_setalarm(FAR const struct timespec *tp, alarmcb_t callback)
{
  int ret = -EBUSY;

  /* Is there already something waiting on the ALARM? */

  if (g_alarmcb == NULL)
    {
      /* No.. Save the callback function pointer */

      g_alarmcb = callback;

      /* Break out the time values */
#warning "Missing logic"

      /* The set the alarm */
#warning "Missing logic"

      ret = OK;
    }

  return ret;
}
#endif

#endif /* CONFIG_RTC */
