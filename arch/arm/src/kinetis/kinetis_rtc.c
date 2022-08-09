/****************************************************************************
 * arch/arm/src/kinetis/kinetis_rtc.c
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/rtc.h>
#include <arch/board/board.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include "arm_internal.h"
#include "kinetis_config.h"
#include "chip.h"
#include "hardware/kinetis_rtc.h"
#include "hardware/kinetis_sim.h"
#include "kinetis.h"
#include "kinetis_alarm.h"

#if defined(CONFIG_RTC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(BOARD_RTC_CAP)
/* Capacitance values 8pF if not already defined */

#  define BOARD_RTC_CAP RTC_CR_SC8P | RTC_CR_SC4P
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static alarmcb_t g_alarmcb;
static bool rtc_irq_state = false;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
static void rtc_dumpregs(const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("   TSR: %08x\n", getreg32(KINETIS_RTC_TSR));
  rtcinfo("   TPR: %08x\n", getreg32(KINETIS_RTC_TPR));
  rtcinfo("   TAR: %08x\n", getreg32(KINETIS_RTC_TAR));
  rtcinfo("    CR: %08x\n", getreg32(KINETIS_RTC_CR));
  rtcinfo("    SR: %08x\n", getreg32(KINETIS_RTC_SR));
  rtcinfo("    LR: %08x\n", getreg32(KINETIS_RTC_LR));
  rtcinfo("   IER: %08x\n", getreg32(KINETIS_RTC_IER));
#if defined(KINETIS_RTC_GEN2)
  rtcinfo("  TTSR: %08x\n", getreg32(KINETIS_RTC_TTSR));
  rtcinfo("   MER: %08x\n", getreg32(KINETIS_RTC_MER));
  rtcinfo("  MCLR: %08x\n", getreg32(KINETIS_RTC_MCLR));
  rtcinfo("  MCHR: %08x\n", getreg32(KINETIS_RTC_MCHR));
  rtcinfo("   WAR: %08x\n", getreg32(KINETIS_RTC_WAR));
  rtcinfo("   RAR: %08x\n", getreg32(KINETIS_RTC_RAR));
#endif
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
static void rtc_dumptime(struct tm *tp, const char *msg)
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
 * Name: kinetis_rtc_interrupt
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

#if defined(CONFIG_RTC_ALARM)
static int kinetis_rtc_interrupt(int irq, void *context, void *arg)
{
  uint16_t rtc_sr;

  /* if alarm */

  rtc_sr = getreg32(KINETIS_RTC_SR);
  if (rtc_sr & RTC_SR_TAF)
    {
      if (g_alarmcb != NULL)
        {
          /* Alarm callback */

          g_alarmcb();
          g_alarmcb = NULL;
        }
    }
  else
    {
      /* other interrupts are serious and should leave a turd
       *
       * RTC_SR_TIF   _TOF   _MOF
       */

      rtcwarn("unexp int src=0x%x, num=", rtc_sr);
    }

  /* Clear pending flags, disable alarm */

  putreg32(0, KINETIS_RTC_TAR); /* Unset alarm (resets flags) */
  putreg32(0, KINETIS_RTC_IER); /* Disable alarm interrupt */

  return 0;
}
#endif

/****************************************************************************
 * Name: rtc_reset
 *
 * Description:
 *    Reset the RTC to known state
 *
 * Input Parameters:
 *    none
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static inline void rtc_reset(void)
{
  putreg32((RTC_CR_SWR | getreg32(KINETIS_RTC_CR)), KINETIS_RTC_CR);
  putreg32((~RTC_CR_SWR & getreg32(KINETIS_RTC_CR)), KINETIS_RTC_CR);

  /* Set TSR register to 0x1 to avoid the timer invalid (TIF) bit being
   * set in the SR register
   */

  putreg32(1, KINETIS_RTC_TSR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_irqinit
 *
 * Description:
 *   Initialize the hardware RTC irq.
 *   This only needs to be called once when first used.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_ALARM)
int up_rtc_irq_attach(void)
{
  uint32_t rtc_sr;

  if (!rtc_irq_state)
    {
      rtc_irq_state = true;

      /* Clear TAF if pending */

      rtc_sr = getreg32(KINETIS_RTC_SR);
      if ((rtc_sr & RTC_SR_TAF) != 0)
        {
          putreg32(0, KINETIS_RTC_TAR);
        }

      /* Enable alarm interrupts.
       * This will not work if part of up_rtc_initialize()
       * as it is called very early in initialization BEFORE the interrupt
       * system will be enabled.  All interrupts will disabled later when
       * the interrupt system is disabled. This must be done later when the
       * alarm is first set.
       *
       * KINETIS_IRQ_RTCS is a separate interrupt for seconds if needed
       */

      irq_attach(KINETIS_IRQ_RTC, kinetis_rtc_interrupt, NULL);
      up_enable_irq(KINETIS_IRQ_RTC);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This
 *   function is called once during the OS initialization sequence
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
  bool rtc_valid = false;

  /* Enable RTC module */

  regval = getreg32(KINETIS_SIM_SCGC6);
  regval |= SIM_SCGC6_RTC;
  putreg32(regval, KINETIS_SIM_SCGC6);

  regval = getreg32(KINETIS_RTC_SR);
  if (!(regval & RTC_SR_TIF))
    {
#ifdef KINETIS_RTC_GEN2
      /* Check if the one-time initialization of the RTC has already been
       * performed. We can determine this by checking if the magic number
       * has been written to the back-up date register DR0.
       */

      regval = getreg32(KINETIS_RTC_MCLR);
      if ((CONFIG_RTC_MAGICL == regval) &&
          (CONFIG_RTC_MAGICH == getreg32(KINETIS_RTC_MCHR)))
#endif
        {
          rtc_valid = true;
        }
    }

  if (rtc_valid)
    {
      rtcinfo("Do resume\n");

      /* RTC already set-up, just resume normal operation */

      rtc_dumpregs("Did resume");
    }
  else
    {
      rtcinfo("Do setup\n");
      rtc_reset();

#ifdef KINETIS_RTC_GEN2
      /* Configure the RTC to be initialized */

      putreg32(CONFIG_RTC_MAGICL, KINETIS_RTC_MCLR);
      putreg32(CONFIG_RTC_MAGICH, KINETIS_RTC_MCHR);
#endif

      /* Setup the update mode and supervisor access mode */

      putreg32((~(RTC_CR_UM | RTC_CR_SUP) & getreg32(KINETIS_RTC_CR)),
               KINETIS_RTC_CR);

      /* Disable counters (just in case) */

      putreg32(0, KINETIS_RTC_SR);

      /* Enable oscilator - must have Vbat else hard fault */

      putreg32((BOARD_RTC_CAP | RTC_CR_OSCE), KINETIS_RTC_CR);

      /* TODO - add capability to accurately tune RTC
       * This is a per individual board customization and requires
       * parameters to be configurable and stored in non-volatile eg flash.
       */

      /* TODO: delay some time (1024 cycles? would be 30ms) */
    }

  /* Disable interrupts */

  putreg32(0, KINETIS_RTC_IER);

  /* Reset flags requires writing the seconds register, the following line
   * avoids altering any stored time value.
   */

  putreg32(getreg32(KINETIS_RTC_TSR), KINETIS_RTC_TSR);

  /* Enable counters */

  putreg32(RTC_SR_TCE, KINETIS_RTC_SR);

  /* Mark RTC enabled */

  g_rtc_enabled = true;

  return OK;
}

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard
 *   time() function.  This interface is only required if the low-resolution
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
  return getreg32(KINETIS_RTC_TSR);
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
      prescaler = getreg32(KINETIS_RTC_TPR);
      seconds = getreg32(KINETIS_RTC_TSR);
      prescaler2 = getreg32(KINETIS_RTC_TPR);
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

int up_rtc_settime(const struct timespec *tp)
{
  irqstate_t flags;
  uint32_t seconds;
  uint32_t prescaler;

  seconds = tp->tv_sec;
  prescaler = tp->tv_nsec / (1000000000 / CONFIG_RTC_FREQUENCY);

  flags = enter_critical_section();

  putreg32(0, KINETIS_RTC_SR);  /* Disable counter */

  putreg32(prescaler, KINETIS_RTC_TPR); /* Always write prescaler first */
  putreg32(seconds, KINETIS_RTC_TSR);

  putreg32(RTC_SR_TCE, KINETIS_RTC_SR); /* Re-enable counter */

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: kinetis_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
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
int kinetis_rtc_setalarm(const struct timespec *tp, alarmcb_t callback)
{
  /* Is there already something waiting on the ALARM? */

  if (g_alarmcb == NULL)
    {
      /* No.. Save the callback function pointer */

      g_alarmcb = callback;

      /* ensure irq is attached */

      up_rtc_irq_attach();

      /* Enable and set RTC alarm */

      putreg32(tp->tv_sec, KINETIS_RTC_TAR);    /* Set alarm (also resets
                                                 * flags) */
      putreg32(RTC_IER_TAIE, KINETIS_RTC_IER);  /* Enable alarm interrupt */

      rtc_dumpregs("set alarmtime");

      return OK;
    }
  else
    {
      return -EBUSY;
    }
}
#endif

/****************************************************************************
 * Name: kinetis_rtc_cancelalarm
 *
 * Description:
 *   Cancel a pending alarm alarm
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int kinetis_rtc_cancelalarm(void)
{
  if (g_alarmcb != NULL)
    {
      /* Cancel the global callback function */

      g_alarmcb = NULL;

      /* Unset the alarm */

      putreg32(0, KINETIS_RTC_IER);     /* Disable alarm interrupt */
      return OK;
    }
  else
    {
      return -ENODATA;
    }
}
#endif

/****************************************************************************
 * Name: kinetis_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  tp - Location to return the timer match register.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int kinetis_rtc_rdalarm(struct timespec *tp)
{
  DEBUGASSERT(tp != NULL);

  tp->tv_sec = getreg32(KINETIS_RTC_TAR);
  tp->tv_nsec = 0;
  return OK;
}
#endif

#endif /* KINETIS_RTC */
