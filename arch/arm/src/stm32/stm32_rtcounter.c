/****************************************************************************
 * arch/arm/src/stm32/stm32_rtcounter.c
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

/* The STM32 RTC Driver offers standard precision of 1 Hz or High Resolution
 * operating at rate up to 16384 Hz. It provides UTC time and alarm interface
 * with external output pin (for wake-up).
 *
 * RTC is based on hardware RTC module which is located in a separate power
 * domain. The 32-bit counter is extended by 16-bit registers in BKP domain
 * STM32_BKP_DR1 to provide system equiv. function to the: time_t time
 * (time_t *).
 *
 * Notation:
 *  - clock refers to 32-bit hardware counter
 *  - time is a combination of clock and upper bits stored in backuped domain
 *    with unit of 1 [s]
 *
 * TODO:
 * Error Handling in case LSE fails during start-up or during operation.
 */

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
#include <errno.h>

#include "arm_internal.h"
#include "stm32_pwr.h"
#include "stm32_rcc.h"
#include "stm32_rtc.h"
#include "stm32_waste.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* In hi-res mode, the RTC operates at 16384Hz.  Overflow interrupts are
 * handled when the 32-bit RTC counter overflows every 3 days and 43 minutes.
 * A BKP register is incremented on each overflow interrupt creating,
 * effectively, a 48-bit RTC counter.
 *
 * In the lo-res mode, the RTC operates at 1Hz.  Overflow interrupts are not
 * handled (because the next overflow is not expected until the year 2106).
 *
 * WARNING:
 * Overflow interrupts are lost whenever the STM32 is powered down.  The
 * overflow interrupt may be lost even if the STM32 is powered down only
 * momentarily. Therefore hi-res solution is only useful in systems where
 * the power is always on.
 */

#ifdef CONFIG_RTC_HIRES
#  ifndef CONFIG_RTC_FREQUENCY
#    error "CONFIG_RTC_FREQUENCY is required for CONFIG_RTC_HIRES"
#  elif CONFIG_RTC_FREQUENCY != 16384
#    error "Only hi-res CONFIG_RTC_FREQUENCY of 16384Hz is supported"
#  endif
#else
#  ifndef CONFIG_RTC_FREQUENCY
#    define CONFIG_RTC_FREQUENCY 1
#  endif
#  if CONFIG_RTC_FREQUENCY != 1
#    error "Only lo-res CONFIG_RTC_FREQUENCY of 1Hz is supported"
#  endif
#endif

#ifndef CONFIG_STM32_BKP
#  error "CONFIG_STM32_BKP is required for CONFIG_STM32_RTC"
#endif

#ifndef CONFIG_STM32_PWR
#  error "CONFIG_STM32_PWR is required for CONFIG_STM32_RTC"
#endif

#ifdef CONFIG_STM32_STM32F10XX
#  if defined(CONFIG_STM32_RTC_HSECLOCK)
#    error "RTC with HSE clock not yet implemented for STM32F10XXX"
#  elif defined(CONFIG_STM32_RTC_LSICLOCK)
#    error "RTC with LSI clock not yet implemented for STM32F10XXX"
#  endif
#endif

/* RTC/BKP Definitions ******************************************************/

/* STM32_RTC_PRESCALAR_VALUE
 *   RTC pre-scalar value.  The RTC is driven by a 32,768Hz input clock.
 *   This input value is divided by this value (plus one) to generate the
 *   RTC frequency.
 * RTC_TIMEMSB_REG
 *   The BKP module register used to hold the RTC overflow value.
 *   Overflows are only handled in hi-res mode.
 * RTC_CLOCKS_SHIFT
 *   The shift used to convert the hi-res timer LSB to one second.
 *   Not used with the lo-res timer.
 */

#ifdef CONFIG_RTC_HIRES
#  define STM32_RTC_PRESCALAR_VALUE STM32_RTC_PRESCALER_MIN
#  define RTC_TIMEMSB_REG           STM32_BKP_DR1
#  define RTC_CLOCKS_SHIFT          14
#else
#  define STM32_RTC_PRESCALAR_VALUE STM32_RTC_PRESCALER_SECOND
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rtc_regvals_s
{
  uint16_t cntl;
  uint16_t cnth;
#ifdef CONFIG_RTC_HIRES
  uint16_t ovf;
#endif
};

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

/* Variable determines the state of the LSE oscillator.
 * Possible errors:
 *   - on start-up
 *   - during operation, reported by LSE interrupt
 */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rtc_beginwr
 *
 * Description:
 *   Enter configuration mode
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_rtc_beginwr(void)
{
  /* Previous write is done? */

  while ((getreg16(STM32_RTC_CRL) & RTC_CRL_RTOFF) == 0)
    {
      stm32_waste();
    }

  /* Enter Config mode, Set Value and Exit */

  modifyreg16(STM32_RTC_CRL, 0, RTC_CRL_CNF);
}

/****************************************************************************
 * Name: stm32_rtc_endwr
 *
 * Description:
 *   Exit configuration mode
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_rtc_endwr(void)
{
  modifyreg16(STM32_RTC_CRL, RTC_CRL_CNF, 0);

  /* Wait for the write to actually reach RTC registers */

  while ((getreg16(STM32_RTC_CRL) & RTC_CRL_RTOFF) == 0)
    {
      stm32_waste();
    }
}

/****************************************************************************
 * Name: stm32_rtc_wait4rsf
 *
 * Description:
 *   Wait for registers to synchronise with RTC module, call after power-up
 *   only
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_rtc_wait4rsf(void)
{
  modifyreg16(STM32_RTC_CRL, RTC_CRL_RSF, 0);
  while ((getreg16(STM32_RTC_CRL) & RTC_CRL_RSF) == 0)
    {
      stm32_waste();
    }
}

/****************************************************************************
 * Name: stm32_rtc_breakout
 *
 * Description:
 *   Set the RTC to the provided time.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_HIRES
static void stm32_rtc_breakout(const struct timespec *tp,
                               struct rtc_regvals_s *regvals)
{
  uint64_t frac;
  uint32_t cnt;
  uint16_t ovf;

  /* Break up the time in seconds + milleconds into the correct values for
   * our use
   */

  frac = ((uint64_t)tp->tv_nsec * CONFIG_RTC_FREQUENCY) / 1000000000;
  cnt  = (tp->tv_sec << RTC_CLOCKS_SHIFT) |
         ((uint32_t)frac & (CONFIG_RTC_FREQUENCY - 1));
  ovf  = (tp->tv_sec >> (32 - RTC_CLOCKS_SHIFT));

  /* Then return the broken out time */

  regvals->cnth = cnt >> 16;
  regvals->cntl = cnt & 0xffff;
  regvals->ovf  = ovf;
}
#else
static inline void stm32_rtc_breakout(const struct timespec *tp,
                                      struct rtc_regvals_s *regvals)
{
  /* The low-res timer is easy... tv_sec holds exactly the value needed
   * by the CNTH/CNTL registers.
   */

  regvals->cnth = (uint16_t)((uint32_t)tp->tv_sec >> 16);
  regvals->cntl = (uint16_t)((uint32_t)tp->tv_sec & 0xffff);
}
#endif

/****************************************************************************
 * Name: stm32_rtc_interrupt
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

#if defined(CONFIG_RTC_HIRES) || defined(CONFIG_RTC_ALARM)
static int stm32_rtc_interrupt(int irq, void *context, void *arg)
{
  uint16_t source = getreg16(STM32_RTC_CRL);

#ifdef CONFIG_RTC_HIRES
  if ((source & RTC_CRL_OWF) != 0)
    {
      stm32_pwr_enablebkp(true);
      putreg16(getreg16(RTC_TIMEMSB_REG) + 1, RTC_TIMEMSB_REG);
      stm32_pwr_enablebkp(false);
    }
#endif

#ifdef CONFIG_RTC_ALARM
  if ((source & RTC_CRL_ALRF) != 0 && g_alarmcb != NULL)
    {
      /* Alarm callback */

      g_alarmcb();
      g_alarmcb = NULL;
    }
#endif

  /* Clear pending flags, leave RSF high */

  putreg16(RTC_CRL_RSF, STM32_RTC_CRL);
  return 0;
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
  uint32_t regval;

  /* Enable write access to the backup domain (RTC registers, RTC backup data
   * registers and backup SRAM).
   */

  stm32_pwr_enablebkp(true);

  regval = getreg32(RTC_MAGIC_REG);
  if (regval != RTC_MAGIC && regval != RTC_MAGIC_TIME_SET)
    {
      /* Reset backup domain if bad magic */

      modifyreg32(STM32_RCC_BDCR, 0, RCC_BDCR_BDRST);
      modifyreg32(STM32_RCC_BDCR, RCC_BDCR_BDRST, 0);
      putreg16(RTC_MAGIC, RTC_MAGIC_REG);
    }

  /* Select the lower power external 32,768Hz (Low-Speed External, LSE)
   * oscillator as RTC Clock Source and enable the Clock.
   */

  modifyreg16(STM32_RCC_BDCR, RCC_BDCR_RTCSEL_MASK, RCC_BDCR_RTCSEL_LSE);

  /* Enable RTC and wait for RSF */

  modifyreg16(STM32_RCC_BDCR, 0, RCC_BDCR_RTCEN);

  /* TODO: Possible stall?
   * should we set the timeout period? and return with -1
   */

  stm32_rtc_wait4rsf();

  /* Configure prescaler, note that these are write-only registers */

  stm32_rtc_beginwr();
  putreg16(STM32_RTC_PRESCALAR_VALUE >> 16,    STM32_RTC_PRLH);
  putreg16(STM32_RTC_PRESCALAR_VALUE & 0xffff, STM32_RTC_PRLL);
  stm32_rtc_endwr();

  stm32_rtc_wait4rsf();

#ifdef CONFIG_RTC_HIRES
  /* Enable overflow interrupt - alarm interrupt is enabled in
   * stm32_rtc_setalarm.
   */

  modifyreg16(STM32_RTC_CRH, 0, RTC_CRH_OWIE);
#endif

  /* TODO: Get state from this function, if everything is
   *   okay and whether it is already enabled (if it was disabled
   *   reset upper time register)
   */

  g_rtc_enabled = true;

  /* Alarm Int via EXTI Line */

  /* STM32_IRQ_RTCALRM  41: RTC alarm through EXTI line interrupt */

  /* Disable write access to the backup domain
   * (RTC registers, RTC backup data registers and backup SRAM).
   */

  stm32_pwr_enablebkp(false);

  return OK;
}

/****************************************************************************
 * Name: stm32_rtc_irqinitialize
 *
 * Description:
 *   Initialize IRQs for RTC, not possible during up_rtc_initialize because
 *   up_irqinitialize is called later.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32_rtc_irqinitialize(void)
{
#if defined(CONFIG_RTC_HIRES) || defined(CONFIG_RTC_ALARM)
  /* Configure RTC interrupt to catch overflow and alarm interrupts. */

  irq_attach(STM32_IRQ_RTC, stm32_rtc_interrupt, NULL);
  up_enable_irq(STM32_IRQ_RTC);
#endif

  return OK;
}

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.
 *   This is similar to the standard time() function.
 *   This interface is only required if the low-resolution RTC/counter
 *   hardware implementation selected.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC is set but
 *   neither CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
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
  irqstate_t flags;
  uint16_t cnth;
  uint16_t cntl;
  uint16_t tmp;

  /* The RTC counter is read from two 16-bit registers to form one 32-bit
   * value.  Because these are non-atomic operations, many things can happen
   * between the two reads:  This thread could get suspended or interrupted
   * or the lower 16-bit counter could rollover between reads.  Disabling
   * interrupts will prevent suspensions and interruptions:
   */

  flags = enter_critical_section();

  /* And the following loop will handle any clock rollover events that may
   * happen between samples.  Most of the time (like 99.9%), the following
   * loop will execute only once.  In the rare rollover case, it should
   * execute no more than 2 times.
   */

  do
    {
      tmp  = getreg16(STM32_RTC_CNTL);
      cnth = getreg16(STM32_RTC_CNTH);
      cntl = getreg16(STM32_RTC_CNTL);
    }

  /* The second sample of CNTL could be less than the first sample of CNTL
   * only if rollover occurred.  In that case, CNTH may or may not be out
   * of sync.  The best thing to do is try again until we know that no
   * rollover occurred.
   */

  while (cntl < tmp);
  leave_critical_section(flags);

  /* Okay.. the samples should be as close together in time as possible and
   * we can be assured that no clock rollover occurred between the samples.
   *
   * Return the time in seconds.
   */

  return (time_t)cnth << 16 | (time_t)cntl;
}
#endif

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC clock/counter.  This
 *   interface is only supported by the high-resolution RTC/counter hardware
 *   implementation.
 *   It is used to replace the system timer.
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
  uint32_t ls;
  uint32_t ms;
  uint16_t ovf;
  uint16_t cnth;
  uint16_t cntl;
  uint16_t tmp;

  /* The RTC counter is read from two 16-bit registers to form one 32-bit
   * value.  Because these are non-atomic operations, many things can happen
   * between the two reads:  This thread could get suspended or interrupted
   * or the lower 16-bit counter could rollover between reads.  Disabling
   * interrupts will prevent suspensions and interruptions:
   */

  flags = enter_critical_section();

  /* And the following loop will handle any clock rollover events that may
   * happen between samples.  Most of the time (like 99.9%), the following
   * loop will execute only once.  In the rare rollover case, it should
   * execute no more than 2 times.
   */

  do
    {
      tmp  = getreg16(STM32_RTC_CNTL);
      cnth = getreg16(STM32_RTC_CNTH);
      ovf  = getreg16(RTC_TIMEMSB_REG);
      cntl = getreg16(STM32_RTC_CNTL);
    }

  /* The second sample of CNTL could be less than the first sample of CNTL
   * only if rollover occurred.  In that case, CNTH may or may not be out
   * of sync.  The best thing to do is try again until we know that no
   * rollover occurred.
   */

  while (cntl < tmp);
  leave_critical_section(flags);

  /* Okay.. the samples should be as close together in time as possible and
   * we can be assured that no clock rollover occurred between the samples.
   *
   * Create a 32-bit value from the LS and MS 16-bit RTC counter values and
   * from the MS and overflow 16-bit counter values.
   */

  ls = (uint32_t)cnth << 16 | (uint32_t)cntl;
  ms = (uint32_t)ovf  << 16 | (uint32_t)cnth;

  /* Then we can save the time in seconds and fractional seconds. */

  tp->tv_sec  = (ms << (32 - RTC_CLOCKS_SHIFT - 16)) |
                (ls >> (RTC_CLOCKS_SHIFT + 16));
  tp->tv_nsec = (ls & (CONFIG_RTC_FREQUENCY - 1)) *
                (1000000000 / CONFIG_RTC_FREQUENCY);
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
  struct rtc_regvals_s regvals;
  irqstate_t flags;
  uint16_t cntl;

  /* Break out the time values */

  stm32_rtc_breakout(tp, &regvals);

  /* Enable write access to the backup domain */

  flags = enter_critical_section();
  stm32_pwr_enablebkp(true);

  /* Then write the broken out values to the RTC counter and BKP overflow
   * register (hi-res mode only)
   */

  do
    {
      stm32_rtc_beginwr();
      putreg16(RTC_MAGIC, RTC_MAGIC_TIME_SET);
      putreg16(regvals.cnth, STM32_RTC_CNTH);
      putreg16(regvals.cntl, STM32_RTC_CNTL);
      cntl = getreg16(STM32_RTC_CNTL);
      stm32_rtc_endwr();
    }
  while (cntl != regvals.cntl);

#ifdef CONFIG_RTC_HIRES
  putreg16(regvals.ovf, RTC_TIMEMSB_REG);
#endif

  stm32_pwr_enablebkp(false);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: stm32_rtc_setalarm
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
int stm32_rtc_setalarm(const struct timespec *tp, alarmcb_t callback)
{
  struct rtc_regvals_s regvals;
  irqstate_t flags;
  uint16_t cr;
  int ret = -EBUSY;

  flags = enter_critical_section();

  /* Is there already something waiting on the ALARM? */

  if (g_alarmcb == NULL)
    {
      /* No.. Save the callback function pointer */

      g_alarmcb = callback;

      /* Break out the time values */

      stm32_rtc_breakout(tp, &regvals);

      stm32_pwr_enablebkp(true);

      /* Enable RTC alarm */

      cr  = getreg16(STM32_RTC_CRH);
      cr |= RTC_CRH_ALRIE;
      putreg16(cr, STM32_RTC_CRH);

      /* The set the alarm */

      stm32_rtc_beginwr();
      putreg16(regvals.cnth, STM32_RTC_ALRH);
      putreg16(regvals.cntl, STM32_RTC_ALRL);
      stm32_rtc_endwr();

      stm32_pwr_enablebkp(false);

      ret = OK;
    }

  leave_critical_section(flags);

  return ret;
}
#endif

/****************************************************************************
 * Name: stm32_rtc_cancelalarm
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
int stm32_rtc_cancelalarm(void)
{
  irqstate_t flags;
  int ret = -ENODATA;

  flags = enter_critical_section();

  if (g_alarmcb != NULL)
    {
      /* Cancel the global callback function */

      g_alarmcb = NULL;

      /* Unset the alarm */

      stm32_pwr_enablebkp(true);
      stm32_rtc_beginwr();
      putreg16(0xffff, STM32_RTC_ALRH);
      putreg16(0xffff, STM32_RTC_ALRL);
      stm32_rtc_endwr();
      stm32_pwr_enablebkp(false);

      ret = OK;
    }

  leave_critical_section(flags);

  return ret;
}
#endif
