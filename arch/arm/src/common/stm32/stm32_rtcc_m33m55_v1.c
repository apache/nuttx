/****************************************************************************
 * arch/arm/src/common/stm32/stm32_rtcc_m33m55_v1.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdbool.h>
#include <sched.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>

#include "arm_internal.h"
#include "stm32.h"
#include "stm32_rtc.h"

#include <arch/board/board.h>

#ifdef CONFIG_STM32_RTC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* This RTC implementation supports the RTC of the STM32N6 and STM32H5:
 *  - date/time RTC hardware with sub-seconds
 *  - Alarm A and Alarm B sharing one RTC interrupt with the wakeup timer
 *  - the periodic wakeup timer
 *  - backup registers in the TAMP peripheral
 *
 * Backup domain write access (PWR DBP) is enabled once and kept enabled:
 * on the STM32N6 an RTC register write with DBP clear never completes and
 * stalls the bus until the next power cycle.  Write protection relies on
 * the RTC_WPR key instead.
 */

#ifndef CONFIG_RTC_DATETIME
#  error "CONFIG_RTC_DATETIME must be set to use this driver"
#endif

#ifdef CONFIG_RTC_HIRES
#  error "CONFIG_RTC_HIRES must NOT be set with this driver"
#endif

/* Constants ****************************************************************/

#define SYNCHRO_TIMEOUT      (0x00020000)
#define INITMODE_TIMEOUT     (0x00010000)

/* RCC glue: the RTC kernel clock enable and source selection, and the RTC
 * APB clock enable.
 *
 * STM32N6: all in APB4LENR and CCIPR7.  CCIPR7 is reset with the system,
 * so the clock selection is not retained.
 * STM32H5: kernel clock in the backup domain BDCR, APB clock in APB3ENR.
 */

#if defined(RCC_APB4LENR_RTCEN)
#  define STM32_RCC_RTCENR        STM32_RCC_APB4LENR
#  define RCC_RTCENR_RTCEN        RCC_APB4LENR_RTCEN
#  define STM32_RCC_RTCAPBENR     STM32_RCC_APB4LENR
#  define RCC_RTCAPBENR_RTCAPBEN  RCC_APB4LENR_RTCAPBEN
#  define STM32_RCC_RTCSELR       STM32_RCC_CCIPR7
#  define RCC_RTCSELR_RTCSEL_MASK RCC_CCIPR7_RTCSEL_MASK
#  define RCC_RTCSELR_RTCSEL_LSE  RCC_CCIPR7_RTCSEL_LSE
#  define RCC_RTCSELR_RTCSEL_LSI  RCC_CCIPR7_RTCSEL_LSI
#  define RCC_RTCSELR_RTCSEL_HSE  RCC_CCIPR7_RTCSEL_HSE
#elif defined(RCC_BDCR_RTCEN)
#  define STM32_RCC_RTCENR        STM32_RCC_BDCR
#  define RCC_RTCENR_RTCEN        RCC_BDCR_RTCEN
#  define STM32_RCC_RTCAPBENR     STM32_RCC_APB3ENR
#  define RCC_RTCAPBENR_RTCAPBEN  RCC_APB3ENR_RTCAPBEN
#  define STM32_RCC_RTCSELR       STM32_RCC_BDCR
#  define RCC_RTCSELR_RTCSEL_MASK RCC_BDCR_RTCSEL_MASK
#  define RCC_RTCSELR_RTCSEL_LSE  RCC_BDCR_RTCSEL_LSE
#  define RCC_RTCSELR_RTCSEL_LSI  RCC_BDCR_RTCSEL_LSI
#  define RCC_RTCSELR_RTCSEL_HSE  RCC_BDCR_RTCSEL_HSE
#else
#  error "Unknown RTC clock enable register"
#endif

#if defined(CONFIG_STM32_RTC_HSECLOCK)
#  define RCC_RTCSELR_RTCSEL      RCC_RTCSELR_RTCSEL_HSE
#elif defined(CONFIG_STM32_RTC_LSICLOCK)
#  define RCC_RTCSELR_RTCSEL      RCC_RTCSELR_RTCSEL_LSI
#else
#  define RCC_RTCSELR_RTCSEL      RCC_RTCSELR_RTCSEL_LSE
#endif

/* The RTC is clocked at 1 MHz from the HSE (see AN4759), divided down by
 * the RCC RTCPRE prescaler.
 */

#define RTC_HSE_FREQUENCY         1000000

/* RTCCLK frequency, the wakeup timer counts RTCCLK/16 for short periods */

#if defined(CONFIG_STM32_RTC_HSECLOCK)
#  define RTC_CLK_FREQUENCY       RTC_HSE_FREQUENCY
#elif defined(CONFIG_STM32_RTC_LSICLOCK)
#  define RTC_CLK_FREQUENCY       STM32_LSI_FREQUENCY
#else
#  define RTC_CLK_FREQUENCY       STM32_LSE_FREQUENCY
#endif

#ifdef CONFIG_STM32_RTC_HSECLOCK
#  if !defined(RCC_CCIPR7_RTCPRE_MASK)
#    error "The RTC HSE clock needs the RCC RTCPRE prescaler"
#  elif STM32_HSE_FREQUENCY % RTC_HSE_FREQUENCY != 0 || \
        STM32_HSE_FREQUENCY / RTC_HSE_FREQUENCY - 1 > \
        (RCC_CCIPR7_RTCPRE_MASK >> RCC_CCIPR7_RTCPRE_SHIFT)
#    error "The HSE frequency cannot be divided down to 1 MHz for the RTC"
#  endif
#endif

#define RTC_ALRMR_ENABLE          (0)

/* The sub second value has 16 bits in BCD mode */

#define RTC_SSR_BCD_MASK          (0xffff)

/* Interrupt flags handled by this driver */

#define RTC_SCR_ALLF              (RTC_SCR_CALRAF | RTC_SCR_CALRBF | \
                                   RTC_SCR_CWUTF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
typedef unsigned int rtc_alarmreg_t;

struct alm_cbinfo_s
{
  volatile alm_callback_t ac_cb; /* Client callback function */
  volatile void *ac_arg;         /* Argument to pass with the callback */
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static struct alm_cbinfo_s g_alarmcb[RTC_ALARM_LAST];
#endif

#ifdef CONFIG_RTC_PERIODIC
static wakeupcb_t g_wakeupcb;
#endif

#if defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_PERIODIC)
static bool g_rtc_irq_enabled;  /* True: the RTC interrupt is attached */
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
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumpregs(const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("      TR: %08" PRIx32 "\n", getreg32(STM32_RTC_TR));
  rtcinfo("      DR: %08" PRIx32 "\n", getreg32(STM32_RTC_DR));
  rtcinfo("     SSR: %08" PRIx32 "\n", getreg32(STM32_RTC_SSR));
  rtcinfo("    ICSR: %08" PRIx32 "\n", getreg32(STM32_RTC_ICSR));
  rtcinfo("    PRER: %08" PRIx32 "\n", getreg32(STM32_RTC_PRER));
  rtcinfo("    WUTR: %08" PRIx32 "\n", getreg32(STM32_RTC_WUTR));
  rtcinfo("      CR: %08" PRIx32 "\n", getreg32(STM32_RTC_CR));
  rtcinfo("  ALRMAR: %08" PRIx32 "\n", getreg32(STM32_RTC_ALRMAR));
  rtcinfo("ALRMASSR: %08" PRIx32 "\n", getreg32(STM32_RTC_ALRMASSR));
  rtcinfo("  ALRMBR: %08" PRIx32 "\n", getreg32(STM32_RTC_ALRMBR));
  rtcinfo("ALRMBSSR: %08" PRIx32 "\n", getreg32(STM32_RTC_ALRMBSSR));
  rtcinfo("      SR: %08" PRIx32 "\n", getreg32(STM32_RTC_SR));
  rtcinfo("    MISR: %08" PRIx32 "\n", getreg32(STM32_RTC_MISR));
  rtcinfo("    CALR: %08" PRIx32 "\n", getreg32(STM32_RTC_CALR));
  rtcinfo("MAGICREG: %08" PRIx32 "\n", getreg32(RTC_MAGIC_REG));
}
#else
#  define rtc_dumpregs(msg)
#endif

/****************************************************************************
 * Name: rtc_dumptime
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumptime(const struct tm *tp, const char *msg)
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
 * Name: rtc_wprunlock
 *
 * Description:
 *    Disable RTC write protection
 *
 ****************************************************************************/

static void rtc_wprunlock(void)
{
  /* The following steps are required to unlock the write protection on all
   * the RTC registers (except for RTC_ICSR[13:8], RTC_SCR and the TAMP
   * registers):
   *
   * 1. Write 0xCA into the RTC_WPR register.
   * 2. Write 0x53 into the RTC_WPR register.
   *
   * Writing a wrong key re-activates the write protection.
   */

  putreg32(RTC_WPR_KEY1, STM32_RTC_WPR);
  putreg32(RTC_WPR_KEY2, STM32_RTC_WPR);
}

/****************************************************************************
 * Name: rtc_wprlock
 *
 * Description:
 *    Enable RTC write protection
 *
 ****************************************************************************/

static inline void rtc_wprlock(void)
{
  /* Writing any wrong key re-activates the write protection. */

  putreg32(0xff, STM32_RTC_WPR);
}

/****************************************************************************
 * Name: rtc_synchwait
 *
 * Description:
 *   Waits until the RTC Time and Date registers (RTC_TR and RTC_DR) are
 *   synchronized with RTC APB clock.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static int rtc_synchwait(void)
{
  volatile uint32_t timeout;
  uint32_t regval;
  int ret;

  /* Disable the write protection for RTC registers */

  rtc_wprunlock();

  /* Clear Registers synchronization flag (RSF) */

  regval  = getreg32(STM32_RTC_ICSR);
  regval &= ~RTC_ICSR_RSF;
  putreg32(regval, STM32_RTC_ICSR);

  /* Now wait the registers to become synchronised */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < SYNCHRO_TIMEOUT; timeout++)
    {
      regval = getreg32(STM32_RTC_ICSR);
      if ((regval & RTC_ICSR_RSF) != 0)
        {
          /* Synchronized */

          ret = OK;
          break;
        }
    }

  /* Re-enable the write protection for RTC registers */

  rtc_wprlock();
  return ret;
}

/****************************************************************************
 * Name: rtc_enterinit
 *
 * Description:
 *   Enter RTC initialization mode.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static int rtc_enterinit(void)
{
  volatile uint32_t timeout;
  uint32_t regval;
  int ret;

  /* Check if the Initialization mode is already set */

  regval = getreg32(STM32_RTC_ICSR);

  ret = OK;
  if ((regval & RTC_ICSR_INITF) == 0)
    {
      /* Set the Initialization mode */

      putreg32(RTC_ICSR_INIT, STM32_RTC_ICSR);

      /* Wait until the RTC is in the INIT state (or a timeout occurs) */

      ret = -ETIMEDOUT;
      for (timeout = 0; timeout < INITMODE_TIMEOUT; timeout++)
        {
          regval = getreg32(STM32_RTC_ICSR);
          if ((regval & RTC_ICSR_INITF) != 0)
            {
              ret = OK;
              break;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rtc_exitinit
 *
 * Description:
 *   Exit RTC initialization mode.
 *
 ****************************************************************************/

static void rtc_exitinit(void)
{
  uint32_t regval;

  regval  = getreg32(STM32_RTC_ICSR);
  regval &= ~RTC_ICSR_INIT;
  putreg32(regval, STM32_RTC_ICSR);
}

/****************************************************************************
 * Name: rtc_bin2bcd
 *
 * Description:
 *   Converts a 2 digit binary to BCD format
 *
 ****************************************************************************/

static uint32_t rtc_bin2bcd(int value)
{
  uint32_t msbcd = 0;

  while (value >= 10)
    {
      msbcd++;
      value -= 10;
    }

  return (msbcd << 4) | value;
}

/****************************************************************************
 * Name: rtc_bcd2bin
 *
 * Description:
 *   Convert from 2 digit BCD to binary.
 *
 ****************************************************************************/

static int rtc_bcd2bin(uint32_t value)
{
  uint32_t tens = (value >> 4) * 10;

  return (int)(tens + (value & 0x0f));
}

/****************************************************************************
 * Name: rtc_setup
 *
 * Description:
 *   Performs first time configuration of the RTC.  A special value written
 *   into the magic backup register will prevent this function from being
 *   called on sub-sequent resets or power up.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static int rtc_setup(void)
{
  uint32_t regval;
  int ret;

  /* Disable the write protection for RTC registers */

  rtc_wprunlock();

  /* Set Initialization mode */

  ret = rtc_enterinit();
  if (ret == OK)
    {
      /* Set the 24 hour format by clearing the FMT bit in the RTC
       * control register
       */

      regval  = getreg32(STM32_RTC_CR);
      regval &= ~RTC_CR_FMT;
      putreg32(regval, STM32_RTC_CR);

      /* Configure RTC pre-scaler with the required values */

#ifdef CONFIG_STM32_RTC_HSECLOCK
      /* STMicro app note AN4759 suggests using 7999 and 124 to
       * get exactly 1Hz when using the RTC at 1MHz.
       */

      putreg32(((uint32_t)7999 << RTC_PRER_PREDIV_S_SHIFT) |
               ((uint32_t)124 << RTC_PRER_PREDIV_A_SHIFT),
               STM32_RTC_PRER);
#else
      /* Correct values for 32.768 KHz LSE clock and inaccurate LSI clock */

      putreg32(((uint32_t)0xff << RTC_PRER_PREDIV_S_SHIFT) |
               ((uint32_t)0x7f << RTC_PRER_PREDIV_A_SHIFT),
               STM32_RTC_PRER);
#endif

      /* Exit RTC initialization mode */

      rtc_exitinit();
    }

  /* Re-enable the write protection for RTC registers */

  rtc_wprlock();

  return ret;
}

/****************************************************************************
 * Name: rtc_resume
 *
 * Description:
 *   Called when the RTC was already initialized on a previous power cycle.
 *   This just brings the RTC back into full operation.
 *
 ****************************************************************************/

static void rtc_resume(void)
{
  /* Clear the RTC alarm and wakeup flags */

  putreg32(RTC_SCR_ALLF, STM32_RTC_SCR);
}

/****************************************************************************
 * Name: stm32_rtc_interrupt
 *
 * Description:
 *   RTC interrupt service routine.  Alarm A, Alarm B and the periodic
 *   wakeup share one NVIC line.
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_PERIODIC)
static int stm32_rtc_interrupt(int irq, void *context, void *arg)
{
#ifdef CONFIG_RTC_ALARM
  struct alm_cbinfo_s *cbinfo;
  alm_callback_t cb;
  void *cb_arg;
#endif
  uint32_t misr;

  misr = getreg32(STM32_RTC_MISR);

  /* Clear the flags first.  RTC_SCR is not protected by RTC_WPR. */

  putreg32(misr & RTC_SCR_ALLF, STM32_RTC_SCR);

#ifdef CONFIG_RTC_ALARM
  if ((misr & RTC_MISR_ALRAMF) != 0)
    {
      cbinfo = &g_alarmcb[RTC_ALARMA];
      if (cbinfo->ac_cb != NULL)
        {
          /* Alarm A callback */

          cb     = cbinfo->ac_cb;
          cb_arg = (void *)cbinfo->ac_arg;

          cbinfo->ac_cb  = NULL;
          cbinfo->ac_arg = NULL;

          cb(cb_arg, RTC_ALARMA);
        }
    }

#if CONFIG_RTC_NALARMS > 1
  if ((misr & RTC_MISR_ALRBMF) != 0)
    {
      cbinfo = &g_alarmcb[RTC_ALARMB];
      if (cbinfo->ac_cb != NULL)
        {
          /* Alarm B callback */

          cb     = cbinfo->ac_cb;
          cb_arg = (void *)cbinfo->ac_arg;

          cbinfo->ac_cb  = NULL;
          cbinfo->ac_arg = NULL;

          cb(cb_arg, RTC_ALARMB);
        }
    }
#endif
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_PERIODIC
  if ((misr & RTC_MISR_WUTMF) != 0 && g_wakeupcb != NULL)
    {
      g_wakeupcb();
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: rtc_enable_irq
 *
 * Description:
 *   Attach and enable the RTC interrupt.  Unlike the older RTC versions
 *   the alarm and wakeup events do not go through the EXTI controller.
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_PERIODIC)
static void rtc_enable_irq(void)
{
  if (!g_rtc_irq_enabled)
    {
      irq_attach(STM32_IRQ_RTC, stm32_rtc_interrupt, NULL);
      up_enable_irq(STM32_IRQ_RTC);
      g_rtc_irq_enabled = true;
    }
}
#endif

/****************************************************************************
 * Name: rtchw_set_alrmXr X is a or b
 *
 * Description:
 *   Set the alarm (A or B) hardware registers.  On this RTC version the
 *   alarm registers may be written as soon as the alarm is disabled.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int rtchw_set_alrmar(rtc_alarmreg_t alarmreg)
{
  /* Disable the write protection for RTC registers */

  rtc_wprunlock();

  /* Disable RTC alarm & Interrupt */

  modifyreg32(STM32_RTC_CR, (RTC_CR_ALRAE | RTC_CR_ALRAIE), 0);

  /* Set the RTC Alarm register and clear a pending flag */

  putreg32(alarmreg, STM32_RTC_ALRMAR);
  putreg32(0, STM32_RTC_ALRMASSR);
  putreg32(RTC_SCR_CALRAF, STM32_RTC_SCR);
  rtcinfo("  ALRMAR: %08" PRIx32 "\n", getreg32(STM32_RTC_ALRMAR));

  /* Enable RTC alarm */

  modifyreg32(STM32_RTC_CR, 0, (RTC_CR_ALRAE | RTC_CR_ALRAIE));

  rtc_wprlock();
  return OK;
}
#endif

#if defined(CONFIG_RTC_ALARM) && CONFIG_RTC_NALARMS > 1
static int rtchw_set_alrmbr(rtc_alarmreg_t alarmreg)
{
  /* Disable the write protection for RTC registers */

  rtc_wprunlock();

  /* Disable RTC alarm B & Interrupt B */

  modifyreg32(STM32_RTC_CR, (RTC_CR_ALRBE | RTC_CR_ALRBIE), 0);

  /* Set the RTC Alarm register and clear a pending flag */

  putreg32(alarmreg, STM32_RTC_ALRMBR);
  putreg32(0, STM32_RTC_ALRMBSSR);
  putreg32(RTC_SCR_CALRBF, STM32_RTC_SCR);
  rtcinfo("  ALRMBR: %08" PRIx32 "\n", getreg32(STM32_RTC_ALRMBR));

  /* Enable RTC alarm B */

  modifyreg32(STM32_RTC_CR, 0, (RTC_CR_ALRBE | RTC_CR_ALRBIE));

  rtc_wprlock();
  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_rtc_getalarmdatetime
 *
 * Description:
 *   Get the current date and time for a RTC alarm.
 *
 * Input Parameters:
 *   reg - RTC alarm register
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int stm32_rtc_getalarmdatetime(rtc_alarmreg_t reg, struct tm *tp)
{
  uint32_t data;
  uint32_t tmp;

  DEBUGASSERT(tp != NULL);

  /* Sample the data time register. */

  data = getreg32(reg);

  /* Convert the RTC time to fields in struct tm format.  All of the STM32
   * ranges of values correspond between struct tm and the time register.
   */

  tmp = (data & (RTC_ALRMR_SU_MASK | RTC_ALRMR_ST_MASK)) >>
        RTC_ALRMR_SU_SHIFT;
  tp->tm_sec = rtc_bcd2bin(tmp);

  tmp = (data & (RTC_ALRMR_MNU_MASK | RTC_ALRMR_MNT_MASK)) >>
        RTC_ALRMR_MNU_SHIFT;
  tp->tm_min = rtc_bcd2bin(tmp);

  tmp = (data & (RTC_ALRMR_HU_MASK | RTC_ALRMR_HT_MASK)) >>
        RTC_ALRMR_HU_SHIFT;
  tp->tm_hour = rtc_bcd2bin(tmp);

  tmp = (data & (RTC_ALRMR_DU_MASK | RTC_ALRMR_DT_MASK)) >>
        RTC_ALRMR_DU_SHIFT;
  tp->tm_mday = rtc_bcd2bin(tmp);

  return OK;
}
#endif

/****************************************************************************
 * Name: rtc_selectclock
 *
 * Description:
 *   Enable the selected RTC clock source and route it to the RTC.  The
 *   backup domain must be writable.
 *
 ****************************************************************************/

static void rtc_selectclock(void)
{
#if defined(CONFIG_STM32_RTC_HSECLOCK)
  /* Divide the HSE down to 1 MHz */

  modifyreg32(STM32_RCC_CCIPR7, RCC_CCIPR7_RTCPRE_MASK,
              (STM32_HSE_FREQUENCY / RTC_HSE_FREQUENCY - 1) <<
              RCC_CCIPR7_RTCPRE_SHIFT);
#elif defined(CONFIG_STM32_RTC_LSICLOCK)
  stm32_rcc_enablelsi();
#else
  stm32_rcc_enablelse();
#endif

  modifyreg32(STM32_RCC_RTCSELR, RCC_RTCSELR_RTCSEL_MASK,
              RCC_RTCSELR_RTCSEL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This
 *   function is called once during the OS initialization sequence
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  uint32_t regval;
  int ret;
  int maxretry = 10;
  int nretry = 0;

  /* Enable write access to the backup domain for good, then the RTC
   * clocks so that the RTC and TAMP registers can be accessed.
   */

  stm32_pwr_enablebkp(true);
  modifyreg32(STM32_RCC_RTCAPBENR, 0, RCC_RTCAPBENR_RTCAPBEN);
  modifyreg32(STM32_RCC_RTCENR, 0, RCC_RTCENR_RTCEN);

  /* Save the token before losing it when resetting */

  regval = getreg32(RTC_MAGIC_REG);

  if (regval != RTC_MAGIC && regval != RTC_MAGIC_TIME_SET)
    {
      /* First start: select the clock source */

      rtc_dumpregs("On reset");
      rtc_selectclock();
    }
  else
    {
      rtc_dumpregs("On reset warm");

      /* The clock selection is reset with the system, only the RTC itself
       * keeps running.  Route the configured clock to it again.
       */

      rtc_selectclock();
    }

  /* Loop, attempting to initialize/resume the RTC.  This loop is necessary
   * because it seems that occasionally it takes longer to initialize the RTC
   * (the actual failure is in rtc_synchwait()).
   */

  do
    {
      /* Wait for the RTC Time and Date registers to be synchronized with RTC
       * APB clock.
       */

      ret = rtc_synchwait();
      if (ret != OK)
        {
          rtcerr("ERROR: rtc_syncwait() failed (%d)\n", ret);
        }
    }
  while (ret != OK && ++nretry < maxretry);

  /* Check if the one-time initialization of the RTC has already been
   * performed. We can determine this by checking if the magic number
   * has been written to the magic backup register.
   */

  if (regval != RTC_MAGIC && regval != RTC_MAGIC_TIME_SET)
    {
      rtcinfo("Do setup\n");

      /* Perform the one-time setup of the clocking to the RTC */

      ret = rtc_setup();

      /* Remember that the RTC is initialized */

      putreg32(RTC_MAGIC, RTC_MAGIC_REG);
    }
  else
    {
      rtcinfo("Do resume\n");

      /* RTC already set-up, just resume normal operation */

      rtc_resume();
      rtc_dumpregs("Did resume");
    }

  if (ret != OK && nretry > 0)
    {
      rtcinfo("setup/resume ran %d times and failed with %d\n",
              nretry, ret);
      return -ETIMEDOUT;
    }

  rtc_dumpregs("After Initialization");

  g_rtc_enabled = true;
  return OK;
}

/****************************************************************************
 * Name: stm32_rtc_irqinitialize
 *
 * Description:
 *   Initialize IRQs for RTC, not possible during up_rtc_initialize because
 *   up_irqinitialize is called later.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32_rtc_irqinitialize(void)
{
  /* Nothing to do, the interrupt is attached on the first use. */

  return OK;
}

/****************************************************************************
 * Name: stm32_rtc_getdatetime_with_subseconds
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS
 *   during initialization to set up the system time when CONFIG_RTC and
 *   CONFIG_RTC_DATETIME are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.
 *   That sub-second accuracy is returned through 'nsec'.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *   nsec - The location to return the subsecond time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_HAVE_RTC_SUBSECONDS
int stm32_rtc_getdatetime_with_subseconds(struct tm *tp, long *nsec)
#else
int up_rtc_getdatetime(struct tm *tp)
#endif
{
#ifdef CONFIG_STM32_HAVE_RTC_SUBSECONDS
  uint32_t ssr;
#endif
  uint32_t dr;
  uint32_t tr;
  uint32_t tmp;

  /* Sample the data time registers.  There is a race condition here... If
   * we sample the time just before midnight on December 31, the date could
   * be wrong because the day rolled over while were sampling. Thus loop for
   * checking overflow here is needed.  There is a race condition with
   * subseconds too. If we sample TR register just before second rolling
   * and subseconds are read at wrong second, we get wrong time.
   */

  do
    {
      dr  = getreg32(STM32_RTC_DR);
      tr  = getreg32(STM32_RTC_TR);
#ifdef CONFIG_STM32_HAVE_RTC_SUBSECONDS
      ssr = getreg32(STM32_RTC_SSR);
      tmp = getreg32(STM32_RTC_TR);
      if (tmp != tr)
        {
          continue;
        }
#endif

      tmp = getreg32(STM32_RTC_DR);
      if (tmp == dr)
        {
          break;
        }
    }
  while (1);

  rtc_dumpregs("Reading Time");

  /* Convert the RTC time to fields in struct tm format.  All of the STM32
   * ranges of values correspond between struct tm and the time register.
   */

  tmp = (tr & (RTC_TR_SU_MASK | RTC_TR_ST_MASK)) >> RTC_TR_SU_SHIFT;
  tp->tm_sec = rtc_bcd2bin(tmp);

  tmp = (tr & (RTC_TR_MNU_MASK | RTC_TR_MNT_MASK)) >> RTC_TR_MNU_SHIFT;
  tp->tm_min = rtc_bcd2bin(tmp);

  tmp = (tr & (RTC_TR_HU_MASK | RTC_TR_HT_MASK)) >> RTC_TR_HU_SHIFT;
  tp->tm_hour = rtc_bcd2bin(tmp);

  /* Now convert the RTC date to fields in struct tm format:
   * Days: 1-31 match in both cases.
   * Month: STM32 is 1-12, struct tm is 0-11.
   * Years: STM32 is 00-99 (2000-2099), struct tm is years since 1900.
   * WeekDay: STM32 is 1 = Mon - 7 = Sun
   */

  tmp = (dr & (RTC_DR_DU_MASK | RTC_DR_DT_MASK)) >> RTC_DR_DU_SHIFT;
  tp->tm_mday = rtc_bcd2bin(tmp);

  tmp = (dr & (RTC_DR_MU_MASK | RTC_DR_MT)) >> RTC_DR_MU_SHIFT;
  tp->tm_mon = rtc_bcd2bin(tmp) - 1;

  tmp = (dr & (RTC_DR_YU_MASK | RTC_DR_YT_MASK)) >> RTC_DR_YU_SHIFT;
  tp->tm_year = rtc_bcd2bin(tmp) + 100;

  tmp = (dr & RTC_DR_WDU_MASK) >> RTC_DR_WDU_SHIFT;
  tp->tm_wday = tmp % 7;
  tp->tm_yday = tp->tm_mday - 1 +
                clock_daysbeforemonth(tp->tm_mon,
                                      clock_isleapyear(tp->tm_year + 1900));
  tp->tm_isdst = 0;

#ifdef CONFIG_STM32_HAVE_RTC_SUBSECONDS
  /* Return RTC sub-seconds if a non-NULL value of nsec has been provided
   * to receive the sub-second value.
   */

  if (nsec)
    {
      uint32_t prediv_s;
      uint32_t usecs;

      prediv_s   = getreg32(STM32_RTC_PRER) & RTC_PRER_PREDIV_S_MASK;
      prediv_s >>= RTC_PRER_PREDIV_S_SHIFT;

      ssr &= RTC_SSR_BCD_MASK;

      /* Maximum prediv_s is 0x7fff, thus we can multiply by 100000 and
       * still fit 32-bit unsigned integer.
       */

      usecs = (((prediv_s - ssr) * 100000) / (prediv_s + 1)) * 10;
      *nsec = usecs * 1000;
    }
#endif /* CONFIG_STM32_HAVE_RTC_SUBSECONDS */

  rtc_dumptime((const struct tm *)tp, "Returning");
  return OK;
}

/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_HAVE_RTC_SUBSECONDS
int up_rtc_getdatetime(struct tm *tp)
{
  return stm32_rtc_getdatetime_with_subseconds(tp, NULL);
}
#endif

/****************************************************************************
 * Name: up_rtc_getdatetime_with_subseconds
 *
 * Description:
 *   Get the current date and time from the date/time RTC with sub-second
 *   accuracy.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *   nsec - The location to return the subsecond time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_RTC_SUBSECONDS
int up_rtc_getdatetime_with_subseconds(struct tm *tp, long *nsec)
{
  return stm32_rtc_getdatetime_with_subseconds(tp, nsec);
}
#endif

/****************************************************************************
 * Name: stm32_rtc_setdatetime
 *
 * Description:
 *   Set the RTC to the provided time. RTC implementations which provide
 *   up_rtc_getdatetime() (CONFIG_RTC_DATETIME is selected) should provide
 *   this function.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32_rtc_setdatetime(const struct tm *tp)
{
  uint32_t tr;
  uint32_t dr;
  int ret;

  rtc_dumptime(tp, "Setting time");

  /* Convert the struct tm format to RTC time register fields.
   * All of the ranges of values correspond between struct tm and the time
   * register.
   */

  tr = (rtc_bin2bcd(tp->tm_sec)  << RTC_TR_SU_SHIFT) |
       (rtc_bin2bcd(tp->tm_min)  << RTC_TR_MNU_SHIFT) |
       (rtc_bin2bcd(tp->tm_hour) << RTC_TR_HU_SHIFT);

  /* Now convert the fields in struct tm format to the RTC date register
   * fields:
   *
   *   Days: 1-31 match in both cases.
   *   Month: STM32 is 1-12, struct tm is 0-11.
   *   Years: STM32 is 00-99, struct tm is years since 1900.
   *   WeekDay: STM32 is 1 = Mon - 7 = Sun
   */

  dr = (rtc_bin2bcd(tp->tm_mday) << RTC_DR_DU_SHIFT) |
       ((rtc_bin2bcd(tp->tm_mon + 1))  << RTC_DR_MU_SHIFT) |
       ((tp->tm_wday == 0 ? 7 : (tp->tm_wday & 7))  << RTC_DR_WDU_SHIFT) |
       ((rtc_bin2bcd(tp->tm_year - 100)) << RTC_DR_YU_SHIFT);

  dr &= ~RTC_DR_RESERVED_BITS;

  /* Disable the write protection for RTC registers */

  rtc_wprunlock();

  /* Set Initialization mode */

  ret = rtc_enterinit();
  if (ret == OK)
    {
      /* Set the RTC TR and DR registers */

      putreg32(tr, STM32_RTC_TR);
      putreg32(dr, STM32_RTC_DR);

      /* Exit Initialization mode and wait for the RTC Time and Date
       * registers to be synchronized with RTC APB clock.
       */

      rtc_exitinit();
      ret = rtc_synchwait();
    }

  /* Remember that the RTC is initialized and had its time set. */

  if (getreg32(RTC_MAGIC_REG) != RTC_MAGIC_TIME_SET)
    {
      putreg32(RTC_MAGIC_TIME_SET, RTC_MAGIC_REG);
    }

  /* Re-enable the write protection for RTC registers */

  rtc_wprlock();
  rtc_dumpregs("New time setting");
  return ret;
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

int up_rtc_settime(const struct timespec *tp)
{
  struct tm newtime;

  /* Break out the time values (not that the time is set only to units of
   * seconds)
   */

  gmtime_r(&tp->tv_sec, &newtime);
  return stm32_rtc_setdatetime(&newtime);
}

/****************************************************************************
 * Name: stm32_rtc_setalarm
 *
 * Description:
 *   Set an alarm to an absolute time using associated hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int stm32_rtc_setalarm(struct alm_setalarm_s *alminfo)
{
  struct alm_cbinfo_s *cbinfo;
  rtc_alarmreg_t alarmreg;
  int ret = -EINVAL;

  DEBUGASSERT(alminfo != NULL);
  DEBUGASSERT(RTC_ALARM_LAST > alminfo->as_id);

  /* Make sure the RTC interrupt is enabled at the NVIC */

  rtc_enable_irq();

  rtc_dumptime(&alminfo->as_time, "New alarm time");

  /* Break out the values to the HW alarm register format.  The values in
   * all STM32 fields match the fields of struct tm in this case.  Notice
   * that the alarm is limited to one month.
   */

  alarmreg = (rtc_bin2bcd(alminfo->as_time.tm_sec)  << RTC_ALRMR_SU_SHIFT) |
             (rtc_bin2bcd(alminfo->as_time.tm_min)  << RTC_ALRMR_MNU_SHIFT) |
             (rtc_bin2bcd(alminfo->as_time.tm_hour) << RTC_ALRMR_HU_SHIFT) |
             (rtc_bin2bcd(alminfo->as_time.tm_mday) << RTC_ALRMR_DU_SHIFT);

  /* Set the alarm in hardware and enable interrupts from the RTC */

  switch (alminfo->as_id)
    {
      case RTC_ALARMA:
        {
          cbinfo         = &g_alarmcb[RTC_ALARMA];
          cbinfo->ac_cb  = alminfo->as_cb;
          cbinfo->ac_arg = alminfo->as_arg;

          ret = rtchw_set_alrmar(alarmreg | RTC_ALRMR_ENABLE);
          if (ret < 0)
            {
              cbinfo->ac_cb  = NULL;
              cbinfo->ac_arg = NULL;
            }

          rtc_dumpregs("Set AlarmA");
        }
        break;

#if CONFIG_RTC_NALARMS > 1
      case RTC_ALARMB:
        {
          cbinfo         = &g_alarmcb[RTC_ALARMB];
          cbinfo->ac_cb  = alminfo->as_cb;
          cbinfo->ac_arg = alminfo->as_arg;

          ret = rtchw_set_alrmbr(alarmreg | RTC_ALRMR_ENABLE);
          if (ret < 0)
            {
              cbinfo->ac_cb  = NULL;
              cbinfo->ac_arg = NULL;
            }

          rtc_dumpregs("Set AlarmB");
        }
        break;
#endif

      default:
        rtcerr("ERROR: Invalid ALARM%d\n", alminfo->as_id);
        break;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: stm32_rtc_cancelalarm
 *
 * Description:
 *   Cancel an alarm.
 *
 * Input Parameters:
 *  alarmid - Identifies the alarm to be cancelled
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int stm32_rtc_cancelalarm(enum alm_id_e alarmid)
{
  int ret = -EINVAL;

  DEBUGASSERT(RTC_ALARM_LAST > alarmid);

  /* Cancel the alarm in hardware and disable interrupts */

  switch (alarmid)
    {
      case RTC_ALARMA:
        {
          /* Cancel the global callback function */

          g_alarmcb[alarmid].ac_cb  = NULL;
          g_alarmcb[alarmid].ac_arg = NULL;

          /* Disable RTC alarm and interrupt, then unset the alarm */

          rtc_wprunlock();
          modifyreg32(STM32_RTC_CR, (RTC_CR_ALRAE | RTC_CR_ALRAIE), 0);
          putreg32(-1, STM32_RTC_ALRMAR);
          putreg32(RTC_SCR_CALRAF, STM32_RTC_SCR);
          rtc_wprlock();
          ret = OK;
        }
        break;

#if CONFIG_RTC_NALARMS > 1
      case RTC_ALARMB:
        {
          /* Cancel the global callback function */

          g_alarmcb[alarmid].ac_cb  = NULL;
          g_alarmcb[alarmid].ac_arg = NULL;

          /* Disable RTC alarm and interrupt, then unset the alarm */

          rtc_wprunlock();
          modifyreg32(STM32_RTC_CR, (RTC_CR_ALRBE | RTC_CR_ALRBIE), 0);
          putreg32(-1, STM32_RTC_ALRMBR);
          putreg32(RTC_SCR_CALRBF, STM32_RTC_SCR);
          rtc_wprlock();
          ret = OK;
        }
        break;
#endif

      default:
        rtcerr("ERROR: Invalid ALARM%d\n", alarmid);
        break;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: stm32_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int stm32_rtc_rdalarm(struct alm_rdalarm_s *alminfo)
{
  int ret = -EINVAL;

  DEBUGASSERT(alminfo != NULL);
  DEBUGASSERT(RTC_ALARM_LAST > alminfo->ar_id);

  switch (alminfo->ar_id)
    {
      case RTC_ALARMA:
        ret = stm32_rtc_getalarmdatetime(STM32_RTC_ALRMAR,
                                         (struct tm *)alminfo->ar_time);
        break;

#if CONFIG_RTC_NALARMS > 1
      case RTC_ALARMB:
        ret = stm32_rtc_getalarmdatetime(STM32_RTC_ALRMBR,
                                         (struct tm *)alminfo->ar_time);
        break;
#endif

      default:
        rtcerr("ERROR: Invalid ALARM%d\n", alminfo->ar_id);
        break;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: stm32_rtc_setperiodic
 *
 * Description:
 *   Set a periodic RTC wakeup
 *
 * Input Parameters:
 *  period   - Time to sleep between wakeups
 *  callback - Function to call when the period expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
int stm32_rtc_setperiodic(const struct timespec *period,
                          wakeupcb_t callback)
{
  volatile uint32_t timeout;
  uint64_t ticks;
  uint32_t wut;
  uint32_t wucksel;
  int ret = -ETIMEDOUT;

  DEBUGASSERT(period != NULL);

  if (period->tv_sec == 0 && period->tv_nsec == 0)
    {
      return -EINVAL;
    }

  /* Up to 2^16 ticks of RTCCLK/16 (32 s with the LSE) the wakeup timer
   * counts RTCCLK/16, longer periods count ck_spre (1 Hz): up to 2^17
   * seconds.
   */

  ticks = ((uint64_t)period->tv_sec * NSEC_PER_SEC + period->tv_nsec) *
          (RTC_CLK_FREQUENCY / 16) / NSEC_PER_SEC;

  if (ticks <= 0x10000)
    {
      wucksel = RTC_CR_WUCKSEL_RTCDIV16;
      wut     = ticks > 0 ? ticks : 1;
    }
  else
    {
      wut = period->tv_sec;
      if (period->tv_nsec > 0)
        {
          wut++;
        }

      if (wut > 0x20000)
        {
          return -EINVAL;
        }

      if (wut > 0x10000)
        {
          wucksel = RTC_CR_WUCKSEL_CKSPREADD;
          wut    -= 0x10000;
        }
      else
        {
          wucksel = RTC_CR_WUCKSEL_CKSPRE;
        }
    }

  rtc_enable_irq();

  g_wakeupcb = callback;

  rtc_wprunlock();

  /* Disable the wakeup timer and wait until it can be reprogrammed */

  modifyreg32(STM32_RTC_CR, RTC_CR_WUTE | RTC_CR_WUTIE, 0);

  for (timeout = 0; timeout < INITMODE_TIMEOUT; timeout++)
    {
      if ((getreg32(STM32_RTC_ICSR) & RTC_ICSR_WUTWF) != 0)
        {
          ret = OK;
          break;
        }
    }

  if (ret == OK)
    {
      putreg32((wut - 1) & RTC_WUTR_WUT_MASK, STM32_RTC_WUTR);
      putreg32(RTC_SCR_CWUTF, STM32_RTC_SCR);
      modifyreg32(STM32_RTC_CR, RTC_CR_WUCKSEL_MASK,
                  wucksel | RTC_CR_WUTE | RTC_CR_WUTIE);
    }
  else
    {
      g_wakeupcb = NULL;
    }

  rtc_wprlock();
  rtc_dumpregs("Set periodic");
  return ret;
}
#endif

/****************************************************************************
 * Name: stm32_rtc_cancelperiodic
 *
 * Description:
 *   Cancel the periodic RTC wakeup
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
int stm32_rtc_cancelperiodic(void)
{
  g_wakeupcb = NULL;

  rtc_wprunlock();
  modifyreg32(STM32_RTC_CR, RTC_CR_WUTE | RTC_CR_WUTIE, 0);
  putreg32(RTC_SCR_CWUTF, STM32_RTC_SCR);
  rtc_wprlock();

  return OK;
}
#endif

#endif /* CONFIG_STM32_RTC */
