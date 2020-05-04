/************************************************************************************
 * arch/arm/src/efm32/efm32_burtc.c
 *
 *   Copyright (C) 2015 Pierre-Noel Bouteville. All rights reserved.
 *   Author: Pierre-Noel Bouteville <pnb990@gmail.com>
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "arm_arch.h"

#include "chip.h"
#include "hardware/efm32_burtc.h"

#include "efm32_rmu.h"
#include "efm32_rtc.h"
#include "efm32_bitband.h"
#include "clock/clock.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#ifdef CONFIG_RTC_HIRES
#  ifndef CONFIG_RTC_FREQUENCY
#    error "CONFIG_RTC_FREQUENCY is required for CONFIG_RTC_HIRES"
#  endif
#else
#  ifndef CONFIG_RTC_FREQUENCY
#    define CONFIG_RTC_FREQUENCY 1
#  endif
#  if CONFIG_RTC_FREQUENCY != 1
#    error "Only lo-res CONFIG_RTC_FREQUENCY of 1Hz is supported"
#  endif
#endif

#ifndef BOARD_BURTC_MODE
#   define BOARD_BURTC_MODE BURTC_CTRL_MODE_EM4EN
#endif

#ifndef BOARD_BURTC_PRESC
#   define BOARD_BURTC_PRESC BURTC_CTRL_PRESC_DIV1
#endif

#ifndef BOARD_BURTC_CLKSRC
#   define BOARD_BURTC_CLKSRC BURTC_CTRL_CLKSEL_LFRCO
#endif

#if    (BOARD_BURTC_PRESC == BURTC_CTRL_PRESC_DIV1)
#   define BURTC_CLK_DIV 1
#elif  (BOARD_BURTC_PRESC == BURTC_CTRL_PRESC_DIV2)
#   define BURTC_CLK_DIV 2
#elif  (BOARD_BURTC_PRESC == BURTC_CTRL_PRESC_DIV4)
#   define BURTC_CLK_DIV 4
#elif  (BOARD_BURTC_PRESC == BURTC_CTRL_PRESC_DIV8)
#   define BURTC_CLK_DIV 8
#elif  (BOARD_BURTC_PRESC == BURTC_CTRL_PRESC_DIV16)
#   define BURTC_CLK_DIV 16
#elif  (BOARD_BURTC_PRESC == BURTC_CTRL_PRESC_DIV32)
#   define BURTC_CLK_DIV 32
#elif  (BOARD_BURTC_PRESC == BURTC_CTRL_PRESC_DIV64)
#   define BURTC_CLK_DIV 64
#elif  (BOARD_BURTC_PRESC == BURTC_CTRL_PRESC_DIV64)
#   define BURTC_CLK_DIV 128
#else
#   error "BOARD_BURTC_PRESC is set with unknown value"
#endif

#if   (BOARD_BURTC_CLKSRC == BURTC_CTRL_CLKSEL_LFRCO)
#   if (CONFIG_RTC_FREQUENCY*BURTC_CLK_DIV != BOARD_LFRCO_FREQUENCY)
#       error "CONFIG_RTC_FREQUENCY is not well be set"
#   endif
#elif (BOARD_BURTC_CLKSRC == BURTC_CTRL_CLKSEL_LFXO)
#   if (CONFIG_RTC_FREQUENCY*BURTC_CLK_DIV != BOARD_LFXO_FREQUENCY)
#       error "CONFIG_RTC_FREQUENCY is not well be set"
#   endif
#elif (BOARD_BURTC_CLKSRC == BURTC_CTRL_CLKSEL_ULFRCO)
#   if (CONFIG_RTC_FREQUENCY*BURTC_CLK_DIV != BOARD_ULFRCO_FREQUENCY)
#       error "CONFIG_RTC_FREQUENCY is not well be set"
#   endif
#else
#   error "BOARD_BURTC_CLKSRC badly set !"
#endif

#define __CNT_TOP               (((uint64_t)(_BURTC_CNT_MASK))+1)
#define __CNT_CARRY_REG         EFM32_BURTC_RET_REG(0)
#define __CNT_ZERO_REG          EFM32_BURTC_RET_REG(1)

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM
static alarmcb_t g_alarmcb;
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* Variable determines the state of the LSE oscillator.
 * Possible errors:
 *   - on start-up
 *   - during operation, reported by LSE interrupt
 */

uint32_t g_efm32_burtc_reset_status;

bool g_efm32_burtc_reseted = false;

volatile bool g_rtc_enabled = false;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: efm32_rtc_interrupt
 *
 * Description:
 *    BURTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ************************************************************************************/

static int efm32_rtc_burtc_interrupt(int irq, void *context, FAR void *arg)
{
  uint32_t source = getreg32(EFM32_BURTC_IF);

  if (source & BURTC_IF_LFXOFAIL)
    {
      rtcerr("ERROR: BURTC_IF_LFXOFAIL");
    }

#ifdef CONFIG_RTC_HIRES
  if (source & BURTC_IF_OF)
    {
      uint32_t regval;

      regval  = getreg32(__CNT_CARRY_REG);
      regval++;
      putreg32(regval, __CNT_CARRY_REG);
    }
#endif

#ifdef CONFIG_RTC_ALARM
  if (source & BURTC_IFC_COMP0)
    {
      if (g_alarmcb != NULL)
        {
          /* Alarm callback */

          g_alarmcb();
          g_alarmcb = NULL;
        }
    }
#endif

  /* Clear pending flags, leave RSF high */

  putreg32(BURTC_IFC_OF | BURTC_IFC_COMP0 | BURTC_IFC_LFXOFAIL, EFM32_BURTC_IFC);

  return 0;
}

/************************************************************************************
 * Name: efm32_burtc_init
 *
 * Description:
 *   board initialization of burtc RTC.
 *   This function is called once in efm32_boardinitialize
 *
 * Note
 *   efm32_rmu_initialize should be called one since boot.
 *
 ************************************************************************************/

static void efm32_rtc_burtc_init(void)
{
  uint32_t regval;
  uint32_t regval2;

  regval = g_efm32_rstcause;
  regval2 = getreg32(EFM32_BURTC_CTRL);

  rtcinfo("BURTC RESETCAUSE=0x%08X BURTC_CTRL=0x%08X\n", regval, regval2);

  if (!(regval2 & BURTC_CTRL_RSTEN) &&
      !(regval  & RMU_RSTCAUSE_BUBODREG) &&
      !(regval  & RMU_RSTCAUSE_BUBODUNREG) &&
      !(regval  & RMU_RSTCAUSE_BUBODBUVIN) &&
      !(regval  & RMU_RSTCAUSE_EXTRST) &&
      !(regval  & RMU_RSTCAUSE_PORST))
    {
      g_efm32_burtc_reset_status = getreg32(EFM32_BURTC_STATUS);

      /* Reset timestamp BURTC clear status */

      putreg32(BURTC_CMD_CLRSTATUS, EFM32_BURTC_CMD);

      /* restore saved base time */

      rtcinfo("BURTC OK\n");
      return;
    }

  rtcinfo("BURTC RESET\n");

  /* Disable reset of BackupDomain */

  bitband_set_peripheral(EFM32_RMU_CTRL, _RMU_CTRL_BURSTEN_SHIFT, 0);

  /* Make sure all registers are updated simultaneously */

  putreg32(BURTC_FREEZE_REGFREEZE_FREEZE, EFM32_BURTC_FREEZE);

  /* Restore all not set BURTC registers to default value */

//  putreg32(_BURTC_LPMODE_RESETVALUE,      EFM32_BURTC_LPMODE);
//  putreg32(_BURTC_LFXOFDET_RESETVALUE,    EFM32_BURTC_LFXOFDET);
//  putreg32(_BURTC_COMP0_RESETVALUE,       EFM32_BURTC_COMP0);

  /* New configuration */

  regval = ((BOARD_BURTC_MODE) |
            (BURTC_CTRL_DEBUGRUN_DEFAULT) |
            (BURTC_CTRL_COMP0TOP_DEFAULT) |
            (BURTC_CTRL_LPCOMP_DEFAULT) |
            (BOARD_BURTC_PRESC) |
            (BOARD_BURTC_CLKSRC) |
            (BURTC_CTRL_BUMODETSEN_DEFAULT));

  /* Clear interrupts */

  putreg32(0xFFFFFFFF, EFM32_BURTC_IFC);

  /* Set new configuration */

  putreg32(regval | BURTC_CTRL_RSTEN, EFM32_BURTC_CTRL);

  /* Clear freeze */

  putreg32(0, EFM32_BURTC_FREEZE);

  /* To enable BURTC counter, we need to disable reset */

  putreg32(regval, EFM32_BURTC_CTRL);

  /* Enable BURTC interrupt on compare match and counter overflow */

  putreg32(BURTC_IF_OF | BURTC_IF_LFXOFAIL, EFM32_BURTC_IEN);

  /* Lock BURTC to avoid modification */

  putreg32(BURTC_LOCK_LOCKKEY_LOCK, EFM32_BURTC_LOCK);

  /* reset BURTC retention REG used */

  putreg32(0, __CNT_CARRY_REG);
  putreg32(0, __CNT_ZERO_REG);

  /* inform rest of software that BURTC was reset at boot */

  g_efm32_burtc_reseted = true;
}

static uint64_t efm32_get_burtc_tick(void)
{
  uint32_t cnt_carry;
  uint32_t cnt_zero;
  uint32_t cnt;
  uint64_t val;
  irqstate_t flags;

  flags = enter_critical_section();

  do
    {
      /* pending IRQ so theat it */

      if (getreg32(EFM32_BURTC_IF) & BURTC_IF_COMP0)
        {
          efm32_rtc_burtc_interrupt(EFM32_IRQ_BURTC, NULL);
        }

      cnt       = getreg32(EFM32_BURTC_CNT);
      cnt_zero  = getreg32(__CNT_ZERO_REG);
      cnt_carry = getreg32(__CNT_CARRY_REG);
    }

  /* Retry if IRQ appear during register reading  */

  while (getreg32(EFM32_BURTC_IF) & BURTC_IF_COMP0);

  leave_critical_section(flags);

  val = (uint64_t)cnt_carry*__CNT_TOP + cnt + cnt_zero;

  rtcinfo("Get Tick carry %u zero %u reg %u\n", cnt_carry, cnt_carry,cnt);

  return val;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This function is
 *   called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_initialize(void)
{
  efm32_rtc_burtc_init();

  /* Configure RTC interrupt to catch overflow and alarm interrupts. */

  irq_attach(EFM32_IRQ_BURTC, efm32_rtc_burtc_interrupt, NULL);
  up_enable_irq(EFM32_IRQ_BURTC);

  g_rtc_enabled = true;

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
  return (time_t)efm32_get_burtc_tick()/CONFIG_RTC_FREQUENCY;
}
#endif

/************************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC clock/counter.  This interface
 *   is only supported by the high-resolution RTC/counter hardware implementation.
 *   It is used to replace the system timer.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_HIRES
int up_rtc_gettime(FAR struct timespec *tp)
{
  uint64_t val;

  val = efm32_get_burtc_tick();

  /* Then we can save the time in seconds and fractional seconds. */

  tp->tv_sec  = val  / CONFIG_RTC_FREQUENCY;
  tp->tv_nsec = (val % CONFIG_RTC_FREQUENCY)*(NSEC_PER_SEC/CONFIG_RTC_FREQUENCY);

  rtcinfo("Get RTC %u.%09u\n", tp->tv_sec, tp->tv_nsec);

  return OK;
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

int up_rtc_settime(FAR const struct timespec *tp)
{
  uint32_t cnt_carry;
  uint32_t cnt;
  uint32_t cnt_reg;
  uint64_t val;
  irqstate_t flags;

  flags = enter_critical_section();

  cnt_reg  = getreg32(EFM32_BURTC_CNT);

  /* Compute Burtc offset because we cannot reset counter */

  val = (((uint64_t)tp->tv_sec) * CONFIG_RTC_FREQUENCY) + \
        (tp->tv_nsec / (NSEC_PER_SEC / CONFIG_RTC_FREQUENCY));

  if (val < cnt_reg)
    {
      val = 0;
    }
  else
    {
      val -= cnt_reg;
    }

  cnt_carry = val / __CNT_TOP;
  cnt       = val % __CNT_TOP;

  rtcinfo("Set RTC %u.%09u carry %u zero %u reg %u\n",
           tp->tv_sec, tp->tv_nsec, cnt_carry, cnt, cnt_reg);

  putreg32(cnt_carry, __CNT_CARRY_REG);
  putreg32(cnt      , __CNT_ZERO_REG);

  leave_critical_section(flags);
  return OK;
}

/************************************************************************************
 * Name: efm32_rtc_setalarm
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
 ************************************************************************************/

#ifdef CONFIG_RTC_ALARM
#error "Sorry ! not yet implemented, just copied from STM32"
int efm32_rtc_setalarm(FAR const struct timespec *tp, alarmcb_t callback)
{
  struct rtc_regvals_s regvals;
  irqstate_t flags;
  uint16_t cr;
  int ret = -EBUSY;

  /* Is there already something waiting on the ALARM? */

  if (g_alarmcb == NULL)
    {
      /* No.. Save the callback function pointer */

      g_alarmcb = callback;

      /* Break out the time values */

      efm32_rtc_breakout(tp, &regvals);

      /* Enable RTC alarm */

      cr  = getreg16(STM32_RTC_CRH);
      cr |= RTC_CRH_ALRIE;
      putreg16(cr, STM32_RTC_CRH);

      /* The set the alarm */

      flags = enter_critical_section();
      stm32_rtc_beginwr();
      putreg16(regvals.cnth, STM32_RTC_ALRH);
      putreg16(regvals.cntl, STM32_RTC_ALRL);
      stm32_rtc_endwr();
      leave_critical_section(flags);

      ret = OK;
    }

  return ret;
}
#endif

/************************************************************************************
 * Name: efm32_rtc_cancelalarm
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
 ************************************************************************************/

#ifdef CONFIG_RTC_ALARM
#error "Sorry ! not yet implemented, just copied from STM32"
int efm32_rtc_cancelalarm(void)
{
  irqstate_t flags;
  int ret = -ENODATA;

  if (g_alarmcb != NULL)
    {
      /* Cancel the global callback function */

      g_alarmcb = NULL;

      /* Unset the alarm */

      flags = enter_critical_section();
      stm32_rtc_beginwr();
      putreg16(0xffff, STM32_RTC_ALRH);
      putreg16(0xffff, STM32_RTC_ALRL);
      stm32_rtc_endwr();
      leave_critical_section(flags);

      ret = OK;
    }

  return ret;
}
#endif
