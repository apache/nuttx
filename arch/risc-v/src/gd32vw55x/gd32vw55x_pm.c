/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_pm.c
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

#include <stdint.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/power/pm.h>

#include "riscv_internal.h"
#include "riscv_mtimer.h"
#include "chip.h"
#include "gd32vw55x_clockconfig.h"
#include "gd32vw55x_pm.h"
#include "hardware/gd32vw55x_rcu.h"
#include "hardware/gd32vw55x_uart.h"

#ifdef CONFIG_PM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PMU_CTL0: LDO and low-driver modes during deep-sleep */

#define GD32VW55X_PMU_CTL0      (GD32VW55X_PMU_BASE + 0x0000)
#define PMU_CTL0_LDOLP          (1 << 0)   /* LDO low power mode */
#define PMU_CTL0_STBMOD         (1 << 1)   /* Standby mode */
#define PMU_CTL0_LDLP           (1 << 10)  /* Low-driver with low power LDO */
#define PMU_CTL0_LDNP           (1 << 11)  /* Low-driver with normal LDO */
#define PMU_CTL0_LDEN           (3 << 18)  /* Low-driver mode enable */

/* Nuclei SLEEPVALUE CSR: WFI enters the deep-sleep mode when set */

#define CSR_SLEEPVALUE          0x811
#define SLEEPVALUE_SHALLOW      0
#define SLEEPVALUE_DEEP         1

/* DBG_CTL0: keep the debugger connection in deep-sleep, so that a sleeping
 * board can still be programmed.
 */

#define GD32VW55X_DBG_CTL0      0xe0044004
#define DBG_CTL0_DSLP_HOLD      (1 << 1)

/* System timer (mtime), stopped in deep-sleep */

#define GD32VW55X_PM_MTIME      (GD32VW55X_SYSTIMER_BASE + 0x0000)
#define GD32VW55X_PM_MTIMECMP   (GD32VW55X_SYSTIMER_BASE + 0x0008)

/* Shorter waits are not worth the deep-sleep: the wake-up timer counts
 * 0.5 ms periods, and the PLL takes time to lock again.
 */

#define GD32VW55X_PM_MIN_SLEEP  2000  /* us */

/* The console UART, emptied before deep-sleep stops its clock */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE          GD32VW55X_USART0_BASE
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE          GD32VW55X_UART1_BASE
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_BASE          GD32VW55X_UART2_BASE
#endif

#define CONSOLE_FLUSH_TIMEOUT   1000000

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32vw55x_pmwait
 *
 * Description:
 *   Wait for an interrupt, in the sleep or the deep-sleep mode.  With the
 *   interrupts disabled, a pending interrupt still ends WFI; it is taken
 *   once they are enabled again.
 *
 ****************************************************************************/

static void gd32vw55x_pmwait(bool deep)
{
  if (deep)
    {
      WRITE_CSR(CSR_SLEEPVALUE, SLEEPVALUE_DEEP);
    }

  asm volatile ("wfi");
  WRITE_CSR(CSR_SLEEPVALUE, SLEEPVALUE_SHALLOW);
}

#ifdef CONFIG_RTC
/****************************************************************************
 * Name: gd32vw55x_mtime_advance
 *
 * Description:
 *   Add 'cycles' to the running system timer.  The low word is cleared
 *   first, so that the counter never goes past the new value and fires an
 *   alarm early.
 *
 ****************************************************************************/

static void gd32vw55x_mtime_advance(uint64_t cycles)
{
  uint64_t value = riscv_mtimer_get(GD32VW55X_PM_MTIME) + cycles;

  putreg32(0, GD32VW55X_PM_MTIME);
  putreg32(value >> 32, GD32VW55X_PM_MTIME + 4);
  putreg32(value, GD32VW55X_PM_MTIME);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_pminitialize
 *
 * Description:
 *   Initialize the power management, early in up_initialize().
 *
 ****************************************************************************/

void riscv_pminitialize(void)
{
  modifyreg32(GD32VW55X_RCU_APB1EN, 0, RCU_APB1EN_PMUEN);
  modifyreg32(GD32VW55X_DBG_CTL0, 0, DBG_CTL0_DSLP_HOLD);
  pm_initialize();
}

/****************************************************************************
 * Name: gd32vw55x_pmstop
 *
 * Description:
 *   Enter the deep-sleep mode until the next timer event: the core and bus
 *   clocks stop, SRAM and registers keep their contents.  The RTC wake-up
 *   timer ends it at the next system timer alarm, at most
 *   CONFIG_GD32VW55X_PM_WAKEUP_MS milliseconds later, and so does any EXTI
 *   interrupt.  On return the clock tree is set up again and the system
 *   timer is advanced by the time spent asleep.  Without the RTC, or when
 *   the next alarm is too close, the core only waits for an interrupt.
 *
 *   Called with the interrupts disabled.
 *
 ****************************************************************************/

void gd32vw55x_pmstop(void)
{
#ifdef CONFIG_RTC
  uint64_t now = riscv_mtimer_get(GD32VW55X_PM_MTIME);
  uint64_t alarm = riscv_mtimer_get(GD32VW55X_PM_MTIMECMP);
  uint64_t elapsed;
  uint64_t awake;
  uint64_t usec;
  uint32_t before;
  uint32_t after;
  uint32_t hz;
#ifdef CONSOLE_BASE
  int timeout;
#endif

  /* Sleep until the next alarm of the system timer */

  usec = alarm > now ? (alarm - now) / (GD32VW55X_MTIME_FREQ / USEC_PER_SEC)
                     : 0;
  usec = MIN(usec, (uint64_t)CONFIG_GD32VW55X_PM_WAKEUP_MS * USEC_PER_MSEC);

  if (usec < GD32VW55X_PM_MIN_SLEEP)
    {
      gd32vw55x_pmwait(false);
      return;
    }

#ifdef CONSOLE_BASE
  /* Let the console finish the character on the wire */

  for (timeout = CONSOLE_FLUSH_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(CONSOLE_BASE + GD32VW55X_UART_STAT_OFFSET) &
           UART_STAT_TC) != 0)
        {
          break;
        }
    }
#endif

  /* Take the system timer and the calendar at the same instant, when a
   * sub-second unit starts: the wake-up comes at a fixed point of a unit,
   * so reading the calendar anywhere in a unit would leave a bias of up to
   * one unit for every sleep.
   */

  before = gd32vw55x_rtc_timeofday(&hz);
  now    = riscv_mtimer_get(GD32VW55X_PM_MTIME);

  if (gd32vw55x_rtc_setwakeup(usec) < 0)
    {
      gd32vw55x_pmwait(false);
      return;
    }

  /* LDO in low power mode and low-driver mode during deep-sleep */

  modifyreg32(GD32VW55X_PMU_CTL0,
              PMU_CTL0_STBMOD | PMU_CTL0_LDOLP | PMU_CTL0_LDEN |
              PMU_CTL0_LDNP | PMU_CTL0_LDLP,
              PMU_CTL0_LDOLP | PMU_CTL0_LDEN | PMU_CTL0_LDLP);

  gd32vw55x_pmwait(true);

  /* The chip wakes up on the IRC16M: bring the PLL back */

  gd32vw55x_clockconfig();
  gd32vw55x_rtc_setwakeup(0);

  /* The system timer was stopped: add the time spent asleep, measured on
   * the calendar, less the time it counted.
   */

  after = gd32vw55x_rtc_timeofday(&hz);
  if (after < before)
    {
      after += SEC_PER_DAY * hz;
    }

  elapsed = ((uint64_t)(after - before) * GD32VW55X_MTIME_FREQ) / hz;
  awake   = riscv_mtimer_get(GD32VW55X_PM_MTIME) - now;

  if (elapsed > awake)
    {
      gd32vw55x_mtime_advance(elapsed - awake);
    }
#else
  /* Nothing would wake the chip up at the next alarm */

  gd32vw55x_pmwait(false);
#endif
}

#endif /* CONFIG_PM */
