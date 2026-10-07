/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_pm_suspend.c
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
 * Suspend to RAM in the POWMAN P1.0 state: the switched core is powered off,
 * the XIP cache and SRAM keep their contents.  Every peripheral loses its
 * registers, so the chip comes back through the ordinary boot, and __start
 * hands control back to the suspended thread (datasheet 6.2).
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <setjmp.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>

#include "arm_internal.h"
#include "nvic.h"

#include "rp23xx_pm.h"
#include "rp23xx_gpio.h"

#ifdef CONFIG_RTC_ALARM
#  include "rp23xx_rtc.h"
#endif

#include "hardware/rp23xx_powman.h"
#include "hardware/rp23xx_pads_bank0.h"
#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* POWMAN ignores a write without this password in the top 16 bits.  The
 * registers above offset 0xac (SCRATCH, BOOT, interrupts) take none.
 */

#define POWMAN_PASSWORD    0x5afe0000
#define POWMAN_SET_ALIAS   0x2000
#define POWMAN_CLR_ALIAS   0x3000

/* STATE.REQ is bits 7:4, and a set bit powers a domain down: bit 3 is
 * SWCORE.  So P1.0 (switched core down) is 0x8.
 */

#define POWMAN_STATE_P1_0  (0x8 << 4)

/* XIP cache clean by set/way, through the top of the maintenance window */

#define XIP_CACHE_SIZE         (16 * 1024)
#define XIP_CACHE_LINE_SIZE    8
#define XIP_CACHE_CLEAN_BASE   (0x18000000 + 0x04000000 - XIP_CACHE_SIZE + 1)

/* Left in SCRATCH0 (always-on domain) to tell the next boot it is a resume */

#define POWMAN_SUSPEND_MAGIC 0x50575231  /* 'PWR1' */

/* The NVIC registers saved over a suspend: 52 interrupts */

#define RP23XX_PM_NVIC_ENABLE_REGS  2
#define RP23XX_PM_NVIC_PRIO_REGS    13

/* The trigger of the wake GPIO */

#ifdef CONFIG_RP23XX_PM_WAKEUP_GPIO_EDGE
#  define RP23XX_PM_SUSPEND_WAKE_EDGE true
#else
#  define RP23XX_PM_SUSPEND_WAKE_EDGE false
#endif

#ifdef CONFIG_RP23XX_PM_WAKEUP_GPIO_HIGH
#  define RP23XX_PM_SUSPEND_WAKE_HIGH true
#else
#  define RP23XX_PM_SUSPEND_WAKE_HIGH false
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp23xx_pm_nvic_s
{
  uint32_t enable[RP23XX_PM_NVIC_ENABLE_REGS];
  uint32_t prio[RP23XX_PM_NVIC_PRIO_REGS];
  uint32_t systick_ctrl;
  uint32_t systick_reload;
  uint32_t shpr2;
  uint32_t shpr3;
  uint32_t vectab;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

uint32_t g_pm_resume_stack[RP23XX_PM_RESUME_STACK_WORDS]
  __attribute__((aligned(8)));

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Where the resume returns to.  .bss is kept over P1.0. */

static jmp_buf g_suspend_ctx;

/* What woke the chip, for the caller */

static volatile uint32_t g_suspend_wake_source;

/* True while the timed wake uses the alarm comparator.  The application
 * alarm it replaced is saved here and given back after the resume.
 */

static bool g_suspend_wake_armed;

#ifdef CONFIG_RTC_ALARM
static struct rp23xx_alarm_state_s g_suspend_alarm;

/* True while the application alarm itself is the timed wake */

static bool g_suspend_alarm_wake;
#endif

/* The NVIC is in the switched core and comes back cleared */

static struct rp23xx_pm_nvic_s g_suspend_nvic;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void powman_write(uint32_t reg, uint32_t value)
{
  putreg32(POWMAN_PASSWORD | (value & 0xffff), reg);
}

static inline void powman_setbits(uint32_t reg, uint32_t bits)
{
  putreg32(POWMAN_PASSWORD | bits, reg + POWMAN_SET_ALIAS);
}

static inline void powman_clrbits(uint32_t reg, uint32_t bits)
{
  putreg32(POWMAN_PASSWORD | bits, reg + POWMAN_CLR_ALIAS);
}

/****************************************************************************
 * Name: rp23xx_pm_xip_clean
 *
 * Description:
 *   Write dirty XIP cache lines back to PSRAM, by set/way through the top
 *   of the maintenance window to avoid erratum RP2350-E11.  The resume boot
 *   discards the cache, and PSRAM can hold task stacks.
 *
 ****************************************************************************/

static void rp23xx_pm_xip_clean(void)
{
  uintptr_t addr;

  for (addr = XIP_CACHE_CLEAN_BASE;
       addr < XIP_CACHE_CLEAN_BASE + XIP_CACHE_SIZE;
       addr += XIP_CACHE_LINE_SIZE)
    {
      putreg8(0, addr);
    }

  UP_DSB();
  UP_ISB();
}

/****************************************************************************
 * Name: rp23xx_pm_nvic_save / rp23xx_pm_nvic_restore
 *
 * Description:
 *   Save and restore the NVIC.  up_irqinitialize() cannot be used: it would
 *   also reset the handler table, which RAM still holds.
 *
 ****************************************************************************/

static void rp23xx_pm_nvic_save(void)
{
  int i;

  for (i = 0; i < RP23XX_PM_NVIC_ENABLE_REGS; i++)
    {
      g_suspend_nvic.enable[i] = getreg32(NVIC_IRQ_ENABLE(i * 32));
    }

  for (i = 0; i < RP23XX_PM_NVIC_PRIO_REGS; i++)
    {
      g_suspend_nvic.prio[i] = getreg32(NVIC_IRQ0_3_PRIORITY + i * 4);
    }

  g_suspend_nvic.systick_ctrl   = getreg32(NVIC_SYSTICK_CTRL);
  g_suspend_nvic.systick_reload = getreg32(NVIC_SYSTICK_RELOAD);
  g_suspend_nvic.shpr2          = getreg32(NVIC_SYSH8_11_PRIORITY);
  g_suspend_nvic.shpr3          = getreg32(NVIC_SYSH12_15_PRIORITY);
  g_suspend_nvic.vectab         = getreg32(NVIC_VECTAB);
}

static void rp23xx_pm_nvic_restore(void)
{
  int i;

  putreg32(g_suspend_nvic.vectab, NVIC_VECTAB);

  for (i = 0; i < RP23XX_PM_NVIC_PRIO_REGS; i++)
    {
      putreg32(g_suspend_nvic.prio[i], NVIC_IRQ0_3_PRIORITY + i * 4);
    }

  putreg32(g_suspend_nvic.shpr2, NVIC_SYSH8_11_PRIORITY);
  putreg32(g_suspend_nvic.shpr3, NVIC_SYSH12_15_PRIORITY);

  putreg32(g_suspend_nvic.systick_reload, NVIC_SYSTICK_RELOAD);
  putreg32(0, NVIC_SYSTICK_CURRENT);
  putreg32(g_suspend_nvic.systick_ctrl, NVIC_SYSTICK_CTRL);

  /* Enables last, so nothing fires before its priority is back */

  for (i = 0; i < RP23XX_PM_NVIC_ENABLE_REGS; i++)
    {
      putreg32(g_suspend_nvic.enable[i], NVIC_IRQ_ENABLE(i * 32));
    }
}

/****************************************************************************
 * Name: rp23xx_pm_wake_release
 *
 * Description:
 *   Disarm the timed wake and give the alarm comparator back to the
 *   application alarm, if one was armed.
 *
 ****************************************************************************/

static void rp23xx_pm_wake_release(void)
{
#ifdef CONFIG_RTC_ALARM
  if (g_suspend_alarm_wake)
    {
      g_suspend_alarm_wake = false;
      powman_clrbits(RP23XX_POWMAN_TIMER,
                     RP23XX_POWMAN_TIMER_PWRUP_ON_ALARM);
    }
#endif

  if (!g_suspend_wake_armed)
    {
      return;
    }

  g_suspend_wake_armed = false;

  powman_clrbits(RP23XX_POWMAN_TIMER, RP23XX_POWMAN_TIMER_PWRUP_ON_ALARM |
                                      RP23XX_POWMAN_TIMER_ALARM_ENAB);
  powman_clrbits(RP23XX_POWMAN_TIMER, RP23XX_POWMAN_TIMER_ALARM);

#ifdef CONFIG_RTC_ALARM
  rp23xx_rtc_restorealarm(&g_suspend_alarm);
#endif
}

/****************************************************************************
 * Name: rp23xx_pm_pwrup_gpio
 *
 * Description:
 *   Arm a GPIO in a POWMAN power-up detector, which, unlike the dormant-wake
 *   detector of the IO bank, keeps working with the switched core off.
 *
 * Input Parameters:
 *   gpio - Pin to watch.
 *   edge - True for a transition, false for a level.
 *   high - True for rising/high, false for falling/low.
 *
 ****************************************************************************/

static void rp23xx_pm_pwrup_gpio(int gpio, bool edge, bool high)
{
  uint32_t regval;

  /* The detector reads the pad: enable the input, remove the isolation */

  modifyreg32(RP23XX_PADS_BANK0_GPIO(gpio), RP23XX_PADS_BANK0_GPIO_ISO,
              RP23XX_PADS_BANK0_GPIO_IE);

  /* Pull away from the wake level.  The reset pull-down on an idle-high
   * UART receive line costs 66 uA while suspended.  A resume does not run
   * arm_pminitialize(), so do it here.
   */

  if (high)
    {
      modifyreg32(RP23XX_PADS_BANK0_GPIO(gpio), RP23XX_PADS_BANK0_GPIO_PUE,
                  RP23XX_PADS_BANK0_GPIO_PDE);
    }
  else
    {
      modifyreg32(RP23XX_PADS_BANK0_GPIO(gpio), RP23XX_PADS_BANK0_GPIO_PDE,
                  RP23XX_PADS_BANK0_GPIO_PUE);
    }

  /* Disable the detector while its source changes */

  powman_clrbits(RP23XX_POWMAN_PWRUP0, RP23XX_POWMAN_PWRUP0_ENABLE);

  regval = (uint32_t)gpio & RP23XX_POWMAN_PWRUP0_SOURCE_MASK;

  if (edge)
    {
      regval |= RP23XX_POWMAN_PWRUP0_MODE;
    }

  if (high)
    {
      regval |= RP23XX_POWMAN_PWRUP0_DIRECTION;
    }

  putreg32(POWMAN_PASSWORD | regval, RP23XX_POWMAN_PWRUP0);

  /* Clear an old edge before enabling, or it ends the suspend at once */

  powman_clrbits(RP23XX_POWMAN_PWRUP0, RP23XX_POWMAN_PWRUP0_STATUS);
  powman_setbits(RP23XX_POWMAN_PWRUP0, RP23XX_POWMAN_PWRUP0_ENABLE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_pm_resume_pending
 *
 * Description:
 *   Tell if this boot is a resume.  Called from __start before .bss is
 *   cleared, so it reads only always-on registers.  The marker survives a
 *   reset too, so CHIP_RESET must also say that the switched core was
 *   powered down.  The marker is consumed, so a failed resume boots cold.
 *
 ****************************************************************************/

bool rp23xx_pm_resume_pending(void)
{
  uint32_t marker;
  uint32_t cause;

  marker = getreg32(RP23XX_POWMAN_SCRATCH0);
  cause  = getreg32(RP23XX_POWMAN_CHIP_RESET);

  putreg32(0, RP23XX_POWMAN_SCRATCH0);

  return marker == POWMAN_SUSPEND_MAGIC &&
         (cause & RP23XX_POWMAN_CHIP_RESET_HAD_SWCORE_PD) != 0;
}

/****************************************************************************
 * Name: rp23xx_pm_resume
 *
 * Description:
 *   Return to the suspended thread in place of nx_start().  The boot has
 *   set up the hardware again; restore the NVIC, which only RAM knew.
 *   Does not return.
 *
 ****************************************************************************/

void rp23xx_pm_resume(void)
{
  g_suspend_wake_source = getreg32(RP23XX_POWMAN_LAST_SWCORE_PWRUP);

  /* No interrupt until the longjmp: this stack belongs to no thread */

  up_irq_save();

  rp23xx_pm_nvic_restore();

  longjmp(g_suspend_ctx, 1);

  for (; ; );
}

/****************************************************************************
 * Name: rp23xx_pm_suspend
 *
 * Description:
 *   Power the switched core down (P1.0) and return after the wake.  Every
 *   peripheral comes back reset.
 *
 * Input Parameters:
 *   wake_ms - Wake after this many milliseconds with the always-on timer
 *             alarm, or 0 for no timed wake.  An RTC alarm that comes
 *             first also wakes the chip.
 *
 * Returned Value:
 *   Zero after the resume.  A negated errno if the request was refused.
 *
 ****************************************************************************/

int rp23xx_pm_suspend(uint32_t wake_ms)
{
  irqstate_t flags;
  uint32_t state;
  uint64_t now;
#ifdef CONFIG_RTC_ALARM
  uint64_t alarm;
#endif

  /* A debugger power-up request otherwise blocks the state change */

  powman_setbits(RP23XX_POWMAN_DBG_PWRCFG, RP23XX_POWMAN_DBG_PWRCFG_IGNORE);

  flags = enter_critical_section();

  /* Non-zero when rp23xx_pm_resume() returns here */

  if (setjmp(g_suspend_ctx) != 0)
    {
      leave_critical_section(flags);
      rp23xx_pm_wake_release();

#ifdef CONFIG_RTC
      /* The system tick stopped; take the time of day from the always-on
       * timer.  CLOCK_MONOTONIC excludes the suspend, as it should.
       */

      clock_synchronize(NULL);
#endif
      return OK;
    }

  rp23xx_pm_nvic_save();

  /* No bootrom resume vector: the resume uses the ordinary boot path */

  putreg32(0, RP23XX_POWMAN_BOOT0);
  putreg32(POWMAN_SUSPEND_MAGIC, RP23XX_POWMAN_SCRATCH0);

#if CONFIG_RP23XX_PM_WAKEUP_GPIO >= 0
  rp23xx_pm_pwrup_gpio(CONFIG_RP23XX_PM_WAKEUP_GPIO,
                       RP23XX_PM_SUSPEND_WAKE_EDGE,
                       RP23XX_PM_SUSPEND_WAKE_HIGH);
#endif

  now = ((uint64_t)getreg32(RP23XX_POWMAN_READ_TIME_UPPER) << 32) |
        getreg32(RP23XX_POWMAN_READ_TIME_LOWER);

#ifdef CONFIG_RTC_ALARM
  if (rp23xx_rtc_rdalarm(&alarm) == OK &&
      (wake_ms == 0 || alarm < now + wake_ms))
    {
      /* The application alarm comes first: let it power the chip up */

      g_suspend_alarm_wake = true;
      powman_setbits(RP23XX_POWMAN_TIMER,
                     RP23XX_POWMAN_TIMER_PWRUP_ON_ALARM);
    }
  else
#endif
  if (wake_ms > 0)
    {
      uint64_t when = now + wake_ms;

#ifdef CONFIG_RTC_ALARM
      rp23xx_rtc_savealarm(&g_suspend_alarm);
#endif

      g_suspend_wake_armed = true;

      powman_clrbits(RP23XX_POWMAN_TIMER, RP23XX_POWMAN_TIMER_ALARM_ENAB);

      powman_write(RP23XX_POWMAN_ALARM_TIME_15TO0,  (uint32_t)(when));
      powman_write(RP23XX_POWMAN_ALARM_TIME_31TO16, (uint32_t)(when >> 16));
      powman_write(RP23XX_POWMAN_ALARM_TIME_47TO32, (uint32_t)(when >> 32));
      powman_write(RP23XX_POWMAN_ALARM_TIME_63TO48, (uint32_t)(when >> 48));

      powman_clrbits(RP23XX_POWMAN_TIMER, RP23XX_POWMAN_TIMER_ALARM);
      powman_setbits(RP23XX_POWMAN_TIMER,
                     RP23XX_POWMAN_TIMER_PWRUP_ON_ALARM |
                     RP23XX_POWMAN_TIMER_ALARM_ENAB);
    }

  /* Write dirty XIP cache lines back to PSRAM.  The resume boot discards
   * the cache, and PSRAM can hold task stacks.
   */

  rp23xx_pm_xip_clean();

  /* Request P1.0.  Clear REQ_IGNORED first, so that it reports on this
   * request only.
   */

  powman_clrbits(RP23XX_POWMAN_STATE, RP23XX_POWMAN_STATE_REQ_IGNORED);
  powman_write(RP23XX_POWMAN_STATE, POWMAN_STATE_P1_0);

  state = getreg32(RP23XX_POWMAN_STATE);

  if ((state & RP23XX_POWMAN_STATE_REQ_IGNORED) != 0)
    {
      putreg32(0, RP23XX_POWMAN_SCRATCH0);
      leave_critical_section(flags);
      rp23xx_pm_wake_release();
      return -EBUSY;
    }

  if ((state & RP23XX_POWMAN_STATE_BAD_SW_REQ) != 0)
    {
      putreg32(0, RP23XX_POWMAN_SCRATCH0);
      leave_critical_section(flags);
      rp23xx_pm_wake_release();
      return -EINVAL;
    }

  /* The transition starts when the processors halt.  Power goes away in
   * the WFI, and execution continues in rp23xx_pm_resume().
   */

  for (; ; )
    {
      __asm__ __volatile__ ("dsb" ::: "memory");
      __asm__ __volatile__ ("wfi");
    }
}

/****************************************************************************
 * Name: rp23xx_pm_wake_source
 *
 * Description:
 *   What caused the last resume, as the raw LAST_SWCORE_PWRUP value.
 *
 ****************************************************************************/

uint32_t rp23xx_pm_wake_source(void)
{
  return g_suspend_wake_source;
}
