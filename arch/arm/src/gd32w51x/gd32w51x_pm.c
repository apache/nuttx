/****************************************************************************
 * arch/arm/src/gd32w51x/gd32w51x_pm.c
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

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/power/pm.h>

#include "arm_internal.h"
#include "nvic.h"
#include "chip.h"
#include "gd32w51x_clockconfig.h"
#include "gd32w51x_pm.h"
#include "hardware/gd32w51x_uart.h"

#ifdef CONFIG_PM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PMU_CTL0: LDO and low-driver modes during deep-sleep */

#define GD32W51X_PMU_CTL0       (GD32W51X_PMU_BASE + 0x0000)
#define PMU_CTL0_LDOLP          (1 << 0)   /* LDO low power mode */
#define PMU_CTL0_STBMOD         (1 << 1)   /* Standby mode */
#define PMU_CTL0_LDLP           (1 << 10)  /* Low-driver with low power LDO */
#define PMU_CTL0_LDNP           (1 << 11)  /* Low-driver with normal LDO */
#define PMU_CTL0_LDEN           (3 << 18)  /* Low-driver mode enable */

/* DBG_CTL0: keep the debugger connection in deep-sleep, so that a sleeping
 * board can still be programmed.
 */

#define GD32W51X_DBG_CTL0       0xe0044004
#define DBG_CTL0_DSLP_HOLD      (1 << 1)

/* The console UART, emptied before deep-sleep stops its clock */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE          GD32W51X_USART0_BASE
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE          GD32W51X_UART1_BASE
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_BASE          GD32W51X_UART2_BASE
#endif

#define CONSOLE_FLUSH_TIMEOUT   1000000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_pminitialize
 *
 * Description:
 *   Initialize the power management, early in up_initialize().
 *
 ****************************************************************************/

void arm_pminitialize(void)
{
  modifyreg32(GD32W51X_DBG_CTL0, 0, DBG_CTL0_DSLP_HOLD);
  pm_initialize();
}

/****************************************************************************
 * Name: gd32w51x_pmstop
 *
 * Description:
 *   Enter the deep-sleep mode: the core and bus clocks stop, SRAM and
 *   registers keep their contents.  Any EXTI interrupt wakes the chip up,
 *   and so does the RTC wake-up timer after
 *   CONFIG_GD32W51X_PM_WAKEUP_MS milliseconds (when it is not zero).  On
 *   return the clock tree is set up again and the system time is
 *   corrected with the time spent asleep.
 *
 *   Called with the interrupts disabled.
 *
 ****************************************************************************/

void gd32w51x_pmstop(void)
{
#ifdef CONSOLE_BASE
  int timeout;
#endif
  uint8_t basepri;

#ifdef CONSOLE_BASE
  /* Let the console finish the character on the wire */

  for (timeout = CONSOLE_FLUSH_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(CONSOLE_BASE + GD32W51X_UART_STAT_OFFSET) &
           UART_STAT_TC) != 0)
        {
          break;
        }
    }
#endif

#if defined(CONFIG_RTC) && CONFIG_GD32W51X_PM_WAKEUP_MS > 0
  gd32w51x_rtc_setwakeup(CONFIG_GD32W51X_PM_WAKEUP_MS);
#endif

  /* LDO in low power mode and low-driver mode during deep-sleep */

  modifyreg32(GD32W51X_PMU_CTL0,
              PMU_CTL0_STBMOD | PMU_CTL0_LDOLP | PMU_CTL0_LDEN |
              PMU_CTL0_LDNP | PMU_CTL0_LDLP,
              PMU_CTL0_LDOLP | PMU_CTL0_LDEN | PMU_CTL0_LDLP);

  /* WFI only wakes up on an interrupt that could be taken: the critical
   * section raises BASEPRI, which would mask the wake-up interrupts.  Mask
   * them with PRIMASK instead while waiting, so that they wake the core up
   * and are serviced once the critical section is left.
   */

  basepri = getbasepri();
  cpsid();
  setbasepri(0);

  modifyreg32(NVIC_SYSCON, 0, NVIC_SYSCON_SLEEPDEEP);
  UP_DSB();
  asm volatile ("wfi");
  UP_ISB();
  modifyreg32(NVIC_SYSCON, NVIC_SYSCON_SLEEPDEEP, 0);

  setbasepri(basepri);
  cpsie();

  /* The chip wakes up on the IRC16M: bring the PLL back */

  gd32w51x_clockconfig();

#ifdef CONFIG_RTC
  gd32w51x_rtc_setwakeup(0);
  gd32w51x_rtc_resync();

  /* The system timer was stopped: add the time spent asleep */

  clock_resynchronize(NULL);
#endif
}

#endif /* CONFIG_PM */
