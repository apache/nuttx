/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_pm.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_PM_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_RP23XX_PM

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_pm_standby
 *
 * Description:
 *   Enter the RP2350 SLEEP state: WFI with the clocks of blocks that have
 *   no driver gated.  Any enabled interrupt wakes the core.
 *
 ****************************************************************************/

void rp23xx_pm_standby(void);

/****************************************************************************
 * Name: rp23xx_pm_sleep
 *
 * Description:
 *   Enter the RP2350 DORMANT state: stop the PLLs and the crystal
 *   oscillator.  Only a GPIO armed with rp23xx_pm_gpio_wakeup() can wake
 *   the chip, so with none armed this enters the standby state instead.
 *
 ****************************************************************************/

void rp23xx_pm_sleep(void);

/****************************************************************************
 * Name: rp23xx_pm_gpio_wakeup
 *
 * Description:
 *   Arm a GPIO in the dormant-wake detector.  The detector watches the pad,
 *   so the pin keeps its function (a UART receive pin can be armed).
 *
 * Input Parameters:
 *   gpio - GPIO number to watch.
 *   edge - True to trigger on a transition, false on a level.
 *   high - True for rising edge / high level, false for falling / low.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure.
 *
 ****************************************************************************/

int rp23xx_pm_gpio_wakeup(int gpio, bool edge, bool high);

/****************************************************************************
 * Name: rp23xx_pm_gpio_wakeup_disable
 *
 * Description:
 *   Disarm a GPIO previously passed to rp23xx_pm_gpio_wakeup().
 *
 ****************************************************************************/

int rp23xx_pm_gpio_wakeup_disable(int gpio);

/****************************************************************************
 * Name: rp23xx_pm_pads_quiesce
 *
 * Description:
 *   Isolate the unused pads and hold the unused blocks in reset.
 *
 ****************************************************************************/

#ifdef CONFIG_RP23XX_PM_QUIESCE_PADS
void rp23xx_pm_pads_quiesce(void);
#endif

/****************************************************************************
 * Name: g_pm_resume_stack
 *
 * Description:
 *   The stack a resume boots on.  The idle stack still holds the frames of
 *   the idle thread, which the resumed system returns to.
 *
 ****************************************************************************/

#ifdef CONFIG_RP23XX_PM_SUSPEND
#  define RP23XX_PM_RESUME_STACK_WORDS (CONFIG_IDLETHREAD_STACKSIZE / 4)

EXTERN uint32_t g_pm_resume_stack[RP23XX_PM_RESUME_STACK_WORDS];
#endif

/****************************************************************************
 * Name: rp23xx_pm_suspend
 *
 * Description:
 *   Power the switched core down (POWMAN P1.0) and return after the wake.
 *   Memory is kept, but every peripheral comes back reset.
 *
 * Input Parameters:
 *   wake_ms - Timed wake in milliseconds, or 0 for none.  An RTC alarm
 *             that comes first also wakes the chip.
 *
 * Returned Value:
 *   Zero after the resume; a negated errno if the request was refused.
 *
 * Name: rp23xx_pm_gpio_wakeup_restore
 *
 * Description:
 *   Arm the dormant-wake GPIOs again after a resume.
 *
 * Name: rp23xx_pm_resume_pending / rp23xx_pm_resume
 *
 * Description:
 *   Called from __start only.  rp23xx_pm_resume_pending() tells if this
 *   boot is a resume, before .bss and .data are touched.
 *   rp23xx_pm_resume() then replaces nx_start() and does not return.
 *
 ****************************************************************************/

#ifdef CONFIG_RP23XX_PM_SUSPEND
int rp23xx_pm_suspend(uint32_t wake_ms);
void rp23xx_pm_gpio_wakeup_restore(void);
uint32_t rp23xx_pm_wake_source(void);
bool rp23xx_pm_resume_pending(void);
void rp23xx_pm_resume(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* CONFIG_RP23XX_PM */
#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_PM_H */
