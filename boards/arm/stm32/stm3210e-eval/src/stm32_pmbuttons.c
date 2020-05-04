/****************************************************************************
 * boards/arm/stm32/stm3210e-eval/src/stm32_pmbuttons.c
 *
 *   Copyright (C) 2012, 2015-2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Diego Sanchez <dsanchez@nx-engineering.com>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <nuttx/board.h>
#include <nuttx/power/pm.h>
#include <arch/irq.h>

#include <stdbool.h>
#include <debug.h>

#include "arm_arch.h"
#include "nvic.h"
#include "stm32_pwr.h"
#include "stm32_pm.h"
#include "stm3210e-eval.h"

#if defined(CONFIG_PM) && defined(CONFIG_ARCH_IDLE_CUSTOM) && defined(CONFIG_PM_BUTTONS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_ARCH_BUTTONS
#  error "CONFIG_ARCH_BUTTONS is not defined in the configuration"
#endif

#define BUTTON_MIN   0
#ifdef CONFIG_DJOYSTICK
#  define BUTTON_MAX 2
#else
#  define BUTTON_MAX 7
#endif

#ifndef CONFIG_PM_BUTTONS_MIN
#  define CONFIG_PM_BUTTONS_MIN BUTTON_MIN
#endif
#ifndef CONFIG_PM_BUTTONS_MAX
#  define CONFIG_PM_BUTTONS_MAX BUTTON_MAX
#endif

#if CONFIG_PM_BUTTONS_MIN > CONFIG_PM_BUTTONS_MAX
#  error "CONFIG_PM_BUTTONS_MIN > CONFIG_PM_BUTTONS_MAX"
#endif

#if CONFIG_PM_BUTTONS_MAX > BUTTON_MAX
#  error "CONFIG_PM_BUTTONS_MAX > BUTTON_MAX"
#endif

#ifndef CONFIG_ARCH_IRQBUTTONS
#  warning "CONFIG_ARCH_IRQBUTTONS is not defined in the configuration"
#endif

#ifndef CONFIG_PM_IRQBUTTONS_MIN
#  define CONFIG_PM_IRQBUTTONS_MIN CONFIG_PM_BUTTONS_MIN
#endif

#ifndef CONFIG_PM_IRQBUTTONS_MAX
#  define CONFIG_PM_IRQBUTTONS_MAX CONFIG_PM_BUTTONS_MAX
#endif

#if CONFIG_PM_IRQBUTTONS_MIN > CONFIG_PM_IRQBUTTONS_MAX
#  error "CONFIG_PM_IRQBUTTONS_MIN > CONFIG_PM_IRQBUTTONS_MAX"
#endif

#if CONFIG_PM_IRQBUTTONS_MAX > 7
#  error "CONFIG_PM_IRQBUTTONS_MAX > 7"
#endif

#ifndef CONFIG_PM_BUTTON_ACTIVITY
#  define CONFIG_PM_BUTTON_ACTIVITY 10
#endif

#define PM_IDLE_DOMAIN 0 /* Revisit */

/* Miscellaneous Definitions ************************************************/

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif
#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

#define MIN_BUTTON MIN(CONFIG_PM_BUTTONS_MIN, CONFIG_PM_IRQBUTTONS_MIN)
#define MAX_BUTTON MAX(CONFIG_PM_BUTTONS_MAX, CONFIG_PM_IRQBUTTONS_MAX)

#define NUM_PMBUTTONS   (MAX_BUTTON - MIN_BUTTON + 1)
#define BUTTON_INDEX(b) ((b)-MIN_BUTTON)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
/****************************************************************************
 * Name: button_handler
 *
 * Description:
 *   Handle a button wake-up interrupt
 *
 ****************************************************************************/
static int button_handler(int irq, FAR void *context, FAR void *arg)
{
  /* At this point the MCU should have already awakened.  The state
   * change will be handled in the IDLE loop when the system is re-awakened
   * The button interrupt handler should be totally ignorant of the PM
   * activities and should report button activity as if nothing
   * special happened.
   */

  pm_activity(PM_IDLE_DOMAIN, CONFIG_PM_BUTTON_ACTIVITY);
  return 0;
}
#endif /* CONFIG_ARCH_IRQBUTTONS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pmbuttons
 *
 * Description:
 *   Configure all the buttons of the STM3210e-eval board as EXTI,
 *   so any button is able to wakeup the MCU from the PM_STANDBY mode
 *
 ****************************************************************************/

void stm32_pmbuttons(void)
{
#ifdef CONFIG_ARCH_IRQBUTTONS
  int ret;
  int i;
#endif

  /* Initialize the button GPIOs */

  board_button_initialize();

#ifdef CONFIG_ARCH_IRQBUTTONS
  for (i = CONFIG_PM_IRQBUTTONS_MIN; i <= CONFIG_PM_IRQBUTTONS_MAX; i++)
    {
      ret = board_button_irq(i, button_handler, (void*)i);
      if (ret < 0)
        {
          serr("ERROR: board_button_irq failed: %d\n", ret);
        }
    }
#endif
}

#endif /* defined(CONFIG_PM) && defined(CONFIG_ARCH_IDLE_CUSTOM) && defined(CONFIG_PM_BUTTONS) */
