/****************************************************************************
 * arch/arm/src/stm32/stm32_pwm.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/pwm.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32_pwm.h"
#include "stm32_internal.h"

/* This module then only compiles if there is at least one enabled timer
 * intended for use with the PWM upper half driver.
 */

#if defined(CONFIG_STM32_TIM1_PWM)  || defined(CONFIG_STM32_TIM2_PWM)  || \
    defined(CONFIG_STM32_TIM3_PWM)  || defined(CONFIG_STM32_TIM4_PWM)  || \
    defined(CONFIG_STM32_TIM5_PWM)  || defined(CONFIG_STM32_TIM6_PWM)  || \
    defined(CONFIG_STM32_TIM7_PWM)  || defined(CONFIG_STM32_TIM8_PWM)  || \
    defined(CONFIG_STM32_TIM9_PWM)  || defined(CONFIG_STM32_TIM10_PWM) || \
    defined(CONFIG_STM32_TIM11_PWM) || defined(CONFIG_STM32_TIM12_PWM) || \
    defined(CONFIG_STM32_TIM13_PWM) || defined(CONFIG_STM32_TIM14_PWM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure representst the state of one PWM timer */

struct stm32_pwmtimer_s
{
  uint32_t base;  /* The base address of the timer */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);
static int pwm_start(FAR struct pwm_lowerhalf_s *dev, FAR const struct pwm_info_s *info);
static int pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int pwm_pulsecount(FAR struct pwm_lowerhalf_s *dev, FAR pwm_count_t *count);
static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* This is the list of lower half PWM driver methods used by the upper half driver */

static const struct pwm_ops_s g_pwmops =
{
  .setup      = pwm_setup(FAR struct pwm_lowerhalf_s *dev);
  .shutdown   = pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);
  .start      = pwm_start(FAR struct pwm_lowerhalf_s *dev, FAR const struct pwm_info_s *info);
  .stop       = pwm_stop(FAR struct pwm_lowerhalf_s *dev);
  .pulsecount = pwm_pulsecount(FAR struct pwm_lowerhalf_s *dev, FAR pwm_count_t *count);
  .ioctl      = pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg);
};

/* The following represent the state of each possible PWM driver */

#ifdef CONFIG_STM32_TIM1_PWM
static struct stm32_pwmtimer_s g_pwm1dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM1_BASE;
};
#endif

#ifdef CONFIG_STM32_TIM2_PWM
static struct stm32_pwmtimer_s g_pwm2dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM2_BASE;
};
#endif

#ifdef CONFIG_STM32_TIM3_PWM
static struct stm32_pwmtimer_s g_pwm3dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM3_BASE;
};
#endif

#ifdef CONFIG_STM32_TIM4_PWM
static struct stm32_pwmtimer_s g_pwm4dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM4_BASE;
};
#endif

#ifdef CONFIG_STM32_TIM5_PWM
static struct stm32_pwmtimer_s g_pwm5dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM5_BASE;
};
#endif

#ifdef CONFIG_STM32_TIM6_PWM
static struct stm32_pwmtimer_s g_pwm6dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM6_BASE;
};
#endif

#ifdef CONFIG_STM32_TIM7_PWM
static struct stm32_pwmtimer_s g_pwm7dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM7_BASE;
};
#endif

#ifdef CONFIG_STM32_TIM8_PWM
static struct stm32_pwmtimer_s g_pwm8dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM8_BASE;
};
#endif

#ifdef CONFIG_STM32_TIM9_PWM
static struct stm32_pwmtimer_s g_pwm9dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM9_BASE;
};
#endif

#ifdef CONFIG_STM32_TIM10_PWM
static struct stm32_pwmtimer_s g_pwm10dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM10_BASE;
};
#endif

#ifdef CONFIG_STM32_TIM11_PWM
static struct stm32_pwmtimer_s g_pwm11dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM11_BASE;
};
#endif

#ifdef CONFIG_STM32_TIM12_PWM
static struct stm32_pwmtimer_s g_pwm12dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM12_BASE;
};
#endif

#ifdef CONFIG_STM32_TIM13_PWM
static struct stm32_pwmtimer_s g_pwm13dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM13_BASE;
};
#endif

#ifdef CONFIG_STM32_TIM14_PWM
static struct stm32_pwmtimer_s g_pwm14dev =
{
  .ops        = *g_pwmops;
  .base       = STM32_TIM14_BASE;
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_start(FAR struct pwm_lowerhalf_s *dev, FAR const struct pwm_info_s *info)
{
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: pwm_pulsecount
 *
 * Description:
 *   Get the number of pulses generated
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   count - A pointer to the location to return the pulse count
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_pulsecount(FAR struct pwm_lowerhalf_s *dev, FAR pwm_count_t *count)
{
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name:
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg)
{
  /* There are no platform-specific ioctl commands */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family but is somewhere in
 *     the range of {1,..,14}.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *stm32_pwminitialize(int timer)
{
  FAR struct stm32_pwmtimer_s *lower;

  switch (timer)
    {
#ifdef CONFIG_STM32_TIM1_PWM
      case 1:
        lower = &g_pwm1dev;
        break;
#endif
#ifdef CONFIG_STM32_TIM2_PWM
      case 2:
        lower = &g_pwm2dev;
        break;
#endif
#ifdef CONFIG_STM32_TIM3_PWM
      case 3:
        lower = &g_pwm3dev;
        break;
#endif
#ifdef CONFIG_STM32_TIM4_PWM
      case 4:
        lower = &g_pwm4dev;
        break;
#endif
#ifdef CONFIG_STM32_TIM5_PWM
      case 5:
        lower = &g_pwm5dev;
        break;
#endif
#ifdef CONFIG_STM32_TIM6_PWM
      case 6:
        lower = &g_pwm6dev;
        break;
#endif
#ifdef CONFIG_STM32_TIM7_PWM
      case 7:
        lower = &g_pwm7dev;
        break;
#endif
#ifdef CONFIG_STM32_TIM8_PWM
      case 8:
        lower = &g_pwm8dev;
        break;
#endif
#ifdef CONFIG_STM32_TIM9_PWM
      case 9:
        lower = &g_pwm9dev;
        break;
#endif
#ifdef CONFIG_STM32_TIM10_PWM
      case 10:
        lower = &g_pwm10dev;
        break;
#endif
#ifdef CONFIG_STM32_TIM11_PWM
      case 11:
        lower = &g_pwm11dev;
        break;
#endif
#ifdef CONFIG_STM32_TIM12_PWM
      case 12:
        lower = &g_pwm12dev;
        break;
#endif
#ifdef CONFIG_STM32_TIM13_PWM
      case 13:
        lower = &g_pwm13dev;
        break;
#endif
#ifdef CONFIG_STM32_TIM14_PWM
      case 14:
        lower = &g_pwm14dev;
        break;
#endif
      default:
        return NULL;
    }

  return (FAR struct pwm_lowerhalf_s *)lower;
}

#endif /* CONFIG_STM32_TIMn_PWM, n = 1,...,14 */
