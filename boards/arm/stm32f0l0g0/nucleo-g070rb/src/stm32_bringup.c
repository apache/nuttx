/****************************************************************************
 * boards/arm/stm32f0l0g0/nucleo-g070rb/src/stm32_bringup.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Daniel Pereira Volpato <dpo@certi.org.br>
 *
 *   Based on: boards/arm/stm32f0l0g0/nucleo-g071rb/src/stm32_bringup.c
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#include <sys/types.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/input/buttons.h>
#include <nuttx/leds/userled.h>

#include "nucleo-g070rb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_LEDS
#undef HAVE_DAC

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
#  define HAVE_LEDS 1
#endif

/* TODO ??? */

#if defined(CONFIG_DAC)
#  define HAVE_DAC1 1
#  define HAVE_DAC2 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y && CONFIG_NSH_ARCHINIT:
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret;

#ifdef HAVE_LEDS
  /* Register the LED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  /* Register the GPIO driver */

  ret = stm32_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_TIMER
  /* Initialize basic timers */

#if defined(CONFIG_STM32F0L0G0_TIM6)
  syslog(LOG_ERR, "Init timer\n");
  ret = stm32_timer_driver_setup("/dev/timer0", 6);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_timer_driver_setup failed. TIM6: %d\n", ret);
    }
#endif

#if defined(CONFIG_STM32F0L0G0_TIM7)
  ret = stm32_timer_driver_setup("/dev/timer1", 7);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_timer_driver_setup failed. TIM7: %d\n", ret);
    }

#endif
#endif

#if defined(CONFIG_PWM)
  /* Initialize PWM and register the PWM device */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
