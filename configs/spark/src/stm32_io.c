/****************************************************************************
 * configs/spark/src/stm32_io.c
 *
 *   Copyright (C) 2011-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <arch/board/board.h>
#include "chip/stm32_tim.h"

#include "spark.h"

#ifndef CONFIG_CC3000_PROBES

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_leds
 *
 * Description:
 *
 ****************************************************************************/

void up_leds(int r, int g ,int b, int freqs)
{
  long fosc = 72000000;
  long prescale = 2048;
  long p1s = fosc/prescale;
  long p0p5s  = p1s/2;
  long p;

  static struct stm32_tim_dev_s *tim1 = 0;

  if (tim1 == 0)
    {
      tim1 = stm32_tim_init(1);
      STM32_TIM_SETMODE(tim1, STM32_TIM_MODE_UP);
      STM32_TIM_SETCLOCK(tim1, p1s-8);
      STM32_TIM_SETPERIOD(tim1, p1s);
      STM32_TIM_SETCOMPARE(tim1, 1, 0);
      STM32_TIM_SETCOMPARE(tim1, 2, 0);
      STM32_TIM_SETCOMPARE(tim1, 3, 0);
      STM32_TIM_SETCHANNEL(tim1, 1, STM32_TIM_CH_OUTPWM | STM32_TIM_CH_POLARITY_NEG);
      STM32_TIM_SETCHANNEL(tim1, 2, STM32_TIM_CH_OUTPWM | STM32_TIM_CH_POLARITY_NEG);
      STM32_TIM_SETCHANNEL(tim1, 3, STM32_TIM_CH_OUTPWM | STM32_TIM_CH_POLARITY_NEG);
    }

  p = freqs == 0 ? p1s : p1s / freqs;
  STM32_TIM_SETPERIOD(tim1, p);

  p = freqs == 0 ? p1s + 1 : p0p5s / freqs;

  STM32_TIM_SETCOMPARE(tim1, 2, (r * p) / 255);
  STM32_TIM_SETCOMPARE(tim1, 1, (b * p) / 255);
  STM32_TIM_SETCOMPARE(tim1, 3, (g * p) / 255);
}

/****************************************************************************
 * Name: up_ioinit
 *
 * Description:
 *
 ****************************************************************************/

void up_ioinit(void)
{
  /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are
   * configured for all pins.
   */

  up_leds(0,0,0,0);
  stm32_configgpio(GPIO_A0); /* Probes */
  stm32_configgpio(GPIO_A1); /* Probes */
  stm32_configgpio(GPIO_A2); /* Smart Config */
  stm32_configgpio(GPIO_A3); /* not used */
  stm32_configgpio(GPIO_D0); /* Sw 1 */
  stm32_configgpio(GPIO_D1); /* Sw 2 */
  stm32_configgpio(GPIO_D2); /* Activate */
}

/****************************************************************************
 * Name: up_read_inputs
 *
 * N.B The return state in true logic, the button polarity is dealt here in
 *
 ****************************************************************************/

uint8_t up_read_inputs(void)
{
  uint8_t bits = 0;
  bits |= stm32_gpioread(GPIO_D0) == 0 ? 1 : 0;
  bits |= stm32_gpioread(GPIO_D1) == 0 ? 2 : 0;
  bits |= stm32_gpioread(GPIO_A2) == 0 ? 4 : 0;
  bits |= stm32_gpioread(GPIO_A3) == 0 ? 8 : 0;
  return bits;
}

/****************************************************************************
 * Name: up_write_outputs
 *
 * N.B The return state in true logic, the button polarity is dealt here in
 *
 ****************************************************************************/

void up_write_outputs(int id, bool bits)
{
  if (id == 2)
    {
      stm32_gpiowrite(GPIO_D2, bits);
    }
  else if (id == 0)
    {
      stm32_gpiowrite(GPIO_A0, bits);
    }
  else if (id == 1)
    {
      stm32_gpiowrite(GPIO_A1, bits);
    }
}

/****************************************************************************
 * Name: up_irqio
 *
 * Description:
 *
 ****************************************************************************/

xcpt_t up_irqio(int id, xcpt_t irqhandler)
{
  xcpt_t oldhandler = NULL;

  /* The following should be atomic */

  if (id == 0)
    {
      oldhandler = stm32_gpiosetevent(GPIO_D0, true, true, true, irqhandler);
    }
  else if (id == 1)
    {
      oldhandler = stm32_gpiosetevent(GPIO_D1, true, true, true, irqhandler);
    }

  return oldhandler;
}
#endif /* CONFIG_CC3000_PROBES */
