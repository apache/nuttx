/****************************************************************************
 * boards/arm/stm32/maple/src/stm32_lcd.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Modified: Librae <librae8226@gmail.com>
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/memlcd.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"
#include "stm32.h"
#include "maple.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#define EXTCOMIN_FREQ    24
#define TIMER_FREQ       1200    /* 72000000/60000 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lcd_dev_s *l_lcddev;
static struct spi_dev_s *spi;
static struct stm32_tim_dev_s *tim;
static xcpt_t g_isr;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int up_lcdextcominisr(int irq, void *context, void *arg)
{
  STM32_TIM_ACKINT(tim, ATIM_SR_UIF);
  if (g_isr == NULL)
    {
      lcderr("ERROR: error, irq not attached, disabled\n");
      STM32_TIM_DISABLEINT(tim, ATIM_DIER_UIE);
      return OK;
    }

  return g_isr(irq, context, arg);
}

static int up_lcdirqattach(xcpt_t isr, void * arg)
{
  lcdinfo("%s IRQ\n", isr == NULL ? "Detach" : "Attach");

  if (isr != NULL)
    {
      STM32_TIM_SETISR(tim, up_lcdextcominisr, arg, ATIM_SR_UIF);
      g_isr = isr;
    }
  else
    {
      STM32_TIM_SETISR(tim, NULL, NULL, ATIM_SR_UIF);
      g_isr = NULL;
    }

  return OK;
}

static void up_lcddispcontrol(bool on)
{
  lcdinfo("set: %s\n", on ? "on" : "off");

  if (on)
    {
      stm32_gpiowrite(GPIO_MEMLCD_DISP, 1);
      STM32_TIM_ENABLEINT(tim, ATIM_DIER_UIE);
    }
  else
    {
      stm32_gpiowrite(GPIO_MEMLCD_DISP, 0);
      STM32_TIM_DISABLEINT(tim, ATIM_DIER_UIE);
    }
}

#ifndef CONFIG_MEMLCD_EXTCOMIN_MODE_HW
static void up_lcdsetpolarity(bool pol)
{
  stm32_gpiowrite(GPIO_LED, pol);
  stm32_gpiowrite(GPIO_MEMLCD_EXTCOMIN, pol);
}
#endif

static void up_lcdsetvcomfreq(unsigned int freq)
{
  lcdinfo("freq: %d\n", freq);
  DEBUGASSERT(freq >= 1 && freq <= 60);
  STM32_TIM_SETPERIOD(tim, TIMER_FREQ / freq);
}

static FAR struct memlcd_priv_s memlcd_priv =
{
  .attachirq   = up_lcdirqattach,
  .dispcontrol = up_lcddispcontrol,
#ifndef CONFIG_MEMLCD_EXTCOMIN_MODE_HW
  .setpolarity = up_lcdsetpolarity,
#endif
  .setvcomfreq = up_lcdsetvcomfreq,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use,
 *   but with the power setting at 0 (full off).
 *
 ****************************************************************************/

FAR int board_lcd_initialize(void)
{
  lcdinfo("Initializing lcd\n");

  lcdinfo("init spi1\n");
  spi = stm32_spibus_initialize(1);
  DEBUGASSERT(spi);

  lcdinfo("configure related io\n");
  stm32_configgpio(GPIO_MEMLCD_EXTCOMIN);
  stm32_configgpio(GPIO_MEMLCD_DISP);

  lcdinfo("configure EXTCOMIN timer\n");
  if (tim == NULL)
    {
      tim = stm32_tim_init(2);
      DEBUGASSERT(tim);
      STM32_TIM_SETPERIOD(tim, TIMER_FREQ / EXTCOMIN_FREQ);
      STM32_TIM_SETCLOCK(tim, TIMER_FREQ);
      STM32_TIM_SETMODE(tim, STM32_TIM_MODE_UP);
    }

  lcdinfo("init lcd\n");
  l_lcddev = memlcd_initialize(spi, &memlcd_priv, 0);
  DEBUGASSERT(l_lcddev);

  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return l_lcddev;
}
