/****************************************************************************
 * boards/arm/stm32/maple/src/stm32_lcd.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/memlcd.h>

#include "chip.h"
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

static struct memlcd_priv_s memlcd_priv =
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

int board_lcd_initialize(void)
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

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return l_lcddev;
}
