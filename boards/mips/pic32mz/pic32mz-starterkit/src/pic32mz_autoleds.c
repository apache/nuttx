/****************************************************************************
 * boards/mips/pic32mz/pic32mz-starterkit/src/pic32mz_leds.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "mips_arch.h"
#include "mips_internal.h"

#include "pic32mz_gpio.h"
#include "pic32mz-starterkit.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED Configuration ********************************************************/

/* The PIC32MZ Ethernet Starter kit has 3 user LEDs labelled LED1-3 on the
 * board:
 *
 *   PIN  LED   Notes
 *   ---  ----- -------------------------
 *   RH0  LED1  High illuminates (RED)
 *   RH1  LED3  High illuminates (YELLOW)
 *   RH2  LED2  High illuminates (GREEN)
 *
 * If CONFIG_ARCH_LEDs is defined, then NuttX will control the 3 LEDs
 * on board the Ethernet Starter Kit.  The following definitions
 * describe how NuttX controls the LEDs:
 *
 *                           ON                  OFF
 * ------------------------- ---- ---- ---- ---- ---- ----
 *                           LED1 LED2 LED3 LED1 LED2 LED3
 * ------------------------- ---- ---- ---- ---- ---- ----
 * LED_STARTED            0  OFF  OFF  OFF  ---  ---  ---
 * LED_HEAPALLOCATE       1  ON   OFF  N/C  ---  ---  ---
 * LED_IRQSENABLED        2  OFF  ON   N/C  ---  ---  ---
 * LED_STACKCREATED       3  ON   ON   N/C  ---  ---  ---
 * LED_INIRQ              4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_SIGNAL             4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_ASSERTION          4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_PANIC              5  ON   N/C  N/C  OFF  N/C  N/C
 */

/* LED Management Definitions ***********************************************/

#define LED_OFF 0
#define LED_ON  1
#define LED_NC  2

/****************************************************************************
 * Private types
 ****************************************************************************/

struct led_setting_s
{
  uint8_t led1   : 2;
  uint8_t led2   : 2;
  uint8_t led3   : 2;
  uint8_t unused : 2;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* If CONFIG_ARCH_LEDS is defined then NuttX will control the LEDs.  The
 * following structures identified the LED settings for each NuttX LED state.
 */

static const struct led_setting_s g_ledonvalues[LED_NVALUES] =
{
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF},
  {LED_ON,  LED_OFF, LED_NC,  LED_OFF},
  {LED_OFF, LED_ON,  LED_NC,  LED_OFF},
  {LED_ON,  LED_ON,  LED_NC,  LED_OFF},
  {LED_NC,  LED_NC,  LED_ON,  LED_OFF},
  {LED_ON,  LED_NC,  LED_NC,  LED_OFF},
};

static const struct led_setting_s g_ledoffvalues[LED_NVALUES] =
{
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF},
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF},
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF},
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF},
  {LED_NC,  LED_NC,  LED_OFF, LED_OFF},
  {LED_OFF, LED_NC,  LED_NC,  LED_OFF},
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_setleds
 ****************************************************************************/

static void pic32mz_setleds(FAR const struct led_setting_s *setting)
{
  if (setting->led1 != LED_NC)
    {
      pic32mz_gpiowrite(GPIO_LED_1, setting->led1 == LED_ON);
    }

  if (setting->led2 != LED_NC)
    {
      pic32mz_gpiowrite(GPIO_LED_2, setting->led2 == LED_ON);
    }

  if (setting->led3 != LED_NC)
    {
      pic32mz_gpiowrite(GPIO_LED_3, setting->led3 == LED_ON);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_led_initialize
 ****************************************************************************/

void pic32mz_led_initialize(void)
{
  /* Configure output pins */

  pic32mz_configgpio(GPIO_LED_1);
  pic32mz_configgpio(GPIO_LED_2);
  pic32mz_configgpio(GPIO_LED_3);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if ((unsigned)led < LED_NVALUES)
    {
      pic32mz_setleds(&g_ledonvalues[led]);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if ((unsigned)led < LED_NVALUES)
    {
      pic32mz_setleds(&g_ledoffvalues[led]);
    }
}

#endif /* CONFIG_ARCH_LEDS */
