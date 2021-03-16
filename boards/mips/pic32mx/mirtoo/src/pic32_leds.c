/****************************************************************************
 * boards/mips/pic32mx/mirtoo/src/pic32_leds.c
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
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "pic32mx.h"
#include "mirtoo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED Configuration ********************************************************/

/* The Mirtoo module has 2 user LEDs labelled LED0 and LED1:
 *
 * ---  ----- --------------------------------------------------------------
 * PIN  Board Notes
 * ---  ----- --------------------------------------------------------------
 * RC8  LED0  Grounded, high value illuminates
 * RC9  LED1  Grounded, high value illuminates
 *
 * The Dimitech DTX1-4000L EV-kit1 supports 3 more LEDs, but there are not
 * controllable from software.
 *
 * If CONFIG_ARCH_LEDS is defined, then NuttX will control these LEDs as
 * follows:
 *                              ON        OFF
 * ------------------------- ---- ---- ---- ----
 *                           LED0 LED1 LED0 LED1
 * ------------------------- ---- ---- ---- ----
 * LED_STARTED            0  OFF  OFF  ---  ---
 * LED_HEAPALLOCATE       1  ON   OFF  ---  ---
 * LED_IRQSENABLED        2  OFF  ON   ---  ---
 * LED_STACKCREATED       3  ON   ON   ---  ---
 * LED_INIRQ              4  ON   N/C  OFF  N/C
 * LED_SIGNAL             4  ON   N/C  OFF  N/C
 * LED_ASSERTION          4  ON   N/C  OFF  N/C
 * LED_PANIC              4  ON   N/C  OFF  N/C
 */

#define GPIO_LED_0   (GPIO_OUTPUT|GPIO_VALUE_ZERO|GPIO_PORTC|GPIO_PIN8)
#define GPIO_LED_1   (GPIO_OUTPUT|GPIO_VALUE_ZERO|GPIO_PORTC|GPIO_PIN9)

/* LED Management Definitions ***********************************************/

#ifdef CONFIG_ARCH_LEDS
#  define LED_OFF 0
#  define LED_ON  1
#  define LED_NC  2
#endif

/****************************************************************************
 * Private types
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
struct led_setting_s
{
  uint8_t led0   : 2;
  uint8_t led1   : 2;
  uint8_t unused : 4;
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* If CONFIG_ARCH_LEDS is defined then NuttX will control the LEDs.  The
 * following structures identified the LED settings for each NuttX LED state.
 */

#ifdef CONFIG_ARCH_LEDS
static const struct led_setting_s g_ledonvalues[LED_NVALUES] =
{
  {LED_OFF, LED_OFF, 0},
  {LED_ON,  LED_OFF, 0},
  {LED_OFF, LED_ON,  0},
  {LED_ON,  LED_ON,  0},
  {LED_ON,  LED_NC,  0},
};

static const struct led_setting_s g_ledoffvalues[LED_NVALUES] =
{
  {LED_NC,  LED_NC,  0},
  {LED_NC,  LED_NC,  0},
  {LED_NC,  LED_NC,  0},
  {LED_NC,  LED_NC,  0},
  {LED_OFF, LED_NC,  0},
};

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following array simply maps the PIC32MX_MIRTOO_LEDn
 * index values to the correct LED pin configuration.
 */

#else
static const uint16_t g_ledpincfg[PIC32MX_MIRTOO_NLEDS] =
{
  GPIO_LED_0, GPIO_LED_1
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_setleds
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
static void pic32mx_setleds(FAR const struct led_setting_s *setting)
{
  /* LEDs are pulled up so writing a low value (false) illuminates them */

  if (setting->led0 != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_LED_0, setting->led0 == LED_ON);
    }

  if (setting->led1 != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_LED_1, setting->led1 == LED_ON);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
uint32_t board_userled_initialize(void)
{
  /* Configure output pins */

  pic32mx_configgpio(GPIO_LED_0);
  pic32mx_configgpio(GPIO_LED_1);
  return 2;
}
#endif

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void board_userled(int led, bool ledon)
{
  /* LEDs are pulled up so writing a low value (false) illuminates them */

  if ((unsigned)led < PIC32MX_MIRTOO_NLEDS)
    {
      pic32mx_gpiowrite(g_ledpincfg[led], ledon);
    }
}
#endif

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void board_userled_all(uint32_t ledset)
{
  /* Call board_userled() with ledon == true to illuminated the LED */

  board_userled(PIC32MX_MIRTOO_LED0, !!(ledset & PIC32MX_MIRTOO_LED0_BIT));
  board_userled(PIC32MX_MIRTOO_LED1, !!(ledset & PIC32MX_MIRTOO_LED1_BIT));
}
#endif

/****************************************************************************
 * Name: pic32mx_led_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void pic32mx_led_initialize(void)
{
  /* Configure output pins */

  pic32mx_configgpio(GPIO_LED_0);
  pic32mx_configgpio(GPIO_LED_1);
}
#endif

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_on(int led)
{
  if ((unsigned)led < LED_NVALUES)
    {
      pic32mx_setleds(&g_ledonvalues[led]);
    }
}
#endif

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_off(int led)
{
  if ((unsigned)led < LED_NVALUES)
    {
      pic32mx_setleds(&g_ledoffvalues[led]);
    }
}
#endif
