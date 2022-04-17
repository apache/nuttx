/****************************************************************************
 * boards/mips/pic32mx/pic32mx7mmb/src/pic32_leds.c
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
#include "pic32mx7mmb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED Configuration ********************************************************/

/* The Mikroelektronika PIC32MX7 MMB has 3 user LEDs labelled LED0-2 in the
 * schematics:
 *
 * ---  ----- --------------------------------------------------------------
 * PIN  Board Notes
 * ---  ----- --------------------------------------------------------------
 * RA0  LED0  Pulled-up, low value illuminates
 * RA1  LED1  Pulled-up, low value illuminates
 * RD9  LED2  Pulled-up, low value illuminates
 * RA9  LED4  Not available for general use*, indicates MMC/SD activity
 * ---  LED5  Not controllable by software, indicates power-on
 *
 * * RA9 is also the SD chip select.  It will illuminate whenever the SD card
 *   is selected.  If SD is not used, then LED4 could also be used as a user-
 *   controlled LED.
 *
 * If CONFIG_ARCH_LEDS is defined, then NuttX will control these LEDs as
 * follows:
 *                           ON                  OFF
 * ------------------------- ---- ---- ---- ---- ---- ----
 *                           LED0 LED1 LED2 LED0 LED1 LED2
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

#define GPIO_LED_0   (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTA|GPIO_PIN0)
#define GPIO_LED_1   (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTA|GPIO_PIN1)
#define GPIO_LED_2   (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTD|GPIO_PIN9)
#define GPIO_LED_4   (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTA|GPIO_PIN9)

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
  uint8_t led2   : 2;
  uint8_t unused : 2;
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

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following array simply maps the PIC32MX_PIC32MX7MMB_LEDn
 * index values to the correct LED pin configuration.
 */

#else
static const uint16_t g_ledpincfg[PIC32MX_PIC32MX7MMB_NLEDS] =
{
  GPIO_LED_0, GPIO_LED_1, GPIO_LED_2
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_setleds
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
static void pic32mx_setleds(const struct led_setting_s *setting)
{
  /* LEDs are pulled up so writing a low value (false) illuminates them */

  if (setting->led0 != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_LED_0, setting->led0 != LED_ON);
    }

  if (setting->led1 != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_LED_1, setting->led1 != LED_ON);
    }

  if (setting->led2 != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_LED_2, setting->led2 != LED_ON);
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
  pic32mx_configgpio(GPIO_LED_2);
  return 3;
}
#endif

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void board_userled(int led, bool ledon)
{
  /* LEDs are pulled up so writing a low value (false) illuminates them */

  if ((unsigned)led < PIC32MX_PIC32MX7MMB_NLEDS)
    {
      pic32mx_gpiowrite(g_ledpincfg[led], !ledon);
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

  board_userled(PIC32MX_PIC32MX7MMB_LED0,
               (ledset & PIC32MX_PIC32MX7MMB_LED0_BIT) != 0);
  board_userled(PIC32MX_PIC32MX7MMB_LED1,
               (ledset & PIC32MX_PIC32MX7MMB_LED1_BIT) != 0);
  board_userled(PIC32MX_PIC32MX7MMB_LED2,
               (ledset & PIC32MX_PIC32MX7MMB_LED2_BIT) != 0);
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
  pic32mx_configgpio(GPIO_LED_2);
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
