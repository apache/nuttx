/****************************************************************************
 * boards/mips/pic32mx/sure-pic32mx/src/pic32mx_autoleds.c
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

#include "chip.h"
#include "mips_internal.h"
#include "pic32mx.h"
#include "pic32mx_ioport.h"
#include "sure-pic32mx.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED Configuration ********************************************************/

/* The Sure PIC32MX board has five LEDs.  One (D4, lablel "Power") is not
 * controllable by software.  Four are controllable by software:
 *
 * D7  "USB"    Yellow  RD7 Low illuminates
 * D8  "SD"     Yellow  RD6 Low illuminates
 * D9  "Flash"  Yellow  RF0 Low illuminates
 * D10 "Error"  Red     RF1 Low illuminates
 *
 *                           ON                  OFF
 *                           USB SD  FLASH ERROR USB SD  FLASH ERROR
 * LED_STARTED            0  OFF OFF OFF   OFF   --- --- ---   ---
 * LED_HEAPALLOCATE       1  ON  OFF N/C   N/C   --- --- ---   ---
 * LED_IRQSENABLED        2  OFF ON  N/C   N/C   --- --- ---   ---
 * LED_STACKCREATED       3  ON  ON  N/C   N/C   --- --- ---   ---
 * LED_INIRQ              4  N/C N/C ON    N/C   N/C N/C OFF   N/C
 * LED_SIGNAL             4  N/C N/C ON    N/C   N/C N/C OFF   N/C
 * LED_ASSERTION          4  N/C N/C ON    N/C   N/C N/C OFF   N/C
 * LED_PANIC              5  N/C N/C N/C   ON    N/C N/C N/C   OFF
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
  uint8_t usb    : 2;
  uint8_t sd     : 2;
  uint8_t flash  : 2;
  uint8_t error  : 2;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct led_setting_s g_ledonvalues[LED_NVALUES] =
{
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF},
  {LED_ON,  LED_OFF, LED_NC,  LED_NC},
  {LED_OFF, LED_ON,  LED_NC,  LED_NC},
  {LED_ON,  LED_ON,  LED_NC,  LED_NC},
  {LED_NC,  LED_NC,  LED_ON,  LED_NC},
  {LED_NC,  LED_NC,  LED_NC,  LED_ON},
};

static const struct led_setting_s g_ledoffvalues[LED_NVALUES] =
{
  {LED_NC,  LED_NC,  LED_NC,  LED_NC},
  {LED_NC,  LED_NC,  LED_NC,  LED_NC},
  {LED_NC,  LED_NC,  LED_NC,  LED_NC},
  {LED_NC,  LED_NC,  LED_NC,  LED_NC},
  {LED_NC,  LED_NC,  LED_OFF, LED_NC},
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_setleds
 ****************************************************************************/

static void pic32mx_setleds(const struct led_setting_s *setting)
{
  if (setting->usb != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_USB_LED, setting->usb != LED_ON);
    }

  if (setting->sd != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_SD_LED, setting->sd != LED_ON);
    }

  if (setting->flash != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_FLASH_LED, setting->flash != LED_ON);
    }

  if (setting->error != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_ERROR_LED, setting->error != LED_ON);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_led_initialize
 ****************************************************************************/

void pic32mx_led_initialize(void)
{
  /* Configure output pins */

  pic32mx_configgpio(GPIO_USB_LED);
  pic32mx_configgpio(GPIO_SD_LED);
  pic32mx_configgpio(GPIO_FLASH_LED);
  pic32mx_configgpio(GPIO_ERROR_LED);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led < LED_NVALUES)
    {
      pic32mx_setleds(&g_ledonvalues[led]);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led < LED_NVALUES)
    {
      pic32mx_setleds(&g_ledoffvalues[led]);
    }
}
#endif /* CONFIG_ARCH_LEDS */
