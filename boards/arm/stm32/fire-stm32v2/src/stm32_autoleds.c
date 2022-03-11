/****************************************************************************
 * boards/arm/stm32/fire-stm32v2/src/stm32_autoleds.c
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
#include <nuttx/power/pm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32.h"
#include "fire-stm32v2.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following definitions map the encoded LED setting to GPIO settings.
 *
 * OFFBITS    ONBITS
 * CLR  SET  CLR  SET
 * 210  210  210  210
 */

#define FIRE_LED1         (1 << 0)
#define FIRE_LED2         (1 << 1)
#define FIRE_LED3         (1 << 2)

#define ON_SETBITS_SHIFT  (0)
#define ON_CLRBITS_SHIFT  (3)
#define OFF_SETBITS_SHIFT (6)
#define OFF_CLRBITS_SHIFT (9)

#define ON_BITS(v)        ((v) & 0x3f)
#define OFF_BITS(v)       (((v) >> 6) & 0x03f)
#define SETBITS(b)        ((b) & 0x07)
#define CLRBITS(b)        (((b) >> 3) & 0x07)

#define ON_SETBITS(v)     (SETBITS(ON_BITS(v))
#define ON_CLRBITS(v)     (CLRBITS(ON_BITS(v))
#define OFF_SETBITS(v)    (SETBITS(OFF_BITS(v))
#define OFF_CLRBITS(v)    (CLRBITS(OFF_BITS(v))

/*                                        ON               OFF
 *  -------------------------- -- ------------------ -----------------
 *                                LED1   LED2  LED3  LED1  LED2  LED3
 *  -------------------------- -- ------ ----- ----- ----- ----- -----
 *  LED_STARTED                0  OFF    OFF   OFF   OFF   OFF   OFF
 *  LED_HEAPALLOCATE           1  ON     OFF   OFF   OFF   OFF   OFF
 *  LED_IRQSENABLED            2  OFF    ON    OFF   ON    OFF   OFF
 *  LED_STACKCREATED           3  OFF    OFF   OFF   OFF   ON    OFF
 *
 *  LED_INIRQ                  4  NC     NC    ON    NC    NC    OFF
 *  LED_SIGNAL                 4  NC     NC    ON    NC    NC    OFF
 *  LED_ASSERTION              4  NC     NC    ON    NC    NC    OFF
 *  LED_PANIC                  4  NC     NC    ON    NC    NC    OFF
 *  -------------------------- -- ------ ----- ----- ----- ----- -----
 */

#define LED_STARTED_ON_SETBITS       (0)
#define LED_STARTED_ON_CLRBITS       ((FIRE_LED1|FIRE_LED2|FIRE_LED3) << ON_CLRBITS_SHIFT)
#define LED_STARTED_OFF_SETBITS      (0)
#define LED_STARTED_OFF_CLRBITS      ((FIRE_LED1|FIRE_LED2|FIRE_LED3) << OFF_CLRBITS_SHIFT)

#define LED_HEAPALLOCATE_ON_SETBITS  ((FIRE_LED1) << ON_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_ON_CLRBITS  ((FIRE_LED2|FIRE_LED3) << ON_CLRBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_SETBITS (0)
#define LED_HEAPALLOCATE_OFF_CLRBITS ((FIRE_LED1|FIRE_LED2|FIRE_LED3) << OFF_CLRBITS_SHIFT)

#define LED_IRQSENABLED_ON_SETBITS   ((FIRE_LED2) << ON_SETBITS_SHIFT)
#define LED_IRQSENABLED_ON_CLRBITS   ((FIRE_LED1|FIRE_LED3) << ON_CLRBITS_SHIFT)
#define LED_IRQSENABLED_OFF_SETBITS  ((FIRE_LED1) << OFF_SETBITS_SHIFT)
#define LED_IRQSENABLED_OFF_CLRBITS  ((FIRE_LED1|FIRE_LED2|FIRE_LED3) << OFF_CLRBITS_SHIFT)

#define LED_STACKCREATED_ON_SETBITS  (0)
#define LED_STACKCREATED_ON_CLRBITS  ((FIRE_LED1|FIRE_LED2|FIRE_LED3) << ON_CLRBITS_SHIFT)
#define LED_STACKCREATED_OFF_SETBITS ((FIRE_LED2) << OFF_SETBITS_SHIFT)
#define LED_STACKCREATED_OFF_CLRBITS ((FIRE_LED1|FIRE_LED3) << OFF_CLRBITS_SHIFT)

#define LED_FLASH_ON_SETBITS         ((FIRE_LED3) << ON_SETBITS_SHIFT)
#define LED_FLASH_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_FLASH_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_FLASH_OFF_CLRBITS        ((FIRE_LED3) << OFF_CLRBITS_SHIFT)

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* LED State Controls */

static inline void led_clrbits(unsigned int clrbits);
static inline void led_setbits(unsigned int setbits);
static void led_setonoff(unsigned int bits);

/* LED Power Management */

#ifdef CONFIG_PM
static void led_pm_notify(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
static int led_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_ledbits[8] =
{
  (LED_STARTED_ON_SETBITS       | LED_STARTED_ON_CLRBITS |
  LED_STARTED_OFF_SETBITS      | LED_STARTED_OFF_CLRBITS),

  (LED_HEAPALLOCATE_ON_SETBITS  | LED_HEAPALLOCATE_ON_CLRBITS |
  LED_HEAPALLOCATE_OFF_SETBITS | LED_HEAPALLOCATE_OFF_CLRBITS),

  (LED_IRQSENABLED_ON_SETBITS   | LED_IRQSENABLED_ON_CLRBITS |
  LED_IRQSENABLED_OFF_SETBITS  | LED_IRQSENABLED_OFF_CLRBITS),

  (LED_STACKCREATED_ON_SETBITS  | LED_STACKCREATED_ON_CLRBITS |
  LED_STACKCREATED_OFF_SETBITS | LED_STACKCREATED_OFF_CLRBITS),

  (LED_FLASH_ON_SETBITS         | LED_FLASH_ON_CLRBITS |
  LED_FLASH_OFF_SETBITS        | LED_FLASH_OFF_CLRBITS)
};

#ifdef CONFIG_PM
static struct pm_callback_s g_ledscb =
{
  .notify  = led_pm_notify,
  .prepare = led_pm_prepare,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_clrbits
 *
 * Description:
 *   Clear all LEDs to the bit encoded state.  The LEDs are pulled up and,
 *   hence, active low.
 *
 ****************************************************************************/

static inline void led_clrbits(unsigned int clrbits)
{
  if ((clrbits & FIRE_LED1) != 0)
    {
      stm32_gpiowrite(GPIO_LED1, true);
    }

  if ((clrbits & FIRE_LED2) != 0)
    {
      stm32_gpiowrite(GPIO_LED2, true);
    }

  if ((clrbits & FIRE_LED3) != 0)
    {
      stm32_gpiowrite(GPIO_LED3, true);
    }
}

/****************************************************************************
 * Name: led_setbits
 *
 * Description:
 *   Set all LEDs to the bit encoded state.  The LEDs are pulled up and,
 *   hence, active low.
 *
 ****************************************************************************/

static inline void led_setbits(unsigned int setbits)
{
  if ((setbits & FIRE_LED1) != 0)
    {
      stm32_gpiowrite(GPIO_LED1, false);
    }

  if ((setbits & FIRE_LED2) != 0)
    {
      stm32_gpiowrite(GPIO_LED2, false);
    }

  if ((setbits & FIRE_LED3) != 0)
    {
      stm32_gpiowrite(GPIO_LED3, false);
    }
}

/****************************************************************************
 * Name: led_setonoff
 *
 * Description:
 *   Set/clear all LEDs to the bit encoded state
 *
 ****************************************************************************/

static void led_setonoff(unsigned int bits)
{
  led_clrbits(CLRBITS(bits));
  led_setbits(SETBITS(bits));
}

/****************************************************************************
 * Name: led_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void led_pm_notify(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case(PM_NORMAL):
        {
          /* Restore normal LEDs operation */
        }
        break;

      case(PM_IDLE):
        {
          /* Entering IDLE mode - Turn leds off */
        }
        break;

      case(PM_STANDBY):
        {
          /* Entering STANDBY mode - Logic for PM_STANDBY goes here */
        }
        break;

      case(PM_SLEEP):
        {
          /* Entering SLEEP mode - Logic for PM_SLEEP goes here */
        }
        break;

      default:
        {
          /* Should not get here */
        }
        break;
    }
}
#endif

/****************************************************************************
 * Name: led_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int led_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate)
{
  /* No preparation to change power modes is required by the LEDs driver.
   * We always accept the state change by returning OK.
   */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void)
{
  /* Configure LED1-4 GPIOs for output */

  stm32_configgpio(GPIO_LED1);
  stm32_configgpio(GPIO_LED2);
  stm32_configgpio(GPIO_LED3);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  led_setonoff(ON_BITS(g_ledbits[led]));
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  led_setonoff(OFF_BITS(g_ledbits[led]));
}

/****************************************************************************
 * Name: up_ledpminitialize
 ****************************************************************************/

#ifdef CONFIG_PM
void up_ledpminitialize(void)
{
  /* Register to receive power management callbacks */

  int ret = pm_register(&g_ledscb);
  if (ret != OK)
    {
      board_autoled_on(LED_ASSERTION);
    }
}
#endif /* CONFIG_PM */

#endif /* CONFIG_ARCH_LEDS */
