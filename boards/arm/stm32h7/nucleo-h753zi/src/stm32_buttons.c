/****************************************************************************
 * boards/arm/stm32h7/nucleo-h753zi/src/stm32_buttons.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/irq.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32_gpio.h"
#include "nucleo-h753zi.h"

#ifdef CONFIG_NUCLEO_H753ZI_BUTTON_SUPPORT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_INPUT_BUTTONS) && !defined(CONFIG_ARCH_IRQBUTTONS)
#  error "The NuttX Buttons Driver depends on IRQ support to work!"
#endif

#define MAX_PIN_CONFIG_LEN 512

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Dynamic button configuration array */

static uint32_t g_buttons[CONFIG_NUCLEO_H753ZI_BUTTON_COUNT];
static int g_button_count = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_exti_line
 *
 * Description:
 *   Extract EXTI line number from GPIO configuration.
 *
 * Input Parameters:
 *   gpio_config - STM32 GPIO configuration
 *
 * Returned Value:
 *   EXTI line number (0-15), or -1 on error
 *
 ****************************************************************************/

static int get_exti_line(uint32_t gpio_config)
{
  /* Extract pin number from GPIO_PIN mask */

  uint32_t pin_mask = gpio_config & GPIO_PIN_MASK;

  switch (pin_mask)
    {
      case GPIO_PIN0:
        return 0;

      case GPIO_PIN1:
        return 1;

      case GPIO_PIN2:
        return 2;

      case GPIO_PIN3:
        return 3;

      case GPIO_PIN4:
        return 4;

      case GPIO_PIN5:
        return 5;

      case GPIO_PIN6:
        return 6;

      case GPIO_PIN7:
        return 7;

      case GPIO_PIN8:
        return 8;

      case GPIO_PIN9:
        return 9;

      case GPIO_PIN10:
        return 10;

      case GPIO_PIN11:
        return 11;

      case GPIO_PIN12:
        return 12;

      case GPIO_PIN13:
        return 13;

      case GPIO_PIN14:
        return 14;

      case GPIO_PIN15:
        return 15;

      default:
        return -1;
    }
}

/****************************************************************************
 * Name: get_gpio_port_letter
 *
 * Description:
 *   Get port letter from GPIO configuration.
 *
 * Input Parameters:
 *   gpio_config - STM32 GPIO configuration
 *
 * Returned Value:
 *   Port letter ('A'-'H'), or '?' on error
 *
 ****************************************************************************/

static char get_gpio_port_letter(uint32_t gpio_config)
{
  uint32_t port = gpio_config & GPIO_PORT_MASK;

  switch (port)
    {
      case GPIO_PORTA:
        return 'A';

      case GPIO_PORTB:
        return 'B';

      case GPIO_PORTC:
        return 'C';

      case GPIO_PORTD:
        return 'D';

      case GPIO_PORTE:
        return 'E';

      case GPIO_PORTF:
        return 'F';

      case GPIO_PORTG:
        return 'G';

      case GPIO_PORTH:
        return 'H';

      default:
        return '?';
    }
}

/****************************************************************************
 * Name: parse_gpio_pin
 *
 * Description:
 *   Parse GPIO pin string like "PF15" into STM32 GPIO configuration.
 *
 * Input Parameters:
 *   pin_str - GPIO pin string (e.g., "PA0", "PF15", "PC13")
 *   error   - Pointer to error code storage
 *
 * Returned Value:
 *   STM32 GPIO configuration value on success, 0 on error
 *
 ****************************************************************************/

static uint32_t parse_gpio_pin(FAR const char *pin_str, FAR int *error)
{
  size_t len;
  char port;
  FAR const char *pin_num_str;
  FAR char *endptr;
  long pin_num;
  uint32_t port_base;
  uint32_t gpio_pin;

  *error = 0;

  if (pin_str == NULL)
    {
      *error = -EINVAL;
      return 0;
    }

  /* Remove leading/trailing spaces */

  while (*pin_str == ' ' || *pin_str == '\t')
    {
      pin_str++;
    }

  len = strlen(pin_str);
  if (len < 3 || len > 4)
    {
      *error = -EINVAL;
      return 0;
    }

  if (pin_str[0] != 'P')
    {
      *error = -EINVAL;
      return 0;
    }

  port = pin_str[1];
  if (port < 'A' || port > 'H')
    {
      *error = -EINVAL;
      return 0;
    }

  pin_num_str = &pin_str[2];
  pin_num = strtol(pin_num_str, &endptr, 10);
  if (*endptr != '\0' || pin_num < 0 || pin_num > 15)
    {
      *error = -EINVAL;
      return 0;
    }

  /* Map port letter to STM32 port base */

  switch (port)
    {
      case 'A':
        port_base = GPIO_PORTA;
        break;

      case 'B':
        port_base = GPIO_PORTB;
        break;

      case 'C':
        port_base = GPIO_PORTC;
        break;

      case 'D':
        port_base = GPIO_PORTD;
        break;

      case 'E':
        port_base = GPIO_PORTE;
        break;

      case 'F':
        port_base = GPIO_PORTF;
        break;

      case 'G':
        port_base = GPIO_PORTG;
        break;

      case 'H':
        port_base = GPIO_PORTH;
        break;

      default:
        *error = -EINVAL;
        return 0;
    }

  /* Use correct STM32 GPIO pin macros */

  switch (pin_num)
    {
      case 0:
        gpio_pin = GPIO_PIN0;
        break;

      case 1:
        gpio_pin = GPIO_PIN1;
        break;

      case 2:
        gpio_pin = GPIO_PIN2;
        break;

      case 3:
        gpio_pin = GPIO_PIN3;
        break;

      case 4:
        gpio_pin = GPIO_PIN4;
        break;

      case 5:
        gpio_pin = GPIO_PIN5;
        break;

      case 6:
        gpio_pin = GPIO_PIN6;
        break;

      case 7:
        gpio_pin = GPIO_PIN7;
        break;

      case 8:
        gpio_pin = GPIO_PIN8;
        break;

      case 9:
        gpio_pin = GPIO_PIN9;
        break;

      case 10:
        gpio_pin = GPIO_PIN10;
        break;

      case 11:
        gpio_pin = GPIO_PIN11;
        break;

      case 12:
        gpio_pin = GPIO_PIN12;
        break;

      case 13:
        gpio_pin = GPIO_PIN13;
        break;

      case 14:
        gpio_pin = GPIO_PIN14;
        break;

      case 15:
        gpio_pin = GPIO_PIN15;
        break;

      default:
        *error = -EINVAL;
        return 0;
    }

  /* CRITICAL FIX: Use GPIO_PULLDOWN instead of GPIO_FLOAT
   *
   * The Nucleo-H753ZI board has external pull-down resistors on button
   * pins. Using GPIO_FLOAT causes instability due to high impedance state
   * combined with variable leakage current. GPIO_PULLDOWN ensures stable
   * LOW state when button is not pressed, working in conjunction with the
   * external pull-down for robust operation.
   */

  return (GPIO_INPUT | GPIO_PULLDOWN | GPIO_EXTI | port_base | gpio_pin);
}

/****************************************************************************
 * Name: print_exti_conflict_error
 *
 * Description:
 *   Print detailed EXTI conflict error message.
 *
 * Input Parameters:
 *   conflicting_button - Index of button that conflicts
 *   current_button     - Index of button being added
 *   pin_str            - Pin string being added
 *   gpio_config        - GPIO config of new pin
 *   exti_line          - EXTI line that conflicts
 *
 ****************************************************************************/

static void print_exti_conflict_error(int conflicting_button,
                                       int current_button,
                                       FAR const char *pin_str,
                                       uint32_t gpio_config,
                                       int exti_line)
{
  char port1 = get_gpio_port_letter(g_buttons[conflicting_button]);
  char port2 = get_gpio_port_letter(gpio_config);
  int pin1 = get_exti_line(g_buttons[conflicting_button]);
  int pin2 = get_exti_line(gpio_config);
  int alt;
  int i;
  bool used;

  syslog(LOG_ERR, "\n");
  syslog(LOG_ERR,
         "======================================================\n");
  syslog(LOG_ERR, "  CRITICAL ERROR: EXTI LINE CONFLICT DETECTED!\n");
  syslog(LOG_ERR,
         "======================================================\n");
  syslog(LOG_ERR, "\n");
  syslog(LOG_ERR,
         "Button %d (P%c%d) and Button %d (%s = P%c%d) both use EXTI%d\n",
         conflicting_button, port1, pin1,
         current_button, pin_str, port2, pin2,
         exti_line);
  syslog(LOG_ERR, "\n");
  syslog(LOG_ERR, "EXPLANATION:\n");
  syslog(LOG_ERR,
         "  STM32 EXTI lines are shared across GPIO ports.\n");
  syslog(LOG_ERR,
         "  Only ONE pin per number can be used as interrupt source.\n");
  syslog(LOG_ERR,
         "  Example: PA3, PB3, PC3, PD3... all share EXTI3.\n");
  syslog(LOG_ERR, "\n");
  syslog(LOG_ERR, "WHAT WILL HAPPEN IF YOU IGNORE THIS:\n");
  syslog(LOG_ERR, "  - The LAST pin (P%c%d) will work\n", port2, pin2);
  syslog(LOG_ERR,
         "  - The FIRST pin (P%c%d) will NOT trigger interrupts\n",
         port1, pin1);
  syslog(LOG_ERR, "  - You will see 'ghost' button presses\n");
  syslog(LOG_ERR, "\n");
  syslog(LOG_ERR, "SOLUTION:\n");
  syslog(LOG_ERR,
         "  Change one of the conflicting pins to a different number.\n");
  syslog(LOG_ERR, "\n");
  syslog(LOG_ERR, "  Example fixes:\n");

  /* Suggest alternative pins */

  for (alt = 0; alt < 16; alt++)
    {
      if (alt == exti_line)
        {
          continue;
        }

      /* Check if this EXTI is already used */

      used = false;
      for (i = 0; i < current_button; i++)
        {
          if (get_exti_line(g_buttons[i]) == alt)
            {
              used = true;
              break;
            }
        }

      if (!used)
        {
          syslog(LOG_ERR,
                 "  - Change P%c%d to P%c%d (EXTI%d is free)\n",
                 port2, pin2, port2, alt, alt);
          break;
        }
    }

  syslog(LOG_ERR, "\n");
  syslog(LOG_ERR, "Current button configuration:\n");
  for (i = 0; i < current_button; i++)
    {
      int line = get_exti_line(g_buttons[i]);
      char prt = get_gpio_port_letter(g_buttons[i]);
      syslog(LOG_ERR, "  Button %d: P%c%d (EXTI%d)%s\n",
             i, prt, line, line,
             (i == conflicting_button) ? " <- CONFLICT" : "");
    }

  syslog(LOG_ERR, "  Button %d: %s (P%c%d, EXTI%d) <- CONFLICT\n",
         current_button, pin_str, port2, pin2, exti_line);
  syslog(LOG_ERR, "\n");
}

/****************************************************************************
 * Name: init_button_configs
 *
 * Description:
 *   Initialize button configuration from Kconfig settings with validation.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

static int init_button_configs(void)
{
  int expected_pins;
  FAR const char *pins_config;
  char pins_str[MAX_PIN_CONFIG_LEN];
  FAR char *pin;
  int error;
  uint32_t gpio_config;
  int i;
  int exti_line;
  size_t config_len;

  /* EXTI conflict detection */

  int exti_usage[16];

  g_button_count = 0;

  /* Initialize EXTI tracking */

  for (i = 0; i < 16; i++)
    {
      exti_usage[i] = -1;  /* -1 = unused */
    }

  /* Calculate how many external pins we expect */

#ifdef CONFIG_NUCLEO_H753ZI_BUTTON_BUILTIN
  g_buttons[g_button_count] = GPIO_BTN_BUILT_IN;

  /* Register built-in button EXTI */

  exti_line = get_exti_line(GPIO_BTN_BUILT_IN);
  if (exti_line >= 0)
    {
      exti_usage[exti_line] = g_button_count;
    }

  g_button_count++;
  expected_pins = CONFIG_NUCLEO_H753ZI_BUTTON_COUNT - 1;
#else
  expected_pins = CONFIG_NUCLEO_H753ZI_BUTTON_COUNT;
#endif

  /* If no external pins needed, we're done */

  if (expected_pins == 0)
    {
      return OK;
    }

  /* Validate pin string is not empty */

  pins_config = CONFIG_NUCLEO_H753ZI_BUTTON_PINS;
  if (pins_config == NULL || strlen(pins_config) == 0)
    {
      syslog(LOG_ERR, "ERROR: Button pins not configured!\n");
      syslog(LOG_ERR,
             "Expected %d GPIO pins but BUTTON_PINS is empty\n",
             expected_pins);
      syslog(LOG_ERR,
             "Configure in: Board Selection -> Button Configuration\n");
      return -EINVAL;
    }

  /* Validate configuration length */

  config_len = strlen(pins_config);
  if (config_len >= MAX_PIN_CONFIG_LEN)
    {
      syslog(LOG_ERR,
             "ERROR: Pin configuration too long (%zu bytes, max %d)\n",
             config_len, MAX_PIN_CONFIG_LEN - 1);
      return -EINVAL;
    }

  /* Make a copy for parsing (strtok modifies the string) */

  strncpy(pins_str, pins_config, sizeof(pins_str) - 1);
  pins_str[sizeof(pins_str) - 1] = '\0';

  /* Parse and validate each pin */

  pin = strtok(pins_str, ", \t\n\r");
  while (pin != NULL && g_button_count < CONFIG_NUCLEO_H753ZI_BUTTON_COUNT)
    {
      /* Parse GPIO pin */

      gpio_config = parse_gpio_pin(pin, &error);
      if (error != 0)
        {
          syslog(LOG_ERR, "ERROR: Invalid GPIO pin: \"%s\"\n", pin);
          syslog(LOG_ERR,
                 "Use format: PORT+PIN (e.g., \"PA0\", \"PF15\")\n");
          syslog(LOG_ERR, "Valid ports: PA-PH, Valid pins: 0-15\n");
          return -EINVAL;
        }

      /* Check for duplicate pins */

      for (i = 0; i < g_button_count; i++)
        {
          if (g_buttons[i] == gpio_config)
            {
              syslog(LOG_ERR, "ERROR: Duplicate GPIO pin: \"%s\"\n", pin);
              syslog(LOG_ERR,
                     "This pin is already used by Button %d\n", i);
              return -EINVAL;
            }
        }

      /* Check for EXTI line conflicts */

      exti_line = get_exti_line(gpio_config);
      if (exti_line < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to determine EXTI line for \"%s\"\n",
                 pin);
          return -EINVAL;
        }

      if (exti_usage[exti_line] >= 0)
        {
          /* EXTI conflict detected */

          print_exti_conflict_error(exti_usage[exti_line], g_button_count,
                                     pin, gpio_config, exti_line);
          return -EINVAL;
        }

      /* Register this EXTI line as used */

      exti_usage[exti_line] = g_button_count;

      /* Add button */

      g_buttons[g_button_count] = gpio_config;
      g_button_count++;

      pin = strtok(NULL, ", \t\n\r");
    }

  /* Validate final count */

  if (g_button_count != CONFIG_NUCLEO_H753ZI_BUTTON_COUNT)
    {
      syslog(LOG_ERR, "ERROR: Button count mismatch!\n");
      syslog(LOG_ERR, "Expected: %d, Got: %d\n",
             CONFIG_NUCLEO_H753ZI_BUTTON_COUNT, g_button_count);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button
 *   resources. After that, board_buttons() may be called to collect the
 *   current state of all buttons or board_button_irq() may be called to
 *   register button interrupt handlers.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  int ret;
  int i;

  ret = init_button_configs();
  if (ret < 0)
    {
      syslog(LOG_ERR, "\n");
      syslog(LOG_ERR,
             "======================================================\n");
      syslog(LOG_ERR, "  BUTTON CONFIGURATION FAILED\n");
      syslog(LOG_ERR,
             "======================================================\n");
      syslog(LOG_ERR, "\n");
      syslog(LOG_ERR, "Please fix the errors above and rebuild.\n");
      syslog(LOG_ERR, "\n");
      return 0;  /* Return 0 to indicate failure */
    }

  /* Configure GPIO pins */

  for (i = 0; i < g_button_count; i++)
    {
      ret = stm32_configgpio(g_buttons[i]);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to configure GPIO for button %d "
                 "(ret=%d)\n", i, ret);
          return 0;
        }
    }

  syslog(LOG_INFO, "Button driver initialized: %d buttons ready\n",
         g_button_count);

  return g_button_count;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   board_buttons() may be called to collect the current state of all
 *   buttons. board_buttons() returns a 32-bit bit set with each bit
 *   associated with a button.  See the BUTTON_*_BIT definitions in
 *   board.h for the meaning of each bit.
 *
 * Returned Value:
 *   32-bit set of button states. Bit set = button pressed.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  bool pressed;
  int i;

  /* Check the state of each button */

  for (i = 0; i < g_button_count; i++)
    {
      /* With pull-down: HIGH = pressed, LOW = released */

      pressed = stm32_gpioread(g_buttons[i]);

      /* Set bit if button is pressed */

      if (pressed)
        {
          ret |= (1 << i);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   board_button_irq() may be called to register an interrupt handler
 *   that will be called when a button is depressed or released.  The ID
 *   value is a button enumeration value that uniquely identifies a button
 *   resource. See the BUTTON_* definitions in board.h for the meaning of
 *   enumeration value.
 *
 * Input Parameters:
 *   id         - Button ID (0-based index)
 *   irqhandler - IRQ handler function
 *   arg        - Argument passed to IRQ handler
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  int ret = -EINVAL;

  /* Validate button ID */

  if (id < 0 || id >= g_button_count)
    {
      syslog(LOG_ERR, "Invalid button ID %d (valid: 0-%d)\n",
             id, g_button_count - 1);
      return ret;
    }

  /* Register interrupt for both rising and falling edges */

  ret = stm32_gpiosetevent(g_buttons[id], true, true, true,
                           irqhandler, arg);

  if (ret < 0)
    {
      syslog(LOG_ERR,
             "Failed to register IRQ for button %d (ret=%d)\n",
             id, ret);
    }

  return ret;
}
#endif

#endif /* CONFIG_NUCLEO_H753ZI_BUTTON_SUPPORT */
