/****************************************************************************
 * boards/mips/pic32mx/sure-pic32mx/src/pic32mx_buttons.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "mips_internal.h"
#include "pic32mx.h"
#include "pic32mx_ioport.h"
#include "pic32mx_adc.h"
#include "sure-pic32mx.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Sure DB_DP11215 PIC32 Storage Demo Board has three buttons.
 *
 * SW1  (SW_UP, left arrow)      RB3 Pulled high, Grounded/low when depressed
 * SW2  (SW_DOWN, down arrow)    RB2 Pulled high, Grounded/low when depressed
 * SW3  (SW_OK, right arrow)     RB4 Pulled high, Grounded/low when depressed
 *
 * The Sure DB-DP11212 PIC32 General Purpose Demo Board also has three
 * buttons, but these are connected differently:
 *
 * SW2-1                         RF0 Pulled high, Grounded/low when depressed
 * SW3-1                         RF1 Pulled high, Grounded/low when depressed
 * SW5-1                         RD6 Pulled high, Grounded/low when depressed
 *
 * Internal pull-ups are not required since the LEDs are pull-up externally.
 * Change notification interrupts are not *automatically* enabled.  Change
 * notification will be enabled when pic32mx_gpioattach() is called.
 */

#ifdef CONFIG_ARCH_DBDP11215
#  define GPIO_SW1 (GPIO_INPUT|GPIO_INT|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_SW2 (GPIO_INPUT|GPIO_INT|GPIO_PORTB|GPIO_PIN2)
#  define GPIO_SW3 (GPIO_INPUT|GPIO_INT|GPIO_PORTB|GPIO_PIN4)
#else /* CONFIG_ARCH_DBDP11212 */
#  define GPIO_SW1 (GPIO_INPUT|GPIO_INT|GPIO_PORTF|GPIO_PIN0)
#  define GPIO_SW2 (GPIO_INPUT|GPIO_INT|GPIO_PORTF|GPIO_PIN1)
#  define GPIO_SW3 (GPIO_INPUT|GPIO_INT|GPIO_PORTD|GPIO_PIN6)
#endif

/* Change notification numbers:
 * RB3 -> CN5
 * RB2 -> CN2
 * RB4 -> CN6
 */

#define CN_SW1 5
#define CN_SW2 2
#define CN_SW3 6

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin configuration for each button */

static const uint16_t g_buttonset[NUM_BUTTONS] =
{
  GPIO_SW1 GPIO_SW2, GPIO_SW3
}

/* Change notification number for each button */

#ifdef CONFIG_ARCH_IRQBUTTONS
static const uint8_t g_buttoncn[NUM_BUTTONS] =
{
  CN_SW1, CN_SW2, CN_SW3
}
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  int i;

  /* Configure input pins */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      pic32mx_configgpio(g_buttonset[i]);
    }

  /* Change AN2/AN3/AN4 to digital */

  putreg32(0xffff, PIC32MX_ADC_CFG);
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int id;

  /* Configure input pins */

  for (id = 0; id < NUM_BUTTONS; id++)
    {
      if (pic32mx_gpioread(g_buttonset[id]))
        {
          ret |= (1 << id);
        }
    }

  return ret;
}

/****************************************************************************
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.
 *   See the BUTTON_*_BIT and JOYSTICK_*_BIT definitions in board.h for the
 *   meaning of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released. The ID value is
 *   a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* and JOYSTICK_* definitions in board.h for the meaning
 *   of enumeration value.
 *
 *   Interrupts are automatically enabled when the button handler is attached
 *   and automatically disabled when the button handler is detached.
 *
 *   When an interrupt occurs, it is due to a change on the GPIO input pin
 *   associated with the button.  In that case, all attached change
 *   notification handlers will be called.  Each handler must maintain state
 *   and determine if the unlying GPIO button input value changed.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret = -EINVAL;

  if (id < NUM_BUTTONS)
    {
      pic32mx_gpioirqdisable(g_buttoncn[id]);
      ret = pic32mx_gpioattach(g_buttonset[id], g_buttoncn[id],
                               irqhandler, arg);
      if (ret >= 0)
        {
          pic32mx_gpioirqenable(g_buttoncn[id]);
        }
    }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
