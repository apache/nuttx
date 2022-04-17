/****************************************************************************
 * boards/arm/kinetis/freedom-k66f/src/k66_buttons.c
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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "kinetis.h"

#include "freedom-k66f.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Two push buttons, SW2 and SW3, are available on FRDM-K66F board, where SW2
 * is connected to PTC6 and SW3 is connected to PTA4.
 * Besides the general purpose input/output functions, SW2 and SW3 can be
 * low-power wake up signal.
 *  Also, only SW3 can be a non-maskable interrupt.
 *
 *   Switch   GPIO Function
 *   -------- ---------------------------------------------------------------
 *   SW2      PTD11/LLWU_P25/SPI2_PCS0/SDHC0_CLKIN/LPUART0_CTS/FB_A19
 *   SW3      PTA10/LLWU_P22/FTM2_CH0/MII0_RXD2/FTM2_QD_PHA/TPM2_CH0/TRACE_D0
 */

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
  /* Configure the two buttons as inputs */

  kinetis_pinconfig(GPIO_SW2);
  kinetis_pinconfig(GPIO_SW3);
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;

  if (kinetis_gpioread(GPIO_SW2))
    {
      ret |= BUTTON_SW2_BIT;
    }

  if (kinetis_gpioread(GPIO_SW3))
    {
      ret |= BUTTON_SW3_BIT;
    }

  return ret;
}

/****************************************************************************
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button
 *   resources.  After that, board_buttons() may be called to collect the
 *   current state of all buttons or board_button_irq() may be called to
 *   register button interrupt handlers.
 *
 *   After board_button_initialize() has been called, board_buttons() may
 *   be called to collect the state of all buttons.  board_buttons() returns
 *   an 32-bit bit set with each bit associated with a button.  See the
 *   BUTTON_*_BIT and JOYSTICK_*_BIT definitions in board.h for the meaning
 *   of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released.  The ID value is
 *   a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* and JOYSTICK_* definitions in board.h for the meaning
 *   of enumeration value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  uint32_t pinset;
  int ret;

  /* Map the button id to the GPIO bit set. */

  if (id == BUTTON_SW2)
    {
      pinset = GPIO_SW2;
    }
  else if (id == BUTTON_SW3)
    {
      pinset = GPIO_SW3;
    }
  else
    {
      return -EINVAL;
    }

  /* The button has already been configured as an interrupting input (by
   * board_button_initialize() above).
   *
   * Attach the new button handler.
   */

  ret = kinetis_pinirqattach(pinset, irqhandler, NULL);
  if (ret >= 0)
    {
      /* Then make sure that interrupts are enabled on the pin */

      kinetis_pinirqenable(pinset);
    }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
