/****************************************************************************
 * boards/avr/avrdx/breadxavr/src/avrdx_buttons.c
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
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include <nuttx/input/buttons.h>

#include <avr/io.h>

#include "avrdx_gpio.h"
#include "avrdx_iodefs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_BOARD_EARLY_INITIALIZE
#  error Button code must be initialized in board_early_initialize
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 *  Function called by the button lower half OS driver code in response
 *  to call of board_early_initialize() (see in avrdx_init.c.) It initializes
 *  button resources, making everything ready so board_buttons() and/or
 *  board_button_irq functions may be called.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  /* Button driver expects logical value of 1 when button is pressed,
   * need to invert the input.
   */

  PORTA.PIN2CTRL |= (PORT_PULLUPEN_bm | PORT_INVEN_bm);
  PORTA.PIN3CTRL |= (PORT_PULLUPEN_bm | PORT_INVEN_bm);
  PORTC.PIN2CTRL |= (PORT_PULLUPEN_bm | PORT_INVEN_bm);
  PORTC.PIN3CTRL |= (PORT_PULLUPEN_bm | PORT_INVEN_bm);

  return BBRD_NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *  Reader function called by button_lower.c (button driver lower half).
 *
 * Returned Value:
 *  Button states returned in integer variable as a bit field.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint8_t ret;
  uint8_t temp;

  /* Making nxstyle happy by using this temporary variable.
   * Comment must not be on the same line either.
   */

  temp = PIN2_bm | PIN3_bm;
  ret = (VPORTA.IN & (temp)) >> 2;
  ret |= (VPORTC.IN & (PIN2_bm | PIN3_bm));

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *  Setter function called by button_lower.c (button driver lower half).
 *
 * Input parameters:
 *   Button ID - numeric value, board code knows what hardware it maps to
 *   IRQ handler. Handler to be called to service pin interrupt. If set
 *     to NULL, then the interrupt handling is to be disabled
 *   Opaque argument arg passed to the handler. Currently always NULL.
 *
 * Returned Value:
 *  Value returned by the ISR multiplexer.
 *
 ****************************************************************************/

int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  int ret;
  uint8_t pin_select;
  uint8_t port_idx;

  /* Pins 2 and 3 on both ports, can use a bit of a trick */

  pin_select = 0x04;
  pin_select <<= (id & 1);

  if (id < 2)
    {
      port_idx = AVRDX_GPIO_PORTA_IDX;
    }
  else
    {
      port_idx = AVRDX_GPIO_PORTC_IDX;
    }

  if (irqhandler)
    {
      /* Note that if we wanted only a specific edge, ie. if the button
       * driver lower half supported only one type of event (press/release),
       * the ISC variable must be inverted because we are inverting
       * the input
       */

      ret = avrdx_irq_attach_gpio_mux(\
          port_idx,
          pin_select,
          irqhandler,
          arg,
          PORT_ISC_BOTHEDGES_GC
      );
    }
  else
    {
      ret = avrdx_irq_detach_gpio_mux(\
          port_idx,
          pin_select,
          true
      );
    }

  return ret;
}
