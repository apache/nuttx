/****************************************************************************
 * boards/arm/samv7/samv71-xult/src/sam_buttons.c
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
#include <nuttx/irq.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_gpio.h"
#include "hardware/sam_matrix.h"
#include "samv71-xult.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
#  define HAVE_IRQBUTTONS 1
#  if !defined(CONFIG_SAMV7_GPIOA_IRQ) && !defined(CONFIG_SAMV7_GPIOB_IRQ)
#    undef HAVE_IRQBUTTONS
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_irqx
 *
 * Description:
 *   This function implements the core of the board_button_irq() logic.
 *
 ****************************************************************************/

#ifdef HAVE_IRQBUTTONS
static int board_button_irqx(gpio_pinset_t pinset, int irq,
                             xcpt_t irqhandler, void *arg)
{
  irqstate_t flags;

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();

  /* Are we attaching or detaching? */

  if (irqhandler != NULL)
    {
      /* Configure the interrupt */

      sam_gpioirq(pinset);
      irq_attach(irq, irqhandler, arg);
      sam_gpioirqenable(irq);
    }
  else
    {
      /* Detach and disable the interrupt */

      irq_detach(irq);
      sam_gpioirqdisable(irq);
    }

  leave_critical_section(flags);
  return OK;
}
#endif

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
  uint32_t regval;

  /* PB12 is set up as a system flash ERASE pin when the firmware boots. To
   * use the SW1, PB12 has to be configured as a normal regular I/O pin in
   * the MATRIX module. For more information see the SAM V71 datasheet.
   */

  regval  = getreg32(SAM_MATRIX_CCFG_SYSIO);
  regval |= MATRIX_CCFG_SYSIO_SYSIO12;
  putreg32(regval, SAM_MATRIX_CCFG_SYSIO);

  /* Configure button GPIOs */

  sam_configgpio(GPIO_SW0);
  sam_configgpio(GPIO_SW1);
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.  See the BUTTON*
 *   definitions above for the meaning of each bit in the returned value.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t retval;

  retval  = sam_gpioread(GPIO_SW0) ? 0 : BUTTON_SW0_BIT;
  retval |= sam_gpioread(GPIO_SW1) ? 0 : BUTTON_SW1_BIT;

  return retval;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is one
 *   of the BUTTON* definitions provided above.
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR32_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature and CONFIG_AVR32_GPIOIRQSETA and/or
 *   CONFIG_AVR32_GPIOIRQSETB must be enabled to select GPIOs to support
 *   interrupts on.  For button support, bits 2 and 3 must be set in
 *   CONFIG_AVR32_GPIOIRQSETB (PB2 and PB3).
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
#ifdef HAVE_IRQBUTTONS

  switch (id)
    {
#ifdef CONFIG_SAMV7_GPIOA_IRQ
      case BUTTON_SW0:
        return board_button_irqx(GPIO_SW0, IRQ_SW0, irqhandler, arg);
#endif

#ifdef CONFIG_SAMV7_GPIOB_IRQ
      case BUTTON_SW1:
        return board_button_irqx(GPIO_SW1, IRQ_SW1, irqhandler, arg);
#endif

      default:
        return -EINVAL;
    }

#else

  return -ENOSYS;

#endif
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
