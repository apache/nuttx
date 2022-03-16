/****************************************************************************
 * arch/arm/src/eoss3/eoss3_gpio.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "chip.h"

#include "eoss3.h"
#include "eoss3_gpio.h"
#include "hardware/eoss3_iomux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eoss3_configgpio
 *
 * Description:
 *   Configure a PIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int eoss3_configgpio(gpio_pinset_t cfgset)
{
  uint16_t pad = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint16_t ctrl = (cfgset & GPIO_CTRL_MASK) >> GPIO_CTRL_SHIFT;
  uint16_t input = (cfgset & GPIO_INPUT_SEL_MASK) >> GPIO_INPUT_SEL_SHIFT;
  uint8_t sel = input & EOSS3_PAD_SEL_VAL_MASK;
  uint16_t sel_idx = \
    (input & EOSS3_PAD_SEL_IDX_MASK) >> EOSS3_PAD_SEL_IDX_SHIFT;

  irqstate_t flags = enter_critical_section();

  /* Check select index, if it is 0 we are not working with an input */

  if (sel_idx != 0)
    {
      putreg32(sel, EOSS3_PAD_SEL(sel_idx));
    }

  /* Check if we are enabling GPIO Register access */

  if (cfgset & GPIO_REG_EN_MASK)
    {
      uint8_t iobit = (cfgset & GPIO_REG_BIT_MASK) >> GPIO_REG_BIT_SHIFT;
      if (pad >= IO_REG_START_HI_PAD)
        {
          putreg32(
            getreg32(EOSS3_IO_REG_SEL) | (1 << iobit),
            EOSS3_IO_REG_SEL);
        }
      else
        {
          putreg32(
            getreg32(EOSS3_IO_REG_SEL) & ~(1 << iobit),
            EOSS3_IO_REG_SEL);
        }
    }

  putreg32(ctrl, EOSS3_PAD_X_CTRL(pad));
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: eoss3_unconfiggpio
 *
 * Description:
 *   unConfigure a PIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int eoss3_unconfiggpio(gpio_pinset_t cfgset)
{
  return OK;
}

/****************************************************************************
 * Name: eoss3_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected PIO pin
 *
 ****************************************************************************/

void eoss3_gpiowrite(gpio_pinset_t cfgset, bool value)
{
  uint8_t iobit = (cfgset & GPIO_REG_BIT_MASK) >> GPIO_REG_BIT_SHIFT;
  if (cfgset & GPIO_REG_EN_MASK)
    {
      irqstate_t flags = enter_critical_section();
      if (value)
        {
          putreg32(
            getreg32(EOSS3_MISC_IO_OUTPUT) | (1 << iobit),
            EOSS3_MISC_IO_OUTPUT);
        }
      else
        {
          putreg32(
            getreg32(EOSS3_MISC_IO_OUTPUT) & ~(1 << iobit),
            EOSS3_MISC_IO_OUTPUT);
        }

      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: eoss3_gpioread
 *
 * Description:
 *   Read one or zero from the selected PIO pin
 *
 ****************************************************************************/

bool eoss3_gpioread(gpio_pinset_t cfgset)
{
  uint8_t iobit = (cfgset & GPIO_REG_BIT_MASK) >> GPIO_REG_BIT_SHIFT;
  return ((getreg32(EOSS3_MISC_IO_INPUT) & (1 << iobit)) != 0);
}

/****************************************************************************
 * Function:  eoss3_dumpgpio
 *
 * Description:
 *   Dump all PIO registers associated with the base address of the provided
 *   pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int eoss3_dumpgpio(uint32_t pinset, const char *msg)
{
#warning Missing logic
  return -ENOSYS;
}
#endif
