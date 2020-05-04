/****************************************************************************
 * arch/risc-v/src/fe310/fe310_gpio.c
 *
 *   Copyright (C) 2019 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "riscv_arch.h"
#include "fe310_gpio.h"
#include "fe310_memorymap.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  fe310_gpio_getpin()
 ****************************************************************************/

static uint32_t fe310_gpio_getpin(uint16_t gpiocfg)
{
  return ((gpiocfg & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

/****************************************************************************
 * Name:  fe310_gpio_configinput()
 ****************************************************************************/

static int fe310_gpio_configinput(uint16_t gpiocfg)
{
  uint32_t pin  = fe310_gpio_getpin(gpiocfg);

  /* Enable input & disable output */

  modifyreg32(FE310_GPIO_INPUT_EN, 0, 0x1 << pin);
  modifyreg32(FE310_GPIO_OUTPUT_EN, 0x1 << pin, 0);
  return 0;
}

/****************************************************************************
 * Name:  fe310_gpio_configirq()
 ****************************************************************************/

static int fe310_gpio_configirq(uint16_t gpiocfg)
{
  uint32_t pin  = fe310_gpio_getpin(gpiocfg);

  /* Enable input & disable output */

  modifyreg32(FE310_GPIO_INPUT_EN, 0, 0x1 << pin);
  modifyreg32(FE310_GPIO_OUTPUT_EN, 0x1 << pin, 0);

  /* Disable all gpio interrupts for the pin */

  modifyreg32(FE310_GPIO_RISE_IE, 0x1 << pin, 0x0);
  modifyreg32(FE310_GPIO_FALL_IE, 0x1 << pin, 0x0);
  modifyreg32(FE310_GPIO_HIGH_IE, 0x1 << pin, 0x0);
  modifyreg32(FE310_GPIO_LOW_IE,  0x1 << pin, 0x0);

  /* Then enable interrupt */

  switch (gpiocfg & GPIO_INT_MASK)
    {
      case GPIO_INT_RISE:
        modifyreg32(FE310_GPIO_RISE_IE, 0, 0x1 << pin);
        break;

      case GPIO_INT_FALL:
        modifyreg32(FE310_GPIO_FALL_IE, 0, 0x1 << pin);
        break;

      case GPIO_INT_BOTH:
        modifyreg32(FE310_GPIO_RISE_IE, 0, 0x1 << pin);
        modifyreg32(FE310_GPIO_FALL_IE, 0, 0x1 << pin);
        break;

      case GPIO_INT_HIGH:
        modifyreg32(FE310_GPIO_HIGH_IE, 0, 0x1 << pin);
        break;

      case GPIO_INT_LOW:
        modifyreg32(FE310_GPIO_LOW_IE, 0, 0x1 << pin);
        break;
    }

  return 0;
}

/****************************************************************************
 * Name:  fe310_gpio_configoutput()
 ****************************************************************************/

static int fe310_gpio_configoutput(uint16_t gpiocfg)
{
  uint32_t pin  = fe310_gpio_getpin(gpiocfg);

  /* TOD: set initial value */

  /* Disable input & enable output */

  modifyreg32(FE310_GPIO_INPUT_EN, 0x1 << pin, 0);
  modifyreg32(FE310_GPIO_OUTPUT_EN, 0, 0x1 << pin);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fe310_gpio_config
 ****************************************************************************/

int fe310_gpio_config(uint16_t gpiocfg)
{
  int ret = 0;
  irqstate_t flags;

  uint32_t pin  = fe310_gpio_getpin(gpiocfg);

  flags = spin_lock_irqsave();

  /* Disable IOF for the pin to be used as GPIO */

  modifyreg32(FE310_GPIO_IOF_EN, 0x1 << pin, 0);

  /* Pullup */

  if (gpiocfg & GPIO_PULLUP)
    {
      modifyreg32(FE310_GPIO_PU_EN, 0, 0x1 << pin);
    }
  else
    {
      modifyreg32(FE310_GPIO_PU_EN, 0x1 << pin, 0);
    }

  switch (gpiocfg & GPIO_MODE_MASK)
    {
      case GPIO_MODE_INPUT:

        ret = fe310_gpio_configinput(gpiocfg);
        break;

      case GPIO_MODE_OUTPUT:

        ret = fe310_gpio_configoutput(gpiocfg);
        break;

      case GPIO_MODE_INIRQ:

        ret = fe310_gpio_configirq(gpiocfg);
        break;
    }

  spin_unlock_irqrestore(flags);

  return ret;
}

/****************************************************************************
 * Name: fe310_gpio_write
 ****************************************************************************/

void fe310_gpio_write(uint16_t gpiocfg, bool value)
{
  uint32_t pin  = fe310_gpio_getpin(gpiocfg);

  if (value)
    {
      modifyreg32(FE310_GPIO_OUTPUT_VAL, 0, 0x1 << pin);
    }
  else
    {
      modifyreg32(FE310_GPIO_OUTPUT_VAL, 0x1 << pin, 0);
    }
}

/****************************************************************************
 * Name: fe310_gpio_read
 ****************************************************************************/

bool fe310_gpio_read(uint16_t gpiocfg)
{
  uint32_t pin  = fe310_gpio_getpin(gpiocfg);
  return (getreg32(FE310_GPIO_INPUT_VAL) & (0x1 << pin)) != 0;
}

/****************************************************************************
 * Name: fe310_gpio_clearpending
 ****************************************************************************/

void fe310_gpio_clearpending(uint32_t pin)
{
  ASSERT(0 <= pin && pin <= 31);

  /* Clear all gpio interrupts for the pin */

  modifyreg32(FE310_GPIO_RISE_IP, 0x0, 0x1 << pin);
  modifyreg32(FE310_GPIO_FALL_IP, 0x0, 0x1 << pin);
  modifyreg32(FE310_GPIO_HIGH_IP, 0x0, 0x1 << pin);
  modifyreg32(FE310_GPIO_LOW_IP,  0x0, 0x1 << pin);
}
