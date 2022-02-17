/****************************************************************************
 * arch/arm/src/rp2040/rp2040_gpio.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "arm_arch.h"

#include "chip.h"
#include "rp2040_gpio.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* GPIO interrupt initialize flag */

static bool g_gpio_irq_init = false;

/* GPIO interrupt handlers information */

static xcpt_t g_gpio_irq_handlers[RP2040_GPIO_NUM];
static FAR void *g_gpio_irq_args[RP2040_GPIO_NUM];
static int g_gpio_irq_modes[RP2040_GPIO_NUM];

/* GPIO pins function assignment */

static int g_gpio_function[RP2040_GPIO_NUM];

/* GPIO pins function mapping table */

static const int g_gpio_function_mapping_spi[2][5] =
{
  {  0,  4, 16, 20, -1 },     /* pin numbers assignable to SPI0 */
  {  8, 12, 24, 28, -1 },     /* pin numbers assignable to SPI1 */
};

static const int g_gpio_function_mapping_uart[2][5] =
{
  {  0, 12, 16, 28, -1 },     /* pin numbers assignable to UART0 */
  {  4,  8, 20, 24, -1 },     /* pin numbers assignable to UART1 */
};

static const int g_gpio_function_mapping_i2c[2][9] =
{
  {  0,  4,  8, 12, 16, 20, 24, 28, -1 }, /* pin numbers assignable to I2C0 */
  {  2,  6, 10, 14, 18, 22, 26, -1, -1 }, /* pin numbers assignable to I2C1 */
};

static const int g_gpio_function_mapping_pwm[8][3] =
{
  {  0, 16, -1 },             /* pin numbers assignable to PWM0 */
  {  2, 18, -1 },             /* pin numbers assignable to PWM1 */
  {  4, 20, -1 },             /* pin numbers assignable to PWM2 */
  {  6, 22, -1 },             /* pin numbers assignable to PWM3 */
  {  8, 24, -1 },             /* pin numbers assignable to PWM4 */
  { 10, 26, -1 },             /* pin numbers assignable to PWM5 */
  { 12, 28, -1 },             /* pin numbers assignable to PWM6 */
  { 14, -1, -1 },             /* pin numbers assignable to PWM7 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_gpio_interrupt
 *
 * Description:
 *  GPIO interrupt handler
 *
 ****************************************************************************/

static int rp2040_gpio_interrupt(int irq, void *context, FAR void *arg)
{
  int i;
  int j;
  uint32_t stat;
  uint32_t gpio;
  xcpt_t handler;

  /* Scan all GPIO interrupt status registers */

  for (i = 0; i < 4; i++)
    {
      /* Get and clear pending GPIO interrupt status */

      stat = getreg32(RP2040_IO_BANK0_PROC_INTS(i * 8, 0));
      if (i == 3)
        {
          stat &= 0x00ffffff;     /* Clear reserved bits */
        }

      putreg32(stat, RP2040_IO_BANK0_INTR(i * 8));

      while (stat != 0)
        {
          /* Scan all GPIO pins in one register */

          for (j = 0; j < 8; j++)
            {
              if (stat & (0xf << (j * 4)))
                {
                  stat &= ~(0xf << (j * 4));

                  gpio = i * 8 + j;
                  handler = g_gpio_irq_handlers[gpio];

                  /* Call GPIO interrupt handler */

                  if (handler)
                    {
                      handler(gpio, context, g_gpio_irq_args[gpio]);
                    }
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: r2040_gpio_get_function_pin
 *
 * Description:
 *   Get the GPIO pin number to which the specified function is assigned
 *
 ****************************************************************************/

int rp2040_gpio_get_function_pin(uint32_t func, uint32_t port)
{
  int i;
  const int *mapping;

  /* Get GPIO pins function mapping table */

  switch (func)
    {
      case RP2040_GPIO_FUNC_SPI:
        if (port >= 2)
          {
            return -1;
          }

        mapping = g_gpio_function_mapping_spi[port];
        break;

      case RP2040_GPIO_FUNC_UART:
        if (port >= 2)
          {
            return -1;
          }

        mapping = g_gpio_function_mapping_uart[port];
        break;

      case RP2040_GPIO_FUNC_I2C:
        if (port >= 2)
          {
            return -1;
          }

        mapping = g_gpio_function_mapping_i2c[port];
        break;

      case RP2040_GPIO_FUNC_PWM:
        if (port >= 8)
          {
            return -1;
          }

        mapping = g_gpio_function_mapping_pwm[port];
        break;

      default:
        return -1;
    }

  /* Find the specified function in the current assignment */

  for (i = 0; mapping[i] >= 0; i++)
    {
      if (g_gpio_function[mapping[i]] == func)
        {
          return mapping[i];
        }
    }

  return -1;
}

/****************************************************************************
 * Name: r2040_gpio_set_function
 *
 * Description:
 *   Assign functions to the specified GPIO pin
 *
 ****************************************************************************/

void rp2040_gpio_set_function(uint32_t gpio, uint32_t func)
{
  DEBUGASSERT(gpio < RP2040_GPIO_NUM);

  modbits_reg32(RP2040_PADS_BANK0_GPIO_IE,
                RP2040_PADS_BANK0_GPIO_IE | RP2040_PADS_BANK0_GPIO_OD,
                RP2040_PADS_BANK0_GPIO(gpio));

  putreg32(func & RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_MASK,
           RP2040_IO_BANK0_GPIO_CTRL(gpio));

  g_gpio_function[gpio] = func;
}

/****************************************************************************
 * Name: r2040_gpio_set_pulls
 *
 * Description:
 *   Set pull-up or pull-down to the specified GPIO pin
 *
 ****************************************************************************/

void rp2040_gpio_set_pulls(uint32_t gpio, int up, int down)
{
  DEBUGASSERT(gpio < RP2040_GPIO_NUM);

  modbits_reg32((up   ? RP2040_PADS_BANK0_GPIO_PUE : 0) |
                (down ? RP2040_PADS_BANK0_GPIO_PDE : 0),
                RP2040_PADS_BANK0_GPIO_PUE | RP2040_PADS_BANK0_GPIO_PDE,
                RP2040_PADS_BANK0_GPIO(gpio));
}

/****************************************************************************
 * Name: r2040_gpio_init
 *
 * Description:
 *   Initialize software-controlled GPIO function
 *
 ****************************************************************************/

void rp2040_gpio_init(uint32_t gpio)
{
  DEBUGASSERT(gpio < RP2040_GPIO_NUM);

  rp2040_gpio_setdir(gpio, false);
  rp2040_gpio_put(gpio, false);
  rp2040_gpio_set_function(gpio, RP2040_GPIO_FUNC_SIO);
}

/****************************************************************************
 * Name: r2040_gpio_irq_attach
 *
 * Description:
 *   Configure the interrupt generated by the specified GPIO pin.
 *
 ****************************************************************************/

int rp2040_gpio_irq_attach(uint32_t gpio, uint32_t intrmode,
                           xcpt_t isr, FAR void *arg)
{
  if (!g_gpio_irq_init)
    {
      /* Initialize - register GPIO interrupt handler */

      g_gpio_irq_init = true;
      irq_attach(RP2040_IO_IRQ_BANK0, rp2040_gpio_interrupt, NULL);
      up_enable_irq(RP2040_IO_IRQ_BANK0);
    }

  DEBUGASSERT(gpio < RP2040_GPIO_NUM);
  DEBUGASSERT(intrmode <= RP2040_GPIO_INTR_EDGE_HIGH);

  /* Save handler information */

  g_gpio_irq_handlers[gpio] = isr;
  g_gpio_irq_args[gpio] = arg;
  g_gpio_irq_modes[gpio] = intrmode;

  /* Clear pending interrupts */

  setbits_reg32(0xf << ((gpio % 8) * 4), RP2040_IO_BANK0_INTR(gpio));

  return OK;
}

/****************************************************************************
 * Name: rp2040_gpio_enable_irq
 *
 * Description:
 *   Enable the GPIO IRQ specified by 'gpio'
 *
 ****************************************************************************/

void rp2040_gpio_enable_irq(uint32_t gpio)
{
  uint32_t reg;

  DEBUGASSERT(gpio < RP2040_GPIO_NUM);

  if (g_gpio_irq_handlers[gpio] != NULL)
    {
      /* Set interrupt enable bit */

      reg = RP2040_IO_BANK0_PROC_INTE(gpio, 0);
      clrbits_reg32(0xf << ((gpio % 8) * 4), reg);
      setbits_reg32(0x1 << ((gpio % 8) * 4 + g_gpio_irq_modes[gpio]), reg);
    }
}

/****************************************************************************
 * Name: rp2040_gpio_disable_irq
 *
 * Description:
 *   Disable the GPIO IRQ specified by 'gpio'
 *
 ****************************************************************************/

void rp2040_gpio_disable_irq(uint32_t gpio)
{
  uint32_t reg;

  DEBUGASSERT(gpio < RP2040_GPIO_NUM);

  if (g_gpio_irq_handlers[gpio] != NULL)
    {
      /* Clear interrupt enable bit */

      reg = RP2040_IO_BANK0_PROC_INTE(gpio, 0);
      clrbits_reg32(0xf << ((gpio % 8) * 4), reg);
    }
}

/****************************************************************************
 * Name: r2040_gpio_initialize
 *
 * Description:
 *   Initialize GPIO function management
 *
 ****************************************************************************/

void rp2040_gpio_initialize(void)
{
  int i;

  for (i = 0; i < RP2040_GPIO_NUM; i++)
    {
      g_gpio_function[i] = RP2040_GPIO_FUNC_NULL;
    }
}
