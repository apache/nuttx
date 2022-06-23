/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_gpio.c
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

#include <stdint.h>

#include "riscv_internal.h"
#include "hardware/mpfs_gpio.h"
#include "mpfs_gpio.h"
#include <arch/barriers.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpio_callback_s
{
  xcpt_t callback;
  void  *arg;
  gpio_pinset_t cfgset;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uintptr_t g_gpio_base[] =
{
  MPFS_GPIO0_LO_BASE,  /* Bank-0 Normal GPIO */
  MPFS_GPIO1_LO_BASE,  /* Bank-1 Normal GPIO */
  MPFS_GPIO2_LO_BASE   /* Bank-2 Fabric only */
};

/* The GPIO interrupts are multiplexed. Total GPIO interrupts are 41.
 * 41 = (14 from GPIO0 + 24 from GPIO1 + 3 non direct interrupts)
 * GPIO2 interrupts are not available by default.
 * Setting the corresponding bitcin GPIO_INTERRUPT_FAB_CR(31:0) will enable
 * GPIO2(31:0) corresponding interrupt on PLIC.
 *
 * PLIC |      GPIO_INTERRUPT_FAB_CR
 *      |             0               1
 * -----------------------------------
 * 0    |   GPIO0 bit 0     GPIO2 bit 0
 * 1    |   GPIO0 bit 1     GPIO2 bit 1
 * .    |
 * .    |
 * 12   |   GPIO0 bit 12    GPIO2 bit 12
 * 13   |   GPIO0 bit 13    GPIO2 bit 13
 * 14   |   GPIO1 bit 0     GPIO2 bit 14
 * 15   |   GPIO1 bit 1     GPIO2 bit 15
 * .    |
 * .    |
 * .    |
 * 30   |   GPIO1 bit 16    GPIO2 bit 30
 * 31   |   GPIO1 bit 17    GPIO2 bit 31
 * 32   |       GPIO1 bit 18
 * 33   |       GPIO1 bit 19
 * 34   |       GPIO1 bit 20
 * 35   |       GPIO1 bit 21
 * 36   |       GPIO1 bit 22
 * 37   |       GPIO1 bit 23
 * 38  Or of all GPIO0 interrupts who do not have a direct connection enabled
 * 39  Or of all GPIO1 interrupts who do not have a direct connection enabled
 * 40  Or of all GPIO2 interrupts who do not have a direct connection enabled
 */

/* IRQ handlers for GPIO2 (FABRIC) */

static struct gpio_callback_s g_fab_gpio_callbacks[GPIO_BANK2_NUM_PINS];

/* IRQ handlers for GPIO0 and GPIO1 (MSSIO) */

static struct gpio_callback_s g_mss_gpio_callbacks[GPIO_BANK0_NUM_PINS +
                                                   GPIO_BANK1_NUM_PINS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mpfs_gpio_irq_clear(int bank, int pin)
{
  putreg32(1 << pin, g_gpio_base[bank] + MPFS_GPIO_INTR_OFFSET);
  __DMB();
}

/****************************************************************************
 * Interrupt Service Routines - Dispatchers
 ****************************************************************************/

static int mpfs_gpio_isr(int irq, void *context, void *arg)
{
  uint8_t irq_n = irq - MPFS_IRQ_GPIO02_BIT0;
  int ret = OK;

  uint8_t mss_bank = irq_n < GPIO_BANK1_NUM_PINS ? 0 : 1;
  uint8_t pin = mss_bank == 1 ? irq_n - GPIO_BANK0_NUM_PINS : irq_n;
  uint32_t event_reg;
  struct gpio_callback_s *cb;

  /* if mss gpio interrupt is enabled and there is an event */

  /* bank 0 and bank 1 pins */

  cb = &g_mss_gpio_callbacks[irq_n];
  event_reg = getreg32(g_gpio_base[mss_bank] + MPFS_GPIO_INTR_OFFSET);

  /* if the callback is registered and there is an interrupt on the mss_pin */

  if (cb->callback != NULL && (event_reg & (1 << pin)) != 0)
    {
      /* clear the pending interrupt */

      mpfs_gpio_irq_clear(mss_bank, pin);

      /* dispatch the interrupt to the handler */

      ret = cb->callback(irq, context, cb->arg);
    }

  /* handle the muxed gpio bank2 interrupt */

  if (irq_n < GPIO_BANK2_NUM_PINS)
    {
      cb = &g_fab_gpio_callbacks[irq_n];
      event_reg = getreg32(g_gpio_base[2] + MPFS_GPIO_INTR_OFFSET);
      if (cb->callback != NULL && (event_reg & (1 << pin)) != 0)
        {
          /* clear the pending interrupt */

          mpfs_gpio_irq_clear(2, pin);

          /* dispatch the interrupt to the handler */

          ret = cb->callback(irq, context, cb->arg);
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

int mpfs_configgpio(gpio_pinset_t cfgset)
{
  uintptr_t baseaddr;
  uint32_t cfg = 0;
  uint8_t pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint8_t bank = (cfgset & GPIO_BANK_MASK) >> GPIO_BANK_SHIFT;
  uint8_t irq_mode = (cfgset & GPIO_IRQ_MASK) >> GPIO_IRQ_SHIFT;
  uint8_t mux = (cfgset & GPIO_AF_MASK) >> GPIO_AF_SHIFT;
  uint16_t ec = (cfgset & GPIO_EC_MASK) >> GPIO_EC_SHIFT;

  if (bank == 3)
    {
      return ERROR;
    }

  /* REVISIT: limit the gpios as
   * bank0  0 - 13
   * bank1  0 - 23
   * bank2  0 - 31
   */

  if (bank == 0 || bank == 1)
    {
      /* Mux the relevant GPIO to IO PAD */

      baseaddr = MPFS_SYSREG_BASE + MSSIO_MUX_BANK_REG_OFFSET(bank, pin);
      modifyreg32(baseaddr, MSSIO_MUX_MASK(pin),
                  mux << MSSIO_MUX_SHIFT(pin));

      /* Set EC configuration for MSSIO pin */

      baseaddr = MSSIO_IO_CFG_CR(bank, pin);
      modifyreg32(baseaddr, MSSIO_IO_CFG_CR_MASK(pin),
                  ec << MSSIO_IO_CFG_CR_SHIFT(pin));
    }
  else
    {
      /* TODO: Always enable to fabric */
    }

  baseaddr = g_gpio_base[bank] + (pin * sizeof(uint32_t));

  if (cfgset & GPIO_INPUT)
    {
      cfg |= GPIO_CONFIG_EN_IN;
    }

  if (cfgset & GPIO_OUTPUT)
    {
      cfg |= GPIO_CONFIG_EN_OUT;
    }

  if (cfgset & GPIO_BUFFER_ENABLE)
    {
      cfg |= GPIO_CONFIG_EN_OE_BUF;
    }

  /* set irq mode bits */

  irq_mode &= GPIO_CONFIG_INT_MASK;
  cfg |= irq_mode << GPIO_CONFIG_INT_SHIFT;

  /* set/clear irq enable */

  if (cfgset & GPIO_IRQ_ENABLE)
    {
      /* clear irq before enabling */

      mpfs_gpio_irq_clear(bank, pin);
      cfg |= GPIO_CONFIG_EN_INT;
    }
  else
    {
      cfg &= ~GPIO_CONFIG_EN_INT;
    }

  putreg32(cfg, baseaddr);

  return OK;
}

/****************************************************************************
 * Name: mpfs_gpio_deinit
 *
 * Description:
 *   Deinit a GPIO (Set GPIO to input state)
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

/* REVISIT: */

int mpfs_gpio_deinit(uint8_t pin)
{
  mpfs_configgpio(pin);
  return OK;
}

/****************************************************************************
 * Name: mpfs_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void mpfs_gpiowrite(gpio_pinset_t pinset, bool value)
{
  uint8_t pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint8_t bank = (pinset & GPIO_BANK_MASK) >> GPIO_BANK_SHIFT;
  uintptr_t baseaddr = g_gpio_base[bank];

  if (bank == 3)
    {
      return;
    }

  if (value)
    {
      putreg32((1 << pin), baseaddr + MPFS_GPIO_SET_BITS_OFFSET);
    }
  else
    {
      putreg32((1 << pin), baseaddr + MPFS_GPIO_CLEAR_BITS_OFFSET);
    }
}

/****************************************************************************
 * Name: mpfs_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool mpfs_gpioread(gpio_pinset_t pinset)
{
  uint8_t pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint8_t bank = (pinset & GPIO_BANK_MASK) >> GPIO_BANK_SHIFT;
  uintptr_t baseaddr = g_gpio_base[bank];

  if (bank == 3)
    {
      return 0;
    }

  return ((getreg32(baseaddr + MPFS_GPIO_GPIN_OFFSET) & (1 << pin)) ? 1 : 0);
}

/****************************************************************************
 * Name: mpfs_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 ****************************************************************************/

int mpfs_gpiosetevent(gpio_pinset_t pinset, bool risingedge,
                      bool fallingedge, bool high, bool low, bool event,
                      xcpt_t func, void *arg)
{
  uint8_t pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint8_t bank = (pinset & GPIO_BANK_MASK) >> GPIO_BANK_SHIFT;
  bool edge = risingedge || fallingedge;
  bool level = high || low;

  uint16_t irq_n = bank == 1 ? pin + GPIO_BANK0_NUM_PINS : pin;
  uint16_t plic_irq;
  struct gpio_callback_s *cb;
  int ret;

  if (bank > 2        ||   /* invalid bank */
      (edge && level) ||   /* edge and level irq at the same */
      (high && low)   ||   /* level high and low at the same */
      irq_n >= GPIO_BANK0_NUM_PINS + GPIO_BANK1_NUM_PINS ||
      (bank == 2 && irq_n >= GPIO_BANK2_NUM_PINS))
    {
      return -EINVAL;
    }

  /* if func is NULL, make sure to disable interrupt */

  if (func == NULL)
    {
      risingedge = false;
      fallingedge = false;
      high = false;
      low = false;
    }

  cb = bank == 2 ? &g_fab_gpio_callbacks[irq_n]
                 : &g_mss_gpio_callbacks[irq_n];

  /* make sure the GPIO is input */

  pinset &= ~GPIO_MODE_MASK;
  pinset |= GPIO_INPUT;

  /* set the irq mask for the gpio */

  pinset &= ~GPIO_IRQ_MASK;

  if (edge)
    {
      if (risingedge && fallingedge)
        {
          pinset |= GPIO_IRQ_EDGE_BOTH;
        }
      else if (fallingedge)
        {
          pinset |= GPIO_IRQ_EDGE_NEG;
        }
      else if (risingedge)
        {
          pinset |= GPIO_IRQ_EDGE_POS;
        }
    }
  else
    {
      if (high)
        {
          pinset |= GPIO_IRQ_HIGH;
        }
      else if (low)
        {
          pinset |= GPIO_IRQ_LOW;
        }
    }

  /* if event, set ENABLE bit, otherwise leave it 0 */

  if (event)
    {
      pinset |= GPIO_IRQ_ENABLE;
    }

  if (mpfs_configgpio(pinset) != OK)
    {
      return -EINVAL;
    }

  /* set/reset the GPIO_INTERRUPT_FAB_CR for bank 2 */

  if (bank == 2)
    {
      if (event)
        {
          modifyreg32(MPFS_SYSREG_BASE +
                      MPFS_SYSREG_GPIO_INTERRUPT_FAB_CR_OFFSET,
                      0, SYSREG_GPIO_INTERRUPT_FAB_CR(pin));
        }
      else
        {
          modifyreg32(MPFS_SYSREG_BASE +
                      MPFS_SYSREG_GPIO_INTERRUPT_FAB_CR_OFFSET,
                      SYSREG_GPIO_INTERRUPT_FAB_CR(pin), 0);
        }
    }

  /* calculate the plic vector location from gpio irq number */

  plic_irq = irq_n + MPFS_IRQ_GPIO02_BIT0;

  /* attach the plic irq */

  if (event)
    {
      ret = irq_attach(plic_irq, mpfs_gpio_isr, NULL);
      if (ret == OK)
        {
          cb->callback = func;
          cb->arg = arg;

          up_enable_irq(plic_irq);
        }
    }
  else
    {
      ret = irq_detach(plic_irq);
      if (ret == OK)
        {
          cb->callback = NULL;
        }

      /* disable the irq even in case detach fails */

      up_disable_irq(plic_irq);
    }

  return ret;
}
