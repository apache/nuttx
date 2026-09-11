/****************************************************************************
 * arch/arm/src/am67/am67_gpio.c
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
#include <assert.h>
#include <stdint.h>

#include "am67_gpio.h"
#include "am67_pinmux.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_PINS_PER_REG_SHIFT  (5U)
#define GPIO_DIRECTION_OUTPUT    (0U)
#define GPIO_DIRECTION_INPUT     (1U)

#define MCU_GPIO0_BASE           0x4201000UL
#define MAIN_GPIO1_BASE          0x601000UL

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct am67_gpio_desc_s
{
  uint32_t base;
  uint32_t pin;
  int16_t  main_pad;
  int16_t  mcu_pad;
  uint32_t pad_setting;
  bool     active_low;
  bool     pad_only;
};

typedef struct
{
  volatile uint32_t DIR;
  volatile uint32_t OUT_DATA;
  volatile uint32_t SET_DATA;
  volatile uint32_t CLR_DATA;
  volatile uint32_t IN_DATA;
  volatile uint32_t SET_RIS_TRIG;
  volatile uint32_t CLR_RIS_TRIG;
  volatile uint32_t SET_FAL_TRIG;
  volatile uint32_t CLR_FAL_TRIG;
  volatile uint32_t INTSTAT;
} gpio_bank_regs_t;

typedef struct
{
  volatile uint32_t PID;
  volatile uint32_t PCR;
  volatile uint32_t BINTEN;
  volatile uint8_t  RSVD0[4];
  gpio_bank_regs_t  BANK_REGISTERS[9];
} gpio_regs_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct am67_gpio_desc_s g_am67_gpio_table[AM67_GPIO_ID_COUNT] =
{
  {
    MAIN_GPIO1_BASE, 52,
    PIN_OLDI0_A0P, -1,
    (PIN_MODE(7) | PIN_PULL_DISABLE),
    false, false
  },
  {
    MAIN_GPIO1_BASE, 53,
    PIN_OLDI0_A0N, -1,
    (PIN_MODE(7) | PIN_PULL_DISABLE),
    false, false
  },
  {
    0, 0,
    -1, PIN_MCU_SPI0_CS1,
    (PIN_MODE(0) | PIN_PULL_DISABLE),
    true, true
  },
  {
    0, 0,
    -1, PIN_MCU_MCAN0_TX,
    (PIN_MODE(2) | PIN_PULL_DISABLE),
    true, true
  },
  {
    MCU_GPIO0_BASE, 12,
    -1, PIN_WKUP_UART0_RTSN,
    (PIN_MODE(7) | PIN_PULL_DISABLE),
    true, false   /* active_low: LOW activates IMU per board design */
  },
  {
    0, 0,
    -1, PIN_WKUP_UART0_RXD,
    (PIN_MODE(2) | PIN_PULL_DISABLE),
    true, true   /* CS2: hardware pin, pad_only */
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t gpio_reg_index(uint32_t pin)
{
  return pin >> GPIO_PINS_PER_REG_SHIFT;
}

static inline uint32_t gpio_bit_pos(uint32_t pin)
{
  return pin - (gpio_reg_index(pin) << GPIO_PINS_PER_REG_SHIFT);
}

static inline uint32_t gpio_bit_mask(uint32_t pin)
{
  return ((uint32_t)1U) << gpio_bit_pos(pin);
}

static inline void gpio_set_dir(uint32_t base, uint32_t pin, uint32_t dir)
{
  gpio_regs_t *gpio = (gpio_regs_t *)(uintptr_t)base;
  uint32_t reg_index = gpio_reg_index(pin);
  uint32_t bit_pos = gpio_bit_pos(pin);

  gpio->BANK_REGISTERS[reg_index].DIR &= ~gpio_bit_mask(pin);
  gpio->BANK_REGISTERS[reg_index].DIR |= ((dir & 0x01u) << bit_pos);
}

static inline void gpio_write_high(uint32_t base, uint32_t pin)
{
  gpio_regs_t *gpio = (gpio_regs_t *)(uintptr_t)base;
  uint32_t reg_index = gpio_reg_index(pin);

  gpio->BANK_REGISTERS[reg_index].SET_DATA = gpio_bit_mask(pin);
}

static inline void gpio_write_low(uint32_t base, uint32_t pin)
{
  gpio_regs_t *gpio = (gpio_regs_t *)(uintptr_t)base;
  uint32_t reg_index = gpio_reg_index(pin);

  gpio->BANK_REGISTERS[reg_index].CLR_DATA = gpio_bit_mask(pin);
}

static inline bool gpio_read_level(uint32_t base, uint32_t pin)
{
  gpio_regs_t *gpio = (gpio_regs_t *)(uintptr_t)base;
  uint32_t reg_index = gpio_reg_index(pin);
  uint32_t reg_val = gpio->BANK_REGISTERS[reg_index].IN_DATA;

  return (reg_val & gpio_bit_mask(pin)) != 0;
}

static const struct am67_gpio_desc_s *
am67_gpio_desc(am67_gpio_t gpio)
{
  if (gpio >= AM67_GPIO_ID_COUNT)
    {
      return NULL;
    }

  return &g_am67_gpio_table[gpio];
}

static void am67_gpio_apply_padconfig(const struct am67_gpio_desc_s *desc)
{
  struct pinmux_conf_s conf[2];

  if (desc->main_pad >= 0)
    {
      conf[0].offset = desc->main_pad;
      conf[0].setting = desc->pad_setting;
      conf[1].offset = PINMUX_END;
      conf[1].setting = PINMUX_END;
      am67_pinmux_config(conf);
    }
  else if (desc->mcu_pad >= 0)
    {
      conf[0].offset = desc->mcu_pad;
      conf[0].setting = desc->pad_setting;
      conf[1].offset = PINMUX_END;
      conf[1].setting = PINMUX_END;
      am67_mcu_pinmux_config(conf);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

am67_gpio_t am67_gpio_hat(unsigned int hat_pin)
{
  (void)hat_pin;
  return AM67_GPIO_ID_COUNT;
}

void am67_configgpio(am67_gpio_t gpio, int pintype)
{
  const struct am67_gpio_desc_s *desc = am67_gpio_desc(gpio);

  DEBUGASSERT(desc != NULL);

  am67_gpio_apply_padconfig(desc);

  if (desc->pad_only || desc->base == 0)
    {
      return;
    }

  if (pintype == GPIO_INPUT)
    {
      gpio_set_dir(desc->base, desc->pin, GPIO_DIRECTION_INPUT);
    }
  else
    {
      gpio_set_dir(desc->base, desc->pin, GPIO_DIRECTION_OUTPUT);
    }
}

void am67_gpiowrite(am67_gpio_t gpio, bool value)
{
  const struct am67_gpio_desc_s *desc = am67_gpio_desc(gpio);
  bool level = value;

  DEBUGASSERT(desc != NULL);

  if (desc->pad_only || desc->base == 0)
    {
      return;
    }

  if (desc->active_low)
    {
      level = !level;
    }

  if (level)
    {
      gpio_write_high(desc->base, desc->pin);
    }
  else
    {
      gpio_write_low(desc->base, desc->pin);
    }
}

bool am67_gpioread(am67_gpio_t gpio)
{
  const struct am67_gpio_desc_s *desc = am67_gpio_desc(gpio);
  bool level;

  DEBUGASSERT(desc != NULL);

  if (desc->pad_only || desc->base == 0)
    {
      return false;
    }

  level = gpio_read_level(desc->base, desc->pin);

  if (desc->active_low)
    {
      level = !level;
    }

  return level;
}

void am67_sensors_power_enable(bool enable)
{
  am67_configgpio(AM67_GPIO_MCU0_PIN12, GPIO_OUTPUT);
  am67_gpiowrite(AM67_GPIO_MCU0_PIN12, enable);
}
