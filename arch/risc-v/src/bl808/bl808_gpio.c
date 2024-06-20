/****************************************************************************
 * arch/risc-v/src/bl808/bl808_gpio.c
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

#include <debug.h>
#include <sys/param.h>

#include "riscv_internal.h"
#include "hardware/bl808_glb.h"
#include "bl808_gpio.h"

////TODO
////#define BL808_GLB_BASE        0x20000000ul  /* glb */
#define BL808_GPIO_BASE 0x200008c4ul /* gpio */
#define BL808_NGPIOS 45
#define reg_gpio_xx_o 24
#define reg_gpio_xx_i 28

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl808_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin attributes.
 *
 ****************************************************************************/

int bl808_configgpio(int pin, gpio_pinattr_t attr)
{
  uintptr_t regaddr;
  uint32_t cfg = 0;

  DEBUGASSERT(pin >= 0 && pin <= BL808_NGPIOS);

  //// TODO: Change GPIO_CFGCTL0_GPIO_0_IE to GPIO_CFG_GPIO_IE
  if (attr & GPIO_INPUT)
    {
      cfg |= GPIO_CFGCTL0_GPIO_0_IE;
    }
  else
    {
      cfg |= GPIO_CFGCTL0_GPIO_0_OE;
    }

  if (attr & GPIO_PULLUP)
    {
      cfg |= GPIO_CFGCTL0_GPIO_0_PU;
    }

  if (attr & GPIO_PULLDOWN)
    {
      cfg |= GPIO_CFGCTL0_GPIO_0_PD;
    }

  if (attr & GPIO_DRV_MASK)
    {
      cfg |= ((attr & GPIO_DRV_MASK) >> GPIO_DRV_SHIFT) <<
        GPIO_CFGCTL0_GPIO_0_DRV_SHIFT;
    }

  if (attr & GPIO_SMT_EN)
    {
      cfg |= GPIO_CFGCTL0_GPIO_0_SMT;
    }

  if (attr & GPIO_FUNC_MASK)
    {
      cfg |= ((attr & GPIO_FUNC_MASK) >> GPIO_FUNC_SHIFT) <<
        GPIO_CFGCTL0_GPIO_0_FUNC_SEL_SHIFT;
    }

  regaddr = BL808_GPIO_BASE + (pin * 4);
  // _info("regaddr=%p, cfg=0x%x\n", regaddr, cfg);////
  putreg32(cfg, regaddr);
  return OK;
}

/****************************************************************************
 * Name: bl808_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void bl808_gpiowrite(int pin, bool value)
{
  uintptr_t regaddr;

  DEBUGASSERT(pin >= 0 && pin <= BL808_NGPIOS);

  regaddr = BL808_GPIO_BASE + (pin * 4);
  if (value)
    {
      up_putc('\n'); _info("regaddr=%p, set=0x%x\n", regaddr, (1 << reg_gpio_xx_o));////
      modifyreg32(regaddr, 0, (1 << reg_gpio_xx_o));
    }
  else
    {
      up_putc('\n'); _info("regaddr=%p, clear=0x%x\n", regaddr, (1 << reg_gpio_xx_o));////
      modifyreg32(regaddr, (1 << reg_gpio_xx_o), 0);
    }
}

/****************************************************************************
 * Name: bl808_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool bl808_gpioread(int pin)
{
  uintptr_t regaddr;
  uint32_t regval;

  DEBUGASSERT(pin >= 0 && pin <= BL808_NGPIOS);

  regaddr = BL808_GPIO_BASE + (pin * 4);
  regval = getreg32(regaddr);
  return (regval & (1 << reg_gpio_xx_i)) != 0;
}