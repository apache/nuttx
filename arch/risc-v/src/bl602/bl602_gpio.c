/****************************************************************************
 * arch/risc-v/src/bl602/bl602_gpio.c
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
#include "hardware/bl602_gpio.h"
#include "hardware/bl602_glb.h"
#include "riscv_arch.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_gpio_init
 *
 * Description:
 *   Init a gpio pin.
 *
 * Input Parameters:
 *   cfg: gpio configuration
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

void bl602_gpio_init(struct gpio_cfg_s *cfg)
{
  uint8_t   gpio_pin = cfg->gpio_pin;
  uint32_t *p_out;
  uint32_t  pos;
  uint32_t  tmp_out;
  uint32_t  tmp_val;

  p_out   = (uint32_t *)(GLB_BASE + GLB_GPIO_OUTPUT_EN_OFFSET +
                       ((gpio_pin >> 5) << 2));
  pos     = gpio_pin % 32;
  tmp_out = *p_out;

  /* Disable output anyway */

  tmp_out &= (~(1 << pos));
  *p_out = tmp_out;

  tmp_val = getreg32(GLB_BASE + GLB_GPIO_OFFSET + gpio_pin / 2 * 4);

  if (gpio_pin % 2 == 0)
    {
      /* Set input or output */

      if (cfg->gpio_mode == GPIO_MODE_OUTPUT)
        {
          tmp_val = tmp_val & GLB_REG_GPIO_0_IE_UMSK;
          tmp_out |= (1 << pos);
        }
      else
        {
          tmp_val = tmp_val | (1 << GLB_REG_GPIO_0_IE_POS);
        }

      /* Set pull up or down */

      tmp_val &= GLB_REG_GPIO_0_PU_UMSK;
      tmp_val &= GLB_REG_GPIO_0_PD_UMSK;
      if (cfg->pull_type == GPIO_PULL_UP)
        {
          tmp_val = tmp_val | (1 << GLB_REG_GPIO_0_PU_POS);
        }
      else if (cfg->pull_type == GPIO_PULL_DOWN)
        {
          tmp_val = tmp_val | (1 << GLB_REG_GPIO_0_PD_POS);
        }

      tmp_val = (tmp_val & GLB_REG_GPIO_0_DRV_UMSK) |
                (cfg->drive << GLB_REG_GPIO_0_DRV_POS);
      tmp_val = (tmp_val & GLB_REG_GPIO_0_SMT_UMSK) |
                (cfg->smt_ctrl << GLB_REG_GPIO_0_SMT_POS);
      tmp_val = (tmp_val & GLB_REG_GPIO_0_FUNC_SEL_UMSK) |
                (cfg->gpio_fun << GLB_REG_GPIO_0_FUNC_SEL_POS);
    }
  else
    {
      /* Set input or output */

      if (cfg->gpio_mode == GPIO_MODE_OUTPUT)
        {
          tmp_val &= GLB_REG_GPIO_1_IE_UMSK;
          tmp_out |= (1 << pos);
        }
      else
        {
          tmp_val = tmp_val | (1 << GLB_REG_GPIO_1_IE_POS);
        }

      /* Set pull up or down */

      tmp_val &= GLB_REG_GPIO_1_PU_UMSK;
      tmp_val &= GLB_REG_GPIO_1_PD_UMSK;
      if (cfg->pull_type == GPIO_PULL_UP)
        {
          tmp_val = tmp_val | (1 << GLB_REG_GPIO_1_PU_POS);
        }
      else if (cfg->pull_type == GPIO_PULL_DOWN)
        {
          tmp_val = tmp_val | (1 << GLB_REG_GPIO_1_PD_POS);
        }

      tmp_val = (tmp_val & GLB_REG_GPIO_1_DRV_UMSK) |
                (cfg->drive << GLB_REG_GPIO_1_DRV_POS);
      tmp_val = (tmp_val & GLB_REG_GPIO_1_SMT_UMSK) |
                (cfg->smt_ctrl << GLB_REG_GPIO_1_SMT_POS);
      tmp_val = (tmp_val & GLB_REG_GPIO_1_FUNC_SEL_UMSK) |
                (cfg->gpio_fun << GLB_REG_GPIO_1_FUNC_SEL_POS);
    }

  putreg32(tmp_val, GLB_BASE + GLB_GPIO_OFFSET + gpio_pin / 2 * 4);

  *p_out = tmp_out;
}
