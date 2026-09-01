/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_iomuxc.c
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

#include <errno.h>

#include "arm_internal.h"
#include "imxrt118x_iomuxc.h"
#include "imxrt_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt118x_iomux_configure
 *
 * Description:
 *   Program a single pad on either IOMUXC instance: writes the MUX_MODE
 *   (and optional daisy) register and, if a pad control register is
 *   provided, the corresponding PAD control value.
 *
 ****************************************************************************/

void imxrt118x_iomux_configure(const struct imxrt118x_iomux_s *cfg,
                               uint32_t padctl)
{
  putreg32(IOMUX_MUX_MODE_ALT(cfg->alt), cfg->base + cfg->mux_off);

  if (cfg->dsy_off != 0)
    {
      putreg32(cfg->dsy, cfg->base + cfg->dsy_off);
    }

  if (cfg->pad_off != 0)
    {
      putreg32(padctl, cfg->base + cfg->pad_off);
    }
}

/****************************************************************************
 * Name: imxrt_config_gpio
 *
 * Description:
 *   Configure an RT118x pin using the board's pad table.  The pinset is a
 *   32-bit value produced by GPIO_RT118X_INDEX(n), where 'n' indexes the
 *   board's g_imxrt118x_padtable[] and g_imxrt118x_padctltable[].
 *
 ****************************************************************************/

int imxrt_config_gpio(gpio_pinset_t pinset)
{
  unsigned int idx = pinset & GPIO_RT118X_INDEX_MASK;

  if ((pinset & GPIO_RT118X_PERIPH) == 0 || idx >= g_imxrt118x_padtable_size)
    {
      return -EINVAL;
    }

  imxrt118x_iomux_configure(&g_imxrt118x_padtable[idx],
                            g_imxrt118x_padctltable[idx]);
  return OK;
}
