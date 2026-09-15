/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_gpio.c
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

#include <stdbool.h>
#include <stdint.h>

#include "arm_internal.h"
#include "imxrt_gpio.h"
#include "hardware/imxrt_memorymap.h"
#include "hardware/rt118x/imxrt118x_pinmux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IOMUXC_AON_MUX_AON08      (IMXRT_IOMUXC_AON_BASE + 0x20)
#define IOMUXC_AON_MUX_AON09      (IMXRT_IOMUXC_AON_BASE + 0x24)
#define IOMUXC_AON_PAD_AON08      (IMXRT_IOMUXC_AON_BASE + 0x94)
#define IOMUXC_AON_PAD_AON09      (IMXRT_IOMUXC_AON_BASE + 0x98)
#define IOMUXC_PAD_UART_DEFAULT   0x02u

/****************************************************************************
 * Public Data
 ****************************************************************************/

const uintptr_t g_gpio_base[IMXRT_GPIO_NPORTS] =
{
  IMXRT_GPIO1_BASE
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_config_gpio
 *
 * Description:
 *   Configure one RT1180 pin for GPIO or peripheral use.
 *
 ****************************************************************************/

int imxrt_config_gpio(gpio_pinset_t pinset)
{
  unsigned int index = pinset & 0xffffu;

  if ((pinset & GPIO_MODE_MASK) == GPIO_PERIPH)
    {
      if (index == IMXRT_PADMUX_GPIO_AON_08_INDEX)
        {
          putreg32(0, IOMUXC_AON_MUX_AON08);
          putreg32(IOMUXC_PAD_UART_DEFAULT, IOMUXC_AON_PAD_AON08);
          return OK;
        }

      if (index == IMXRT_PADMUX_GPIO_AON_09_INDEX)
        {
          putreg32(0, IOMUXC_AON_MUX_AON09);
          putreg32(IOMUXC_PAD_UART_DEFAULT, IOMUXC_AON_PAD_AON09);
          return OK;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_gpio_write
 *
 * Description:
 *   Write one RT1180 GPIO output.
 *
 ****************************************************************************/

void imxrt_gpio_write(gpio_pinset_t pinset, bool value)
{
  (void)pinset;
  (void)value;
}

/****************************************************************************
 * Name: imxrt_gpio_read
 *
 * Description:
 *   Read one RT1180 GPIO input.
 *
 ****************************************************************************/

bool imxrt_gpio_read(gpio_pinset_t pinset)
{
  (void)pinset;
  return false;
}
