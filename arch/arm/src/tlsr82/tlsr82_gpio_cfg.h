/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_gpio_cfg.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_TLSR82_GPIO_CFG_H
#define __ARCH_ARM_SRC_TLSR82_TLSR82_GPIO_CFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/tlsr82/chip.h>

#include "hardware/tlsr82_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_NUM              (4 * 8)

#define TLSR82_MUX_NONE       (0)
#define TLSR82_MUX_INPUT      (1 << 0)
#define TLSR82_MUX_OUTPUT     (1 << 1)
#define TLSR82_MUX_UART       (1 << 2)
#define TLSR82_MUX_PWM        (1 << 3)
#define TLSR82_MUX_DMIC       (1 << 4)
#define TLSR82_MUX_I2C        (1 << 5)
#define TLSR82_MUX_SPI        (1 << 6)
#define TLSR82_MUX_U7816      (1 << 7)
#define TLSR82_MUX_SDM        (1 << 8)
#define TLSR82_MUX_I2S        (1 << 9)
#define TLSR82_MUX_ATSEL      (1 << 10)
#define TLSR82_MUX_SWS        (1 << 11)
#define TLSR82_MUX_AMP        (1 << 12)
#define TLSR82_MUX_USB        (1 << 13)
#define TLSR82_MUX_ADC        (1 << 14)
#define TLSR82_MUX_LPC        (1 << 15)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct tlsr82_gpio_mux_s
{
  uint32_t muxs[4];
};

struct tlsr82_gpio_cfg_s
{
  uint32_t mux;
  uint32_t cfg;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_gpio_cfg_check
 *
 * Description:
 *   Check weather the mux function is supported or not by the input pin.
 *
 * Input Parameters:
 *   cfg - the pin config information, include the group and pin num
 *         information.
 *   mux - mux function.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int tlsr82_gpio_cfg_check(uint32_t cfg, uint32_t mux);

/****************************************************************************
 * Name: tlsr82_gpio_init
 *
 * Description:
 *   GPIO config initialization.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tlsr82_gpio_init(void);

#endif /* __ARCH_ARM_SRC_TLSR82_TLSR82_GPIO_CFG_H */
