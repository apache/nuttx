/****************************************************************************
 * arch/arm/src/phy62xx/phyplus_gpio.h
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

#ifndef __ARCH_ARM_SRC_PHY62XX_PHYPLUS_GPIO_H
#define __ARCH_ARM_SRC_PHY62XX_PHYPLUS_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/ioexpander/gpio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PHYPLUS_GPIO_INPUT     0
#define PHYPLUS_GPIO_OUTPUT    1
#define PHYPLUS_GPIO_INTERRUPT 2
#define PIN_LOW                0
#define PIN_HIGH               1

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct phyplus_gpio_param_s
{
    uint8_t pin_idx;       /* range to 23 */
    uint8_t mode;          /* input,output,interrupt */
    uint8_t trig_mode;     /* raise, fall */
    uint8_t default_val;   /* high, low */
    uint8_t pin_pull;      /* gpio_pupd_e */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int phyplus_gpio_register(struct phyplus_gpio_param_s
                *phyplus_gpio_param);
int phyplus_gpio_unregister(struct phyplus_gpio_param_s
                *phyplus_gpio_param);
#endif /* __ARCH_ARM_SRC_PHY62XX_PHYPLUS_GPIO_H */
