/****************************************************************************
 * boards/risc-v/ab32vg1/ab32vg1-generic/src/ab32vg1-generic.h
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

#ifndef __BOARDS_RISCV_AB32VG1_AB32VG1_GENERIC_SRC_AB32VG1_GENERIC_H
#define __BOARDS_RISCV_AB32VG1_AB32VG1_GENERIC_SRC_AB32VG1_GENERIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ab32vg1_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RGB LED, active low */

#define GPIO_LED_RED    (GPIO_PORTE | GPIO_PIN(1) | GPIO_OUTPUT | GPIO_VALUE_ONE)
#define GPIO_LED_GREEN  (GPIO_PORTE | GPIO_PIN(4) | GPIO_OUTPUT | GPIO_VALUE_ONE)
#define GPIO_LED_BLUE   (GPIO_PORTA | GPIO_PIN(1) | GPIO_OUTPUT | GPIO_VALUE_ONE)

/* User buttons, active low */

#define GPIO_KEY1       (GPIO_PORTF | GPIO_PIN(1) | GPIO_INPUT | GPIO_PULLUP)
#define GPIO_KEY2       (GPIO_PORTF | GPIO_PIN(0) | GPIO_INPUT | GPIO_PULLUP)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ab32vg1_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int ab32vg1_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISCV_AB32VG1_AB32VG1_GENERIC_SRC_AB32VG1_GENERIC_H */
