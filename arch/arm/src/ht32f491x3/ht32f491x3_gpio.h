/****************************************************************************
 * arch/arm/src/ht32f491x3/ht32f491x3_gpio.h
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

#ifndef __ARCH_ARM_SRC_HT32F491X3_HT32F491X3_GPIO_H
#define __ARCH_ARM_SRC_HT32F491X3_HT32F491X3_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HT32_GPIO_MODE_INPUT           0u
#define HT32_GPIO_MODE_OUTPUT          1u
#define HT32_GPIO_MODE_ALTFN           2u
#define HT32_GPIO_MODE_ANALOG          3u

#define HT32_GPIO_DRIVE_LOW            0u
#define HT32_GPIO_DRIVE_MEDIUM         1u
#define HT32_GPIO_DRIVE_HIGH           2u

#define HT32_GPIO_PULL_NONE            0u
#define HT32_GPIO_PULL_UP              1u
#define HT32_GPIO_PULL_DOWN            2u

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void ht32f491x3_gpioconfig(uintptr_t base, unsigned int pin,
                           unsigned int mode, bool opendrain,
                           unsigned int drive, unsigned int pull,
                           unsigned int af);
void ht32f491x3_gpiowrite(uintptr_t base, unsigned int pin, bool value);
bool ht32f491x3_gpioread(uintptr_t base, unsigned int pin);

#endif /* __ARCH_ARM_SRC_HT32F491X3_HT32F491X3_GPIO_H */
