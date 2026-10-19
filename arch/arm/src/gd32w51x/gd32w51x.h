/****************************************************************************
 * arch/arm/src/gd32w51x/gd32w51x.h
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

#ifndef __ARCH_ARM_SRC_GD32W51X_GD32W51X_H
#define __ARCH_ARM_SRC_GD32W51X_GD32W51X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: gd32w51x_boardinitialize
 *
 * Description:
 *   Board-specific initialization, provided by the board logic.
 *
 ****************************************************************************/

void gd32w51x_boardinitialize(void);

/****************************************************************************
 * Name: gd32w51x_tzinit
 *
 * Description:
 *   Release the GPIO pins and the SRAM blocks to the non-secure world, so
 *   that the secure-state core reaches every peripheral through its
 *   non-secure alias.  Called from __start() before anything else touches
 *   the hardware.
 *
 ****************************************************************************/

void gd32w51x_tzinit(void);

/****************************************************************************
 * Name: gd32w51x_gpio_setsecure
 *
 * Description:
 *   Mark one pin (a gd32_gpio_config() pin set) secure or non-secure.
 *
 ****************************************************************************/

void gd32w51x_gpio_setsecure(uint32_t pinset, bool secure);

/****************************************************************************
 * Name: gd32w51x_icache_enable
 *
 * Description:
 *   Enable the instruction cache in front of the SIP flash.
 *
 ****************************************************************************/

void gd32w51x_icache_enable(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32W51X_GD32W51X_H */
