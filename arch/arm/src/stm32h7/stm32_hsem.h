/****************************************************************************
 * arch/arm/src/stm32h7/stm32_hsem.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_HSEM_H
#define __ARCH_ARM_SRC_STM32H7_STM32_HSEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/stm32_hsem.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*hsem_callback_t)(uint8_t id, void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

void stm32_hsem_subscribe(uint8_t id, hsem_callback_t callback, void *args);
void stm32_hsem_signal(uint8_t id);
void stm32_hsem_busywait_lock(uint8_t id);
void stm32_hsem_busywait_free(uint8_t id);
void stm32_hsem_wait_take(uint8_t id);
bool stm32_hsem_take(uint8_t id);
void stm32_hsem_free(uint8_t id);
void stm32_hsem_init(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32H7_STM32_HSEM_H */
