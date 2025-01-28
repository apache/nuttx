/****************************************************************************
 * arch/arm/src/stm32h5/stm32_icache.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_ICACHE_H
#define __ARCH_ARM_SRC_STM32H5_STM32_ICACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "chip.h"
#include "hardware/stm32_icache.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_get_icache_linesize
 *
 * Description:
 *   Returns the icache linesize.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   16
 *
 ****************************************************************************/

size_t stm32_get_icache_linesize(void);

/****************************************************************************
 * Name: stm32_get_icache_size
 *
 * Description:
 *   Returns the icache size.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   8192
 *
 ****************************************************************************/

size_t stm32_get_icache_size(void);

/****************************************************************************
 * Name: stm32_enable_icache
 *
 * Description:
 *   Initializes the STM32H5 ICACHE
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_enable_icache(void);

/****************************************************************************
 * Name: stm32_disable_icache
 *
 * Description:
 *   Disables the STM32H5 ICACHE.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_disable_icache(void);

/****************************************************************************
 * Name: stm32_reset_monitors
 *
 * Description:
 *   Reset the ICACHE Hit and Miss Counters
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_icache_reset_monitors(void);

/****************************************************************************
 * Name: stm32_invalidate_icache
 *
 * Description:
 *   Invalidate the icache and wait for its completion.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_invalidate_icache(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_ICACHE_H */
