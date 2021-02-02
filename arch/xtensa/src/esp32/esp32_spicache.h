/****************************************************************************
 * arch/xtensa/src/esp32/esp32_spicache.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_SPICACHE_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_SPICACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <nuttx/mtd/mtd.h>

#include "xtensa_attr.h"

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
 * Name: spi_disable_cache
 *
 * Description:
 *   Disable cache for SPI FLASH on a CPU
 *
 * Input Parameters:
 *   - cpu: ID of CPU to disable cache
 *   - state: pointer to cache reg state that will be returned
 *
 * Returned Value:
 *   None (the return will be over *state)
 *
 ****************************************************************************/

void IRAM_ATTR spi_disable_cache(int cpu, uint32_t *state);

/****************************************************************************
 * Name: spi_enable_cache
 *
 * Description:
 *   Enable cache for SPI FLASH on a CPU
 *
 * Input Parameters:
 *   - cpu: ID of CPU to enable cache
 *   - state: the cache reg state to be set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR spi_enable_cache(int cpu, uint32_t state);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_SPICACHE_H */
