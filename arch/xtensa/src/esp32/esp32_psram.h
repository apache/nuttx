/****************************************************************************
 * arch/xtensa/src/esp32/esp32_psram.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_PSRAM_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_PSRAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

typedef enum
{
  PSRAM_CACHE_F80M_S40M = 0,
  PSRAM_CACHE_F40M_S40M,
  PSRAM_CACHE_F80M_S80M,
  PSRAM_CACHE_MAX,
} psram_cache_mode_t;

typedef enum
{
  PSRAM_SIZE_16MBITS = 0,
  PSRAM_SIZE_32MBITS = 1,
  PSRAM_SIZE_64MBITS = 2,
  PSRAM_SIZE_MAX,
} psram_size_t;

/* See the TRM, chapter PID/MPU/MMU, header 'External RAM' for the
 * definitions of these modes. Important is that NORMAL works with the app
 * CPU cache disabled, but gives huge cache coherency issues when both app
 * and pro CPU are enabled. LOWHIGH and EVENODD do not have these coherency
 * issues but cannot be used when the app CPU cache is disabled.
 */

typedef enum
{
  PSRAM_VADDR_MODE_NORMAL = 0, /* App and Pro CPU use their own flash cache
                                * for external RAM access
                                */

  PSRAM_VADDR_MODE_LOWHIGH,    /* App and Pro CPU share external RAM caches:
                                  * pro CPU has low 2M, app CPU has high 2M
                                  */
  PSRAM_VADDR_MODE_EVENODD,    /* App and Pro CPU share external RAM caches:
                                * pro CPU does even 32yte ranges, app does
                                * odd ones.
                                */
} psram_vaddr_mode_t;

/* Description: Get PSRAM size
 * return:
 *   - PSRAM_SIZE_MAX if psram not enabled or not valid
 *   - PSRAM size
 */

psram_size_t psram_get_size(void);

/* Description: PSRAM cache enable function
 *
 * Esp-idf uses this to initialize cache for psram, mapping it into the main
 * memory address space.
 *
 * param:
 *   mode       SPI mode to access psram in
 *   vaddrmode  Mode the psram cache works in.
 * return:
 *   OK on success
 *   EINVAL when VSPI peripheral is needed but cannot be
 *          claimed.
 */

int psram_enable(psram_cache_mode_t mode, psram_vaddr_mode_t
                       vaddrmode);

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_HIMEM_H */
