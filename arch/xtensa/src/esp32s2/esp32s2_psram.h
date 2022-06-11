/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_psram.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_PSRAM_H
#define __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_PSRAM_H

#define PSRAM_SIZE_2MB                  (2 * 1024 * 1024)
#define PSRAM_SIZE_4MB                  (4 * 1024 * 1024)
#define PSRAM_SIZE_8MB                  (8 * 1024 * 1024)
#define PSRAM_SIZE_16MB                 (16 * 1024 * 1024)
#define PSRAM_SIZE_32MB                 (32 * 1024 * 1024)

#define PSRAM_SIZE_16MBITS      0
#define PSRAM_SIZE_32MBITS      1
#define PSRAM_SIZE_64MBITS      2
#define PSRAM_SIZE_MAX          3

#define PSRAM_CACHE_S80M                1
#define PSRAM_CACHE_S40M                2
#define PSRAM_CACHE_S26M                3
#define PSRAM_CACHE_S20M                4
#define PSRAM_CACHE_MAX                 5

#define SPIRAM_WRAP_MODE_16B            0
#define SPIRAM_WRAP_MODE_32B            1
#define SPIRAM_WRAP_MODE_64B            2
#define SPIRAM_WRAP_MODE_DISABLE        3

/* See the TRM, chapter PID/MPU/MMU, header 'External RAM' for the
 * definitions of these modes. Important is that NORMAL works with the
 * cache disabled.
 */

#define PSRAM_VADDR_MODE_NORMAL   0  /* CPU use their own flash
                                      * cache for external RAM access
                                      */

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: psram_get_physical_size
 *
 * Description:
 *   Get the physical psram size in bytes.
 *
 ****************************************************************************/

int psram_get_physical_size(uint32_t *out_size_bytes);

/****************************************************************************
 * Name: psram_get_available_size
 *
 * Description:
 *   Get the available physical psram size in bytes.
 *
 * If ECC is enabled, available PSRAM size will be 15/16 times its
 * physical size. If not, it equals to the physical psram size.
 * Note: For now ECC is only enabled on ESP32S2 Octal PSRAM
 *
 * Input Parameters:
 *   out_size_bytes - availabe physical psram size in bytes.
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int psram_get_available_size(uint32_t *out_size_bytes);

/****************************************************************************
 * Name: psram_get_available_size
 *
 * Description:
 *   Enable psram cache
 *
 * Esp-idf uses this to initialize cache for psram, mapping it into the
 * main memory address space.
 *
 * Input Parameters:
 *   mode      - SPI mode to access psram in.
 *   vaddrmode - Mode the psram cache works in.
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int psram_enable(int mode, int vaddrmode);

/****************************************************************************
 * Name: psram_get_cs_io
 *
 * Description:
 *   Get the psram CS IO
 *
 * Returned Value:
 *   The psram CS IO pin.
 *
 ****************************************************************************/

uint8_t psram_get_cs_io(void);

/****************************************************************************
 * Name: psram_get_size
 *
 * Description:
 *   Return size of PSRAM in bytes
 *
 * Returned Value:
 *   PSRAM size in bytes
 *
 ****************************************************************************/

int psram_get_size(void);

#endif /* __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_PSRAM_H */
