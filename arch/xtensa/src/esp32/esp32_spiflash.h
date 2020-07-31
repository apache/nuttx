/****************************************************************************
 * arch/xtensa/src/esp32/esp32_spiflash.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_SPIFLASH_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_SPIFLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <nuttx/mtd/mtd.h>

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
 * Name: esp32_spiflash_init
 *
 * Description:
 *   Alloc ESP32 SPI Flash MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP32 SPI Flash MTD data pointer if success or NULL if fail.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *esp32_spiflash_alloc_mtdpart(void);

/****************************************************************************
 * Name: esp32_spiflash_get_mtd
 *
 * Description:
 *   Get ESP32 SPI Flash raw MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP32 SPI Flash raw MTD data pointer.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *esp32_spiflash_get_mtd(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_SPIFLASH_H */
