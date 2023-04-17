/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_spi_timing.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_SPI_TIMING_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_SPI_TIMING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ESP32S3_FLASH_FREQ_20M)
#  define ESP32S3_SPI_TIMING_FLASH_CLOCK            20
#elif defined(CONFIG_ESP32S3_FLASH_FREQ_40M)
#  define ESP32S3_SPI_TIMING_FLASH_CLOCK            40
#elif defined(CONFIG_ESP32S3_FLASH_FREQ_80M)
#  define ESP32S3_SPI_TIMING_FLASH_CLOCK            80
#elif defined(CONFIG_ESP32S3_FLASH_FREQ_120M)
#  define ESP32S3_SPI_TIMING_FLASH_CLOCK            120
#endif

#if defined(CONFIG_ESP32S3_FLASH_SAMPLE_MODE_DTR)
#  if ESP32S3_SPI_TIMING_FLASH_CLOCK > 40
#    define ESP32S3_SPI_TIMING_FLASH_TUNING         1
#  else
#    define ESP32S3_SPI_TIMING_FLASH_TUNING         0
#  endif
#elif defined(CONFIG_ESP32S3_FLASH_SAMPLE_MODE_STR)
#  if ESP32S3_SPI_TIMING_FLASH_CLOCK > 80
#    define ESP32S3_SPI_TIMING_FLASH_TUNING         1
#  else
#    define ESP32S3_SPI_TIMING_FLASH_TUNING         0
#  endif
#else
#  define ESP32S3_SPI_TIMING_FLASH_TUNING           0
#endif

#if ESP32S3_SPI_TIMING_FLASH_TUNING
#  error "SPI flash tuning is not supported"
#endif

#if defined(CONFIG_ESP32S3_SPIRAM)
#  if defined(CONFIG_ESP32S3_SPIRAM_SPEED_40M)
#    define ESP32S3_SPI_TIMING_PSRAM_CLOCK          40
#  elif defined(CONFIG_ESP32S3_SPIRAM_SPEED_80M)
#    define ESP32S3_SPI_TIMING_PSRAM_CLOCK          80
#  elif defined(CONFIG_ESP32S3_SPIRAM_SPEED_120M)
#    define ESP32S3_SPI_TIMING_PSRAM_CLOCK          120
#  endif
#else
#  define ESP32S3_SPI_TIMING_PSRAM_CLOCK            10
#endif

#if defined(CONFIG_ESP32S3_SPIRAM_MODE_OCT)
#  if ESP32S3_SPI_TIMING_PSRAM_CLOCK > 40
#    define ESP32S3_SPI_TIMING_PSRAM_TUNING         1
#  else
#    define ESP32S3_SPI_TIMING_PSRAM_TUNING         0
#  endif
#elif defined(CONFIG_ESP32S3_SPIRAM_MODE_QUAD)
#  if ESP32S3_SPI_TIMING_PSRAM_CLOCK > 80
#    define ESP32S3_SPI_TIMING_PSRAM_TUNING         1
#  else
#    define ESP32S3_SPI_TIMING_PSRAM_TUNING         0
#  endif
#else
#  define ESP32S3_SPI_TIMING_PSRAM_TUNING           0
#endif

#if ESP32S3_SPI_TIMING_PSRAM_TUNING
#  error "SPI PSRAM tuning is not supported"
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_spi_timing_set_mspi_low_speed
 *
 * Description:
 *   Make MSPI work under 20MHz and remove the timing tuning required delays.
 *
 * Input Parameters:
 *   spi1 - Select whether to control SPI1
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_spi_timing_set_mspi_low_speed(bool control_spi1);

/****************************************************************************
 * Name: esp32s3_spi_timing_set_mspi_high_speed
 *
 * Description:
 *   Make MSPI work under the frequency as users set, may add certain
 *   delays to MSPI RX direction to meet timing requirements.
 *
 * Input Parameters:
 *   spi1 - Select whether to control SPI1
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_spi_timing_set_mspi_high_speed(bool control_spi1);

/****************************************************************************
 * Name: esp32s3_spi_timing_set_pin_drive_strength
 *
 * Description:
 *   Make SPI all GPIO strength to be 3 under default clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_spi_timing_set_pin_drive_strength(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_SPI_H */
