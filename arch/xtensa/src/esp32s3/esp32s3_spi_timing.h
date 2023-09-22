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

#if ESP32S3_SPI_TIMING_FLASH_TUNING || ESP32S3_SPI_TIMING_PSRAM_TUNING

/* This should be larger than the max available timing config num */

#define MSPI_TIMING_CONFIG_NUM_DEFAULT              20

#define __GET_TUNING_CONFIG(type, core_clock, module_clock, mode) \
        (tuning_config_s) { .config_table = MSPI_TIMING_##type##_CONFIG_TABLE_CORE_CLK_##core_clock##M_MODULE_CLK_##module_clock##M_##mode, \
                            .config_num = MSPI_TIMING_##type##_CONFIG_NUM_CORE_CLK_##core_clock##M_MODULE_CLK_##module_clock##M_##mode, \
                            .default_config_id = MSPI_TIMING_##type##_DEFAULT_CONFIG_ID_CORE_CLK_##core_clock##M_MODULE_CLK_##module_clock##M_##mode }

#define _GET_TUNING_CONFIG(type, core_clock, module_clock, mode) __GET_TUNING_CONFIG(type, core_clock, module_clock, mode)
#define MSPI_TIMING_FLASH_GET_TUNING_CONFIG(core_clock_mhz, module_clock_mhz, mode) _GET_TUNING_CONFIG(FLASH, core_clock_mhz, module_clock_mhz, mode)
#define MSPI_TIMING_PSRAM_GET_TUNING_CONFIG(core_clock_mhz, module_clock_mhz, mode) _GET_TUNING_CONFIG(PSRAM, core_clock_mhz, module_clock_mhz, mode)

/* Timing Tuning Parameters */

/* FLASH: core clock 160M, module clock 40M, DTR mode */

#define MSPI_TIMING_FLASH_CONFIG_TABLE_CORE_CLK_160M_MODULE_CLK_40M_DTR_MODE         {{1, 0, 0}, {0, 0, 0}, {2, 1, 1}, {2, 0, 1}, {2, 2, 2}, {2, 1, 2}, {1, 0, 1}, {0, 0, 1}}
#define MSPI_TIMING_FLASH_CONFIG_NUM_CORE_CLK_160M_MODULE_CLK_40M_DTR_MODE           8
#define MSPI_TIMING_FLASH_DEFAULT_CONFIG_ID_CORE_CLK_160M_MODULE_CLK_40M_DTR_MODE    2

/* FLASH: core clock 160M, module clock 80M, DTR mode */

#define MSPI_TIMING_FLASH_CONFIG_TABLE_CORE_CLK_160M_MODULE_CLK_80M_DTR_MODE         {{0, 0, 0}, {4, 2, 2}, {2, 1, 2}, {4, 1, 2}, {1, 0, 1}, {4, 0, 2}, {0, 0, 1}, {4, 2, 3}, {2, 1, 3}, {4, 1, 3}, {1, 0, 2}, {4, 0, 3}, {0, 0, 2}, {4, 2, 4}}
#define MSPI_TIMING_FLASH_CONFIG_NUM_CORE_CLK_160M_MODULE_CLK_80M_DTR_MODE           14
#define MSPI_TIMING_FLASH_DEFAULT_CONFIG_ID_CORE_CLK_160M_MODULE_CLK_80M_DTR_MODE    1

/* FLASH: core clock 240M, module clock 120M, DTR mode */

#define MSPI_TIMING_FLASH_CONFIG_TABLE_CORE_CLK_240M_MODULE_CLK_120M_DTR_MODE        {{0, 0, 0}, {4, 1, 2}, {1, 0, 1}, {4, 0, 2}, {0, 0, 1}, {4, 1, 3}, {1, 0, 2}, {4, 0, 3}, {0, 0, 2}, {4, 1, 4}, {1, 0, 3}, {4, 0, 4}, {0, 0, 3}, {4, 1, 5}}
#define MSPI_TIMING_FLASH_CONFIG_NUM_CORE_CLK_240M_MODULE_CLK_120M_DTR_MODE          14
#define MSPI_TIMING_FLASH_DEFAULT_CONFIG_ID_CORE_CLK_240M_MODULE_CLK_120M_DTR_MODE   1

/* FLASH: core clock 160M, module clock 80M, STR mode */

#define MSPI_TIMING_FLASH_CONFIG_TABLE_CORE_CLK_160M_MODULE_CLK_80M_STR_MODE         {{1, 0, 0}, {0, 0, 0}, {2, 1, 1}, {2, 0, 1}, {2, 2, 2}, {2, 1, 2}, {1, 0, 1}, {0, 0, 1}}
#define MSPI_TIMING_FLASH_CONFIG_NUM_CORE_CLK_160M_MODULE_CLK_80M_STR_MODE           8
#define MSPI_TIMING_FLASH_DEFAULT_CONFIG_ID_CORE_CLK_160M_MODULE_CLK_80M_STR_MODE    2

/* FLASH: core clock 120M, module clock 120M, STR mode */

#define MSPI_TIMING_FLASH_CONFIG_TABLE_CORE_CLK_120M_MODULE_CLK_120M_STR_MODE        {{2, 0, 1}, {0, 0, 0}, {2, 2, 2}, {1, 0, 1}, {2, 0, 2}, {0, 0, 1}, {2, 2, 3}, {1, 0, 2}, {2, 0, 3}, {0, 0, 2}, {2, 2, 4}, {1, 0, 3}}
#define MSPI_TIMING_FLASH_CONFIG_NUM_CORE_CLK_120M_MODULE_CLK_120M_STR_MODE          12
#define MSPI_TIMING_FLASH_DEFAULT_CONFIG_ID_CORE_CLK_120M_MODULE_CLK_120M_STR_MODE   2

/* FLASH: core clock 240M, module clock 120M, STR mode */

#define MSPI_TIMING_FLASH_CONFIG_TABLE_CORE_CLK_240M_MODULE_CLK_120M_STR_MODE        {{1, 0, 0}, {0, 0, 0}, {1, 1, 1}, {2, 3, 2}, {1, 0, 1}, {0, 0, 1}, {1, 1, 2}, {2, 3, 3}, {1, 0, 2}, {0, 0, 2}, {1, 1, 3}, {2, 3, 4}}
#define MSPI_TIMING_FLASH_CONFIG_NUM_CORE_CLK_240M_MODULE_CLK_120M_STR_MODE          12
#define MSPI_TIMING_FLASH_DEFAULT_CONFIG_ID_CORE_CLK_240M_MODULE_CLK_120M_STR_MODE   2

/* PSRAM: core clock 80M, module clock 40M, DTR mode */

#define MSPI_TIMING_PSRAM_CONFIG_TABLE_CORE_CLK_80M_MODULE_CLK_40M_DTR_MODE          {{1, 0, 0}, {2, 1, 1}, {2, 0, 1}, {0, 0, 0}, {3, 1, 1}, {3, 0, 1}, {1, 0, 1}, {2, 1, 2}, {2, 0, 2}, {0, 0, 1}, {3, 1, 2}, {3, 0, 2}}
#define MSPI_TIMING_PSRAM_CONFIG_NUM_CORE_CLK_80M_MODULE_CLK_40M_DTR_MODE            12
#define MSPI_TIMING_PSRAM_DEFAULT_CONFIG_ID_CORE_CLK_80M_MODULE_CLK_40M_DTR_MODE     4

/* PSRAM: core clock 160M, module clock 80M, DTR mode */

#define MSPI_TIMING_PSRAM_CONFIG_TABLE_CORE_CLK_160M_MODULE_CLK_80M_DTR_MODE         {{0, 0, 0}, {4, 2, 2}, {2, 1, 2}, {4, 1, 2}, {1, 0, 1}, {4, 0, 2}, {0, 0, 1}, {4, 2, 3}, {2, 1, 3}, {4, 1, 3}, {1, 0, 2}, {4, 0, 3}, {0, 0, 2}, {4, 2, 4}}
#define MSPI_TIMING_PSRAM_CONFIG_NUM_CORE_CLK_160M_MODULE_CLK_80M_DTR_MODE           14
#define MSPI_TIMING_PSRAM_DEFAULT_CONFIG_ID_CORE_CLK_160M_MODULE_CLK_80M_DTR_MODE    5

/* PSRAM: core clock 240M, module clock 120M, STR mode */

#define MSPI_TIMING_PSRAM_CONFIG_TABLE_CORE_CLK_240M_MODULE_CLK_120M_STR_MODE        {{1, 0, 0}, {0, 0, 0}, {1, 1, 1}, {2, 3, 2}, {1, 0, 1}, {0, 0, 1}, {1, 1, 2}, {2, 3, 3}, {1, 0, 2}, {0, 0, 2}, {1, 1, 3}, {2, 3, 4}}
#define MSPI_TIMING_PSRAM_CONFIG_NUM_CORE_CLK_240M_MODULE_CLK_120M_STR_MODE          12
#define MSPI_TIMING_PSRAM_DEFAULT_CONFIG_ID_CORE_CLK_240M_MODULE_CLK_120M_STR_MODE   2

/* PSRAM: core clock 120M, module clock 120M, STR mode */

#define MSPI_TIMING_PSRAM_CONFIG_TABLE_CORE_CLK_120M_MODULE_CLK_120M_STR_MODE        {{2, 0, 1}, {0, 0, 0}, {2, 2, 2}, {1, 0, 1}, {2, 0, 2}, {0, 0, 1}, {2, 2, 3}, {1, 0, 2}, {2, 0, 3}, {0, 0, 2}, {2, 2, 4}, {1, 0, 3}}
#define MSPI_TIMING_PSRAM_CONFIG_NUM_CORE_CLK_120M_MODULE_CLK_120M_STR_MODE          12
#define MSPI_TIMING_PSRAM_DEFAULT_CONFIG_ID_CORE_CLK_120M_MODULE_CLK_120M_STR_MODE   2

/* PSRAM: core clock 240M, module clock 120M, DTR mode */

#define MSPI_TIMING_PSRAM_CONFIG_TABLE_CORE_CLK_240M_MODULE_CLK_120M_DTR_MODE        {{0, 0, 0}, {4, 1, 2}, {1, 0, 1}, {4, 0, 2}, {0, 0, 1}, {4, 1, 3}, {1, 0, 2}, {4, 0, 3}, {0, 0, 2}, {4, 1, 4}, {1, 0, 3}, {4, 0, 4}, {0, 0, 3}, {4, 1, 5}}
#define MSPI_TIMING_PSRAM_CONFIG_NUM_CORE_CLK_240M_MODULE_CLK_120M_DTR_MODE          14
#define MSPI_TIMING_PSRAM_DEFAULT_CONFIG_ID_CORE_CLK_240M_MODULE_CLK_120M_DTR_MODE   1

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#if ESP32S3_SPI_TIMING_FLASH_TUNING || ESP32S3_SPI_TIMING_PSRAM_TUNING

/* SPI timing tuning registers.
 * Upper layer rely on these 3 registers to tune the timing.
 */

typedef struct
{
  uint8_t spi_din_mode;    /* input signal delay mode */
  uint8_t spi_din_num;     /* input signal delay number */
  uint8_t extra_dummy_len; /* extra dummy length */
} tuning_param_s;

typedef struct
{
  tuning_param_s config_table[MSPI_TIMING_CONFIG_NUM_DEFAULT];
  uint32_t config_num;
  uint32_t default_config_id; /* If tuning fails, we use this one as default */
} tuning_config_s;

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

/****************************************************************************
 * Name: esp32s3_spi_timing_set_mspi_psram_tuning
 *
 * Description:
 *   Tune MSPI psram timing to make it work under high frequency
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_spi_timing_set_mspi_psram_tuning(void);

/****************************************************************************
 * Name: esp32s3_spi_timing_set_mspi_flash_tuning
 *
 * Description:
 *   Tune MSPI flash timing to make it work under high frequency
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_spi_timing_set_mspi_flash_tuning(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_SPI_H */
