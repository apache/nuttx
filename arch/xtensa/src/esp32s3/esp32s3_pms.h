/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_pms.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_PMS_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_PMS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <stdint.h>

#include "esp32s3_wcl.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Split lines partition a memory type into areas.  Up to 4 split lines
 * (PMS_SPLIT_LINE_0..3) exist per applicable memory type.
 */

enum pms_split_line_e
{
  PMS_SPLIT_LINE_0 = 0,
  PMS_SPLIT_LINE_1,
  PMS_SPLIT_LINE_2,
  PMS_SPLIT_LINE_3
};

/* Areas are the regions delimited by the BASE, the split lines and the END
 * of a memory type.  Up to 4 areas (PMS_AREA_0..3) exist per memory type.
 */

enum pms_area_e
{
  PMS_AREA_0 = 0,    /* Area between BASE and split_line 0 */
  PMS_AREA_1,        /* Area between split_line 0 and split_line 1 */
  PMS_AREA_2,        /* Area between split_line 1 and END */
  PMS_AREA_3,
  PMS_AREA_INVALID
};

/* Per-operation access flags applied to a world within an area. */

enum pms_flags_e
{
  PMS_ACCESS_NONE = 0,
  PMS_ACCESS_R = 1,
  PMS_ACCESS_W = 2,
  PMS_ACCESS_X = 4,
  PMS_ACCESS_ALL = PMS_ACCESS_X | PMS_ACCESS_W | PMS_ACCESS_R
};

/* There are 55 peripherals, each having 2 bits for permission configuration.
 * These are spread across 4 registers, each register having maximum of 16
 * peripheral entries.
 *
 * Enum defined as per the bit field position of the peripheral in the
 * register:
 *    FIELD = 30 - 2 * (ENUM % 16)
 */

enum pms_peripheral_e
{
  PMS_UART1 = 0,
  PMS_I2S0,
  PMS_I2C,
  PMS_MISC,
  PMS_HINF = 5,
  PMS_IO_MUX = 7,
  PMS_RTC,
  PMS_FE = 10,
  PMS_FE2 = 11,
  PMS_GPIO,
  PMS_G0SPI_0,
  PMS_G0SPI_1,
  PMS_UART,
  PMS_SYSTIMER,
  PMS_TIMERGROUP1,
  PMS_TIMERGROUP,
  PMS_PWM0,
  PMS_BB,
  PMS_BACKUP = 22,
  PMS_LEDC,
  PMS_SLC,
  PMS_PCNT,
  PMS_RMT,
  PMS_SLCHOST,
  PMS_UHCI0,
  PMS_I2C_EXT0,
  PMS_BT = 31,
  PMS_PWR = 33,
  PMS_WIFIMAC,
  PMS_RWBT = 36,
  PMS_UART2 = 39,
  PMS_I2S1,
  PMS_PWM1,
  PMS_CAN,
  PMS_SDIO_HOST,
  PMS_I2C_EXT1,
  PMS_APB_CTRL,
  PMS_SPI_3,
  PMS_SPI_2,
  PMS_WORLD_CONTROLLER,
  PMS_DIO,
  PMS_AD,
  PMS_CACHE_CONFIG,
  PMS_DMA_COPY,
  PMS_INTERRUPT,
  PMS_SENSITIVE,
  PMS_SYSTEM,
  PMS_USB,
  PMS_BT_PWR,
  PMS_LCD_CAM,
  PMS_APB_ADC,
  PMS_CRYPTO_DMA,
  PMS_CRYPTO_PERI,
  PMS_USB_WRAP,
  PMS_USB_DEVICE,
  PMS_MAX
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_pms_set_sram_main_split_line
 *
 * Description:
 *   Configure the main Internal SRAM1 Instruction/Data split line.
 *
 ****************************************************************************/

void esp32s3_pms_set_sram_main_split_line(uintptr_t addr);

/****************************************************************************
 * Name: esp32s3_pms_set_iram_split_line
 *
 * Description:
 *   Set one of the IRAM region split lines.
 *
 ****************************************************************************/

void esp32s3_pms_set_iram_split_line(enum pms_split_line_e line,
                                     uintptr_t addr);

/****************************************************************************
 * Name: esp32s3_pms_set_dram_split_line
 *
 * Description:
 *   Set one of the DRAM region split lines.
 *
 ****************************************************************************/

void esp32s3_pms_set_dram_split_line(enum pms_split_line_e line,
                                     uintptr_t addr);

/****************************************************************************
 * Name: esp32s3_pms_set_flash_cache_split_line
 *
 * Description:
 *   Split the External Flash cached region into sub regions.  The starting
 *   address is aligned to 64 KB and the length is expressed in 64 KB pages.
 *
 ****************************************************************************/

void esp32s3_pms_set_flash_cache_split_line(enum pms_split_line_e line,
                                            uintptr_t addr, size_t length);

/****************************************************************************
 * Name: esp32s3_pms_configure_iram_region
 *
 * Description:
 *   Configure access permissions to a given IRAM split region for a world.
 *
 ****************************************************************************/

void esp32s3_pms_configure_iram_region(enum pms_area_e area,
                                       enum esp32s3_pms_world_e world,
                                       enum pms_flags_e flags);

/****************************************************************************
 * Name: esp32s3_pms_configure_dram_region
 *
 * Description:
 *   Configure access permissions to a given DRAM split region for a world.
 *
 ****************************************************************************/

void esp32s3_pms_configure_dram_region(enum pms_area_e area,
                                       enum esp32s3_pms_world_e world,
                                       enum pms_flags_e flags);

/****************************************************************************
 * Name: esp32s3_pms_configure_icache
 *
 * Description:
 *   Configure access permissions to the Internal SRAM0 blocks not used as
 *   Instruction Cache, for a world.
 *
 ****************************************************************************/

void esp32s3_pms_configure_icache(enum pms_area_e area,
                                  enum esp32s3_pms_world_e world,
                                  enum pms_flags_e flags);

/****************************************************************************
 * Name: esp32s3_pms_configure_dcache
 *
 * Description:
 *   Configure access permissions to the Internal SRAM2 blocks not used as
 *   Data Cache, for a world.
 *
 ****************************************************************************/

void esp32s3_pms_configure_dcache(enum esp32s3_pms_world_e world,
                                  enum pms_flags_e flags);

/****************************************************************************
 * Name: esp32s3_pms_configure_flash_cache_region
 *
 * Description:
 *   Configure access permissions to a given External Flash cached split
 *   region for a world.
 *
 ****************************************************************************/

void esp32s3_pms_configure_flash_cache_region(enum pms_area_e area,
                                              enum esp32s3_pms_world_e world,
                                              enum pms_flags_e flags);

/****************************************************************************
 * Name: esp32s3_pms_set_sram_split_line
 *
 * Description:
 *   Place one of the four external-SRAM (PSRAM) split regions.  Address and
 *   length are physical offsets into the PSRAM device, both 64 KB aligned.
 *
 ****************************************************************************/

void esp32s3_pms_set_sram_split_line(enum pms_split_line_e line,
                                     uintptr_t addr, size_t length);

/****************************************************************************
 * Name: esp32s3_pms_configure_sram_region
 *
 * Description:
 *   Configure a world's access permissions to one external-SRAM region.
 *
 ****************************************************************************/

void esp32s3_pms_configure_sram_region(enum pms_area_e area,
                                       enum esp32s3_pms_world_e world,
                                       enum pms_flags_e flags);

/****************************************************************************
 * Name: esp32s3_pms_configure_peripheral
 *
 * Description:
 *   Configure access permissions to a given peripheral for a world.
 *
 ****************************************************************************/

void esp32s3_pms_configure_peripheral(enum pms_peripheral_e periph,
                                      enum esp32s3_pms_world_e world,
                                      enum pms_flags_e flags);

/****************************************************************************
 * Name: esp32s3_pms_configure_irom_access
 *
 * Description:
 *   Configure the kernel (all) / user (none) access permissions to the IROM
 *   region.
 *
 ****************************************************************************/

void esp32s3_pms_configure_irom_access(void);

/****************************************************************************
 * Name: esp32s3_pms_configure_drom_access
 *
 * Description:
 *   Configure the kernel (all) / user (none) access permissions to the DROM
 *   region.
 *
 ****************************************************************************/

void esp32s3_pms_configure_drom_access(void);

#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_PMS_H */
