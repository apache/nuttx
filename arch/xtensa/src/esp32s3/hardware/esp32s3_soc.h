/****************************************************************************
 * arch/xtensa/src/esp32s3/hardware/esp32s3_soc.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_SOC_H
#define __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_SOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#include <stdint.h>
#include <stdbool.h>
#endif

#include <nuttx/bits.h>

#include "soc/soc.h"
#include "soc/hwcrypto_reg.h"
#include "soc/system_reg.h"
#include "esp_attr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DR_REG_USB_BASE                         0x60080000
#define DR_REG_EMAC_BASE                        0x600CD000
#define DR_REG_ASSIST_DEBUG_BASE                0x600CE000
#define DR_REG_WORLD_CNTL_BASE                  0x600D0000
#define DR_REG_DPORT_END                        0x600D3FFC

#ifndef __ASSEMBLY__

/* Extract the field from the register and shift it to avoid wrong reading */

#define REG_MASK(_reg, _field) ((_reg & (_field##_M)) >> (_field##_S))

/* Helper to place a value in a field */

#define VALUE_TO_FIELD(_value, _field) (((_value) << (_field##_S)) & (_field##_M))

#endif /* __ASSEMBLY__ */

/* Peripheral Clock */

#define  RTC_CLK_FREQ                                (20*1000000)

/* Core voltage needs to be increased in two cases:
 * 1. running at 240 MHz
 * 2. running with 80MHz Flash frequency
 */

#if defined(CONFIG_ESP32S3_FLASH_FREQ_80M) || defined(CONFIG_ESP32S3_FLASH_FREQ_120M)
#define DIG_DBIAS_80M_160M          RTC_CNTL_DBIAS_1V25
#else
#define DIG_DBIAS_80M_160M          RTC_CNTL_DBIAS_1V10
#endif
#define DIG_DBIAS_240M              RTC_CNTL_DBIAS_1V25
#define DIG_DBIAS_XTAL              RTC_CNTL_DBIAS_1V10
#define DIG_DBIAS_2M                RTC_CNTL_DBIAS_1V00

#define MHZ (1000000)
#define RTC_PLL_FREQ_320M           320
#define RTC_PLL_FREQ_480M           480

#define DPORT_CPUPERIOD_SEL_80      0
#define DPORT_CPUPERIOD_SEL_160     1
#define DPORT_CPUPERIOD_SEL_240     2

#define DPORT_SOC_CLK_SEL_XTAL      0
#define DPORT_SOC_CLK_SEL_PLL       1
#define DPORT_SOC_CLK_SEL_8M        2

#define RTC_FAST_CLK_FREQ_8M        8500000

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_sp_dram
 *
 * Description:
 *   Check if the pointer is dma capable.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32s3_ptr_dma_capable(const void *p)
{
    return (intptr_t)p >= SOC_DMA_LOW && (intptr_t)p < SOC_DMA_HIGH;
}

/****************************************************************************
 * Name: esp32s3_sp_dram
 *
 * Description:
 *   Check if the stack pointer is in DRAM.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32s3_sp_dram(uint32_t sp)
{
  return (sp >= SOC_DRAM_LOW + 0x10 && sp < SOC_DRAM_HIGH - 0x10);
}

/****************************************************************************
 * Name: esp32s3_ptr_extram
 *
 * Description:
 *   Check if the buffer comes from the external RAM
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32s3_ptr_extram(const void *p)
{
  return ((intptr_t)p >= SOC_EXTRAM_DATA_LOW &&
          (intptr_t)p < SOC_EXTRAM_DATA_HIGH);
}

/****************************************************************************
 * Name: esp32s3_ptr_iram
 *
 * Description:
 *   Check if the pointer is in IRAM
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32s3_ptr_iram(const void *p)
{
  return ((intptr_t)p >= SOC_IRAM_LOW && (intptr_t)p < SOC_IRAM_HIGH);
}

/****************************************************************************
 * Name: esp32s3_ptr_exec
 *
 * Description:
 *   Check if the pointer is within an executable range.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32s3_ptr_exec(const void *p)
{
  intptr_t ip = (intptr_t)p;
  return (ip >= SOC_IROM_LOW && ip < SOC_IROM_HIGH)
      || (ip >= SOC_IRAM_LOW && ip < SOC_IRAM_HIGH)
      || (ip >= SOC_IROM_MASK_LOW && ip < SOC_IROM_MASK_HIGH)
#if defined(SOC_CACHE_APP_LOW) && !defined(CONFIG_SMP)
      || (ip >= SOC_CACHE_APP_LOW && ip < SOC_CACHE_APP_HIGH)
#endif
      || (ip >= SOC_RTC_IRAM_LOW && ip < SOC_RTC_IRAM_HIGH);
}

/****************************************************************************
 * Name: esp32s3_ptr_rtc
 *
 * Description:
 *   Check if the buffer comes from the RTC RAM.
 *
 * Parameters:
 *   p - Address of the buffer.
 *
 * Return Value:
 *   True if given buffer comes from RTC RAM. False if not.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32s3_ptr_rtc(const void *p)
{
  return ((intptr_t)p >= SOC_RTC_DATA_LOW &&
          (intptr_t)p < SOC_RTC_DATA_HIGH);
}

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_SOC_H */
