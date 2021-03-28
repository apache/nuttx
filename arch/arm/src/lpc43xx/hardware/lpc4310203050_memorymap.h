/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc4310203050_memorymap.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC4310203050_MEMORYMAP_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC4310203050_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map ***************************************************************/

/* See arch/arm/include/lpc43xx/chip.h for the actual sizes of FLASH and
 * SRAM regions
 */

#define LPC43_SHADOW_BASE          0x00000000 /* -0x0fffffff: 256Mb shadow area */
#define LPC43_LOCSRAM_BASE         0x10000000 /* -0x1fffffff: Local SRAM and external memory */
#define LPC43_AHBSRAM_BASE         0x20000000 /* -0x27ffffff: AHB SRAM */
#define LPC43_DYCS0_BASE           0x28000000 /* -0x2fffffff: 128Mb dynamic external memory */
#define LPC43_DYCS1_BASE           0x30000000 /* -0x2fffffff: 256Mb dynamic external memory */
#define LPC43_PERIPH_BASE          0x40000000 /* -0x5fffffff: Peripherals */
#define LPC43_DYCS2_BASE           0x60000000 /* -0x6fffffff: 256Mb dynamic external memory */
#define LPC43_DYCS3_BASE           0x70000000 /* -0x7fffffff: 256Mb dynamic external memory */
#define LPC43_SPIFI_DATA_BASE      0x80000000 /* -0x87ffffff: 256Mb dynamic external memory */
#define LPC43_ARM_BASE             0xe0000000 /* -0xe00fffff: ARM private */

/* Local SRAM Banks and external memory */

#define LPC43_LOCSRAM_BANK0_BASE   (LPC43_LOCSRAM_BASE + 0x00000000)
#define LPC43_LOCSRAM_BANK1_BASE   (LPC43_LOCSRAM_BASE + 0x00080000)
#define LPC43_ROM_BASE             (LPC43_LOCSRAM_BASE + 0x00400000)
#define LPC43_LOCSRAM_SPIFI_BASE   (LPC43_LOCSRAM_BASE + 0x04000000)
#define LPC43_EXTMEM_CS0_BASE      (LPC43_LOCSRAM_BASE + 0x0c000000)
#define LPC43_EXTMEM_CS1_BASE      (LPC43_LOCSRAM_BASE + 0x0d000000)
#define LPC43_EXTMEM_CS2_BASE      (LPC43_LOCSRAM_BASE + 0x0e000000)
#define LPC43_EXTMEM_CS3_BASE      (LPC43_LOCSRAM_BASE + 0x0f000000)

/* ROM Driver Table */

#define LPC43_ROM_DRIVER_TABLE     (LPC43_ROM_BASE+0x00000100)
#define LPC43_ROM_DRIVER_TABLE0    (LPC43_ROM_DRIVER_TABLE+0x0000)
#define LPC43_ROM_DRIVER_TABLE1    (LPC43_ROM_DRIVER_TABLE+0x0004)
#define LPC43_ROM_DRIVER_TABLE2    (LPC43_ROM_DRIVER_TABLE+0x0008)
#define LPC43_ROM_DRIVER_TABLE3    (LPC43_ROM_DRIVER_TABLE+0x000c)
#define LPC43_ROM_DRIVER_TABLE4    (LPC43_ROM_DRIVER_TABLE+0x0010)
#define LPC43_ROM_DRIVER_TABLE5    (LPC43_ROM_DRIVER_TABLE+0x0014)
#define LPC43_ROM_DRIVER_TABLE6    (LPC43_ROM_DRIVER_TABLE+0x0018)
#define LPC43_ROM_DRIVER_TABLE7    (LPC43_ROM_DRIVER_TABLE+0x001c)

/* AHB SRAM */

#define LPC43_AHBSRAM_BANK0_BASE   (LPC43_AHBSRAM_BASE)
#define LPC43_AHBSRAM_BANK1_BASE   (LPC43_AHBSRAM_BASE + 0x00008000)
#define LPC43_AHBSRAM_BANK2_BASE   (LPC43_AHBSRAM_BASE + 0x0000c000)
#define LPC43_AHBSRAM_BITBAND_BASE (LPC43_AHBSRAM_BASE + 0x0000c000)

/* Peripherals */

#define LPC43_AHBPERIPH_BASE       (LPC43_PERIPH_BASE + 0x00000000)
#define LPC43_RTCPERIPH_BASE       (LPC43_PERIPH_BASE + 0x00040000)
#define LPC43_CLKPERIPH_BASE       (LPC43_PERIPH_BASE + 0x00050000)
#define LPC43_APB0PERIPH_BASE      (LPC43_PERIPH_BASE + 0x00080000)
#define LPC43_APB1PERIPH_BASE      (LPC43_PERIPH_BASE + 0x000a0000)
#define LPC43_APB2PERIPH_BASE      (LPC43_PERIPH_BASE + 0x000c0000)
#define LPC43_APB3PERIPH_BASE      (LPC43_PERIPH_BASE + 0x000e0000)
#define LPC43_GPIO_BASE            (LPC43_PERIPH_BASE + 0x000f4000)
#define LPC43_SPI_BASE             (LPC43_PERIPH_BASE + 0x00100000)
#define LPC43_SGPIO_BASE           (LPC43_PERIPH_BASE + 0x00101000)
#define LPC43_PERIPH_BITBAND_BASE  (LPC43_PERIPH_BASE + 0x02000000)

/* AHB Peripherals */

#define LPC43_SCT_BASE             (LPC43_AHBPERIPH_BASE + 0x00000000)
#define LPC43_DMA_BASE             (LPC43_AHBPERIPH_BASE + 0x00002000)
#define LPC43_SPIFI_BASE           (LPC43_AHBPERIPH_BASE + 0x00003000)
#define LPC43_SDMMC_BASE           (LPC43_AHBPERIPH_BASE + 0x00004000)
#define LPC43_EMC_BASE             (LPC43_AHBPERIPH_BASE + 0x00005000)
#define LPC43_USB0_BASE            (LPC43_AHBPERIPH_BASE + 0x00006000)
#define LPC43_USB1_BASE            (LPC43_AHBPERIPH_BASE + 0x00007000)
#define LPC43_LCD_BASE             (LPC43_AHBPERIPH_BASE + 0x00008000)
#define LPC43_ETHERNET_BASE        (LPC43_AHBPERIPH_BASE + 0x00010000)

/* RTC Domain Peripherals */

#define LPC43_ATIMER_BASE          (LPC43_RTCPERIPH_BASE + 0x00000000)
#define LPC43_BACKUP_BASE          (LPC43_RTCPERIPH_BASE + 0x00001000)
#define LPC43_PMC_BASE             (LPC43_RTCPERIPH_BASE + 0x00002000)
#define LPC43_CREG_BASE            (LPC43_RTCPERIPH_BASE + 0x00003000)
#define LPC43_EVNTRTR_BASE         (LPC43_RTCPERIPH_BASE + 0x00004000)
#define LPC43_OTPC_BASE            (LPC43_RTCPERIPH_BASE + 0x00005000)
#define LPC43_RTC_BASE             (LPC43_RTCPERIPH_BASE + 0x00006000)
#define LPC43_EVNTMNTR_BASE        (LPC43_RTC_BASE       + 0x00000080)

/* Clocking and Reset Peripherals */

#define LPC43_CGU_BASE             (LPC43_CLKPERIPH_BASE + 0x00000000)
#define LPC43_CCU1_BASE            (LPC43_CLKPERIPH_BASE + 0x00001000)
#define LPC43_CCU2_BASE            (LPC43_CLKPERIPH_BASE + 0x00002000)
#define LPC43_RGU_BASE             (LPC43_CLKPERIPH_BASE + 0x00003000)

/* APB0 Peripherals */

#define LPC43_WWDT_BASE            (LPC43_APB0PERIPH_BASE + 0x00000000)
#define LPC43_USART0_BASE          (LPC43_APB0PERIPH_BASE + 0x00001000)
#define LPC43_UART1_BASE           (LPC43_APB0PERIPH_BASE + 0x00002000)
#define LPC43_SSP0_BASE            (LPC43_APB0PERIPH_BASE + 0x00003000)
#define LPC43_TIMER0_BASE          (LPC43_APB0PERIPH_BASE + 0x00004000)
#define LPC43_TIMER1_BASE          (LPC43_APB0PERIPH_BASE + 0x00005000)
#define LPC43_SCU_BASE             (LPC43_APB0PERIPH_BASE + 0x00006000)
#define LPC43_GPIOINT_BASE         (LPC43_APB0PERIPH_BASE + 0x00007000)
#define LPC43_GRP0INT_BASE         (LPC43_APB0PERIPH_BASE + 0x00008000)
#define LPC43_GRP1INT_BASE         (LPC43_APB0PERIPH_BASE + 0x00009000)

/* APB1 Peripherals */

#define LPC43_MCPWM_BASE           (LPC43_APB1PERIPH_BASE + 0x00000000)
#define LPC43_I2C0_BASE            (LPC43_APB1PERIPH_BASE + 0x00001000)
#define LPC43_I2S0_BASE            (LPC43_APB1PERIPH_BASE + 0x00002000)
#define LPC43_I2S1_BASE            (LPC43_APB1PERIPH_BASE + 0x00003000)
#define LPC43_CAN1_BASE            (LPC43_APB1PERIPH_BASE + 0x00004000)

/* APB2 Peripherals */

#define LPC43_RIT_BASE             (LPC43_APB2PERIPH_BASE + 0x00000000)
#define LPC43_USART2_BASE          (LPC43_APB2PERIPH_BASE + 0x00001000)
#define LPC43_USART3_BASE          (LPC43_APB2PERIPH_BASE + 0x00002000)
#define LPC43_TIMER2_BASE          (LPC43_APB2PERIPH_BASE + 0x00003000)
#define LPC43_TIMER3_BASE          (LPC43_APB2PERIPH_BASE + 0x00004000)
#define LPC43_SSP1_BASE            (LPC43_APB2PERIPH_BASE + 0x00005000)
#define LPC43_QEI_BASE             (LPC43_APB2PERIPH_BASE + 0x00006000)
#define LPC43_GIMA_BASE            (LPC43_APB2PERIPH_BASE + 0x00007000)

/* APB3 Peripherals */

#define LPC43_I2C1_BASE            (LPC43_APB3PERIPH_BASE + 0x00000000)
#define LPC43_DAC_BASE             (LPC43_APB3PERIPH_BASE + 0x00001000)
#define LPC43_CAN0_BASE            (LPC43_APB3PERIPH_BASE + 0x00002000)
#define LPC43_ADC0_BASE            (LPC43_APB3PERIPH_BASE + 0x00003000)
#define LPC43_ADC1_BASE            (LPC43_APB3PERIPH_BASE + 0x00004000)

/* ARM Private */

#define LPC43_SCS_BASE             (LPC43_ARM_BASE + 0x0000e000)
#define LPC43_DEBUGMCU_BASE        (LPC43_ARM_BASE + 0x00042000)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC4310203050_MEMORYMAP_H */
