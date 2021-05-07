/****************************************************************************
 * arch/arm/src/nuc1xx/hardware/nuc_memorymap.h
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

#ifndef __ARCH_ARM_SRC_NUC1XX_HARDWARE_NUC_MEMORYMAP_H
#define __ARCH_ARM_SRC_NUC1XX_HARDWARE_NUC_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System Memory Map */

#define NUC_FLASH_BASE   0x00000000 /* -0x0001ffff: FLASH memory space (128KB) */
#define NUC_SRAM_BASE    0x20000000 /* -0x20003fff: SRAM memory space (16KB) */
#define NUC_APB1_BASE    0x40000000 /* -0x400fffff: APB1 controller space */
#define NUC_APB2_BASE    0x40100000 /* -0x401fffff: APB2 controller space */
#define NUC_AHB_BASE     0x50000000 /* -0x501fffff: AHB controller space */
#define NUC_EXTMEM_BASE  0x60000000 /* -0x6001ffff: External Memory space (128KB) */
#define NUC_SYSCON_BASE  0x60000000 /* -0x6001ffff: ARMv6-M system controller space
                                     *              See nvic.h */

/* FLASH space */

#define NUC_CONFIG_BASE  0x00300000 /* -0x00300004: User configuration */

/* AHB controller space */

#define NUC_GCR_BASE     0x50000000 /* -0x500001ff: System global control registers */
#define NUC_CLK_BASE     0x50000200 /* -0x500002ff: Clock control registers */
#define NUC_INT_BASE     0x50000300 /* -0x500003ff: Interrupt multiplexor control registers */
#define NUC_GPIO_BASE    0x50004000 /* -0x50007fff: GPIO control registers */
#define NUC_PDMA_BASE    0x50008000 /* -0x5000bfff: Peripheral DMA control registers */
#define NUC_FMC_BASE     0x5000c000 /* -0x5000ffff: Flash memory control registers */
#define NUC_EBI_BASE     0x50010000 /* -0x500103ff: External bus interface control registers */

/* APB1 controller space */

#define NUC_WDT_BASE     0x40004000 /* -0x40007fff: Watchdog timer control registers */
#define NUC_RTC_BASE     0x40008000 /* -0x4000bfff: Real time clock (RTc) control registers */
#define NUC_TMR01_BASE   0x40010000 /* -0x40013fff: Timer0/Timer0 control registers */
#define NUC_I2C0_BASE    0x40020000 /* -0x40023fff: I2C interface control registers */
#define NUC_SPI0_BASE    0x40030000 /* -0x40033fff: SPI0 master/slave control registers */
#define NUC_SPI1_BASE    0x40034000 /* -0x40037fff: SPI1 master/slave control registers */
#define NUC_PWMA_BASE    0x40040000 /* -0x40043fff: PWM0/1/2/3 control registers */
#define NUC_UART0_BASE   0x40050000 /* -0x40053fff: UART0 control registers */
#define NUC_USBD_BASE    0x40060000 /* -0x40063fff: USB 2.0 FS device controller registers */
#define NUC_ACMP_BASE    0x400d0000 /* -0x400d3fff: Analog comparator control registers */
#define NUC_ADC_BASE     0x400e0000 /* -0x400effff: Analog-digital-converter (ADC) control
                                     *              registers */

/* APB2 controller space */

#define NUC_PS2_BASE     0x40100000 /* -0x40103fff: PS/2 interface control registers */
#define NUC_TIMR23_BASE  0x40110000 /* -0x40113fff: Timer2/Timer3 control registers */
#define NUC_I2C1_BASE    0x40120000 /* -0x40123fff: I2C1 interface control registers */
#define NUC_SPI2_BASE    0x40130000 /* -0x40133fff: SPI2 master/slave control registers */
#define NUC_SPI3_BASE    0x40134000 /* -0x40137fff: SPI3 master/slave control registers */
#define NUC_PWMB_BASE    0x40140000 /* -0x40143fff: PWM4/5/6/7 control registers */
#define NUC_UART1_BASE   0x40150000 /* -0x40153fff: UART1 control registers */
#define NUC_UART2_BASE   0x40154000 /* -0x40157fff: UART2 control registers */
#define NUC_I2S_BASE     0x401a0000 /* -0x401a3fff: I2S interface control registers */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_NUC1XX_HARDWARE_NUC_MEMORYMAP_H */
