/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mzef_memorymap.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZEF_MEMORYMAP_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZEF_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "mips32-memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Physical Memory Map ******************************************************/

/* Memory Regions */

#define PIC32MZ_DATAMEM_PBASE     0x00000000 /* Size depends on CHIP_DATAMEM_KB */
#define PIC32MZ_PROGFLASH_PBASE   0x1d000000 /* Size depends on CHIP_PROGFLASH_KB */
#define PIC32MZ_SFR_PBASE         0x1f800000 /* Special function registers */
#define PIC32MZ_BOOTFLASH_PBASE   0x1fc00000 /* Size depends on CHIP_BOOTFLASH_KB */
#define PIC32MZ_EBIMEM_PBASE      0x20000000 /* External memory via EBI */
#define PIC32MZ_SQIMEM_PBASE      0x30000000 /* External memory via SQI */

/* Boot FLASH */

#define PIC32MZ_LOWERBOOT_PBASE   0x1fc00000 /* Lower boot alias */
#define PIC32MZ_BOOTCFG_PBASE     0x1fc0ff00 /* Configuration space */
#define PIC32MZ_UPPERBOOT_PBASE   0x1fc20000 /* Upper boot alias */
#define PIC32MZ_BOOT1_PBASE       0x1fc40000 /* Boot flash 1 */
#define PIC32MZ_SEQCFG1_PBASE     0x1fc4ff00 /* Sequence/configuration space 1 */
#define PIC32MZ_DEVSN_PBASE       0x1fc54000 /* Device serial number */
#define PIC32MZ_BOOT2_PBASE       0x1fc60000 /* Boot flash 2 */
#define PIC32MZ_SEQCFG2_PBASE     0x1fc6ff00 /* Sequence/configuration space 2 */

/* Virtual Memory Map *******************************************************/

#define PIC32MZ_DATAMEM_K0BASE      (KSEG0_BASE + PIC32MZ_DATAMEM_PBASE)
#define PIC32MZ_PROGFLASH_K0BASE    (KSEG0_BASE + PIC32MZ_PROGFLASH_PBASE)
#define PIC32MZ_SFR_K0BASE          (KSEG0_BASE + PIC32MZ_SFR_PBASE)
#define PIC32MZ_BOOTFLASH_K0BASE    (KSEG0_BASE + PIC32MZ_BOOTFLASH_PBASE)
#define PIC32MZ_EBIMEM_K0BASE       (KSEG0_BASE + PIC32MZ_EBIMEM_PBASE)
#define PIC32MZ_SQIMEM_K0BASE       (KSEG0_BASE + PIC32MZ_SQIMEM_PBASE)

#define PIC32MZ_DATAMEM_K1BASE      (KSEG1_BASE + PIC32MZ_DATAMEM_PBASE)
#define PIC32MZ_PROGFLASH_K1BASE    (KSEG1_BASE + PIC32MZ_PROGFLASH_PBASE)
#define PIC32MZ_SFR_K1BASE          (KSEG1_BASE + PIC32MZ_SFR_PBASE)
#define PIC32MZ_BOOTFLASH_K1BASE    (KSEG1_BASE + PIC32MZ_BOOTFLASH_PBASE)
#define PIC32MZ_EBIMEM_K1BASE       (KSEG1_BASE + PIC32MZ_EBIMEM_PBASE)
#define PIC32MZ_SQIMEM_K1BASE       (KSEG1_BASE + PIC32MZ_SQIMEM_PBASE)

/* Boot FLASH */

#define PIC32MZ_LOWERBOOT_K0BASE    (KSEG0_BASE + PIC32MZ_LOWERBOOT_PBASE)
#define PIC32MZ_BOOTCFG_K0BASE      (KSEG0_BASE + PIC32MZ_BOOTCFG_PBASE)
#define PIC32MZ_UPPERBOOT_K0BASE    (KSEG0_BASE + PIC32MZ_UPPERBOOT_PBASE)
#define PIC32MZ_BOOT1_K0BASE        (KSEG0_BASE + PIC32MZ_BOOT1_PBASE)
#define PIC32MZ_SEQCFG1_K0BASE      (KSEG0_BASE + PIC32MZ_SEQCFG1_PBASE)
#define PIC32MZ_DEVSN_K0BASE        (KSEG0_BASE + PIC32MZ_DEVSN_PBASE)
#define PIC32MZ_BOOT2_K0BASE        (KSEG0_BASE + PIC32MZ_BOOT2_PBASE)
#define PIC32MZ_SEQCFG2_K0BASE      (KSEG0_BASE + PIC32MZ_SEQCFG2_PBASE)

#define PIC32MZ_LOWERBOOT_K1BASE    (KSEG1_BASE + PIC32MZ_LOWERBOOT_PBASE)
#define PIC32MZ_BOOTCFG_K1BASE      (KSEG1_BASE + PIC32MZ_BOOTCFG_PBASE)
#define PIC32MZ_UPPERBOOT_K1BASE    (KSEG1_BASE + PIC32MZ_UPPERBOOT_PBASE)
#define PIC32MZ_BOOT1_K1BASE        (KSEG1_BASE + PIC32MZ_BOOT1_PBASE)
#define PIC32MZ_SEQCFG1_K1BASE      (KSEG1_BASE + PIC32MZ_SEQCFG1_PBASE)
#define PIC32MZ_DEVSN_K1BASE        (KSEG1_BASE + PIC32MZ_DEVSN_PBASE)
#define PIC32MZ_BOOT2_K1BASE        (KSEG1_BASE + PIC32MZ_BOOT2_PBASE)
#define PIC32MZ_SEQCFG2_K1BASE      (KSEG1_BASE + PIC32MZ_SEQCFG2_PBASE)

/* Register Base Addresses **************************************************/

#define PIC32MZ_CONFIG_K1BASE       (PIC32MZ_SFR_K1BASE + 0x00000000) /* Configuration */
#define PIC32MZ_FLASHC_K1BASE       (PIC32MZ_SFR_K1BASE + 0x00000600) /* Flash Controller */
#define PIC32MZ_WDT_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00000800) /* Watchdog Timer */
#define PIC32MZ_DMT_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00000a00) /* Deadman Timer */
#define PIC32MZ_RTCC_K1BASE         (PIC32MZ_SFR_K1BASE + 0x00000c00) /* RTCC */
#define PIC32MZ_CVREF_K1BASE        (PIC32MZ_SFR_K1BASE + 0x00000e00) /* CVREF */
#define PIC32MZ_OSC_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00001200) /* Oscillator */
#define PIC32MZ_PPS_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00001400) /* PPS */

#define PIC32MZ_INT_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00010000) /* Interrupt Controller */
#define PIC32MZ_DMA_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00011000) /* DMA */

#define PIC32MZ_I2C_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00020000) /* I2C1-I2C5 */
#define PIC32MZ_SPI_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00021000) /* SPI1-SPI6 */
#define PIC32MZ_UART_K1BASE         (PIC32MZ_SFR_K1BASE + 0x00022000) /* UART1-UART6 */
#define PIC32MZ_PMP_K1BASE          (PIC32MZ_SFR_K1BASE + 0x0002e000) /* PMP */

#define PIC32MZ_TIMER_K1BASE        (PIC32MZ_SFR_K1BASE + 0x00040000) /* Timer1-Timer9 */
#define PIC32MZ_IC_K1BASE           (PIC32MZ_SFR_K1BASE + 0x00042000) /* IC1-IC9 */
#define PIC32MZ_OC_K1BASE           (PIC32MZ_SFR_K1BASE + 0x00044000) /* OC1-OC9 */
#define PIC32MZ_ADC1_K1BASE         (PIC32MZ_SFR_K1BASE + 0x0004b000) /* ADC1 */
#define PIC32MZ_CMP_K1BASE          (PIC32MZ_SFR_K1BASE + 0x0004c000) /* Comparator 1, 2 */

#define PIC32MZ_IOPORT_K1BASE       (PIC32MZ_SFR_K1BASE + 0x00060000) /* PORTA-PORTK */

#define PIC32MZ_CAN_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00080000) /* CAN1 and CAN2 */
#define PIC32MZ_ETH_K1BASE          (PIC32MZ_SFR_K1BASE + 0x00082000) /* Ethernet */
#define PIC32MZ_USBCR_K1BASE        (PIC32MZ_SFR_K1BASE + 0x00084000) /* USBCR */

#define PIC32MZ_PREFETCH_K1BASE     (PIC32MZ_SFR_K1BASE + 0x000e0000) /* Prefetch */
#define PIC32MZ_EBI_K1BASE          (PIC32MZ_SFR_K1BASE + 0x000e1000) /* EBI */
#define PIC32MZ_SQI1_K1BASE         (PIC32MZ_SFR_K1BASE + 0x000e2000) /* SQI1 */
#define PIC32MZ_USB_K1BASE          (PIC32MZ_SFR_K1BASE + 0x000e3000) /* USB */
#define PIC32MZ_CRYPTO_K1BASE       (PIC32MZ_SFR_K1BASE + 0x000e5000) /* Crypto */
#define PIC32MZ_RNG_K1BASE          (PIC32MZ_SFR_K1BASE + 0x000e6000) /* RNG */

#define PIC32MZ_SYSBUS_K1BASE       (PIC32MZ_SFR_K1BASE + 0x000f0000) /* System Bus */

#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZEF_MEMORYMAP_H */
