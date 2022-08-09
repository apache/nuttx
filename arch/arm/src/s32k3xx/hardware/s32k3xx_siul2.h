/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_siul2.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SIUL2_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SIUL2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define S32K3XX_PORTA                    (0)
#define S32K3XX_PORTB                    (1)
#define S32K3XX_PORTC                    (2)
#define S32K3XX_PORTD                    (3)
#define S32K3XX_PORTE                    (4)
#define S32K3XX_PORTF                    (5)
#define S32K3XX_PORTG                    (6)

#define S32K3XX_NPORTS                   (7)  /* Number of available ports */
#define S32K3XX_NPINS                    (32) /* Maximum amount of pins per port */

/* SIUL2 Register Offsets ***************************************************/

#define S32K3XX_SIUL2_MIDR1_OFFSET       (0x0004) /* SIUL2 MCU ID Register #1 (MIDR1) */
#define S32K3XX_SIUL2_MIDR2_OFFSET       (0x0008) /* SIUL2 MCU ID Register #2 (MIDR2) */
#define S32K3XX_SIUL2_DISR0_OFFSET       (0x0010) /* SIUL2 DMA/Interrupt Status Flag Register 0 (DISR0) */
#define S32K3XX_SIUL2_DIRER0_OFFSET      (0x0018) /* SIUL2 DMA/Interrupt Request Enable Register 0 (DIRER0) */
#define S32K3XX_SIUL2_DIRSR0_OFFSET      (0x0020) /* SIUL2 DMA/Interrupt Request Select Register 0 (DIRSR0) */
#define S32K3XX_SIUL2_IREER0_OFFSET      (0x0028) /* SIUL2 Interrupt Rising-Edge Event Enable Register 0 (IREER0) */
#define S32K3XX_SIUL2_IFEER0_OFFSET      (0x0030) /* SIUL2 Interrupt Falling-Edge Event Enable Register 0 (IFEER0) */
#define S32K3XX_SIUL2_IFER0_OFFSET       (0x0038) /* SIUL2 Interrupt Filter Enable Register 0 (IFER0) */

#define S32K3XX_SIUL2_IFMCR_OFFSET(n)    (0x0040 + ((n) << 2)) /* SIUL2 Interrupt Filter Maximum Counter Register n=0..31 (IFMCRn) */

#define S32K3XX_SIUL2_IFCPR_OFFSET       (0x00c0) /* SIUL2 Interrupt Filter Clock Prescaler Register */
#define S32K3XX_SIUL2_MIDR3_OFFSET       (0x0200) /* SIUL2 MCU ID Register #3 (MIDR3) */
#define S32K3XX_SIUL2_MIDR4_OFFSET       (0x0204) /* SIUL2 MCU ID Register #4 (MIDR4) */

#define S32K3XX_SIUL2_MSCR_OFFSET(n)     (0x0240 + ((n) << 2)) /* SIUL2 Multiplexed Signal Configuration Register n=0..219 (MSCRn) */
#define S32K3XX_SIUL2_IMCR_OFFSET(n)     (0x0a40 + ((n) << 2)) /* SIUL2 Input Multiplexed Signal Configuration Register n=0..378 (IMCRn) */

#define S32K3XX_SIUL2_GPDO_OFFSET(n)     (0x1300 + ((n) + 3 - 2 * ((n) % 4))) /* SIUL2 GPIO Pad Data Output Register n=0..219 (GPDOn) */
#define S32K3XX_SIUL2_GPDI_OFFSET(n)     (0x1500 + ((n) + 3 - 2 * ((n) % 4))) /* SIUL2 GPIO Pad Data Input Register n=0..219 (GPDIn) */

#define S32K3XX_SIUL2_PGPDO1_OFFSET      (0x1700) /* SIUL2 Parallel GPIO Pad Data Out Register 1 (PGPDO1) */
#define S32K3XX_SIUL2_PGPDO0_OFFSET      (0x1702) /* SIUL2 Parallel GPIO Pad Data Out Register 0 (PGPDO0) */
#define S32K3XX_SIUL2_PGPDO3_OFFSET      (0x1704) /* SIUL2 Parallel GPIO Pad Data Out Register 3 (PGPDO3) */
#define S32K3XX_SIUL2_PGPDO2_OFFSET      (0x1706) /* SIUL2 Parallel GPIO Pad Data Out Register 2 (PGPDO2) */
#define S32K3XX_SIUL2_PGPDO5_OFFSET      (0x1708) /* SIUL2 Parallel GPIO Pad Data Out Register 5 (PGPDO5) */
#define S32K3XX_SIUL2_PGPDO4_OFFSET      (0x170a) /* SIUL2 Parallel GPIO Pad Data Out Register 4 (PGPDO4) */
#define S32K3XX_SIUL2_PGPDO7_OFFSET      (0x170c) /* SIUL2 Parallel GPIO Pad Data Out Register 7 (PGPDO7) */
#define S32K3XX_SIUL2_PGPDO6_OFFSET      (0x170e) /* SIUL2 Parallel GPIO Pad Data Out Register 6 (PGPDO6) */
#define S32K3XX_SIUL2_PGPDO9_OFFSET      (0x1710) /* SIUL2 Parallel GPIO Pad Data Out Register 9 (PGPDO9) */
#define S32K3XX_SIUL2_PGPDO8_OFFSET      (0x1712) /* SIUL2 Parallel GPIO Pad Data Out Register 8 (PGPDO8) */
#define S32K3XX_SIUL2_PGPDO11_OFFSET     (0x1714) /* SIUL2 Parallel GPIO Pad Data Out Register 11 (PGPDO11) */
#define S32K3XX_SIUL2_PGPDO10_OFFSET     (0x1716) /* SIUL2 Parallel GPIO Pad Data Out Register 10 (PGPDO10) */
#define S32K3XX_SIUL2_PGPDO13_OFFSET     (0x1718) /* SIUL2 Parallel GPIO Pad Data Out Register 13 (PGPDO13) */
#define S32K3XX_SIUL2_PGPDO12_OFFSET     (0x171a) /* SIUL2 Parallel GPIO Pad Data Out Register 12 (PGPDO12) */
#define S32K3XX_SIUL2_PGPDI1_OFFSET      (0x1740) /* SIUL2 Parallel GPIO Pad Data In Register 1 (PGPDI1) */
#define S32K3XX_SIUL2_PGPDI0_OFFSET      (0x1742) /* SIUL2 Parallel GPIO Pad Data In Register 0 (PGPDI0) */
#define S32K3XX_SIUL2_PGPDI3_OFFSET      (0x1744) /* SIUL2 Parallel GPIO Pad Data In Register 3 (PGPDI3) */
#define S32K3XX_SIUL2_PGPDI2_OFFSET      (0x1746) /* SIUL2 Parallel GPIO Pad Data In Register 2 (PGPDI2) */
#define S32K3XX_SIUL2_PGPDI5_OFFSET      (0x1748) /* SIUL2 Parallel GPIO Pad Data In Register 5 (PGPDI5) */
#define S32K3XX_SIUL2_PGPDI4_OFFSET      (0x174a) /* SIUL2 Parallel GPIO Pad Data In Register 4 (PGPDI4) */
#define S32K3XX_SIUL2_PGPDI7_OFFSET      (0x174c) /* SIUL2 Parallel GPIO Pad Data In Register 7 (PGPDI7) */
#define S32K3XX_SIUL2_PGPDI6_OFFSET      (0x174e) /* SIUL2 Parallel GPIO Pad Data In Register 6 (PGPDI6) */
#define S32K3XX_SIUL2_PGPDI9_OFFSET      (0x1750) /* SIUL2 Parallel GPIO Pad Data In Register 9 (PGPDI9) */
#define S32K3XX_SIUL2_PGPDI8_OFFSET      (0x1752) /* SIUL2 Parallel GPIO Pad Data In Register 8 (PGPDI8) */
#define S32K3XX_SIUL2_PGPDI11_OFFSET     (0x1754) /* SIUL2 Parallel GPIO Pad Data In Register 11 (PGPDI11) */
#define S32K3XX_SIUL2_PGPDI10_OFFSET     (0x1756) /* SIUL2 Parallel GPIO Pad Data In Register 10 (PGPDI10) */
#define S32K3XX_SIUL2_PGPDI13_OFFSET     (0x1758) /* SIUL2 Parallel GPIO Pad Data In Register 13 (PGPDI13) */
#define S32K3XX_SIUL2_PGPDI12_OFFSET     (0x175a) /* SIUL2 Parallel GPIO Pad Data In Register 12 (PGPDI12) */

#define S32K3XX_SIUL2_MPGPDO_OFFSET(n)   (0x1780 + ((n) << 2)) /* SIUL2 Masked Parallel GPIO Pad Data Out Register n=0..13 (MPGPDOn) */

/* SIUL2 Register Addresses *************************************************/

#define S32K3XX_SIUL2_MIDR1              (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_MIDR1_OFFSET)
#define S32K3XX_SIUL2_MIDR2              (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_MIDR2_OFFSET)
#define S32K3XX_SIUL2_DISR0              (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_DISR0_OFFSET)
#  define S32K3XX_SIUL2_DISR0_IRQ0       (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_DISR0_OFFSET + 0x00)
#  define S32K3XX_SIUL2_DISR0_IRQ1       (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_DISR0_OFFSET + 0x01)
#  define S32K3XX_SIUL2_DISR0_IRQ2       (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_DISR0_OFFSET + 0x02)
#  define S32K3XX_SIUL2_DISR0_IRQ3       (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_DISR0_OFFSET + 0x03)
#define S32K3XX_SIUL2_DIRER0             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_DIRER0_OFFSET)
#define S32K3XX_SIUL2_DIRSR0             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_DIRSR0_OFFSET)
#define S32K3XX_SIUL2_IREER0             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_IREER0_OFFSET)
#define S32K3XX_SIUL2_IFEER0             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_IFEER0_OFFSET)
#define S32K3XX_SIUL2_IFER0              (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_IFER0_OFFSET)
#define S32K3XX_SIUL2_IFMCR(n)           (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_IFMCR_OFFSET(n))
#define S32K3XX_SIUL2_IFCPR              (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_IFCPR_OFFSET)
#define S32K3XX_SIUL2_MIDR3              (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_MIDR3_OFFSET)
#define S32K3XX_SIUL2_MIDR4              (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_MIDR4_OFFSET)
#define S32K3XX_SIUL2_MSCR(n)            (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_MSCR_OFFSET(n))
#define S32K3XX_SIUL2_IMCR(n)            (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_IMCR_OFFSET(n))
#define S32K3XX_SIUL2_GPDO(n)            (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_GPDO_OFFSET(n))
#define S32K3XX_SIUL2_GPDI(n)            (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_GPDI_OFFSET(n))
#define S32K3XX_SIUL2_PGPDO1             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO1_OFFSET)
#define S32K3XX_SIUL2_PGPDO0             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO0_OFFSET)
#define S32K3XX_SIUL2_PGPDO3             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO3_OFFSET)
#define S32K3XX_SIUL2_PGPDO2             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO2_OFFSET)
#define S32K3XX_SIUL2_PGPDO5             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO5_OFFSET)
#define S32K3XX_SIUL2_PGPDO4             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO4_OFFSET)
#define S32K3XX_SIUL2_PGPDO7             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO7_OFFSET)
#define S32K3XX_SIUL2_PGPDO6             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO6_OFFSET)
#define S32K3XX_SIUL2_PGPDO9             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO9_OFFSET)
#define S32K3XX_SIUL2_PGPDO8             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO8_OFFSET)
#define S32K3XX_SIUL2_PGPDO11            (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO11_OFFSET)
#define S32K3XX_SIUL2_PGPDO10            (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO10_OFFSET)
#define S32K3XX_SIUL2_PGPDO13            (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO13_OFFSET)
#define S32K3XX_SIUL2_PGPDO12            (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDO12_OFFSET)
#define S32K3XX_SIUL2_PGPDI1             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI1_OFFSET)
#define S32K3XX_SIUL2_PGPDI0             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI0_OFFSET)
#define S32K3XX_SIUL2_PGPDI3             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI3_OFFSET)
#define S32K3XX_SIUL2_PGPDI2             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI2_OFFSET)
#define S32K3XX_SIUL2_PGPDI5             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI5_OFFSET)
#define S32K3XX_SIUL2_PGPDI4             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI4_OFFSET)
#define S32K3XX_SIUL2_PGPDI7             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI7_OFFSET)
#define S32K3XX_SIUL2_PGPDI6             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI6_OFFSET)
#define S32K3XX_SIUL2_PGPDI9             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI9_OFFSET)
#define S32K3XX_SIUL2_PGPDI8             (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI8_OFFSET)
#define S32K3XX_SIUL2_PGPDI11            (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI11_OFFSET)
#define S32K3XX_SIUL2_PGPDI10            (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI10_OFFSET)
#define S32K3XX_SIUL2_PGPDI13            (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI13_OFFSET)
#define S32K3XX_SIUL2_PGPDI12            (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_PGPDI12_OFFSET)
#define S32K3XX_SIUL2_MPGPDO(n)          (S32K3XX_SIUL2_BASE + S32K3XX_SIUL2_MPGPDO_OFFSET(n))

/* SIUL2 Register Bitfield Definitions **************************************/

/* SIUL2 MCU ID Register #1 (MIDR1) */

#define SIUL2_MIDR1_MINOR_SHIFT          (0)        /* Bits 0-3: Minor Mask Revision (MINOR_MASK) */
#define SIUL2_MIDR1_MINOR_MASK           (0x0f << SIUL2_MIDR1_MINOR_SHIFT)
#define SIUL2_MIDR1_MAJOR_SHIFT          (4)        /* Bits 4-7: Major Mask Revision (MAJOR_MASK) */
#define SIUL2_MIDR1_MAJOR_MASK           (0x0f << SIUL2_MIDR1_MAJOR_SHIFT)
                                                    /* Bits 8-15: Reserved */
#define SIUL2_MIDR1_PART_NO_SHIFT        (16)       /* Bits 16-25: MCU Part Number (PART_NO) */
#define SIUL2_MIDR1_PART_NO_MASK         (0x03ff << SIUL2_MIDR1_PART_NO_SHIFT)
#  define SIUL2_MIDR1_PART_NO_S32K311    (0x0137 << SIUL2_MIDR1_PART_NO_SHIFT) /* S32K311 */
#  define SIUL2_MIDR1_PART_NO_S32K312    (0x0138 << SIUL2_MIDR1_PART_NO_SHIFT) /* S32K312 */
#  define SIUL2_MIDR1_PART_NO_S32K314    (0x013a << SIUL2_MIDR1_PART_NO_SHIFT) /* S32K314 */
#  define SIUL2_MIDR1_PART_NO_S32K322    (0x0142 << SIUL2_MIDR1_PART_NO_SHIFT) /* S32K322 */
#  define SIUL2_MIDR1_PART_NO_S32K324    (0x0144 << SIUL2_MIDR1_PART_NO_SHIFT) /* S32K324 */
#  define SIUL2_MIDR1_PART_NO_S32K328    (0x0148 << SIUL2_MIDR1_PART_NO_SHIFT) /* S32K328 */
#  define SIUL2_MIDR1_PART_NO_S32K338    (0x0152 << SIUL2_MIDR1_PART_NO_SHIFT) /* S32K338 */
#  define SIUL2_MIDR1_PART_NO_S32K342    (0x0156 << SIUL2_MIDR1_PART_NO_SHIFT) /* S32K342 */
#  define SIUL2_MIDR1_PART_NO_S32K344    (0x0158 << SIUL2_MIDR1_PART_NO_SHIFT) /* S32K344 */
#  define SIUL2_MIDR1_PART_NO_S32K348    (0x015c << SIUL2_MIDR1_PART_NO_SHIFT) /* S32K348 */
#  define SIUL2_MIDR1_PART_NO_S32K358    (0x0166 << SIUL2_MIDR1_PART_NO_SHIFT) /* S32K358 */

#define SIUL2_MIDR1_PRODUCT_LINE_LETTER_SHIFT (26)  /* Bits 26-31: Product Line Letter (PRODUCT_LINE_LETTER) */
#define SIUL2_MIDR1_PRODUCT_LINE_LETTER_MASK  (0x3f << SIUL2_MIDR1_PRODUCT_LINE_LETTER_SHIFT)

/* SIUL2 MCU ID Register #2 (MIDR2) */

#define SIUL2_MIDR2_FLASH_SIZE_CODE_SHIFT (0)       /* Bits 0-7: Code Flash Size (FLASH_SIZE_CODE) */
#define SIUL2_MIDR2_FLASH_SIZE_CODE_MASK  (0xff << SIUL2_MIDR2_FLASH_SIZE_CODE_SHIFT)
#define SIUL2_MIDR2_FLASH_SIZE_DATA_SHIFT (8)       /* Bits 8-11: Data Flash Size (FLASH_SIZE_DATA) */
#define SIUL2_MIDR2_FLASH_SIZE_DATA_MASK  (0x0f << SIUL2_MIDR2_FLASH_SIZE_DATA_SHIFT)

#define SIUL2_MIDR2_FLASH_DATA_SHIFT     (12)       /* Bits 12-13: Data Flash Location (FLASH_DATA) */
#define SIUL2_MIDR2_FLASH_DATA_MASK      (0x03 << SIUL2_MIDR2_FLASH_DATA_SHIFT)
#define SIUL2_MIDR2_FLASH_CODE_SHIFT     (14)       /* Bits 14-15: Code Flash Location (FLASH_CODE) */
#define SIUL2_MIDR2_FLASH_CODE_MASK      (0x03 << SIUL2_MIDR2_FLASH_CODE_SHIFT)
#define SIUL2_MIDR2_FREQUENCY_SHIFT      (16)       /* Bits 16-19: Maximum Core Frequency (FREQUENCY) */
#define SIUL2_MIDR2_FREQUENCY_MASK       (0x0f << SIUL2_MIDR2_FREQUENCY_SHIFT)
#define SIUL2_MIDR2_PACKAGE_SHIFT        (20)       /* Bits 20-25: Package Type (PACKAGE) */
#define SIUL2_MIDR2_PACKAGE_MASK         (0x3f << SIUL2_MIDR2_PACKAGE_SHIFT)
#define SIUL2_MIDR2_TEMPERATURE_SHIFT    (26)       /* Bits 26-28: Ambient Temperature Range (TEMPERATURE) */
#define SIUL2_MIDR2_TEMPERATURE_MASK     (0x07 << SIUL2_MIDR2_TEMPERATURE_SHIFT)
#define SIUL2_MIDR2_TECHNOLOGY_SHIFT     (29)       /* Bits 29-31: Silicon Technology (TECHNOLOGY) */
#define SIUL2_MIDR2_TECHNOLOGY_MASK      (0x07 << SIUL2_MIDR2_TECHNOLOGY_SHIFT)

/* SIUL2 DMA/Interrupt Status Flag Register 0 (DISR0) */

#define SIUL2_DISR0_EIF(b)               (1 << (b)) /* Bits 0-31: External Interrupt Status Flag 0-31 (EIF0-EIF31) */

/* SIUL2 DMA/Interrupt Request Enable Register 0 (DIRER0) */

#define SIUL2_DIRER0_EIRE(b)             (1 << (b)) /* Bits 0-31: External Interrupt Request Enable 0-31 (EIRE0-EIRE31) */

/* SIUL2 DMA/Interrupt Request Select Register 0 (DIRSR0) */

#define SIUL2_DIRSR0_DIRSR(b)            (1 << (b)) /* Bits 0-31: DMA/Interrupt Request Select Register 0-31 (DIRSR0-DIRSR31) */

/* SIUL2 Interrupt Rising-Edge Event Enable Register 0 (IREER0) */

#define SIUL2_IREER0_IREE(b)             (1 << (b)) /* Bits 0-31: Interrupt Rising-Edge Event Enable 0-31 (IREE0-IREE31) */

/* SIUL2 Interrupt Falling-Edge Event Enable Register 0 (IFEER0) */

#define SIUL2_IFEER0_IFEE(b)             (1 << (b)) /* Bits 0-31: Interrupt Falling-Edge Event Enable 0-31 (IFEE0-IFEE31) */

/* SIUL2 Interrupt Filter Enable Register 0 (IFER0) */

#define SIUL2_IFER0_IFE(b)               (1 << (b)) /* Bits 0-31: Interrupt Filter Enable 0-31 (IFE0-IFE31) */

/* SIUL2 Interrupt Filter Maximum Counter Register n=0..31 (IFMCRn) */

#define SIUL2_IFMCR_MAXCNT_SHIFT         (0)        /* Bits 0-3: Maximum Interrupt Filter Counter Setting (MAXCNT) */
#define SIUL2_IFMCR_MAXCNT_MASK          (0x0f << SIUL2_IFMCR_MAXCNT_SHIFT)
                                                    /* Bits 4-31: Reserved */

/* SIUL2 Interrupt Filter Clock Prescaler Register */

#define SIUL2_IFCPR_IFCP_SHIFT           (0)        /* Bits 0-3: Interrupt Filter Clock Prescaler Setting (IFCP) */
#define SIUL2_IFCPR_IFCP_MASK            (0x0f << SIUL2_IFCPR_IFCP_SHIFT)
                                                    /* Bits 4-31: Reserved */

/* SIUL2 MCU ID Register #3 (MIDR3) */

#define SIUL2_MIDR3_SYS_RAM_SIZE_SHIFT   (0)        /* Bits 0-5: System RAM Size (SYS_RAM_SIZE) */
#define SIUL2_MIDR3_SYS_RAM_SIZE_MASK    (0x3f << SIUL2_MIDR3_SYS_RAM_SIZE_SHIFT)
                                                    /* Bits 6-9: Reserved */
#define SIUL2_MIDR3_PART_NO_SUF_SHIFT    (10)       /* Bits 10-15: Part Number Suffix (PART_NO_SUFFIX) */
#define SIUL2_MIDR3_PART_NO_SUF_MASK     (0x3f << SIUL2_MIDR3_PART_NO_SUF_SHIFT)
#define SIUL2_MIDR3_PROD_FAM_NO_SHIFT    (16)       /* Bits 16-25: Product Family Number (PROD_FAM_NO) */
#define SIUL2_MIDR3_PROD_FAM_NO_MASK     (0x03ff << SIUL2_MIDR3_PROD_FAM_NO_SHIFT)
#define SIUL2_MIDR3_PROD_FAM_LET_SHIFT   (26)       /* Bits 26-31: Product Family Letter (PROD_FAM_LET) */
#define SIUL2_MIDR3_PROD_FAM_LET_MASK    (0x3f << SIUL2_MIDR3_PROD_FAM_LET_SHIFT)

/* SIUL2 MCU ID Register #4 (MIDR4) */

#define SIUL2_MIDR4_CORE_PLAT_FET_SHIFT  (0)        /* Bits 0-2: Core Platform Options Feature (CORE_PLAT_FET) */
#define SIUL2_MIDR4_CORE_PLAT_FET_MASK   (0x07 << SIUL2_MIDR4_CORE_PLAT_FET_SHIFT)
#define SIUL2_MIDR4_EMAC_FET_SHIFT       (3)        /* Bits 3-4: Ethernet Feature (EMAC_FET) */
#define SIUL2_MIDR4_EMAC_FET_MASK        (0x03 << SIUL2_MIDR4_EMAC_FET_SHIFT)
#define SIUL2_MIDR4_SEC_FET_SHIFT        (5)        /* Bits 5-6: Security Feature (SEC_FET) */
#define SIUL2_MIDR4_SEC_FET_MASK         (0x03 << SIUL2_MIDR4_SEC_FET_SHIFT)
                                                    /* Bits 7-31: Reserved */

/* SIUL2 Multiplexed Signal Configuration Register n=0..219 (MSCRn) */

#define SIUL2_MSCR_SSS_SHIFT             (0)        /* Bit 0: Source Signal Select (SSS) */
#define SIUL2_MSCR_SSS_MASK              (0x07 << SIUL2_MSCR_SSS_SHIFT)
#  define SIUL2_MSCR_SSS(n)              (((n) << SIUL2_MSCR_SSS_SHIFT) & SIUL2_MSCR_SSS_MASK)
                                                    /* Bits 3-4: Reserved */
#define SIUL2_MSCR_SMC                   (1 << 5)   /* Bit 5: Safe Mode Control (SMC) */
#define SIUL2_MSCR_IFE                   (1 << 6)   /* Bit 6: Input Filter Enable (IFE) */
                                                    /* Bit 7: Reserved */
#define SIUL2_MSCR_DSE                   (1 << 8)   /* Bit 8: Drive Strength Enable (DSE) */
                                                    /* Bits 9-10: Reserved */
#define SIUL2_MSCR_PUS                   (1 << 11)  /* Bit 11: Pull Select (PUS) */
                                                    /* Bit 12: Reserved */
#define SIUL2_MSCR_PUE                   (1 << 13)  /* Bit 13: Pull Enable (PUE) */
#define SIUL2_MSCR_SRC                   (1 << 14)  /* Bit 14: Slew Rate Control (SRC) */
                                                    /* Bit 15: Reserved */
#define SIUL2_MSCR_PKE                   (1 << 16)  /* Bit 16: Pad Keeping Enable (PKE) */
#define SIUL2_MSCR_INV                   (1 << 17)  /* Bit 17: Invert (INV) */
                                                    /* Bit 18: Reserved */
#define SIUL2_MSCR_IBE                   (1 << 19)  /* Bit 19: Input Buffer Enable (IBE) */
                                                    /* Bit 20: Reserved */
#define SIUL2_MSCR_OBE                   (1 << 21)  /* Bit 21: GPIO Output Buffer Enable (OBE) */
                                                    /* Bits 22-31: Reserved */

/* SIUL2 Input Multiplexed Signal Configuration Register n=0..378 (IMCRn) */

#define SIUL2_IMCR_SSS_SHIFT             (0)        /* Bits 0-3: Source Signal Select (SSS) */
#define SIUL2_IMCR_SSS_MASK              (0x0f << SIUL2_IMCR_SSS_SHIFT)
#  define SIUL2_IMCR_SSS(n)              (((n) << SIUL2_IMCR_SSS_SHIFT) & SIUL2_IMCR_SSS_MASK)
                                                    /* Bits 4-31: Reserved */

/* SIUL2 GPIO Pad Data Output Register n=0..219 (GPDOn) */

#define SIUL2_GPDO_PDO                   (1 << 0)   /* Bit 0: Pad Data Out (PDO) */
                                                    /* Bits 1-7: Reserved */

/* SIUL2 GPIO Pad Data Input Register n=0..219 (GPDIn) */

#define SIUL2_GPDI_PDI                   (1 << 0)   /* Bit 0: Pad Data In (PDI) */
                                                    /* Bits 1-7: Reserved */

/* SIUL2 Parallel GPIO Pad Data Out Register n=0..13 (PGPDOn) */

#define SIUL2_PGPDO_PPDO(b)              (1 << (b)) /* Bits 0-15: Parallel Pad Data Out 0-15 (PPDO0-PPDO15) */

/* SIUL2 Parallel GPIO Pad Data In Register n=0..13 (PGPDIn) */

#define SIUL2_PGPDI_PPDI(b)              (1 << (b)) /* Bits 0-15: Parallel Pad Data In 0-15 (PPDI0-PPDI15) */

/* SIUL2 Masked Parallel GPIO Pad Data Out Register n=0..13 (MPGPDOn) */

#define SIUL2_MPGPDO_MPPDO(b)            (1 << (b)) /* Bits 0-15: Masked Parallel Pad Data Out 0-15 (MPPDO0-MPPDO15) */
#define SIUL2_MPGPDO_MPPDO_SHIFT         (0)
#define SIUL2_MPGPDO_MPPDO_MASK          (0xffff << SIUL2_MPGPDO_MPPDO_SHIFT)
#define SIUL2_MPGPDO_MASK(b)             (1 << (b)) /* Bits 16-31: Mask Field 0-15 (MASK0-MASK15) */
#define SIUL2_MPGPDO_MASK_SHIFT          (16)
#define SIUL2_MPGPDO_MASK_MASK           (0xffff << SIUL2_MPGPDO_MASK_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SIUL2_H */
