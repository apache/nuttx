/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mzec_features.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZEC_FEATURES_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZEC_FEATURES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register/Flash Offsets ***************************************************/

/* Device ID, Revision, and Configuration (SFR PIC32MZ_CONFIG_K1BASE) */

#define PIC32MZ_CFGCON_OFFSET    0x0000 /* Configuration control register */
#define PIC32MZ_DEVID_OFFSET     0x0020 /* Device ID and revision register */
#define PIC32MZ_SYSKEY_OFFSET    0x0030 /* System key register */
#define PIC32MZ_CFGEBIA_OFFSET   0x00c0 /* External bus interface address pin configuration register */
#define PIC32MZ_CFGEBIC_OFFSET   0x00d0 /* External bus interface address pin control register */
#define PIC32MZ_CFGPG_OFFSET     0x00e0 /* Permission group configuration register */

/* Alternate Device Configuration (Boot Flash PIC32MZ_BOOTCFG_K1BASE) */

#define PIC32MZ_ADEVCFG3_OFFSET  0x0000 /* Alternate device configuration word 3 */
#define PIC32MZ_ADEVCFG2_OFFSET  0x0004 /* Alternate device configuration word 2 */
#define PIC32MZ_ADEVCFG1_OFFSET  0x0008 /* Alternate device configuration word 1 */
#define PIC32MZ_ADEVCFG0_OFFSET  0x000c /* Alternate device configuration word 0 */
#define PIC32MZ_ADEVCP3_OFFSET   0x0010 /* Alternate device code protect word 3 */
#define PIC32MZ_ADEVCP2_OFFSET   0x0014 /* Alternate device code protect word 2 */
#define PIC32MZ_ADEVCP1_OFFSET   0x0018 /* Alternate device code protect word 1 */
#define PIC32MZ_ADEVCP0_OFFSET   0x001c /* Alternate device code protect word 0 */
#define PIC32MZ_ADEVSIGN3_OFFSET 0x0020 /* Alternate device signature word 3 */
#define PIC32MZ_ADEVSIGN2_OFFSET 0x0024 /* Alternate device signature word 2 */
#define PIC32MZ_ADEVSIGN1_OFFSET 0x0028 /* Alternate device signature word 1 */
#define PIC32MZ_ADEVSIGN0_OFFSET 0x002c /* Alternate device signature word 0 */

/* Device Configuration (Boot Flash PIC32MZ_BOOTCFG_K1BASE) */

#define PIC32MZ_DEVCFG3_OFFSET   0x0080 /* Device configuration word 3 */
#define PIC32MZ_DEVCFG2_OFFSET   0x0084 /* Device configuration word 2 */
#define PIC32MZ_DEVCFG1_OFFSET   0x0088 /* Device configuration word 1 */
#define PIC32MZ_DEVCFG0_OFFSET   0x008c /* Device configuration word 0 */
#define PIC32MZ_DEVCP3_OFFSET    0x0090 /* Device code protect word 3 */
#define PIC32MZ_DEVCP2_OFFSET    0x0094 /* Device code protect word 2 */
#define PIC32MZ_DEVCP1_OFFSET    0x0098 /* Device code protect word 1 */
#define PIC32MZ_DEVCP0_OFFSET    0x009c /* Device code protect word 0 */
#define PIC32MZ_DEVSIGN3_OFFSET  0x00a0 /* Device signature word 3 */
#define PIC32MZ_DEVSIGN2_OFFSET  0x00a4 /* Device signature word 2 */
#define PIC32MZ_DEVSIGN1_OFFSET  0x00a8 /* Device signature word 1 */
#define PIC32MZ_DEVSIGN0_OFFSET  0x00ac /* Device signature word 0 */

/* Device ADC Calibration (Boot Flash PIC32MZ_ADCCALIB_K1BASE) */

#define PIC32MZ_DEVADC1_OFFSET   0x0000 /* ADC1 Calibration */
#define PIC32MZ_DEVADC2_OFFSET   0x0004 /* ADC2 Calibration */
#define PIC32MZ_DEVADC3_OFFSET   0x0008 /* ADC3 Calibration */
#define PIC32MZ_DEVADC4_OFFSET   0x000c /* ADC4 Calibration */
#define PIC32MZ_DEVADC5_OFFSET   0x0010 /* ADC5 Calibration */

/* Device Serial Number (Boot Flash PIC32MZ_DEVSN_K1BASE) */

#define PIC32MZ_DEVSN0_OFFSET    0x0000 /* Device serial number 0 */
#define PIC32MZ_DEVSN1_OFFSET    0x0004 /* Device serial number 1 */

/* Register/Flash Addresses *************************************************/

/* Device ID, Revision, and Configuration (SFR PIC32MZ_CONFIG_K1BASE) */

#define PIC32MZ_CFGCON           (PIC32MZ_CONFIG_K1BASE+PIC32MZ_CFGCON_OFFSET)
#define PIC32MZ_DEVID            (PIC32MZ_CONFIG_K1BASE+PIC32MZ_DEVID_OFFSET)
#define PIC32MZ_SYSKEY           (PIC32MZ_CONFIG_K1BASE+PIC32MZ_SYSKEY_OFFSET)
#define PIC32MZ_CFGEBIA          (PIC32MZ_CONFIG_K1BASE+PIC32MZ_CFGEBIA_OFFSET)
#define PIC32MZ_CFGEBIC          (PIC32MZ_CONFIG_K1BASE+PIC32MZ_CFGEBIC_OFFSET)
#define PIC32MZ_CFGPG            (PIC32MZ_CONFIG_K1BASE+PIC32MZ_CFGPG_OFFSET)

/* Alternate Device Configuration (Boot Flash PIC32MZ_BOOTCFG_K1BASE) */

#define PIC32MZ_ADEVCFG3         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCFG3_OFFSET)
#define PIC32MZ_ADEVCFG2         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCFG2_OFFSET)
#define PIC32MZ_ADEVCFG1         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCFG1_OFFSET)
#define PIC32MZ_ADEVCFG0         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCFG0_OFFSET)
#define PIC32MZ_ADEVCP3          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCP3_OFFSET)
#define PIC32MZ_ADEVCP2          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCP2_OFFSET)
#define PIC32MZ_ADEVCP1          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCP1_OFFSET)
#define PIC32MZ_ADEVCP0          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCP0_OFFSET)
#define PIC32MZ_ADEVSIGN3        (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVSIGN3_OFFSET)
#define PIC32MZ_ADEVSIGN2        (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVSIGN2_OFFSET)
#define PIC32MZ_ADEVSIGN1        (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVSIGN1_OFFSET)
#define PIC32MZ_ADEVSIGN0        (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVSIGN0_OFFSET)

/* Device Configuration (Boot Flash PIC32MZ_BOOTCFG_K1BASE) */

#define PIC32MZ_DEVCFG3          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCFG3_OFFSET)
#define PIC32MZ_DEVCFG2          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCFG2_OFFSET)
#define PIC32MZ_DEVCFG1          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCFG1_OFFSET)
#define PIC32MZ_DEVCFG0          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCFG0_OFFSET)
#define PIC32MZ_DEVCP3           (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCP3_OFFSET)
#define PIC32MZ_DEVCP2           (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCP2_OFFSET)
#define PIC32MZ_DEVCP1           (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCP1_OFFSET)
#define PIC32MZ_DEVCP0           (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCP0_OFFSET)
#define PIC32MZ_DEVSIGN3         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVSIGN3_OFFSET)
#define PIC32MZ_DEVSIGN2         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVSIGN2_OFFSET)
#define PIC32MZ_DEVSIGN1         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVSIGN1_OFFSET)
#define PIC32MZ_DEVSIGN0         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVSIGN0_OFFSET)

/* Device ADC Calibration (Boot Flash PIC32MZ_ADCCALIB_K1BASE) */

#define PIC32MZ_DEVADC1          (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVADC1_OFFSET)
#define PIC32MZ_DEVADC2          (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVADC2_OFFSET)
#define PIC32MZ_DEVADC3          (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVADC3_OFFSET)
#define PIC32MZ_DEVADC4          (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVADC4_OFFSET)
#define PIC32MZ_DEVADC5          (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVADC5_OFFSET)

/* Device Serial Number
 * (Boot Flash PIC32MZ_DEVSN_K1BASEPIC32MZ_DEVSN_K1BASE)
 */

#define PIC32MZ_DEVSN0           (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVSN0_OFFSET)
#define PIC32MZ_DEVSN1           (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVSN1_OFFSET)

/* Register/Flash Bit Field Definitions *************************************/

/* Device ID, Revision, and Configuration (SFR PIC32MZ_CONFIG_K1BASE) */

/* Configuration control register
 *
 * NOTE:
 * To change many of the bits in the register, the unlock sequence must first
 * be performed.
 */

#define CFGCON_TDOEN             (1 << 0)  /* Bit 0:  TDO Enable for 2-Wire JTAG */
#define CFGCON_TROEN             (1 << 2)  /* Bit 2:  Trace Output Enable bit */
#define CFGCON_JTAGEN            (1 << 3)  /* Bit 3:  JTAG Port Enable bit */
#define CFGCON_ECCCON_SHIFT      (4)       /* Bits 4-5: Flash ECC Configuration bits */
#define CFGCON_ECCCON_MASK       (7 << CFGCON_ECCCON_SHIFT)
#  define CFGCON_ECCCON_ECC      (0 << CFGCON_ECCCON_SHIFT) /* Flash ECC enabled */
#  define CFGCON_ECCCON_DYNECC   (1 << CFGCON_ECCCON_SHIFT) /* Dynamic Flash ECC enabled */
#  define CFGCON_ECCCON_DISLCK   (2 << CFGCON_ECCCON_SHIFT) /* ECC / dynamic ECC disabled (locked) */
#  define CFGCON_ECCCON_DISWR    (3 << CFGCON_ECCCON_SHIFT) /* ECC / dynamic ECC disabled (writable) */

#define CFGCON_USBSSEN           (1 << 8)  /* Bit 8:  USB Suspend Sleep Enable bit */
#define CFGCON_PGLOCK            (1 << 11) /* Bit 11: Permission Group Lock bit */
#define CFGCON_PMDLOCK           (1 << 12) /* Bit 12: Peripheral Module Disable bit */
#define CFGCON_IOLOCK            (1 << 13) /* Bit 13: Peripheral Pin Select Lock bit */
#define CFGCON_OCACLK            (1 << 16) /* Bit 16: Output Compare Alternate Clock Selection bit */
#define CFGCON_ICACLK            (1 << 17) /* Bit 17: Input Capture Alternate Clock Selection bit */
#define CFGCON_CPUPRI            (1 << 24) /* Bit 24: CPU Arbitration Priority to SRAM */
#define CFGCON_DMAPRI            (1 << 25) /* Bit 25: DMA Read and DMA Write Arbitration Priority */

/* Device ID and revision register */

#define DEVID_SHIFT              (0)       /* Bits 0-27: Device ID */
#define DEVID_MASK               (0x0ffffff << DEVID_SHIFT)
#define DEVID_VER_SHIFT          (28)      /* Bits 28-31: Revision Identifier bits */
#define DEVID_VER_MASK           (15 << DEVID_VER_SHIFT)

/* System key register: 32-bit key value */

#define UNLOCK_SYSKEY_0          (0xaa996655ul)
#define UNLOCK_SYSKEY_1          (0x556699aaul)

/* External bus interface address pin configuration register */

#define CFGEBIA_EBIA0N_SHIFT     (0)       /* Bits 0-23: EBI address pin 0 enable */
#define CFGEBIA_EBIA0N_MASK      (0x00ffffff << CFGEBIA_EBIA0N_SHIFT)
#  define CFGEBIA_EBIA0EN        (1 << 0)  /* Bit 0:  EBI address pin 0 enable */
#  define CFGEBIA_EBIA1EN        (1 << 1)  /* Bit 1:  EBI address pin 1 enable */
#  define CFGEBIA_EBIA2EN        (1 << 2)  /* Bit 2:  EBI address pin 2 enable */
#  define CFGEBIA_EBIA3EN        (1 << 3)  /* Bit 3:  EBI address pin 3 enable */
#  define CFGEBIA_EBIA4EN        (1 << 4)  /* Bit 4:  EBI address pin 4 enable */
#  define CFGEBIA_EBIA5EN        (1 << 5)  /* Bit 5:  EBI address pin 5 enable */
#  define CFGEBIA_EBIA6EN        (1 << 6)  /* Bit 6:  EBI address pin 6 enable */
#  define CFGEBIA_EBIA7EN        (1 << 7)  /* Bit 7:  EBI address pin 7 enable */
#  define CFGEBIA_EBIA8EN        (1 << 8)  /* Bit 8:  EBI address pin 8 enable */
#  define CFGEBIA_EBIA9EN        (1 << 9)  /* Bit 9:  EBI address pin 9 enable */
#  define CFGEBIA_EBIA10EN       (1 << 10) /* Bit 10: EBI address pin 10 enable */
#  define CFGEBIA_EBIA11EN       (1 << 11) /* Bit 11: EBI address pin 11 enable */
#  define CFGEBIA_EBIA12EN       (1 << 12) /* Bit 12: EBI address pin 12 enable */
#  define CFGEBIA_EBIA13EN       (1 << 13) /* Bit 13: EBI address pin 13 enable */
#  define CFGEBIA_EBIA14EN       (1 << 14) /* Bit 14: EBI address pin 14 enable */
#  define CFGEBIA_EBIA15EN       (1 << 15) /* Bit 15: EBI address pin 15 enable */
#  define CFGEBIA_EBIA16EN       (1 << 16) /* Bit 16: EBI address pin 16 enable */
#  define CFGEBIA_EBIA17EN       (1 << 17) /* Bit 17: EBI address pin 17 enable */
#  define CFGEBIA_EBIA18EN       (1 << 18) /* Bit 18: EBI address pin 18 enable */
#  define CFGEBIA_EBIA19EN       (1 << 19) /* Bit 19: EBI address pin 19 enable */
#  define CFGEBIA_EBIA20EN       (1 << 20) /* Bit 20: EBI address pin 20 enable */
#  define CFGEBIA_EBIA21EN       (1 << 21) /* Bit 21: EBI address pin 21 enable */
#  define CFGEBIA_EBIA22EN       (1 << 22) /* Bit 22: EBI address pin 22 enable */
#  define CFGEBIA_EBIA23EN       (1 << 23) /* Bit 23: EBI address pin 23 enable */
#define CFGEBIA_EBIPINEN         (1 << 31) /* Bit 31: EBI Pin Enable bit */

/* External bus interface address pin control register */

#define CFGEBIC_EBIRPEN          (1 << 16) /* Bit 16: EBIRP Pin Sensitivity Control bit */
#define CFGEBIC_EBIRDYLVL        (1 << 17) /* Bit 17: EBIRDYx Pin Sensitivity Control bit */
#define CFGEBIC_EBIRDYEN1        (1 << 24) /* Bit 24: EBIRDY1 Pin Enable bit */
#define CFGEBIC_EBIRDYEN2        (1 << 25) /* Bit 25: EBIRDY2 Pin Enable bit */
#define CFGEBIC_EBIRDYEN3        (1 << 26) /* Bit 26: EBIRDY3 Pin Enable bit */
#define CFGEBIC_EBIRDYINV1       (1 << 28) /* Bit 28: EBIRDY1 Inversion Control bit */
#define CFGEBIC_EBIRDYINV2       (1 << 29) /* Bit 29: EBIRDY2 Inversion Control bit */
#define CFGEBIC_EBIRDYINV3       (1 << 30) /* Bit 30: EBIRDY3 Inversion Control bit */

/* Permission group configuration register */

#define CFGPG_GROUP0             0         /* Initiator is assigned to Permission Group 0 */
#define CFGPG_GROUP1             1         /* Initiator is assigned to Permission Group 1 */
#define CFGPG_GROUP2             2         /* Initiator is assigned to Permission Group 2 */
#define CFGPG_GROUP3             3         /* Initiator is assigned to Permission Group 3 */

#define CFGPG_CPUPG_SHIFT        (0)       /* Bits 0-1: CPU Permission Group bits */
#define CFGPG_CPUPG_MASK         (3 << CFGPG_CPUPG_SHIFT)
#  define CFGPG_CPUPG(n)         ((uint32_t)(n) << CFGPG_CPUPG_SHIFT)
#  define CFGPG_CPUPG_GROUP0     CFGPG_CPUPG(CFGPG_GROUP0)
#  define CFGPG_CPUPG_GROUP1     CFGPG_CPUPG(CFGPG_GROUP1)
#  define CFGPG_CPUPG_GROUP2     CFGPG_CPUPG(CFGPG_GROUP2)
#  define CFGPG_CPUPG_GROUP3     CFGPG_CPUPG(CFGPG_GROUP3)
#define CFGPG_DMAPG_SHIFT        (4)       /* Bits 4-5: DMA Module Permission Group bits */
#define CFGPG_DMAPG_MASK         (3 << CFGPG_DMAPG_SHIFT)
#  define CFGPG_DMAPG(n)         ((uint32_t)(n) << CFGPG_DMAPG_SHIFT)
#  define CFGPG_DMAPG_GROUP0     CFGPG_DMAPG(CFGPG_GROUP0)
#  define CFGPG_DMAPG_GROUP1     CFGPG_DMAPG(CFGPG_GROUP1)
#  define CFGPG_DMAPG_GROUP2     CFGPG_DMAPG(CFGPG_GROUP2)
#  define CFGPG_DMAPG_GROUP3     CFGPG_DMAPG(CFGPG_GROUP3)
#define CFGPG_USBPG_SHIFT        (8)       /*  Bits 8-9: USB Module Permission Group bits */
#define CFGPG_USBPG_MASK         (3 << CFGPG_USBPG_SHIFT)
#  define CFGPG_USBPG(n)         ((uint32_t)(n) << CFGPG_USBPG_SHIFT)
#  define CFGPG_USBPG_GROUP0     CFGPG_USBPG(CFGPG_GROUP0)
#  define CFGPG_USBPG_GROUP1     CFGPG_USBPG(CFGPG_GROUP1)
#  define CFGPG_USBPG_GROUP2     CFGPG_USBPG(CFGPG_GROUP2)
#  define CFGPG_USBPG_GROUP3     CFGPG_USBPG(CFGPG_GROUP3)
#define CFGPG_CAN1PG_SHIFT       (12)      /* Bits 12-13: CAN1 Module Permission Group bits */
#define CFGPG_CAN1PG_MASK        (3 << CFGPG_CAN1PG_SHIFT)
#  define CFGPG_CAN1PG(n)        ((uint32_t)(n) << CFGPG_CAN1PG_SHIFT)
#  define CFGPG_CAN1PG_GROUP0    CFGPG_CAN1PG(CFGPG_GROUP0)
#  define CFGPG_CAN1PG_GROUP1    CFGPG_CAN1PG(CFGPG_GROUP1)
#  define CFGPG_CAN1PG_GROUP2    CFGPG_CAN1PG(CFGPG_GROUP2)
#  define CFGPG_CAN1PG_GROUP3    CFGPG_CAN1PG(CFGPG_GROUP3)
#define CFGPG_CAN2PG_SHIFT       (14)      /* Bits 14-15: CAN2 Module Permission Group bits */
#define CFGPG_CAN2PG_MASK        (3 << CFGPG_CAN2PG_SHIFT)
#  define CFGPG_CAN2PG(n)        ((uint32_t)(n) << CFGPG_CAN2PG_SHIFT)
#  define CFGPG_CAN2PG_GROUP0    CFGPG_CAN2PG(CFGPG_GROUP0)
#  define CFGPG_CAN2PG_GROUP1    CFGPG_CAN2PG(CFGPG_GROUP1)
#  define CFGPG_CAN2PG_GROUP2    CFGPG_CAN2PG(CFGPG_GROUP2)
#  define CFGPG_CAN2PG_GROUP3    CFGPG_CAN2PG(CFGPG_GROUP3)
#define CFGPG_ETHPG_SHIFT        (16)      /* Bits 16-17: Ethernet Module Permission Group bits */
#define CFGPG_ETHPG_MASK         (3 << CFGPG_ETHPG_SHIFT)
#  define CFGPG_ETHPG(n)         ((uint32_t)(n) << CFGPG_ETHPG_SHIFT)
#  define CFGPG_ETHPG_GROUP0     CFGPG_ETHPG(CFGPG_GROUP0)
#  define CFGPG_ETHPG_GROUP1     CFGPG_ETHPG(CFGPG_GROUP1)
#  define CFGPG_ETHPG_GROUP2     CFGPG_ETHPG(CFGPG_GROUP2)
#  define CFGPG_ETHPG_GROUP3     CFGPG_ETHPG(CFGPG_GROUP3)
#define CFGPG_SQI1PG_SHIFT       (20)      /* Bits 20-21: SQI Module Permission Group bits */
#define CFGPG_SQI1PG_MASK        (3 << CFGPG_SQI1PG_SHIFT)
#  define CFGPG_SQI1PG(n)        ((uint32_t)(n) << CFGPG_SQI1PG_SHIFT)
#  define CFGPG_SQI1PG_GROUP0    CFGPG_SQI1PG(CFGPG_GROUP0)
#  define CFGPG_SQI1PG_GROUP1    CFGPG_SQI1PG(CFGPG_GROUP1)
#  define CFGPG_SQI1PG_GROUP2    CFGPG_SQI1PG(CFGPG_GROUP2)
#  define CFGPG_SQI1PG_GROUP3    CFGPG_SQI1PG(CFGPG_GROUP3)
#define CFGPG_FCPG_SHIFT         (22)      /* Bits 22-23: Flash Control Permission Group bits */
#define CFGPG_FCPG_MASK          (3 << CFGPG_FCPG_SHIFT)
#  define CFGPG_FCPG(n)          ((uint32_t)(n) << CFGPG_FCPG_SHIFT)
#  define CFGPG_FCPG_GROUP0      CFGPG_FCPG(CFGPG_GROUP0)
#  define CFGPG_FCPG_GROUP1      CFGPG_FCPG(CFGPG_GROUP1)
#  define CFGPG_FCPG_GROUP2      CFGPG_FCPG(CFGPG_GROUP2)
#  define CFGPG_FCPG_GROUP3      CFGPG_FCPG(CFGPG_GROUP3)
#define CFGPG_CRYPTPG_SHIFT      (24)      /* Bits 24-25: Crypto Engine Permission Group bits */
#define CFGPG_CRYPTPG_MASK       (3 << CFGPG_CRYPTPG_SHIFT)
#  define CFGPG_CRYPTPG(n)       ((uint32_t)(n) << CFGPG_CRYPTPG_SHIFT)
#  define CFGPG_CRYPTPG_GROUP0   CFGPG_CRYPTPG(CFGPG_GROUP0)
#  define CFGPG_CRYPTPG_GROUP1   CFGPG_CRYPTPG(CFGPG_GROUP1)
#  define CFGPG_CRYPTPG_GROUP2   CFGPG_CRYPTPG(CFGPG_GROUP2)
#  define CFGPG_CRYPTPG_GROUP3   CFGPG_CRYPTPG(CFGPG_GROUP3)

/* Alternate Device Configuration (Boot Flash PIC32MZ_BOOTCFG_K1BASE) */

/* Device Configuration (Boot Flash PIC32MZ_BOOTCFG_K1BASE) */

/* Device configuration word 3 / Alternate device configuration word 3 */

#define DEVCFG3_USERID_SHIFT     (0)       /* Bit 0-15: 16-bit user defined value */
#define DEVCFG3_USERID_MASK      (0xffff << DEVCFG3_USERID_SHIFT)
#  define DEVCFG3_USERID(n)      ((uint32_t)(n) << DEVCFG3_USERID_SHIFT)
#define DEVCFG3_FMIIEN_SHIFT     (24)      /* Bit 24: Ethernet MII Enable Configuration bit */
#define DEVCFG3_FMIIEN           (1 << 24) /* Bit 24: Ethernet MII Enable Configuration bit */
#define DEVCFG3_FETHIO_SHIFT     (25)      /* Bit 25: Ethernet I/O Pin Selection Configuration bit */
#define DEVCFG3_FETHIO           (1 << 25) /* Bit 25: Ethernet I/O Pin Selection Configuration bit */
#define DEVCFG3_PGL1WAY_SHIFT    (27)      /* Bit 27: Permission Group Lock One Way Configuration bit */
#define DEVCFG3_PGL1WAY          (1 << 27) /* Bit 27: Permission Group Lock One Way Configuration bit */
#define DEVCFG3_PMDL1WAY_SHIFT   (28)      /* Bit 28: Peripheral Module Disable Configuration bit */
#define DEVCFG3_PMDL1WAY         (1 << 28) /* Bit 28: Peripheral Module Disable Configuration bit */
#define DEVCFG3_IOL1WAY_SHIFT    (29)      /* Bit 29: Peripheral Pin Select Configuration bit */
#define DEVCFG3_IOL1WAY          (1 << 29) /* Bit 29: Peripheral Pin Select Configuration bit */
#define DEVCFG3_FUSBIDIO_SHIFT   (30)      /* Bit 30: USB USBID Selection bit */
#define DEVCFG3_FUSBIDIO         (1 << 30) /* Bit 30: USB USBID Selection bit */

#define DEVCFG3_RWO              0x84ff0000 /* Bits 16-23, 31: Reserved, write as one */

/* Device configuration word 2 / Alternate device configuration word 2 */

#define DEVCFG2_FPLLIDIV_SHIFT   (0)        /* Bits 0-2: PLL Input Divider bits */
#define DEVCFG2_FPLLIDIV_MASK    (7 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV(n)    ((uint32_t)((n)-1) << DEVCFG2_FPLLIDIV_SHIFT) /* n=1..8 */

#  define DEVCFG2_FPLLIDIV_1     (0 << DEVCFG2_FPLLIDIV_SHIFT) /* Divide by 1 */
#  define DEVCFG2_FPLLIDIV_2     (1 << DEVCFG2_FPLLIDIV_SHIFT) /* Divide by 2 */
#  define DEVCFG2_FPLLIDIV_3     (2 << DEVCFG2_FPLLIDIV_SHIFT) /* Divide by 3 */
#  define DEVCFG2_FPLLIDIV_4     (3 << DEVCFG2_FPLLIDIV_SHIFT) /* Divide by 4 */
#  define DEVCFG2_FPLLIDIV_5     (4 << DEVCFG2_FPLLIDIV_SHIFT) /* Divide by 5 */
#  define DEVCFG2_FPLLIDIV_6     (5 << DEVCFG2_FPLLIDIV_SHIFT) /* Divide by 6 */
#  define DEVCFG2_FPLLIDIV_7     (6 << DEVCFG2_FPLLIDIV_SHIFT) /* Divide by 7 */
#  define DEVCFG2_FPLLIDIV_8     (7 << DEVCFG2_FPLLIDIV_SHIFT) /* Divide by 8 */

#define DEVCFG2_FPLLRNG_SHIFT    (4)       /* Bits 4-6: System PLL Divided Input Clock Frequency Range bits */
#define DEVCFG2_FPLLRNG_MASK     (7 << DEVCFG2_FPLLRNG_SHIFT)
#  define DEVCFG2_FPLLRNG_BYPASS    (0 << DEVCFG2_FPLLRNG_SHIFT) /* Bypass */
#  define DEVCFG2_FPLLRNG_5_10MHZ   (1 << DEVCFG2_FPLLRNG_SHIFT) /* 5-10 MHz */
#  define DEVCFG2_FPLLRNG_8_16MHZ   (2 << DEVCFG2_FPLLRNG_SHIFT) /* 8-16 MHz */
#  define DEVCFG2_FPLLRNG_13_26MHZ  (3 << DEVCFG2_FPLLRNG_SHIFT) /* 13-26 MHz */
#  define DEVCFG2_FPLLRNG_21_42MHZ  (4 << DEVCFG2_FPLLRNG_SHIFT) /* 21-42 MHz */
#  define DEVCFG2_FPLLRNG_34_64MHZ  (5 << DEVCFG2_FPLLRNG_SHIFT) /* 34-64 MHz */

#define DEVCFG2_FPLLICLK         (1 << 7)  /* Bit 7: System PLL Input Clock Select bit */
#define DEVCFG2_FPLLMULT_SHIFT   (8)       /* Bits 8-14: System PLL Feedback Divider bits */
#define DEVCFG2_FPLLMULT_MASK    (0x7f << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT(n)    ((uint32_t)((n)-1) << DEVCFG2_FPLLMULT_SHIFT) /* n=1..128 */

#define DEVCFG2_FPLLODIV_SHIFT   (16)      /* Bits 16-18: Default System PLL Output Divisor bits */
#define DEVCFG2_FPLLODIV_MASK    (7 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_2     (1 << DEVCFG2_FPLLODIV_SHIFT) /* PLL output divided by 2 */
#  define DEVCFG2_FPLLODIV_4     (2 << DEVCFG2_FPLLODIV_SHIFT) /* PLL output divided by 4 */
#  define DEVCFG2_FPLLODIV_8     (3 << DEVCFG2_FPLLODIV_SHIFT) /* PLL output divided by 8 */
#  define DEVCFG2_FPLLODIV_16    (4 << DEVCFG2_FPLLODIV_SHIFT) /* PLL output divided by 16 */
#  define DEVCFG2_FPLLODIV_32    (5 << DEVCFG2_FPLLODIV_SHIFT) /* PLL output divided by 32 */

#define DEVCFG2_UPLLFSEL         (1 << 30) /* Bit 30: USB PLL Input Frequency Select bit */
#  define DEVCFG2_UPLLFSEL_24MHZ (1 << 30) /*   Bit 30=1: UPLL input clock is 24 MHz */
#  define DEVCFG2_UPLLFSEL_12MHZ (0 << 30) /*   Bit 30=0: UPLL input clock is 12 MHz */

#define DEVCFG2_RWO 0xbff88008 /* Bits 3, 15, 19-29, 31: Reserved, write as one */

/* Device configuration word 1 / Alternate device configuration word 1 */

#define DEVCFG1_FNOSC_SHIFT      (0)       /* Bits 0-2: Oscillator Selection bits */
#define DEVCFG1_FNOSC_MASK       (7 << DEVCFG1_FNOSC_SHIFT)
#  define DEVCFG1_FNOSC_FRC      (0 << DEVCFG1_FNOSC_SHIFT) /* FRC divided by FRCDIV */
#  define DEVCFG1_FNOSC_SPLL     (1 << DEVCFG1_FNOSC_SHIFT) /* SPLL */
#  define DEVCFG1_FNOSC_POSC     (2 << DEVCFG1_FNOSC_SHIFT) /* POSC (HS, EC) */
#  define DEVCFG1_FNOSC_SOSC     (4 << DEVCFG1_FNOSC_SHIFT) /* SOSC */
#  define DEVCFG1_FNOSC_LPRC     (5 << DEVCFG1_FNOSC_SHIFT) /* LPRC */
#  define DEVCFG1_FNOSC_FRCDIV   (7 << DEVCFG1_FNOSC_SHIFT) /* FRC divided by FRCDIV<2:0> */

#define DEVCFG1_DMTINV_SHIFT     (3)       /* Bits 3-5: Deadman Timer Count Window Interval bits */
#define DEVCFG1_DMTINV_MASK      (7 << DEVCFG1_DMTINV_SHIFT)
#  define DEVCFG1_DMTINV_0       (0 << DEVCFG1_DMTINV_SHIFT) /* Window/Interval value zero */
#  define DEVCFG1_DMTINV_1_2     (1 << DEVCFG1_DMTINV_SHIFT) /* Window/Interval value 1/2 counter */
#  define DEVCFG1_DMTINV_3_4     (2 << DEVCFG1_DMTINV_SHIFT) /* Window/Interval value 3/4 counter */
#  define DEVCFG1_DMTINV_7_8     (3 << DEVCFG1_DMTINV_SHIFT) /* Window/Interval value 7/8 counter */
#  define DEVCFG1_DMTINV_15_16   (4 << DEVCFG1_DMTINV_SHIFT) /* Window/Interval value 15/16 counter */
#  define DEVCFG1_DMTINV_31_32   (5 << DEVCFG1_DMTINV_SHIFT) /* Window/Interval value 31/32 counter */
#  define DEVCFG1_DMTINV_63_64   (6 << DEVCFG1_DMTINV_SHIFT) /* Window/Interval value 63/64 counter */
#  define DEVCFG1_DMTINV_127_128 (7 << DEVCFG1_DMTINV_SHIFT) /* Window/Interval value 127/128 counter */

#define DEVCFG1_FSOSCEN          (1 << 6)  /* Bit 6:  Secondary Oscillator Enable bit */
#define DEVCFG1_IESO             (1 << 7)  /* Bit 7:  Internal External Switchover bit */
#define DEVCFG1_POSCMOD_SHIFT    (8)       /* Bits 8-9: Primary Oscillator Configuration bits */
#define DEVCFG1_POSCMOD_MASK     (3 << DEVCFG1_POSCMOD_SHIFT)
#  define DEVCFG1_POSCMOD_EC     (0 << DEVCFG1_POSCMOD_SHIFT) /* EC mode selected */
#  define DEVCFG1_POSCMOD_HS     (2 << DEVCFG1_POSCMOD_SHIFT) /* HS Oscillator mode selected */
#  define DEVCFG1_POSCMOD_DIS    (3 << DEVCFG1_POSCMOD_SHIFT) /* POSC disabled */

#define DEVCFG1_OSCIOFNC         (1 << 10) /* Bit 10: CLKO Enable Configuration bit */
#define DEVCFG1_FCKSM_SHIFT      (14)      /* Bits 14-15: Clock Switching and Monitoring Selection */
#define DEVCFG1_FCKSM_MASK       (3 << DEVCFG1_FCKSM_SHIFT)
#  define DEVCFG1_FCKSM_NONE     (0 << DEVCFG1_FCKSM_SHIFT) /* Clock switching/monitoring disabled */
#  define DEVCFG1_FCKSM_SWITCH   (1 << DEVCFG1_FCKSM_SHIFT) /* Clock switching enabled */
#  define DEVCFG1_FCKSM_MONITOR  (2 << DEVCFG1_FCKSM_SHIFT) /* Clock monitoring enabled */
#  define DEVCFG1_FCKSM_BOTH     (3 << DEVCFG1_FCKSM_SHIFT) /* Clock switching/monitoring enabled */

#define DEVCFG1_WDTPS_SHIFT      (16)      /* Bits 16-20: Watchdog Timer Postscale Select bits */
#define DEVCFG1_WDTPS_MASK       (31 << DEVCFG1_WDTPS_SHIFT)
#  define DEVCFG1_WDTPS_1        (0 << DEVCFG1_WDTPS_SHIFT)  /* 1:1 */
#  define DEVCFG1_WDTPS_2        (1 << DEVCFG1_WDTPS_SHIFT)  /* 1:2 */
#  define DEVCFG1_WDTPS_4        (2 << DEVCFG1_WDTPS_SHIFT)  /* 1:4 */
#  define DEVCFG1_WDTPS_8        (3 << DEVCFG1_WDTPS_SHIFT)  /* 1:8 */
#  define DEVCFG1_WDTPS_16       (4 << DEVCFG1_WDTPS_SHIFT)  /* 1:16 */
#  define DEVCFG1_WDTPS_32       (5 << DEVCFG1_WDTPS_SHIFT)  /* 1:32 */
#  define DEVCFG1_WDTPS_64       (6 << DEVCFG1_WDTPS_SHIFT)  /* 1:64 */
#  define DEVCFG1_WDTPS_128      (7 << DEVCFG1_WDTPS_SHIFT)  /* 1:128 */
#  define DEVCFG1_WDTPS_256      (8 << DEVCFG1_WDTPS_SHIFT)  /* 1:256 */
#  define DEVCFG1_WDTPS_512      (9 << DEVCFG1_WDTPS_SHIFT)  /* 1:512 */
#  define DEVCFG1_WDTPS_1024     (10 << DEVCFG1_WDTPS_SHIFT) /* 1:1024 */
#  define DEVCFG1_WDTPS_2048     (11 << DEVCFG1_WDTPS_SHIFT) /* 1:2048 */
#  define DEVCFG1_WDTPS_4096     (12 << DEVCFG1_WDTPS_SHIFT) /* 1:4096 */
#  define DEVCFG1_WDTPS_8192     (13 << DEVCFG1_WDTPS_SHIFT) /* 1:8192 */
#  define DEVCFG1_WDTPS_16384    (14 << DEVCFG1_WDTPS_SHIFT) /* 1:16384 */
#  define DEVCFG1_WDTPS_32768    (15 << DEVCFG1_WDTPS_SHIFT) /* 1:32768 */
#  define DEVCFG1_WDTPS_65536    (16 << DEVCFG1_WDTPS_SHIFT) /* 1:65536 */
#  define DEVCFG1_WDTPS_131072   (17 << DEVCFG1_WDTPS_SHIFT) /* 1:131072 */
#  define DEVCFG1_WDTPS_262144   (18 << DEVCFG1_WDTPS_SHIFT) /* 1:262144 */
#  define DEVCFG1_WDTPS_524288   (19 << DEVCFG1_WDTPS_SHIFT) /* 1:524288 */
#  define DEVCFG1_WDTPS_1048576  (20 << DEVCFG1_WDTPS_SHIFT) /* 1:1048576 */

#define DEVCFG1_WDTSPGM          (1 << 21) /* Bit 21: WDT stop/run during flash programming bit */
#  define DEVCFG1_WDTSPGM_STOP   (1 << 21) /*   Bit 21=1: WDT stops during flash programming */
#  define DEVCFG1_WDTSPGM_RUN    (0 << 21) /*   Bit 21=0: WDT runs during flash programming */
#define DEVCFG1_WINDIS           (1 << 22) /* Bit 22: Watchdog Timer Window Enable bit */
#  define DEVCFG1_WDT_NORMAL     (1 << 22) /*   Bit 22=1: Watchdog normal mode */
#  define DEVCFG1_WDT_WINDOW     (0 << 22) /*   Bit 22=0: Watchdog window mode */
#define DEVCFG1_FWDTEN           (1 << 23) /* Bit 23: Watchdog Timer Enable bit */
#  define DEVCFG1_FWDT_ENABLED   (1 << 23) /*   Bit 23=1: Watchdog enabled, cannot be disabled */
#  define DEVCFG1_FWDT_DISABLED  (0 << 23) /*   Bit 23=0: Watchdog disabled, can be enabled */
#define DEVCFG1_FWDTWINSZ_SHIFT  (24)      /* Bits 24-25: Watchdog Timer Window Size bits */
#define DEVCFG1_FWDTWINSZ_MASK   (3 << DEVCFG1_FWDTWINSZ_SHIFT)
#  define DEVCFG1_FWDTWINSZ_75   (0 << DEVCFG1_FWDTWINSZ_SHIFT) /* Window size is 75% */
#  define DEVCFG1_FWDTWINSZ_50   (1 << DEVCFG1_FWDTWINSZ_SHIFT) /* Window size is 50% */
#  define DEVCFG1_FWDTWINSZ_37p5 (2 << DEVCFG1_FWDTWINSZ_SHIFT) /* Window size is 37.5% */
#  define DEVCFG1_FWDTWINSZ_25   (3 << DEVCFG1_FWDTWINSZ_SHIFT) /* Window size is 25% */

#define DEVCFG1_DMTCNT_SHIFT     (26)      /* Bits 26-30: Deadman Timer Count Select bits */
#define DEVCFG1_DMTCNT_MASK      (31 << DEVCFG1_DMTCNT_SHIFT)
#  define DEVCFG1_DMTCNT(n)      ((uint32_t)((n)-8) << DEVCFG1_DMTCNT_SHIFT) /* 2**n, n=8..31 */

#  define DEVCFG1_DMTCNT_MIN     (0  << DEVCFG1_DMTCNT_SHIFT) /* 2**8   (256) */
#  define DEVCFG1_DMTCNT_MAX     (23 << DEVCFG1_DMTCNT_SHIFT) /* 2**31 (2147483648) */

#define DEVCFG1_FDMTEN           (1 << 31) /* Bit 31: Deadman Timer enable bit */

#define DEVCFG1_RWO              0x00003800 /* Bits 11-13: Reserved, write as one */

/*  Device configuration word 0  / Alternate device configuration word 0 */

#define DEVCFG0_DEBUG_SHIFT      (0)       /* Bits 0-1: Background Debugger Enable bits */
#define DEVCFG0_DEBUG_MASK       (3 << DEVCFG0_DEBUG_SHIFT)
#  define DEVCFG0_DEBUG_ENABLED  (2 << DEVCFG0_DEBUG_SHIFT) /* Debugger is enabled */
#  define DEVCFG0_DEBUG_DISABLED (3 << DEVCFG0_DEBUG_SHIFT) /* Debugger is disabled */

#define DEVCFG0_JTAGEN           (1 << 2)  /* Bit 2: JTAG Enable bit(1) */
#define DEVCFG0_ICESEL_SHIFT     (3)       /* Bits 3-4: ICE Communication Channel Select bits */
#define DEVCFG0_ICESEL_MASK      (3 << DEVCFG0_ICESEL_SHIFT)
#  define DEVCFG0_ICESEL_1       (3 << DEVCFG0_ICESEL_SHIFT) /* PGEC1/PGED1 pair is used */
#  define DEVCFG0_ICESEL_2       (2 << DEVCFG0_ICESEL_SHIFT) /* PGEC2/PGED2 pair is used */

#define DEVCFG0_TRCEN            (1 << 5)  /* Bit 5: Trace Enable bit */
#define DEVCFG0_BOOTISA          (1 << 6)  /* Bit 6: Boot ISA Selection bit */
#  define DEVCFG0_BOOT_MIPS32    (1 << 6)  /*   Bit 6=1: Boot code and Exception code is MIPS32 */
#  define DEVCFG0_BOOT_MICROMIPS (0 << 6)  /*   Bit 6=0: Boot code and Exception code is microMIPS */
#define DEVCFG0_FECCCON_SHIFT    (8)       /* Bit 8-9: Dynamic Flash ECC Configuration bits */
#define DEVCFG0_FECCCON_MASK     (3 << DEVCFG0_FECCCON_SHIFT)
#  define DEVCFG0_FECCCON_ECC    (0 << DEVCFG0_FECCCON_SHIFT) /* Flash ECC enabled (locked) */
#  define DEVCFG0_FECCCON_DYNECC (1 << DEVCFG0_FECCCON_SHIFT) /* Dynamic Flash ECC enabled (locked) */
#  define DEVCFG0_FECCCON_DISLCK (2 << DEVCFG0_FECCCON_SHIFT) /* ECC / dynamic ECC disabled (locked) */
#  define DEVCFG0_FECCCON_DISWR  (3 << DEVCFG0_FECCCON_SHIFT) /* ECC / dynamic ECC disabled (writable) */

#define DEVCFG0_FSLEEP           (1 << 10) /* Bit 10: Flash Sleep Mode bit */
#  define DEVCFG0_FSLEEP_OFF     (1 << 10) /*   Bit 10=1: Flash powered down in sleep mode */
#  define DEVCFG0_FSLEEP_ON      (0 << 10) /*   Bit 10=0: Flash powerdown controlled by VREGS bit */
#define DEVCFG0_DBGPER_SHIFT     (12)      /* Bits 12-14: Debug Mode CPU Access Permission bits */
#define DEVCFG0_DBGPER_MASK      (7 << DEVCFG0_DBGPER_SHIFT)
#  define DEVCFG0_DBGPER_GROUP0  (1 << DEVCFG0_DBGPER_SHIFT) /* Allow/deny access to group 0 regions */
#  define DEVCFG0_DBGPER_GROUP1  (2 << DEVCFG0_DBGPER_SHIFT) /* Allow/deny access to group 1 regions */
#  define DEVCFG0_DBGPER_GROUP2  (4 << DEVCFG0_DBGPER_SHIFT) /* Allow/deny access to group 2 regions */
#  define DEVCFG0_DBGPER_ALL     (7 << DEVCFG0_DBGPER_SHIFT) /* Allow/deny access to all regions */

#define DEVCFG0_EJTAGBEN         (1 << 30) /* Bit 30: EJTAG Boot Enable bit */
#  define DEVCFG0_EJTAG_NORMAL   (1 << 30) /*   Bit 30=1: Normal EJTAG functionality */
#  define DEVCFG0_EJTAG_REDUCED  (0 << 30) /*   Bit 30=0: Reduced EJTAG functionality */

#define DEVCFG0_RW0              0xbfff8880 /* Bits 7, 11, 15-29, 31: Reserved, write as one */

/* Device code protect words 1-3 / Alternate device code protect words 1-3
 *
 * The DEVCP1 through DEVCP3 and ADEVCP1 through ADEVCP3 registers are used
 * for Quad Word programming operation when programming the DEVCP0/ADEVCP0
 * registers, and do not contain any valid information.
 */

/* Device code protect word 0 / Alternate device code protect word 0 */

#define DEVCP0_CP                (1 << 28)  /* Bit 28: Code-protect bit */
#define DEVCP0_RWO               0xefffffff /* Bits 0-27, 28-31: Reserved, write as one */

/* Device signature words 1-3 / Alternate device signature words 1-3
 *
 * The DEVSIGN1 through DEVSIGN3 and ADEVSIGN1 through ADEVSIGN3 registers
 * are used for Quad Word programming operation when programming the
 * DEVSIGN0/ADESIGN0 registers, and do not contain any valid information.
 */

/* Device signature word 0 / Alternate device signature word 0 */

#define DEVSIGN0_RWZ             0x80000000 /* Bit 31: Reserved, write as zero */
#define DEVSIGN0_RWO             0x7fffffff /* Bits 0-30: Reserved, write as one */

/* Device ADC Calibration (Boot Flash PIC32MZ_ADCCALIB_K1BASE) */

/* ADC1-5 Calibration:  32-bit calibration values */

/* Device Serial Number (Boot Flash PIC32MZ_DEVSN_K1BASE) */

/* Device serial number 0-1: 32-bit serial number data */

#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZEC_FEATURES_H */
