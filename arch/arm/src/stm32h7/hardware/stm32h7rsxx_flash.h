/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7rsxx_flash.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_FLASH_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_FLASH_ACR_OFFSET              0x0000                /* Access control */
#define STM32_FLASH_KEYR_OFFSET             0x0004                /* Control unlock key */
#define STM32_FLASH_CR_OFFSET               0x0010                /* Control */
#define STM32_FLASH_SR_OFFSET               0x0014                /* Status */
#define STM32_FLASH_IER_OFFSET              0x0020                /* Interrupt enable */
#define STM32_FLASH_ISR_OFFSET              0x0024                /* Interrupt status */
#define STM32_FLASH_ICR_OFFSET              0x0028                /* Interrupt clear */
#define STM32_FLASH_CRCCR_OFFSET            0x0030                /* CRC control */
#define STM32_FLASH_CRCSADDR_OFFSET         0x0034                /* CRC start address */
#define STM32_FLASH_CRCEADDR_OFFSET         0x0038                /* CRC end address */
#define STM32_FLASH_CRCDATAR_OFFSET         0x003c                /* CRC data */
#define STM32_FLASH_ECCSFADDR_OFFSET        0x0040                /* ECC single error address */
#define STM32_FLASH_ECCDFADDR_OFFSET        0x0044                /* ECC double error address */
#define STM32_FLASH_OPTKEYR_OFFSET          0x0100                /* Option unlock key */
#define STM32_FLASH_OPTCR_OFFSET            0x0104                /* Option control */
#define STM32_FLASH_OPTISR_OFFSET           0x0108                /* Option interrupt status */
#define STM32_FLASH_OPTICR_OFFSET           0x010c                /* Option interrupt clear */
#define STM32_FLASH_OBKCR_OFFSET            0x0110                /* Option-byte key control */
#define STM32_FLASH_OBKDR_OFFSET(n)         (0x0118 + ((n) << 2)) /* Option-byte key data */
#define STM32_FLASH_NVSR_OFFSET             0x0200                /* Non-volatile state */
#define STM32_FLASH_NVSRP_OFFSET            0x0204                /* Non-volatile state program */
#define STM32_FLASH_ROTSR_OFFSET            0x0208                /* Root-of-trust state */
#define STM32_FLASH_ROTSRP_OFFSET           0x020c                /* Root-of-trust program */
#define STM32_FLASH_OTPLSR_OFFSET           0x0210                /* OTP lock state */
#define STM32_FLASH_OTPLSRP_OFFSET          0x0214                /* OTP lock program */
#define STM32_FLASH_WRPSR_OFFSET            0x0218                /* Write protection state */
#define STM32_FLASH_WRPSRP_OFFSET           0x021c                /* Write protection program */
#define STM32_FLASH_HDPSR_OFFSET            0x0230                /* Hide protection state */
#define STM32_FLASH_HDPSRP_OFFSET           0x0234                /* Hide protection program */
#define STM32_FLASH_EPOCHSR_OFFSET          0x0250                /* Epoch state */
#define STM32_FLASH_EPOCHSRP_OFFSET         0x0254                /* Epoch program */
#define STM32_FLASH_OBW1SR_OFFSET           0x0260                /* Option word 1 state */
#define STM32_FLASH_OBW1SRP_OFFSET          0x0264                /* Option word 1 program */
#define STM32_FLASH_OBW2SR_OFFSET           0x0268                /* Option word 2 state */
#define STM32_FLASH_OBW2SRP_OFFSET          0x026c                /* Option word 2 program */

/* Register Addresses *******************************************************/

#define STM32_FLASH_ACR                     (STM32_FLASHIF_BASE + STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_KEYR                    (STM32_FLASHIF_BASE + STM32_FLASH_KEYR_OFFSET)
#define STM32_FLASH_CR                      (STM32_FLASHIF_BASE + STM32_FLASH_CR_OFFSET)
#define STM32_FLASH_SR                      (STM32_FLASHIF_BASE + STM32_FLASH_SR_OFFSET)
#define STM32_FLASH_IER                     (STM32_FLASHIF_BASE + STM32_FLASH_IER_OFFSET)
#define STM32_FLASH_ISR                     (STM32_FLASHIF_BASE + STM32_FLASH_ISR_OFFSET)
#define STM32_FLASH_ICR                     (STM32_FLASHIF_BASE + STM32_FLASH_ICR_OFFSET)
#define STM32_FLASH_CRCCR                   (STM32_FLASHIF_BASE + STM32_FLASH_CRCCR_OFFSET)
#define STM32_FLASH_CRCSADDR                (STM32_FLASHIF_BASE + STM32_FLASH_CRCSADDR_OFFSET)
#define STM32_FLASH_CRCEADDR                (STM32_FLASHIF_BASE + STM32_FLASH_CRCEADDR_OFFSET)
#define STM32_FLASH_CRCDATAR                (STM32_FLASHIF_BASE + STM32_FLASH_CRCDATAR_OFFSET)
#define STM32_FLASH_ECCSFADDR               (STM32_FLASHIF_BASE + STM32_FLASH_ECCSFADDR_OFFSET)
#define STM32_FLASH_ECCDFADDR               (STM32_FLASHIF_BASE + STM32_FLASH_ECCDFADDR_OFFSET)
#define STM32_FLASH_OPTKEYR                 (STM32_FLASHIF_BASE + STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_OPTCR                   (STM32_FLASHIF_BASE + STM32_FLASH_OPTCR_OFFSET)
#define STM32_FLASH_OPTISR                  (STM32_FLASHIF_BASE + STM32_FLASH_OPTISR_OFFSET)
#define STM32_FLASH_OPTICR                  (STM32_FLASHIF_BASE + STM32_FLASH_OPTICR_OFFSET)
#define STM32_FLASH_OBKCR                   (STM32_FLASHIF_BASE + STM32_FLASH_OBKCR_OFFSET)
#define STM32_FLASH_OBKDR(n)                (STM32_FLASHIF_BASE + STM32_FLASH_OBKDR_OFFSET(n))
#define STM32_FLASH_NVSR                    (STM32_FLASHIF_BASE + STM32_FLASH_NVSR_OFFSET)
#define STM32_FLASH_NVSRP                   (STM32_FLASHIF_BASE + STM32_FLASH_NVSRP_OFFSET)
#define STM32_FLASH_ROTSR                   (STM32_FLASHIF_BASE + STM32_FLASH_ROTSR_OFFSET)
#define STM32_FLASH_ROTSRP                  (STM32_FLASHIF_BASE + STM32_FLASH_ROTSRP_OFFSET)
#define STM32_FLASH_OTPLSR                  (STM32_FLASHIF_BASE + STM32_FLASH_OTPLSR_OFFSET)
#define STM32_FLASH_OTPLSRP                 (STM32_FLASHIF_BASE + STM32_FLASH_OTPLSRP_OFFSET)
#define STM32_FLASH_WRPSR                   (STM32_FLASHIF_BASE + STM32_FLASH_WRPSR_OFFSET)
#define STM32_FLASH_WRPSRP                  (STM32_FLASHIF_BASE + STM32_FLASH_WRPSRP_OFFSET)
#define STM32_FLASH_HDPSR                   (STM32_FLASHIF_BASE + STM32_FLASH_HDPSR_OFFSET)
#define STM32_FLASH_HDPSRP                  (STM32_FLASHIF_BASE + STM32_FLASH_HDPSRP_OFFSET)
#define STM32_FLASH_EPOCHSR                 (STM32_FLASHIF_BASE + STM32_FLASH_EPOCHSR_OFFSET)
#define STM32_FLASH_EPOCHSRP                (STM32_FLASHIF_BASE + STM32_FLASH_EPOCHSRP_OFFSET)
#define STM32_FLASH_OBW1SR                  (STM32_FLASHIF_BASE + STM32_FLASH_OBW1SR_OFFSET)
#define STM32_FLASH_OBW1SRP                 (STM32_FLASHIF_BASE + STM32_FLASH_OBW1SRP_OFFSET)
#define STM32_FLASH_OBW2SR                  (STM32_FLASHIF_BASE + STM32_FLASH_OBW2SR_OFFSET)
#define STM32_FLASH_OBW2SRP                 (STM32_FLASHIF_BASE + STM32_FLASH_OBW2SRP_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* FLASH keys ***************************************************************/

#define FLASH_KEY1                          0x45670123
#define FLASH_KEY2                          0xcdef89ab
#define FLASH_OPTKEY1                       0x08192a3b
#define FLASH_OPTKEY2                       0x4c5d6e7f

#define FLASH_KEYR_CUKEY_SHIFT              (0)
#define FLASH_KEYR_CUKEY_MASK               (0xffffffff << FLASH_KEYR_CUKEY_SHIFT)
#define FLASH_OPTKEYR_OCUKEY_SHIFT          (0)
#define FLASH_OPTKEYR_OCUKEY_MASK           (0xffffffff << FLASH_OPTKEYR_OCUKEY_SHIFT)

/* Access control register **************************************************/

#define FLASH_ACR_LATENCY_SHIFT             (0)
#define FLASH_ACR_LATENCY_MASK              (15 << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)              ((n) << FLASH_ACR_LATENCY_SHIFT)
#define FLASH_ACR_WRHIGHFREQ_SHIFT          (4)
#define FLASH_ACR_WRHIGHFREQ_MASK           (3 << FLASH_ACR_WRHIGHFREQ_SHIFT)
#  define FLASH_ACR_WRHIGHFREQ(n)           ((n) << FLASH_ACR_WRHIGHFREQ_SHIFT)

/* Control and status registers *********************************************/

#define FLASH_CR_LOCK                       (1 << 0)  /* Configuration lock */
#define FLASH_CR_PG                         (1 << 1)  /* Program enable */
#define FLASH_CR_SER                        (1 << 2)  /* Sector erase */
#define FLASH_CR_BER                        (1 << 3)  /* Bank erase */
#define FLASH_CR_FW                         (1 << 4)  /* Force write */
#define FLASH_CR_START                      (1 << 5)  /* Start operation */
#define FLASH_CR_SSN_SHIFT                  (6)
#define FLASH_CR_SSN_MASK                   (7 << FLASH_CR_SSN_SHIFT)
#  define FLASH_CR_SSN(n)                   ((n) << FLASH_CR_SSN_SHIFT)
#define FLASH_CR_PG_OTP                     (1 << 16) /* OTP program enable */
#define FLASH_CR_CRC_EN                     (1 << 17) /* CRC enable */
#define FLASH_CR_ALL_BANKS                  (1 << 24) /* Select all banks */

#define FLASH_SR_BUSY                       (1 << 0) /* Flash busy */
#define FLASH_SR_WBNE                       (1 << 1) /* Write buffer not empty */
#define FLASH_SR_QW                         (1 << 2) /* Wait queue */
#define FLASH_SR_CRC_BUSY                   (1 << 3) /* CRC busy */
#define FLASH_SR_IS_PROGRAM                 (1 << 4) /* Program operation */
#define FLASH_SR_IS_ERASE                   (1 << 5) /* Erase operation */
#define FLASH_SR_IS_OPTCHANGE               (1 << 6) /* Option change */

/* Interrupt registers ******************************************************/

#define FLASH_INT_EOP                       (1 << 16) /* End of operation */
#define FLASH_INT_WRPERR                    (1 << 17) /* Write protection error */
#define FLASH_INT_PGSERR                    (1 << 18) /* Programming sequence error */
#define FLASH_INT_STRBERR                   (1 << 19) /* Strobe error */
#define FLASH_INT_OBLERR                    (1 << 20) /* Option loading error */
#define FLASH_INT_INCERR                    (1 << 21) /* Inconsistency error */
#define FLASH_INT_RDSERR                    (1 << 24) /* Read security error */
#define FLASH_INT_SNECCERR                  (1 << 25) /* Single ECC error */
#define FLASH_INT_DBECCERR                  (1 << 26) /* Double ECC error */
#define FLASH_INT_CRCEND                    (1 << 27) /* CRC complete */
#define FLASH_INT_CRCRDERR                  (1 << 28) /* CRC read error */
#define FLASH_INT_ALL                       ((0x3f << 16) | (0x1f << 24))

#define FLASH_IER_EOPIE                     FLASH_INT_EOP
#define FLASH_IER_WRPERRIE                  FLASH_INT_WRPERR
#define FLASH_IER_PGSERRIE                  FLASH_INT_PGSERR
#define FLASH_IER_STRBERRIE                 FLASH_INT_STRBERR
#define FLASH_IER_OBLERRIE                  FLASH_INT_OBLERR
#define FLASH_IER_INCERRIE                  FLASH_INT_INCERR
#define FLASH_IER_RDSERRIE                  FLASH_INT_RDSERR
#define FLASH_IER_SNECCERRIE                FLASH_INT_SNECCERR
#define FLASH_IER_DBECCERRIE                FLASH_INT_DBECCERR
#define FLASH_IER_CRCENDIE                  FLASH_INT_CRCEND
#define FLASH_IER_CRCRDERRIE                FLASH_INT_CRCRDERR
#define FLASH_ISR_EOPF                      FLASH_INT_EOP
#define FLASH_ISR_WRPERRF                   FLASH_INT_WRPERR
#define FLASH_ISR_PGSERRF                   FLASH_INT_PGSERR
#define FLASH_ISR_STRBERRF                  FLASH_INT_STRBERR
#define FLASH_ISR_OBLERRF                   FLASH_INT_OBLERR
#define FLASH_ISR_INCERRF                   FLASH_INT_INCERR
#define FLASH_ISR_RDSERRF                   FLASH_INT_RDSERR
#define FLASH_ISR_SNECCERRF                 FLASH_INT_SNECCERR
#define FLASH_ISR_DBECCERRF                 FLASH_INT_DBECCERR
#define FLASH_ISR_CRCENDF                   FLASH_INT_CRCEND
#define FLASH_ISR_CRCRDERRF                 FLASH_INT_CRCRDERR
#define FLASH_ICR_EOPF                      FLASH_INT_EOP
#define FLASH_ICR_WRPERRF                   FLASH_INT_WRPERR
#define FLASH_ICR_PGSERRF                   FLASH_INT_PGSERR
#define FLASH_ICR_STRBERRF                  FLASH_INT_STRBERR
#define FLASH_ICR_OBLERRF                   FLASH_INT_OBLERR
#define FLASH_ICR_INCERRF                   FLASH_INT_INCERR
#define FLASH_ICR_RDSERRF                   FLASH_INT_RDSERR
#define FLASH_ICR_SNECCERRF                 FLASH_INT_SNECCERR
#define FLASH_ICR_DBECCERRF                 FLASH_INT_DBECCERR
#define FLASH_ICR_CRCENDF                   FLASH_INT_CRCEND
#define FLASH_ICR_CRCRDERRF                 FLASH_INT_CRCRDERR

/* CRC registers ************************************************************/

#define FLASH_CRCCR_CRC_SECT_SHIFT          (0)       /* Bits 0-2: CRC sector */
#define FLASH_CRCCR_CRC_SECT_MASK           (7 << FLASH_CRCCR_CRC_SECT_SHIFT)
#  define FLASH_CRCCR_CRC_SECT(n)           ((n) << FLASH_CRCCR_CRC_SECT_SHIFT)
#define FLASH_CRCCR_CRC_BY_SECT             (1 << 9)  /* Bit 9: Sector mode */
#define FLASH_CRCCR_ADD_SECT                (1 << 10) /* Bit 10: Add sector */
#define FLASH_CRCCR_CLEAN_SECT              (1 << 11) /* Bit 11: Clear sector list */
#define FLASH_CRCCR_START_CRC               (1 << 16) /* Bit 16: Start CRC */
#define FLASH_CRCCR_CLEAN_CRC               (1 << 17) /* Bit 17: Clear CRC */
#define FLASH_CRCCR_BURST_SHIFT             (20)      /* Bits 20-21: Burst size */
#define FLASH_CRCCR_BURST_MASK              (3 << FLASH_CRCCR_BURST_SHIFT)
#  define FLASH_CRCCR_BURST_4               (0 << FLASH_CRCCR_BURST_SHIFT)
#  define FLASH_CRCCR_BURST_16              (1 << FLASH_CRCCR_BURST_SHIFT)
#  define FLASH_CRCCR_BURST_64              (2 << FLASH_CRCCR_BURST_SHIFT)
#  define FLASH_CRCCR_BURST_256             (3 << FLASH_CRCCR_BURST_SHIFT)
#define FLASH_CRCCR_ALL_SECT                (1 << 24) /* Bit 24: All sectors */

#define FLASH_CRCSADDR_START_SHIFT          (6) /* Bits 6-16: CRC start address */
#define FLASH_CRCSADDR_START_MASK           (0x7ff << FLASH_CRCSADDR_START_SHIFT)
#  define FLASH_CRCSADDR_START(n)           ((n) << FLASH_CRCSADDR_START_SHIFT)
#define FLASH_CRCEADDR_END_SHIFT            (6) /* Bits 6-16: CRC end address */
#define FLASH_CRCEADDR_END_MASK             (0x7ff << FLASH_CRCEADDR_END_SHIFT)
#  define FLASH_CRCEADDR_END(n)             ((n) << FLASH_CRCEADDR_END_SHIFT)

/* Option control and state registers ***************************************/

#define FLASH_OPTCR_OPTLOCK                 (1 << 0)  /* Option configuration lock */
#define FLASH_OPTCR_PG_OPT                  (1 << 1)  /* Option byte programming */
#define FLASH_OPTCR_KVEIE                   (1 << 27) /* Key validity error IRQ */
#define FLASH_OPTCR_KTEIE                   (1 << 28) /* Key transfer error IRQ */
#define FLASH_OPTCR_OPTERRIE                (1 << 30) /* Option byte error IRQ */
#define FLASH_OPTISR_KVEF                   (1 << 27) /* Key validity error flag */
#define FLASH_OPTISR_KTEF                   (1 << 28) /* Key transfer error flag */
#define FLASH_OPTISR_OPTERRF                (1 << 30) /* Option byte error flag */
#define FLASH_OPTICR_KVEF                   (1 << 27) /* Clear key validity error */
#define FLASH_OPTICR_KTEF                   (1 << 28) /* Clear key transfer error */
#define FLASH_OPTICR_OPTERRF                (1 << 30) /* Clear option byte error */

#define FLASH_OBKCR_OBKINDEX_SHIFT          (0)       /* Bits 0-4: OB key index */
#define FLASH_OBKCR_OBKINDEX_MASK           (31 << FLASH_OBKCR_OBKINDEX_SHIFT)
#  define FLASH_OBKCR_OBKINDEX(n)           ((n) << FLASH_OBKCR_OBKINDEX_SHIFT)
#define FLASH_OBKCR_NEXTKL_SHIFT            (8)       /* Bits 8-9: Next key level */
#define FLASH_OBKCR_NEXTKL_MASK             (3 << FLASH_OBKCR_NEXTKL_SHIFT)
#  define FLASH_OBKCR_NEXTKL(n)             ((n) << FLASH_OBKCR_NEXTKL_SHIFT)
#define FLASH_OBKCR_OBKSIZE_SHIFT           (10)      /* Bits 10-11: OB key size */
#define FLASH_OBKCR_OBKSIZE_MASK            (3 << FLASH_OBKCR_OBKSIZE_SHIFT)
#  define FLASH_OBKCR_OBKSIZE(n)            ((n) << FLASH_OBKCR_OBKSIZE_SHIFT)
#define FLASH_OBKCR_KEYPROG                 (1 << 14) /* Key programming mode */
#define FLASH_OBKCR_KEYSTART                (1 << 15) /* Start key operation */

#define FLASH_NVSR_NVSTATE_SHIFT            (0) /* Bits 0-7: Non-volatile state */
#define FLASH_NVSR_NVSTATE_MASK             (0xff << FLASH_NVSR_NVSTATE_SHIFT)
#  define FLASH_NVSR_NVSTATE(n)             ((n) << FLASH_NVSR_NVSTATE_SHIFT)
#define FLASH_NVSRP_NVSTATE_SHIFT           (0) /* Bits 0-7: Programmed NV state */
#define FLASH_NVSRP_NVSTATE_MASK            (0xff << FLASH_NVSRP_NVSTATE_SHIFT)
#  define FLASH_NVSRP_NVSTATE(n)            ((n) << FLASH_NVSRP_NVSTATE_SHIFT)

#define FLASH_ROTSR_OEM_PROVD_SHIFT         (0)  /* Bits 0-7: OEM provision state */
#define FLASH_ROTSR_OEM_PROVD_MASK          (0xff << FLASH_ROTSR_OEM_PROVD_SHIFT)
#define FLASH_ROTSR_DBG_AUTH_SHIFT          (8)  /* Bits 8-15: Debug authentication */
#define FLASH_ROTSR_DBG_AUTH_MASK           (0xff << FLASH_ROTSR_DBG_AUTH_SHIFT)
#define FLASH_ROTSR_IROT_SELECT_SHIFT       (24) /* Bits 24-31: Immutable root */
#define FLASH_ROTSR_IROT_SELECT_MASK        (0xff << FLASH_ROTSR_IROT_SELECT_SHIFT)
#define FLASH_ROTSRP_OEM_PROVD_SHIFT        (0)  /* Bits 0-7: Programmed provision */
#define FLASH_ROTSRP_OEM_PROVD_MASK         (0xff << FLASH_ROTSRP_OEM_PROVD_SHIFT)
#define FLASH_ROTSRP_DBG_AUTH_SHIFT         (8)  /* Bits 8-15: Programmed debug auth */
#define FLASH_ROTSRP_DBG_AUTH_MASK          (0xff << FLASH_ROTSRP_DBG_AUTH_SHIFT)
#define FLASH_ROTSRP_IROT_SELECT_SHIFT      (24) /* Bits 24-31: Programmed iRoT */
#define FLASH_ROTSRP_IROT_SELECT_MASK       (0xff << FLASH_ROTSRP_IROT_SELECT_SHIFT)

#define FLASH_OTPLSR_OTPL_SHIFT             (0)  /* Bits 0-15: OTP lock */
#define FLASH_OTPLSR_OTPL_MASK              (0xffff << FLASH_OTPLSR_OTPL_SHIFT)
#define FLASH_OTPLSRP_OTPL_SHIFT            (0)  /* Bits 0-15: OTP lock program */
#define FLASH_OTPLSRP_OTPL_MASK             (0xffff << FLASH_OTPLSRP_OTPL_SHIFT)
#define FLASH_WRPSR_WRPS_SHIFT              (0)  /* Bits 0-7: Write protection */
#define FLASH_WRPSR_WRPS_MASK               (0xff << FLASH_WRPSR_WRPS_SHIFT)
#define FLASH_WRPSRP_WRPS_SHIFT             (0)  /* Bits 0-7: Program protection */
#define FLASH_WRPSRP_WRPS_MASK              (0xff << FLASH_WRPSRP_WRPS_SHIFT)
#define FLASH_HDPSR_HDP_AREA_START_SHIFT    (0)  /* Bits 0-7: HDP start */
#define FLASH_HDPSR_HDP_AREA_START_MASK     (0xff << FLASH_HDPSR_HDP_AREA_START_SHIFT)
#define FLASH_HDPSR_HDP_AREA_END_SHIFT      (16) /* Bits 16-23: HDP end */
#define FLASH_HDPSR_HDP_AREA_END_MASK       (0xff << FLASH_HDPSR_HDP_AREA_END_SHIFT)
#define FLASH_HDPSRP_HDP_AREA_START_SHIFT   (0)  /* Bits 0-7: Program start */
#define FLASH_HDPSRP_HDP_AREA_START_MASK    (0xff << FLASH_HDPSRP_HDP_AREA_START_SHIFT)
#define FLASH_HDPSRP_HDP_AREA_END_SHIFT     (16) /* Bits 16-23: Program end */
#define FLASH_HDPSRP_HDP_AREA_END_MASK      (0xff << FLASH_HDPSRP_HDP_AREA_END_SHIFT)
#define FLASH_EPOCHSR_EPOCH_SHIFT           (0)  /* Bits 0-23: Epoch */
#define FLASH_EPOCHSR_EPOCH_MASK            (0xffffff << FLASH_EPOCHSR_EPOCH_SHIFT)
#define FLASH_EPOCHSRP_EPOCH_SHIFT          (0)  /* Bits 0-23: Program epoch */
#define FLASH_EPOCHSRP_EPOCH_MASK           (0xffffff << FLASH_EPOCHSRP_EPOCH_SHIFT)

#define FLASH_OBW1SR_BOR_LEVEL_SHIFT        (2)       /* Bits 2-3: BOR level */
#define FLASH_OBW1SR_BOR_LEVEL_MASK         (3 << FLASH_OBW1SR_BOR_LEVEL_SHIFT)
#define FLASH_OBW1SR_IWDG_HW                (1 << 4)  /* Hardware IWDG */
#define FLASH_OBW1SR_NRST_STOP              (1 << 6)  /* Reset in stop mode */
#define FLASH_OBW1SR_NRST_STBY              (1 << 7)  /* Reset in standby mode */
#define FLASH_OBW1SR_XSPI1_HSLV             (1 << 8)  /* XSPI1 high-speed mode */
#define FLASH_OBW1SR_XSPI2_HSLV             (1 << 9)  /* XSPI2 high-speed mode */
#define FLASH_OBW1SR_IWDG_FZ_STOP           (1 << 17) /* Freeze IWDG in stop */
#define FLASH_OBW1SR_IWDG_FZ_STBY           (1 << 18) /* Freeze IWDG in standby */
#define FLASH_OBW1SR_PERSO_OK               (1 << 28) /* Personalization complete */
#define FLASH_OBW1SR_VDDIO_HSLV             (1 << 29) /* VDDIO high-speed mode */

#define FLASH_OBW1SRP_BOR_LEVEL_SHIFT       (2)       /* Bits 2-3: Programmed BOR */
#define FLASH_OBW1SRP_BOR_LEVEL_MASK        (3 << FLASH_OBW1SRP_BOR_LEVEL_SHIFT)
#define FLASH_OBW1SRP_IWDG_HW               (1 << 4)  /* Program hardware IWDG */
#define FLASH_OBW1SRP_NRST_STOP             (1 << 6)  /* Program reset in stop */
#define FLASH_OBW1SRP_NRST_STBY             (1 << 7)  /* Program reset in standby */
#define FLASH_OBW1SRP_XSPI1_HSLV            (1 << 8)  /* Program XSPI1 high speed */
#define FLASH_OBW1SRP_XSPI2_HSLV            (1 << 9)  /* Program XSPI2 high speed */
#define FLASH_OBW1SRP_IWDG_FZ_STOP          (1 << 17) /* Program IWDG stop freeze */
#define FLASH_OBW1SRP_IWDG_FZ_STBY          (1 << 18) /* Program IWDG standby freeze */
#define FLASH_OBW1SRP_VDDIO_HSLV            (1 << 29) /* Program VDDIO high speed */

#define FLASH_OBW2SR_ITCM_AXI_SHARE_SHIFT   (0)      /* Bits 0-2: ITCM/AXI share */
#define FLASH_OBW2SR_ITCM_AXI_SHARE_MASK    (7 << FLASH_OBW2SR_ITCM_AXI_SHARE_SHIFT)
#define FLASH_OBW2SR_DTCM_AXI_SHARE_SHIFT   (4)      /* Bits 4-6: DTCM/AXI share */
#define FLASH_OBW2SR_DTCM_AXI_SHARE_MASK    (7 << FLASH_OBW2SR_DTCM_AXI_SHARE_SHIFT)
#define FLASH_OBW2SR_ECC_ON_SRAM            (1 << 8) /* SRAM ECC enable */
#define FLASH_OBW2SR_I2C_NI3C               (1 << 9) /* I2C instead of I3C */

#define FLASH_OBW2SRP_ITCM_AXI_SHARE_SHIFT  (0)      /* Bits 0-2: Program ITCM share */
#define FLASH_OBW2SRP_ITCM_AXI_SHARE_MASK   (7 << FLASH_OBW2SRP_ITCM_AXI_SHARE_SHIFT)
#define FLASH_OBW2SRP_DTCM_AXI_SHARE_SHIFT  (4)      /* Bits 4-6: Program DTCM share */
#define FLASH_OBW2SRP_DTCM_AXI_SHARE_MASK   (7 << FLASH_OBW2SRP_DTCM_AXI_SHARE_SHIFT)
#define FLASH_OBW2SRP_ECC_ON_SRAM           (1 << 8) /* Program SRAM ECC */
#define FLASH_OBW2SRP_I2C_NI3C              (1 << 9) /* Program I2C/I3C mode */

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_FLASH_H */
