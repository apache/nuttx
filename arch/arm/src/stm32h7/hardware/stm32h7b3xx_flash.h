/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7b3xx_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7B3XX_FLASH_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7B3XX_FLASH_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_FLASH_ACR_OFFSET         0x0000 /* Access control register */
#define STM32_FLASH_KEYR_OFFSET        0x0004 /* Key register for bank 1 */
#define STM32_FLASH_OPTKEYR_OFFSET     0x0008 /* Option key register */
#define STM32_FLASH_CR_OFFSET          0x000c /* Control register for bank 1 */
#define STM32_FLASH_SR_OFFSET          0x0010 /* Status register for bank 1 */
#define STM32_FLASH_CCR_OFFSET         0x0014 /* Clear control register for bank 1 */
#define STM32_FLASH_OPTCR_OFFSET       0x0018 /* Option control register */
#define STM32_FLASH_OPTSR_CUR_OFFSET   0x001c /* Option status register (CUR) */
#define STM32_FLASH_OPTSR_PRG_OFFSET   0x0020 /* Option status register (PRG) */
#define STM32_FLASH_OPTCCR_OFFSET      0x0024 /* Option clear control register */
#define STM32_FLASH_PRAR_CUR_OFFSET    0x0028 /* Protection address for bank 1 */
#define STM32_FLASH_PRAR_PRG_OFFSET    0x002C /* Protection address for bank 1 */
#define STM32_FLASH_SCAR_CUR_OFFSET    0x0030 /* Secure address for bank 1 (CUR) */
#define STM32_FLASH_SCAR_PRG_OFFSET    0x0034 /* Secure address for bank 1 (PRG) */
#define STM32_FLASH_WPSN_CURR_OFFSET   0x0038 /* Write sector protection for bank 1 (CUR) */
#define STM32_FLASH_WPSN_PRGR_OFFSET   0x003C /* Write sector protection for bank 1 (PRG) */
#define STM32_FLASH_BOOT_CUR_OFFSET    0x0040 /* Boot address (CUR) */
#define STM32_FLASH_BOOT_PRGR_OFFSET   0x0044 /* Boot address (PRG) */
#define STM32_FLASH_CRCCR_OFFSET       0x0050 /* CRC control register for bank 1 */
#define STM32_FLASH_CRCSADDR_OFFSET    0x0054 /* CRC start address register for bank 1 */
#define STM32_FLASH_CRCEADDR_OFFSET    0x0058 /* CRC end address register for bank 1 */
#define STM32_FLASH_CRCDATAR_OFFSET    0x005C /* CRC data register */
#define STM32_FLASH_ECC_FAR_OFFSET     0x0060 /* ECC fail address register for bank 1 */
#define STM32_FLASH_OTPBL_CUR_OFFSET   0x0068 /* current values of option bits */
#define STM32_FLASH_OTPBL_PRG_OFFSET   0x006C /* program values in option bits */

#define STM32_FLASH_BANK1_OFFSET       0x0000 /* Bank 1 registers offset */
#define STM32_FLASH_BANK2_OFFSET       0x0100 /* Bank 2 registers offset */

/* Register Addresses *******************************************************/

#define STM32_FLASH_ACR                (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_KEYR1              (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_KEYR1_OFFSET)
#define STM32_FLASH_OPTKEYR            (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_CR1                (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_CR1_OFFSET)
#define STM32_FLASH_SR1                (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_SR1_OFFSET)
#define STM32_FLASH_CCR1               (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_CCR1_OFFSET)
#define STM32_FLASH_OPTCR              (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_OPTCR_OFFSET)
#define STM32_FLASH_OPTSR_CUR          (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_OPTSR_CUR_OFFSET)
#define STM32_FLASH_OPTSR_PRG          (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_OPTSR_PRG_OFFSET)
#define STM32_FLASH_OPTCCR             (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_OPTCCR_OFFSET)
#define STM32_FLASH_PRAR_CUR1          (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_PRAR_CUR1_OFFSET)
#define STM32_FLASH_PRAR_PRG1          (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_PRAR_PRG1_OFFSET)
#define STM32_FLASH_SCAR_CUR1          (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_SCAR_CUR1_OFFSET)
#define STM32_FLASH_SCAR_PRG1          (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_SCAR_PRG1_OFFSET)
#define STM32_FLASH_WPSN_CUR1R         (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_WPSN_CUR1R_OFFSET)
#define STM32_FLASH_WPSN_PRG1R         (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_WPSN_PRG1R_OFFSET)
#define STM32_FLASH_BOOT_CUR           (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_BOOT_CUR_OFFSET)
#define STM32_FLASH_BOOT_PRGR          (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_BOOT_PRGR_OFFSET)
#define STM32_FLASH_CRCCR1             (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_CRCCR1_OFFSET)
#define STM32_FLASH_CRCSADD1R          (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_CRCSADD1R_OFFSET)
#define STM32_FLASH_CRCEADD1R          (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_CRCEADD1R_OFFSET)
#define STM32_FLASH_CRCDATAR           (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_CRCDATAR_OFFSET)
#define STM32_FLASH_ECC_FA1R           (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_ECC_FA1R_OFFSET)
#define STM32_FLASH_OTPBL_CUR          (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_OTPBL_CUR_OFFSET)
#define STM32_FLASH_OTPBL_PRG          (STM32_FLASHIF_BASE + STM32_FLASH_BANK1_OFFSET + STM32_FLASH_OTPBL_PRG_OFFSET)

#define STM32_FLASH_KEYR2              (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_KEYR2_OFFSET)
#define STM32_FLASH_CR2                (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_CR2_OFFSET)
#define STM32_FLASH_SR2                (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_SR2_OFFSET)
#define STM32_FLASH_CCR2               (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_CCR2_OFFSET)
#define STM32_FLASH_PRAR_CUR2          (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_PRAR_CUR2_OFFSET)
#define STM32_FLASH_PRAR_PRG2          (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_PRAR_PRG2_OFFSET)
#define STM32_FLASH_SCAR_CUR2          (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_SCAR_CUR2_OFFSET)
#define STM32_FLASH_SCAR_PRG2          (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_SCAR_PRG2_OFFSET)
#define STM32_FLASH_WPSN_CUR2R         (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_WPSN_CUR2R_OFFSET)
#define STM32_FLASH_WPSN_PRG2R         (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_WPSN_PRG2R_OFFSET)
#define STM32_FLASH_CRCCR2             (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_CRCCR2_OFFSET)
#define STM32_FLASH_CRCSADD2R          (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_CRCSADD2R_OFFSET)
#define STM32_FLASH_CRCEADD2R          (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_CRCEADD2R_OFFSET)
#define STM32_FLASH_ECC_FA2R           (STM32_FLASHIF_BASE + STM32_FLASH_BANK2_OFFSET + STM32_FLASH_ECC_FA2R_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Flash Access Control Register (ACR) Bank 1 or 2 */

#define FLASH_ACR_LATENCY_SHIFT        (0)                              /* Bits 0-3: Latency */
#define FLASH_ACR_LATENCY_MASK         (15 << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)         ((n) << FLASH_ACR_LATENCY_SHIFT) /* n wait states */
#  define FLASH_ACR_LATENCY_0          (0 << FLASH_ACR_LATENCY_SHIFT)   /* 0000: Zero wait states */
#  define FLASH_ACR_LATENCY_1          (1 << FLASH_ACR_LATENCY_SHIFT)   /* 0001: One wait state */
#  define FLASH_ACR_LATENCY_2          (2 << FLASH_ACR_LATENCY_SHIFT)   /* 0010: Two wait states */
#  define FLASH_ACR_LATENCY_3          (3 << FLASH_ACR_LATENCY_SHIFT)   /* 0011: Three wait states */
#  define FLASH_ACR_LATENCY_4          (4 << FLASH_ACR_LATENCY_SHIFT)   /* 0100: Four wait states */
#  define FLASH_ACR_LATENCY_5          (5 << FLASH_ACR_LATENCY_SHIFT)   /* 0101: Five wait states */
#  define FLASH_ACR_LATENCY_6          (6 << FLASH_ACR_LATENCY_SHIFT)   /* 0110: Six wait states */
#define FLASH_ACR_WRHIGHFREQ_SHIFT     (4)                              /* Bitd 4-5: Flash signal delay */
#define FLASH_ACR_WRHIGHFREQ_MASK      (3 << FLASH_ACR_WRHIGHFREQ_SHIFT)
#  define FLASH_ACR_WRHIGHFREQ(n)      ((n) << FLASH_ACR_WRHIGHFREQ_SHIFT)

/* Flash Control Register (CR) Bank 1 or 2 */

#define FLASH_CR_LOCK                  (1 << 0)                       /* Bit 0:  Lock */
#define FLASH_CR_PG                    (1 << 1)                       /* Bit 1:  Programming */
#define FLASH_CR_SER                   (1 << 2)                       /* Bit 2:  Sector erase */
#define FLASH_CR_BER                   (1 << 3)                       /* Bit 3: Bank erase */
#define FLASH_CR_FW                    (1 << 4)                       /* Bit 4: Force write */
#define FLASH_CR_START                 (1 << 5)                       /* Bit 5: Erase start */
#define FLASH_CR_SSN_SHIFT             (6)                            /* Bits 6-12: Sector number */
#define FLASH_CR_SSN_MASK              (0x7f << FLASH_CR_SSN_SHIFT)   /* Used to clear FLASH_CR_SSN bits */

#  define FLASH_CR_SSN(n)              ((uint32_t)((n) & 0x7f) << FLASH_CR_SSN_SHIFT) /* Sector n, n=0..127 */

#define FLASH_CR_CRCEN                 (1 << 15)  					/* Bit 15: CRC control enable */
#define FLASH_CR_EOPIE                 (1 << 16)  					/* Bit 16: End-of-program interrupt enable */
#define FLASH_CR_WRPERRIE              (1 << 17)  					/* Bit 17: Write protection error interrupt enable */
#define FLASH_CR_PGSERRIE              (1 << 18)  					/* Bit 18: Programming sequence error interrupt enable */
#define FLASH_CR_STRBERRIE             (1 << 19)  					/* Bit 19: Strobe error interrupt enable */
#define FLASH_CR_INCERRIE              (1 << 21)  					/* Bit 21: Inconsistency error interrupt enable */
#define FLASH_CR_RDPERRIE              (1 << 23)  					/* Bit 23: Read protection error interrupt enable */
#define FLASH_CR_RDSERRIE              (1 << 24)  					/* Bit 24: Secure error interrupt enable */
#define FLASH_CR_SNECCERRIE            (1 << 25)  					/* Bit 25: ECC single correction error interrupt enable */
#define FLASH_CR_DBECCERRIE            (1 << 26)  					/* Bit 26: ECC double detection error interrupt enable */
#define FLASH_CR_CRCENDIE              (1 << 27)  					/* Bit 27: CRC end of calculation interrupt enable */
#define FLASH_CR_CRCRDERRIE            (1 << 28)  					/* Bit 28: CRC read error interrupt enable bit */

/* Flash Status Register (SR)  Bank 1 or 2 */

#define FLASH_SR_BSY                   (1 << 0)   /* Bit 0: Busy */
#define FLASH_SR_WBNE                  (1 << 1)   /* Bit 1: write buffer not empty */
#define FLASH_SR_QW                    (1 << 2)   /* Bit 2: wait queue flag */
#define FLASH_SR_CRCBUSY               (1 << 3)   /* Bit 3: CRC busy flag */
                                                  /* Bits 4-15: Reserved */
#define FLASH_SR_EOP                   (1 << 16)  /* Bit 16: End of program */
#define FLASH_SR_WRPERR                (1 << 17)  /* Bit 17: Write protection error */
#define FLASH_SR_PGSERR                (1 << 18)  /* Bit 18: Programming sequence error */
#define FLASH_SR_STRBERR               (1 << 19)  /* Bit 19: Strobe error */
                                                  /* Bit 20: Reserved */
#define FLASH_SR_INCERR                (1 << 21)  /* Bit 21: Inconsistency error */
#define FLASH_SR_RDPERR                (1 << 23)  /* Bit 23: Read protection error */
#define FLASH_SR_RDSERR                (1 << 24)  /* Bit 24: Secure error */
#define FLASH_SR_SNECCERR              (1 << 25)  /* Bit 25: ECC single error */
#define FLASH_SR_DBECCERR              (1 << 26)  /* Bit 26: ECC double detection error */
#define FLASH_SR_CRCEND                (1 << 27)  /* Bit 27: CRC end of calculation */
#define FLASH_SR_CRCRDERR              (1 << 28)  /* Bit 28: CRC read error flag */

/* Flash Clear control register Bank 1 or 2 */

#define FLASH_CLR_EOP                  (1 << 16)  /* Bit 16: Clear end of program */
#define FLASH_CLR_WRPERR               (1 << 17)  /* Bit 17: Clear write protection error */
#define FLASH_CLR_PGSERR               (1 << 18)  /* Bit 18: Clear programming sequence error */
#define FLASH_CLR_STRBERR              (1 << 19)  /* Bit 19: Clear strobe error */
                                                  /* Bit 20: Reserved */
#define FLASH_CLR_INCERR               (1 << 21)  /* Bit 21: Clear inconsistency error */
#define FLASH_CLR_RDPERR               (1 << 23)  /* Bit 23: Clear read protection error */
#define FLASH_CLR_RDSERR               (1 << 24)  /* Bit 24: Clear secure error */
#define FLASH_CLR_SNECCERR             (1 << 25)  /* Bit 25: Clear ECC single error */
#define FLASH_CLR_DBECCERR             (1 << 26)  /* Bit 26: Clear ECC double detection error */
#define FLASH_CLR_CRCEND               (1 << 27)  /* Bit 27: Clear CRC end of calculation */
#define FLASH_CLR_CRCRDERR             (1 << 28)  /* Bit 28: Clear CRC read error flag */

/* Flash Option Control Register (OPTCR) Bank 1 or 2 */

#define FLASH_OPTCR_OPTLOCK            (1 << 0)   /* Bit 0: Option lock */
#define FLASH_OPTCR_OPTSTRT            (1 << 1)   /* Bit 1: Option start */
                                                  /* Bits 2-3: Reserved */
#define FLASH_OPTCR_MER                (1 << 4)   /* Bit 4: Mass erase  */
#define FLASH_OPTCR_PGOTP              (1 << 5)   /* Bit 5: Enable OTP write */
                                                  /* Bits 6-29: Reserved */
#define FLASH_OPTCR_CHANGEERRIE        (1 << 30)  /* Bit 30: Option byte change error interrupt enable */
#define FLASH_OPTCR_SWAPBANK           (1 << 31)  /* Bit 31: Bank swapping */

/* Flash Option Status Register (OPTSR) */

#define FLASH_OPTSR_BUSYV              (1 << 0)   /* Bit 0: Option byte change busy */
                                                  /* Bit 1: Reserved */
#define FLASH_OPTSR_BORLEV_SHIFT       (2)        /* Bits 2-3: Brownout level option */
#define FLASH_OPTSR_BORLEV_MASK        (3 << FLASH_OPTSR_BORLEV_SHIFT)
#  define FLASH_OPTSR_BORLEV_0         (0 << FLASH_OPTSR_BORLEV_SHIFT)
#  define FLASH_OPTSR_BORLEV_1         (1 << FLASH_OPTSR_BORLEV_SHIFT)
#  define FLASH_OPTSR_BORLEV_2         (2 << FLASH_OPTSR_BORLEV_SHIFT)
#  define FLASH_OPTSR_BORLEV_3         (3 << FLASH_OPTSR_BORLEV_SHIFT)
#define FLASH_OPTSR_IWDGSW             (1 << 4)   /* Bit 4: IWDG control mode */
                                                  /* Bit 5: Reserved */
#define FLASH_OPTSR_NRSTSTOP           (1 << 6)   /* Bit 6: DStop entry reset */
#define FLASH_OPTSR_NRSTSTDY           (1 << 7)   /* Bit 7: DStandby entry reset */
#define FLASH_OPTSR_RDP_SHIFT          (8)        /* Bits 8-15: Readout protection level */
#define FLASH_OPTSR_RDP_MASK           (0xff << FLASH_OPTSR_RDP_SHIFT)
#  define FLASH_OPTSR_RDP(n)           ((uint32_t)(n) << FLASH_OPTSR_RDP_SHIFT)
#define FLASH_OPTSR_VDDMMCHSLV         (1 << 16)  /* Bit 16:  high-speed at low-voltage status bit */
#define FLASH_OPTSR_IWDGFZSTOP         (1 << 17)  /* Bit 17: IWDG Stop mode freeze */
#define FLASH_OPTSR_IWDGFZSTBY         (1 << 18)  /* Bit 18: IWDG Standby mode freeze */
#define FLASH_OPTSR_STRAMSIZE_SHIFT    (19)       /* Bits 19-20: ST RAM size */
#define FLASH_OPTSR_STRAMSIZE_MASK     (3 << FLASH_OPTSR_STRAMSIZE_SHIFT)
#  define FLASH_OPTSR_STRAMSIZE_2      (0 << FLASH_OPTSR_STRAMSIZE_SHIFT)
#  define FLASH_OPTSR_STRAMSIZE_4      (1 << FLASH_OPTSR_STRAMSIZE_SHIFT)
#  define FLASH_OPTSR_STRAMSIZE_8      (2 << FLASH_OPTSR_STRAMSIZE_SHIFT)
#  define FLASH_OPTSR_STRAMSIZE_16     (3 << FLASH_OPTSR_STRAMSIZE_SHIFT)
#define FLASH_OPTSR_SECURITY           (1 << 21)  /* Bit 21: Security enable*/
                                                  /* Bits 22-28: Reserved */
#define FLASH_OPTSR_IOHSLV             (1 << 29)  /* Bit 29: IO high-speed at low-volateg */
#define FLASH_OPTSR_CHANGEERR          (1 << 30)  /* Bit 30: Option byte change error */
#define FLASH_OPTSR_SWAPBANK           (1 << 31)  /* Bit 31: Bank swapping status */

/* Flash Option Clear Control Register (OPTCCR) */

                                                  /* Bits 0-29: Reserved */
#define FLASH_OPTCCR_OPTCHANGEERR      (1 << 30)  /* Bit 30: OPTCHANGEERR reset */
                                                  /* Bit 31: Reserved */

/* Flash Protection Address (PRAR) Bank 1 or 2 */

#define FLASH_PRAR_START_SHIFT         (0)        /* Bits 0-11 Bank PCROP area start status bits */
#define FLASH_PRAR_START_MASK          (0xfff << FLASH_PRAR_START_SHIFT)
                                                  /* Bits 12-15: Reserved */
#define FLASH_PRAR_END_SHIFT           (16)       /* Bits 16-27 Bank PCROP area end configuration bits */
#define FLASH_PRAR_END_MASK            (0xfff << FLASH_PRAR_END_SHIFT)
                                                  /* Bits 28-30: Reserved */
#define FLASH_PRAR_DMEP                (1 << 31)  /* Bit 31: Bank PCROP protected erase enable option configuration bit */

/* Flash Secure Address (SCAR) Bank 1 or 2 */

#define FLASH_SCAR_START_SHIFT         (0)        /* Bits 0-11 Bank secure-only area start status bits */
#define FLASH_SCAR_START_MASK          (0xfff << FLASH_SCAR_START_SHIFT)
                                                  /* Bits 12-15: Reserved */
#define FLASH_SCAR_END_SHIFT           (16)       /* Bits 16-27 Bank secure-only area end configuration bits */
#define FLASH_SCAR_END_MASK            (0xfff << FLASH_SCAR_END_SHIFT)
                                                  /* Bits 28-30: Reserved */
#define FLASH_SCAR_DMES                (1 << 31)  /* Bit 31: Bank secure access protected erase enable option status bit */

/* Flash Write Sector Group Protection (WPSGN) Bank 1 or 2 */

#define FLASH_WPSGN_SHIFT         		(0)        /* Bits 0-31: Sector group write protection option */
#define FLASH_WPSGN_MASK          		(0xffffffff << FLASH_WPSGN_SHIFT)

/* Flash Register Boot Address (BOOT) */

#define FLASH_BOOT_ADD0_SHIFT          (0)        /* Bits 0-15: Boot address 0 */
#define FLASH_BOOT_ADD0_MASK           (0xffff << FLASH_BOOT_ADD0_SHIFT)
#define FLASH_BOOT_ADD1_SHIFT          (16)       /* Bits 16-31: Boot address 1 */
#define FLASH_BOOT_ADD1_MASK           (0xffff << FLASH_BOOT_ADD1_SHIFT)

/* Flash CRC Control Register (CRCCR) Bank 1 or 2 */

#define FLASH_CRCCR_CRC_SEC_SHIFT      (0)        /* Bits 0-6: CRC sector number */
#define FLASH_CRCCR_CRC_SEC_MASK       (0x7f << FLASH_CRCCR_CRC_SEC_SHIFT)
#define FLASH_CRCCR_CRC_BY_SECT        (1 << 8)   /* Bit 9: Bank CRC sector mode select bit */
#define FLASH_CRCCR_ADD_SECT           (1 << 9)   /* Bit 9: Bank CRC sector select bit */
#define FLASH_CRCCR_CLEAN_SECT         (1 << 10)  /* Bit 10: Bank CRC sector list clear bit */
#define FLASH_CRCCR_START_CRC          (1 << 16)  /* Bit 16: Bank CRC start bit */
#define FLASH_CRCCR_CLEAN_CRC          (1 << 17)  /* Bit 16: Bank CRC clean bit */
#define FLASH_CRCCR_BURST_SHIFT        (20)       /* Bits 20-21: Bank CRC burst size */
#define FLASH_CRCCR_BURST_MASK         (3 << FLASH_CRCCR_BURST_SHIFT)
#  define FLASH_CRCCR_BURST_4          (0 << FLASH_CRCCR_BURST_SHIFT)
#  define FLASH_CRCCR_BURST_16         (1 << FLASH_CRCCR_BURST_SHIFT)
#  define FLASH_CRCCR_BURST_64         (2 << FLASH_CRCCR_BURST_SHIFT)
#  define FLASH_CRCCR_BURST_256        (3 << FLASH_CRCCR_BURST_SHIFT)
#define FLASH_CRCCR_ALL_BANK           (1 << 22)   /* Bit 22: Bank CRC select bit */

/* Flash CRC Start Address Register (CRCSADDR) Bank 1 or 2 */

                                                  /* Bits 0-1: Reserved */
#define FLASH_CRCSADDR_START_SHIFT     (2)        /* Bits 2-19 CRC start address on bank */
#define FLASH_CRCSADDR_START_MASK      (0x3ffff << FLASH_CRCSADDR_START_SHIFT)
                                                  /* Bits 20-31: Reserved */

/* Flash CRC End Address Register (CRCSEDDR) Bank 1 or 2 */

                                                  /* Bits 0-1: Reserved */
#define FLASH_CRCSEDDR_START_SHIFT     (2)        /* Bits 2-19 CRC end address on bank */
#define FLASH_CRCSEDDR_START_MASK      (0x3ffff << FLASH_CRCSEDDR_START_SHIFT)
                                                  /* Bits 20-31: Reserved */

/* Flash ECC fail Address(FAnR) Bank 1 or 2 */

#define FLASH_ECC_FAR_SHIFT            (0)        /* Bits 0-15 Bank 1 ECC error add */
#define FLASH_ECC_FAR_MASK             (0xffff << FLASH_CRCSEDDR_START_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7B3XX_FLASH_H */
