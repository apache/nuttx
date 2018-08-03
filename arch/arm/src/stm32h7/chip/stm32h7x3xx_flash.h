/************************************************************************************
 * arch/arm/src/stm32h7/chip/stm32h7x3xx_flash.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Mateusz Szafoni <raiden00@railab.me>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM327_CHIP_STM32H7X3XX_FLASH_H
#define __ARCH_ARM_SRC_STM327_CHIP_STM32H7X3XX_FLASH_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_FLASH_ACR_OFFSET      0x0000 /* Access control register */
#define STM32_FLASH_KEYR_OFFSET     0x0004 /* Key register */
#define STM32_FLASH_OPTKEYR_OFFSET  0x0008 /* Option key register */
#define STM32_FLASH_CR_OFFSET       0x000c /* Control register */
#define STM32_FLASH_SR_OFFSET       0x0010 /* Status register */
#define STM32_FLASH_CCR_OFFSET      0x0014 /* Clear control register */
#define STM32_FLASH_OPTCR_OFFSET    0x0018 /* Option control register */
#define STM32_FLASH_OPTSRCUR_OFFSET 0x001c /* Option status register (CUR) */
#define STM32_FLASH_OPTSRPRG_OFFSET 0x0020 /* Option status register (PRG) */
#define STM32_FLASH_OPTCCR_OFFSET   0x0024 /* Option clear control register */
#define STM32_FLASH_PRARCUR_OFFSET  0x0028 /* Protection address (CUR) */
#define STM32_FLASH_PRARPRG_OFFSET  0x002C /* Protection address (PRG) */
#define STM32_FLASH_SCARCUR_OFFSET  0x0030 /* Secure address (CUR) */
#define STM32_FLASH_SCARPRG_OFFSET  0x0034 /* Secure address (PRG) */
#define STM32_FLASH_WPSNCUR_OFFSET  0x0038 /* Write sector protection (CUR) */
#define STM32_FLASH_WPSNPRG_OFFSET  0x003C /* Write sector protection (PRG) */
#define STM32_FLASH_BOOTCUR_OFFSET  0x0040 /* Boot address (CUR) */
#define STM32_FLASH_BOOTPRGR_OFFSET 0x0044 /* Boot address (PRG) */
#define STM32_FLASH_CRCCR_OFFSET    0x0050 /* CRC control register */
#define STM32_FLASH_CRCSADDR_OFFSET 0x0054 /* CRC start address register */
#define STM32_FLASH_CRCEADDR_OFFSET 0x0058 /* CRC end address register */
#define STM32_FLASH_CRCDATAR_OFFSET 0x005C /* CRC data register */
#define STM32_FLASH_ECCFAR_OFFSET   0x0060 /* ECC fail address */

#define STM32_FLASH_BANK1_OFFSET    0x0000 /* Bank 1 registers offset */
#define STM32_FLASH_BANK2_OFFSET    0x0100 /* Bank 2 registers offset */

/* Register Addresses ***************************************************************/

#define STM32_FLASH_ACR             (STM32_FLASH_BASE+STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_KEYR            (STM32_FLASH_BASE+STM32_FLASH_KEYR_OFFSET)
#define STM32_FLASH_OPTKEYR         (STM32_FLASH_BASE+STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_CR              (STM32_FLASH_BASE+STM32_FLASH_CR_OFFSET)
#define STM32_FLASH_SR              (STM32_FLASH_BASE+STM32_FLASH_SR_OFFSET)
#define STM32_FLASH_CCR             (STM32_FLASH_BASE+STM32_FLASH_CCR_OFFSET)
#define STM32_FLASH_OPTCR           (STM32_FLASH_BASE+STM32_FLASH_OPTCR_OFFSET)
#define STM32_FLASH_OPTSRCUR        (STM32_FLASH_BASE+STM32_FLASH_OPTSRCUR_OFFSET)
#define STM32_FLASH_OPTSRPRG        (STM32_FLASH_BASE+STM32_FLASH_OPTSRPRG_OFFSET)
#define STM32_FLASH_OPTCCR          (STM32_FLASH_BASE+STM32_FLASH_OPTCCR_OFFSET)
#define STM32_FLASH_PRARCUR         (STM32_FLASH_BASE+STM32_FLASH_PRARCUR_OFFSET)
#define STM32_FLASH_PRARPRG         (STM32_FLASH_BASE+STM32_FLASH_PRARPRG_OFFSET)
#define STM32_FLASH_SCARCUR         (STM32_FLASH_BASE+STM32_FLASH_SCARCUR_OFFSET)
#define STM32_FLASH_SCARPRG         (STM32_FLASH_BASE+STM32_FLASH_SCARPRG_OFFSET)
#define STM32_FLASH_WPSNCUR         (STM32_FLASH_BASE+STM32_FLASH_WPSNCUR_OFFSET)
#define STM32_FLASH_BOOTCUR         (STM32_FLASH_BASE+STM32_FLASH_BOOTCUR_OFFSET)
#define STM32_FLASH_BOOTPRGR        (STM32_FLASH_BASE+STM32_FLASH_BOOTPRGR_OFFSET)
#define STM32_FLASH_CRCCR           (STM32_FLASH_BASE+STM32_FLASH_CRCCR_OFFSET)
#define STM32_FLASH_CRCSADDR        (STM32_FLASH_BASE+STM32_FLASH_CRCSADDR_OFFSET)
#define STM32_FLASH_CRCEADDR        (STM32_FLASH_BASE+STM32_FLASH_CRCEADDR_OFFSET)
#define STM32_FLASH_CRCDATAR        (STM32_FLASH_BASE+STM32_FLASH_CRCDATAR_OFFSET)
#define STM32_FLASH_ECCFAR          (STM32_FLASH_BASE+STM32_FLASH_ECCFAR_OFFSET)

/* Register Bitfield Definitions ****************************************************/
/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY_SHIFT    (0)       /* Bits 0-3: Latency */
#define FLASH_ACR_LATENCY_MASK     (15 << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)     ((n) << FLASH_ACR_LATENCY_SHIFT)  /* n wait states */
#  define FLASH_ACR_LATENCY_0      (0 << FLASH_ACR_LATENCY_SHIFT)    /* 0000: Zero wait states */
#  define FLASH_ACR_LATENCY_1      (1 << FLASH_ACR_LATENCY_SHIFT)    /* 0001: One wait state */
#  define FLASH_ACR_LATENCY_2      (2 << FLASH_ACR_LATENCY_SHIFT)    /* 0010: Two wait states */
#  define FLASH_ACR_LATENCY_3      (3 << FLASH_ACR_LATENCY_SHIFT)    /* 0011: Three wait states */
#  define FLASH_ACR_LATENCY_4      (4 << FLASH_ACR_LATENCY_SHIFT)    /* 0100: Four wait states */
#  define FLASH_ACR_LATENCY_5      (5 << FLASH_ACR_LATENCY_SHIFT)    /* 0101: Five wait states */
#  define FLASH_ACR_LATENCY_6      (6 << FLASH_ACR_LATENCY_SHIFT)    /* 0110: Six wait states */
#  define FLASH_ACR_LATENCY_7      (7 << FLASH_ACR_LATENCY_SHIFT)    /* 0111: Seven wait states */
#  define FLASH_ACR_LATENCY_8      (8 << FLASH_ACR_LATENCY_SHIFT)    /* 1000: Eight wait states */
#  define FLASH_ACR_LATENCY_9      (9 << FLASH_ACR_LATENCY_SHIFT)    /* 1001: Nine wait states */
#  define FLASH_ACR_LATENCY_10     (10 << FLASH_ACR_LATENCY_SHIFT)   /* 1010: Ten wait states */
#  define FLASH_ACR_LATENCY_11     (11 << FLASH_ACR_LATENCY_SHIFT)   /* 1011: Eleven wait states */
#  define FLASH_ACR_LATENCY_12     (12 << FLASH_ACR_LATENCY_SHIFT)   /* 1100: Twelve wait states */
#  define FLASH_ACR_LATENCY_13     (13 << FLASH_ACR_LATENCY_SHIFT)   /* 1101: Thirteen wait states */
#  define FLASH_ACR_LATENCY_14     (14 << FLASH_ACR_LATENCY_SHIFT)   /* 1110: Fourteen wait states */
#  define FLASH_ACR_LATENCY_15     (15 << FLASH_ACR_LATENCY_SHIFT)   /* 1111: Fifteen wait states */
#define FLASH_ACR_WRHIGHFREQ_SHIFT (4)       /* Bitd 4-5: Flash signal delay */
#define FLASH_ACR_WRHIGHFREQ_MASK  (3 << FLASH_ACR_WRHIGHFREQ_SHIFT)
#  define FLASH_ACR_WRHIGHFREQ(n)  ((n) << FLASH_ACR_WRHIGHFREQ_SHIFT)

/* Flash Control Register (CR) */

#define FLASH_CR_LOCK              (1 << 0)  /* Bit 0:  Lock */
#define FLASH_CR_PG                (1 << 1)  /* Bit 1:  Progrramming */
#define FLASH_CR_SER               (1 << 2)  /* Bit 2:  Sector erase */
#define FLASH_CR_BER               (1 << 3)  /* Bit 3: Bank erase */
#define FLASH_CR_PSIZE_SHIFT       (4)       /* Bits 4-5: Program size */
#define FLASH_CR_PSIZE_MASK        (3 << FLASH_CR_PSIZE_SHIFT)
#  define FLASH_CR_PSIZE_X8        (0 << FLASH_CR_PSIZE_SHIFT) /* 00: x8 */
#  define FLASH_CR_PSIZE_X16       (1 << FLASH_CR_PSIZE_SHIFT) /* 01: x16 */
#  define FLASH_CR_PSIZE_X32       (2 << FLASH_CR_PSIZE_SHIFT) /* 10: x32 */
#  define FLASH_CR_PSIZE_X64       (3 << FLASH_CR_PSIZE_SHIFT) /* 11: x64 */
#define FLASH_CR_FW                (1 << 6)  /* Bit 6: Force write */
#define FLASH_CR_START             (1 << 7)  /* Bit 7: Erase start */
#define FLASH_CR_SNB_SHIFT         (8)       /* Bits 8-10: Sector number */
#define FLASH_CR_SNB_MASK          (15 << FLASH_CR_SNB_SHIFT)  /* Used to clear FLASH_CR_SNB bits */
#  define FLASH_CR_SNB(n)          ((uint32_t)(n & 0x7) << FLASH_CR_SNB_SHIFT) /* Sector n, n=0..7 */
                                             /* Bits 11-14: Reserved */
#define FLASH_CR_CRCEN             (1 << 15) /* Bit 15: CRC control enable */
#define FLASH_CR_EOPIE             (1 << 16) /* Bit 16: End-of-program interrupt enable */
#define FLASH_CR_WRPERRIE          (1 << 17) /* Bit 17: Write protection error interrupt enable */
#define FLASH_CR_PGSERRIE          (1 << 18) /* Bit 18: Programming sequence error interrupt enable */
#define FLASH_CR_STRBERRIE         (1 << 19) /* Bit 19: Strobe error interrupt enable */
                                             /* Bit 20: Reserver */
#define FLASH_CR_INCERRIE          (1 << 21) /* Bit 21: Inconsistency error interrupt enbale */
#define FLASH_CR_OPERRIE           (1 << 22) /* Bit 22: Write/erase error interrupt enable */
#define FLASH_CR_RDPERRIE          (1 << 23) /* Bit 23: Read protection error interrupt enable */
#define FLASH_CR_RDSERRIE          (1 << 24) /* Bit 24: Secure error interrupt enable */
#define FLASH_CR_SNECCERRIE        (1 << 25) /* Bit 25: ECC single correction error interrupt enable */
#define FLASH_CR_DBECCERRIE        (1 << 26) /* Bit 26: ECC double detection error interrupt enable */
#define FLASH_CR_CRCENDIE          (1 << 27) /* Bit 27: CRC end of calculation interrupt enable */
                                             /* Bits 28-31: Reserverd */

/* Flash Status Register (SR) */

#define FLASH_SR_BSY               (1 << 0)  /* Bit 0: Busy */
#define FLASH_SR_WBNE              (1 << 1)  /* Bit 1: write buffer not empty */
#define FLASH_SR_QW                (1 << 2)  /* Bit 2: wait queue flag */
#define FLASH_SR_CRCBUSY           (1 << 3)  /* Bit 3: CRC busy flag */
                                             /* Bits 4-15: Reserved */
#define FLASH_SR_EOP               (1 << 16) /* Bit 16: End of program */
#define FLASH_SR_WROERR            (1 << 17) /* Bit 17: Write protection error */
#define FLASH_SR_PGSERR            (1 << 18) /* Bit 18: Programming sequence error */
#define FLASH_SR_STRBERR           (1 << 19) /* Bit 19: Strobe error */
                                             /* Bit 20: Reserved */
#define FLASH_SR_INCERR            (1 << 21) /* Bit 21: Inconsistency error */
#define FLASH_SR_OPERR             (1 << 22) /* Bit 22: Write/erase error */
#define FLASH_SR_RDPERR            (1 << 23) /* Bit 23: Read protection error */
#define FLASH_SR_RDSERR            (1 << 24) /* Bit 24: Secure error */
#define FLASH_SR_SNECCERR          (1 << 25) /* Bit 25: ECC single error */
#define FLASH_SR_DBECCERR          (1 << 26) /* Bit 26: ECC double detection error */
#define FLASH_SR_CRCEND            (1 << 27) /* Bit 27: CRC end of calculation */
                                             /* Bits 28-31: Reserved */

/* Flash Clear control register */

                                              /* Bits 0-15: Reserved */
#define FLASH_CLR_EOP               (1 << 16) /* Bit 16: Clear end of program */
#define FLASH_CLR_WROERR            (1 << 17) /* Bit 17: Clear write protection error */
#define FLASH_CLR_PGSERR            (1 << 18) /* Bit 18: Clear programming sequence error */
#define FLASH_CLR_STRBERR           (1 << 19) /* Bit 19: Clear strobe error */
                                              /* Bit 20: Reserved */
#define FLASH_CLR_INCERR            (1 << 21) /* Bit 21: Clear inconsistency error */
#define FLASH_CLR_OPERR             (1 << 22) /* Bit 22: Clear write/erase error */
#define FLASH_CLR_RDPERR            (1 << 23) /* Bit 23: Clear read protection error */
#define FLASH_CLR_RDSERR            (1 << 24) /* Bit 24: Clear secure error */
#define FLASH_CLR_SNECCERR          (1 << 25) /* Bit 25: Clear ECC single error */
#define FLASH_CLR_DBECCERR          (1 << 26) /* Bit 26: Clear ECC double detection error */
#define FLASH_CLR_CRCEND            (1 << 27) /* Bit 27: Clear CRC end of calculation */
                                              /* Bits 28-31: Reserved */

/* Flash Option Control Register (OPTCR) */

#define FLASH_OPTCR_OPTLOCK         (1 << 0)  /* Bit 0: Option lock */
#define FLASH_OPTCR_OPTSTRT         (1 << 1)  /* Bit 1: Option start */
                                              /* Bits 2-3: Reserved */
#define FLASH_OPTCR_MER             (1 << 4)  /* Bit 4: Mass erase  */
                                              /* Bits 5-29: Reserved */
#define FLASH_OPTCR_CHANGEERRIE     (1 << 30) /* Bit 30: Option byte change error interrupt enable */
#define FLASH_OPTCR_SWAPBANK        (1 << 31) /* Bit 31: Bank swapping */

/* Flash Option Status Register (OPTSR) */

#define FLASH_OPTSR_BUSYV           (1 << 0)  /* Bit 0: Option byte change busy */
                                              /* Bit 1: Reserved */
#define FLASH_OPTSR_BORLEV_SHIFT    (2)       /* Bits 2-3: Brownout level option */
#define FLASH_OPTSR_BORLEV_MASK     (3 << FLASH_OPTSR_BORLEV_SHIFT)
#  define FLASH_OPTSR_BORLEV_0      (0 << FLASH_OPTSR_BORLEV_SHIFT)
#  define FLASH_OPTSR_BORLEV_1      (1 << FLASH_OPTSR_BORLEV_SHIFT)
#  define FLASH_OPTSR_BORLEV_2      (2 << FLASH_OPTSR_BORLEV_SHIFT)
#  define FLASH_OPTSR_BORLEV_3      (3 << FLASH_OPTSR_BORLEV_SHIFT)
#define FLASH_OPTSR_IWDGSW          (1 << 4)  /* Bit 4: IWDG control mode */
                                              /* Bit 5: Reserved */
#define FLASH_OPTSR_NRSTSTOP        (1 << 6)  /* Bit 6: DStop entry reset */
#define FLASH_OPTSR_NRSTSTDY        (1 << 7)  /* Bit 7: DStandby entry reset */
#define FLASH_OPTSR_RDP_SHIFT       (8)       /* Bits 8-15: Readout protection level */
#define FLASH_OPTSR_RDP_MASK        (0xff << FLASH_OPTSR_RDP_SHIFT)
#  define FLASH_OPTSR_RDP(n)        ((uint32_t)(n) << FLASH_OPTSR_RDP_SHIFT)
#define FLASH_OPTSR_IWDGFZSTOP      (1 << 17)  /* Bit 17: IWDG Stop mode freeze */
#define FLASH_OPTSR_IWDGFZSTBY      (1 << 18)  /* Bit 18: IWDG Standby mode freeze */
#define FLASH_OPTSR_STRAMSIZE_SHIFT (19)       /* Bits 19-20: ST RAM size */
#define FLASH_OPTSR_STRAMSIZE_MASK  (3 << FLASH_OPTSR_STRAMSIZE_SHIFT)
#  define FLASH_OPTSR_STRAMSIZE_2   (0 << FLASH_OPTSR_STRAMSIZE_SHIFT)
#  define FLASH_OPTSR_STRAMSIZE_4   (1 << FLASH_OPTSR_STRAMSIZE_SHIFT)
#  define FLASH_OPTSR_STRAMSIZE_8   (2 << FLASH_OPTSR_STRAMSIZE_SHIFT)
#  define FLASH_OPTSR_STRAMSIZE_16  (3 << FLASH_OPTSR_STRAMSIZE_SHIFT)
#define FLASH_OPTSR_SECURITY        (1 << 21) /* Bit 21: Security enable*/
                                              /* Bits 22-28: Reserved */
#define FLASH_OPTSR_IOHSLV          (1 << 29) /* Bit 29: IO high-speed at low-volateg */
#define FLASH_OPTSR_CHANGEERR       (1 << 30) /* Bit 30: Option byte change error */
#define FLASH_OPTSR_SWAPBANK        (1 << 31) /* Bit 31: Bank swapping status */

/* Flash Option Clear Control Register (OPTCCR) */

                                               /* Bits 0-29: Reserved */
#define FLASH_OPTCCR_OPTLOCK        (1 << 30)  /* Bit 30: OPTCHANGEERR reset */
                                               /* Bit 31: Reserved */

/* TODO: Flash Protection Address (PRAR) */

/* TODO: Flash Secure Address (SCAR) */

/* Flash Write Sector Protection (WPSN) */

#define FLASH_WPSN_WRPSN_SHIFT     (0) /* Bits 0-7L Sector write protection option */
#define FLASH_WPSN_WRPSN_MASK      (15 << FLASH_WPSN_WRPSN_SHIFT)
                                       /* Bits 8-31: Reserved */

/* Flash Register Boot Address (BOOT) */

#define FLASH_BOOT_ADD0_SHIFT      (0)  /* Bits 0-15: Boot addres 0 */
#define FLASH_BOOT_ADD0_MASK       (0xff << FLASH_BOOT_ADD0_SHIFT)
#define FLASH_BOOT_ADD1_SHIFT      (16) /* Bits 16-31: Boot addres 1 */
#define FLASH_BOOT_ADD2_MASK       (0xff << FLASH_BOOT_ADD2_SHIFT)

/* TODO: Flash CRC Control Register (CRCCR) */

/* TODO: Flash CRC Start Address Register (CRCSADDR) */

/* TODO: Flash CRC End Address Register (CRCSEDDR) */

#endif /* __ARCH_ARM_SRC_STM327_CHIP_STM32H7X3XX_FLASH_H */
