/************************************************************************************
 * arch/arm/src/stm32f7/hardware/stm32f72xx73xx_flash.h
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            Bob Feretich <bob.feretich@rafresearch.com>
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

#ifndef __ARCH_ARM_SRC_STM327_CHIP_STM32F72XX73XX_FLASH_H
#define __ARCH_ARM_SRC_STM327_CHIP_STM32F72XX73XX_FLASH_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Flash size is known from the chip selection:
 *
 *   When CONFIG_STM32F7_FLASH_OVERRIDE_DEFAULT is set the
 *   CONFIG_STM32F7_FLASH_CONFIG_x selects the default FLASH size based on the chip
 *   part number. This value can be overridden with CONFIG_STM32F7_FLASH_OVERRIDE_x
 *
 *   Parts STM32F72xxC have 256Kb of FLASH
 *   Parts STM32F72xxE have 512Kb of FLASH
 *   Parts STM32F73xxC have 256Kb of FLASH
 *   Parts STM32F73xxE have 512Kb of FLASH
 *
 *   N.B. Only Single bank mode is supported
 */

#define _K(x) ((x)*1024)

#if !defined(CONFIG_STM32F7_FLASH_OVERRIDE_DEFAULT) && \
    !defined(CONFIG_STM32F7_FLASH_OVERRIDE_E) && \
    !defined(CONFIG_STM32F7_FLASH_OVERRIDE_C) && \
    !defined(CONFIG_STM32F7_FLASH_CONFIG_E) && \
    !defined(CONFIG_STM32F7_FLASH_CONFIG_C)
#  define CONFIG_STM32F7_FLASH_OVERRIDE_C
#  warning "Flash size not defined defaulting to 256KiB (C)"
#endif

#if !defined(CONFIG_STM32F7_FLASH_OVERRIDE_DEFAULT)

#  undef CONFIG_STM32F7_FLASH_CONFIG_C
#  undef CONFIG_STM32F7_FLASH_CONFIG_E
#  undef CONFIG_STM32F7_FLASH_CONFIG_G

#  if defined(CONFIG_STM32F7_FLASH_OVERRIDE_C)
#    define CONFIG_STM32F7_FLASH_CONFIG_C

#  elif defined(CONFIG_STM32F7_FLASH_OVERRIDE_E)
#    define CONFIG_STM32F7_FLASH_CONFIG_E

#  endif
#endif

#if defined(CONFIG_STM32F7_FLASH_CONFIG_C)

#  define STM32_FLASH_NPAGES      6
#  define STM32_FLASH_SIZE        _K((4 * 16) + (1 * 64) + (1 * 128))
#  define STM32_FLASH_SIZES       {_K(16), _K(16), _K(16), _K(16),  \
                                   _K(64), _K(128)}

#elif defined(CONFIG_STM32F7_FLASH_CONFIG_E)

#  define STM32_FLASH_NPAGES      8
#  define STM32_FLASH_SIZE        _K((4 * 16) + (1 * 64) + (3 * 128))
#  define STM32_FLASH_SIZES       {_K(16), _K(16), _K(16), _K(16),  \
                                   _K(64), _K(128), _K(128), _K(128)}

#endif

/* Register Offsets *****************************************************************/

#define STM32_FLASH_ACR_OFFSET     0x0000
#define STM32_FLASH_KEYR_OFFSET    0x0004
#define STM32_FLASH_OPTKEYR_OFFSET 0x0008
#define STM32_FLASH_SR_OFFSET      0x000c
#define STM32_FLASH_CR_OFFSET      0x0010
#define STM32_FLASH_OPTCR_OFFSET   0x0014
#define STM32_FLASH_OPTCR1_OFFSET  0x0018

/* Register Addresses ***************************************************************/

#define STM32_FLASH_ACR            (STM32_FLASHIF_BASE+STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_KEYR           (STM32_FLASHIF_BASE+STM32_FLASH_KEYR_OFFSET)
#define STM32_FLASH_OPTKEYR        (STM32_FLASHIF_BASE+STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_SR             (STM32_FLASHIF_BASE+STM32_FLASH_SR_OFFSET)
#define STM32_FLASH_CR             (STM32_FLASHIF_BASE+STM32_FLASH_CR_OFFSET)
#define STM32_FLASH_OPTCR          (STM32_FLASHIF_BASE+STM32_FLASH_OPTCR_OFFSET)
#define STM32_FLASH_OPTCR1         (STM32_FLASHIF_BASE+STM32_FLASH_OPTCR1_OFFSET)

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
#define FLASH_ACR_PRFTEN           (1 << 8)  /* FLASH prefetch enable */
#define FLASH_ACR_ARTEN            (1 << 9)  /* Bit 9:  ART Accelerator Enable */
#define FLASH_ACR_ARTRST           (1 << 11) /* Bit 11: ART Accelerator reset */

/* Flash Status Register (SR) */

#define FLASH_SR_EOP               (1 << 0)  /* Bit 0:  End of operation */
#define FLASH_SR_OPERR             (1 << 1)  /* Bit 1:  Operation error */
#define FLASH_SR_WRPERR            (1 << 4)  /* Bit 4:  Write protection error */
#define FLASH_SR_PGAERR            (1 << 5)  /* Bit 5:  Programming alignment error */
#define FLASH_SR_PGPERR            (1 << 6)  /* Bit 6:  Programming parallelism error */
#define FLASH_SR_PGSERR            (1 << 7)  /* Bit 7:  Programming sequence error */
#define FLASH_SR_PGRERR            (1 << 7)  /* Bit 7:  PCROP protection error */
#define FLASH_SR_BSY               (1 << 16) /* Bit 16: Busy */

/* Flash Control Register (CR) */

#define FLASH_CR_PG                (1 << 0)  /* Bit 0:  Programming */
#define FLASH_CR_SER               (1 << 1)  /* Bit 1:  Sector Erase */
#define FLASH_CR_MER               (1 << 2)  /* Bit 2:  Mass Erase sectors 0..11 */
#define FLASH_CR_SNB_SHIFT         (3)       /* Bits 3-6: Sector number */
#define FLASH_CR_SNB_MASK          (0xf << FLASH_CR_SNB_SHIFT)  /* Used to clear FLASH_CR_SNB bits */
#  define FLASH_CR_SNB(n)          ((uint32_t)(n & 0x7) << FLASH_CR_SNB_SHIFT) /* Sector n, n=0..7 */
#define FLASH_CR_PSIZE_SHIFT       (8)       /* Bits 8-9: Program size */
#define FLASH_CR_PSIZE_MASK        (3 << FLASH_CR_PSIZE_SHIFT)
#  define FLASH_CR_PSIZE_X8        (0 << FLASH_CR_PSIZE_SHIFT) /* Program x8 */
#  define FLASH_CR_PSIZE_X16       (1 << FLASH_CR_PSIZE_SHIFT) /* Program x16 */
#  define FLASH_CR_PSIZE_X32       (2 << FLASH_CR_PSIZE_SHIFT) /* Program x32 */
#  define FLASH_CR_PSIZE_X64       (3 << FLASH_CR_PSIZE_SHIFT) /* Program x64 */
#define FLASH_CR_STRT              (1 << 16) /* Bit 16: Start Erase */
#define FLASH_CR_EOPIE             (1 << 24) /* Bit 24: End of operation interrupt enable */
#define FLASH_CR_ERRIE             (1 << 25) /* Bit 25: Error interrupt enable */
#define FLASH_CR_RDERRIE           (1 << 26) /* Bit 26: PCROP error interrupt enable */
#define FLASH_CR_LOCK              (1 << 31) /* Bit 31: Lock */

/* Flash Option Control Register (OPTCR) */

#define FLASH_OPTCR_OPTLOCK        (1 << 0)  /* Bit 0:  Option lock */
#define FLASH_OPTCR_OPTSTRT        (1 << 1)  /* Bit 1:  Option start */
#define FLASH_OPTCR_BORLEV_SHIFT   (2)       /* Bits 2-3: BOR reset Level */
#define FLASH_OPTCR_BORLEV_MASK    (3 << FLASH_OPTCR_BORLEV_SHIFT)
#  define FLASH_OPTCR_VBOR3        (0 << FLASH_OPTCR_BORLEV_SHIFT) /* BOR Level 3 */
#  define FLASH_OPTCR_VBOR2        (1 << FLASH_OPTCR_BORLEV_SHIFT) /* BOR Level 2 */
#  define FLASH_OPTCR_VBOR1        (2 << FLASH_OPTCR_BORLEV_SHIFT) /* BOR Level 1 */
#  define FLASH_OPTCR_VBOR0        (3 << FLASH_OPTCR_BORLEV_SHIFT) /* BOR off */
#define FLASH_OPTCR_USER_SHIFT     (4)       /* Bits 5-7: User option bytes */
#define FLASH_OPTCR_USER_MASK      (15 << FLASH_OPTCR_USER_SHIFT)
#  define FLASH_OPTCR_WWDG_SW      (1 << 4)  /* Bit 4: WWDG_SW */
#  define FLASH_OPTCR_IWDG_SW      (1 << 5)  /* Bit 5: IWDG_SW */
#  define FLASH_OPTCR_NRST_STOP    (1 << 6)  /* Bit 6: nRST_STOP */
#  define FLASH_OPTCR_NRST_STDBY   (1 << 7)  /* Bit 7: nRST_STDBY */
#define FLASH_OPTCR_RDP_SHIFT      (8)       /* Bits 8-15: Read protect */
#define FLASH_OPTCR_RDP_MASK       (0xff << FLASH_OPTCR_RDP_SHIFT)
#  define FLASH_OPTCR_RDP(n)       ((uint32_t)(n) << FLASH_OPTCR_RDP_SHIFT)
#define FLASH_OPTCR_NWRP_SHIFT     (16)      /* Bits 16-23: Not write protect */
#define FLASH_OPTCR_NWRP_MASK      (0xff << FLASH_OPTCR_NWRP_SHIFT)
#   define FLASH_OPTCR_NWRP(n)     ((uint32_t)(n) << FLASH_OPTCR_NWRP_SHIFT)
#define FLASH_OPTCR_IWDG_STDBY     (1 << 30) /* Bit 30: IWDG freeze in stop mode */
#define FLASH_OPTCR_IWDG_STOP      (1 << 31) /* Bit 31: IWDG freeze in standby mode */

/* Flash Option Control Register (OPTCR1) */

#define FLASH_OPTCR1_BOOTADD0_SHIFT (0)      /* Bits 0-15: Boot base address when Boot pin=0 */
#define FLASH_OPTCR1_BOOTADD0_MASK  (0xffff << FLASH_OPTCR1_BOOTADD0_SHIFT)
#  define FLASH_OPTCR1_BOOTADD0(n)  ((uint32_t)(n) << FLASH_OPTCR1_BOOTADD0_SHIFT)
#define FLASH_OPTCR1_BOOTADD1_SHIFT (16)     /* Bits 16-31:Boot base address when Boot pin=1 */
#define FLASH_OPTCR1_BOOTADD1_MASK  (0xffff << FLASH_OPTCR1_BOOTADD1_SHIFT)
#  define FLASH_OPTCR1_BOOTADD1(n)  ((uint32_t)(n) << FLASH_OPTCR1_BOOTADD1_SHIFT)

#endif /* __ARCH_ARM_SRC_STM327_CHIP_STM32F72XX73XX_FLASH_H */
