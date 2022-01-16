/****************************************************************************
 * arch/arm/src/stm32f7/hardware/stm32f74xx75xx_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F74XX75XX_FLASH_H
#define __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F74XX75XX_FLASH_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash size is known from the chip selection:
 *
 *   When CONFIG_STM32F7_FLASH_OVERRIDE_DEFAULT is set the
 *   CONFIG_STM32F7_FLASH_CONFIG_x selects the default FLASH size based on
 *   the chip part number.
 *   This value can be overridden with CONFIG_STM32F7_FLASH_OVERRIDE_x
 *
 *   Parts STM32F74xxE have 512Kb of FLASH
 *   Parts STM32F74xxG have 1024Kb of FLASH
 *
 *   N.B. Only Single bank mode is supported
 */

#define _K(x) ((x)*1024)

#if !defined(CONFIG_STM32F7_FLASH_OVERRIDE_DEFAULT) && \
    !defined(CONFIG_STM32F7_FLASH_OVERRIDE_E) && \
    !defined(CONFIG_STM32F7_FLASH_OVERRIDE_G) && \
    !defined(CONFIG_STM32F7_FLASH_CONFIG_E) && \
    !defined(CONFIG_STM32F7_FLASH_CONFIG_G)
#  define CONFIG_STM32F7_FLASH_OVERRIDE_E
#  warning "Flash size not defined defaulting to 512KiB (E)"
#endif

#if !defined(CONFIG_STM32F7_FLASH_OVERRIDE_DEFAULT)

#  undef CONFIG_STM32F7_FLASH_CONFIG_E
#  undef CONFIG_STM32F7_FLASH_CONFIG_G

#  if defined(CONFIG_STM32F7_FLASH_OVERRIDE_E)

#    define CONFIG_STM32F7_FLASH_CONFIG_E

#  elif defined(CONFIG_STM32F7_FLASH_OVERRIDE_G)

#    define CONFIG_STM32F7_FLASH_CONFIG_G

#  endif
#endif

#if defined(CONFIG_STM32F7_FLASH_CONFIG_E)

#  define STM32_FLASH_NPAGES      6
#  define STM32_FLASH_SIZE        _K((4 * 32) + (1 * 128) + (1 * 256))
#  define STM32_FLASH_SIZES       {_K(32), _K(32), _K(32), _K(32),  \
                                   _K(128), _K(256)}

#elif defined(CONFIG_STM32F7_FLASH_CONFIG_G)

#  define STM32_FLASH_NPAGES      8
#  define STM32_FLASH_SIZE        _K((4 * 32) + (1 * 128) + (3 * 256))
#  define STM32_FLASH_SIZES       {_K(32), _K(32), _K(32), _K(32),  \
                                  _K(128), _K(256), _K(256), _K(256)}

#endif

/* Register Offsets *********************************************************/

#define STM32_FLASH_ACR_OFFSET     0x0000
#define STM32_FLASH_KEYR_OFFSET    0x0004
#define STM32_FLASH_OPTKEYR_OFFSET 0x0008
#define STM32_FLASH_SR_OFFSET      0x000c
#define STM32_FLASH_CR_OFFSET      0x0010
#define STM32_FLASH_OPTCR_OFFSET   0x0014
#define STM32_FLASH_OPTCR1_OFFSET  0x0018

/* Register Addresses *******************************************************/

#define STM32_FLASH_ACR            (STM32_FLASHIF_BASE+STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_KEYR           (STM32_FLASHIF_BASE+STM32_FLASH_KEYR_OFFSET)
#define STM32_FLASH_OPTKEYR        (STM32_FLASHIF_BASE+STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_SR             (STM32_FLASHIF_BASE+STM32_FLASH_SR_OFFSET)
#define STM32_FLASH_CR             (STM32_FLASHIF_BASE+STM32_FLASH_CR_OFFSET)
#define STM32_FLASH_OPTCR          (STM32_FLASHIF_BASE+STM32_FLASH_OPTCR_OFFSET)
#define STM32_FLASH_OPTCR1         (STM32_FLASHIF_BASE+STM32_FLASH_OPTCR1_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY_SHIFT    (0)       /* Bits 0-1: Latency */
#define FLASH_ACR_LATENCY_MASK     (7 << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)     ((n) << FLASH_ACR_LATENCY_SHIFT)  /* n wait states */
#  define FLASH_ACR_LATENCY_0      (0 << FLASH_ACR_LATENCY_SHIFT)    /* 000: Zero wait states */
#  define FLASH_ACR_LATENCY_1      (1 << FLASH_ACR_LATENCY_SHIFT)    /* 001: One wait state */
#  define FLASH_ACR_LATENCY_2      (2 << FLASH_ACR_LATENCY_SHIFT)    /* 010: Two wait states */
#  define FLASH_ACR_LATENCY_3      (3 << FLASH_ACR_LATENCY_SHIFT)    /* 011: Three wait states */
#  define FLASH_ACR_LATENCY_4      (4 << FLASH_ACR_LATENCY_SHIFT)    /* 100: Four wait states */
#  define FLASH_ACR_LATENCY_5      (5 << FLASH_ACR_LATENCY_SHIFT)    /* 101: Five wait states */
#  define FLASH_ACR_LATENCY_6      (6 << FLASH_ACR_LATENCY_SHIFT)    /* 110: Six wait states */
#  define FLASH_ACR_LATENCY_7      (7 << FLASH_ACR_LATENCY_SHIFT)    /* 111: Seven wait states */

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
#define FLASH_SR_BSY               (1 << 16) /* Bit 16: Busy */

/* Flash Control Register (CR) */

#define FLASH_CR_PG                (1 << 0)  /* Bit 0:  Programming */
#define FLASH_CR_SER               (1 << 1)  /* Bit 1:  Sector Erase */
#define FLASH_CR_MER               (1 << 2)  /* Bit 2:  Mass Erase sectors 0..11 */
#define FLASH_CR_SNB_SHIFT         (3)       /* Bits 3-6: Sector number */
#define FLASH_CR_SNB_MASK          (0xf << FLASH_CR_SNB_SHIFT)
#  define FLASH_CR_SNB(n)          (((uint32_t)((n) % 8) << FLASH_CR_SNB_SHIFT) | ((n / 8) << 6)) /* Sector n, n=0..23 */

#define FLASH_CR_PSIZE_SHIFT       (8)       /* Bits 8-9: Program size */
#define FLASH_CR_PSIZE_MASK        (3 << FLASH_CR_PSIZE_SHIFT)
#  define FLASH_CR_PSIZE_X8        (0 << FLASH_CR_PSIZE_SHIFT) /* Program x8 */
#  define FLASH_CR_PSIZE_X16       (1 << FLASH_CR_PSIZE_SHIFT) /* Program x16 */
#  define FLASH_CR_PSIZE_X32       (2 << FLASH_CR_PSIZE_SHIFT) /* Program x32 */
#  define FLASH_CR_PSIZE_X64       (3 << FLASH_CR_PSIZE_SHIFT) /* Program x64 */

#define FLASH_CR_STRT              (1 << 16) /* Bit 16: Start Erase */
#define FLASH_CR_EOPIE             (1 << 24) /* Bit 24: End of operation interrupt enable */
#define FLASH_CR_ERRIE             (1 << 25) /* Bit 25: Error interrupt enable */
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

#endif /* __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F74XX75XX_FLASH_H */
