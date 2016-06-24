/************************************************************************************
 * arch/arm/src/stm32/chip/stm32_flash.h
 *
 *   Copyright (C) 2009, 2011, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David Sidrane <david_s5@nscdg.com>
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_FLASH_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_FLASH_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define _K(x) ((x)*1024)

#if !defined(CONFIG_STM32_FLASH_CONFIG_DEFAULT) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_4) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_6) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_8) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_B) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_C) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_D) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_E) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_F) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_G) && \
    !defined(CONFIG_STM32_FLASH_CONFIG_I)
#  define CONFIG_STM32_FLASH_CONFIG_DEFAULT
#endif

#if defined(CONFIG_STM32_FLASH_CONFIG_DEFAULT)
#  if defined(CONFIG_STM32_STM32L15XX)

/* The STM32 L15xx/L16xx can support up to 384KB of FLASH.  (In reality, supported
 * L15xx parts have no more than 128KB). The program memory block is divided into
 * 96 sectors of 4 Kbytes each, and each sector is further split up into 16 pages of
 * 256 bytes each. The sector is the write protection granularity. In total, the
 * program memory block contains 1536 pages.
 */

#    define STM32_FLASH_NPAGES        1536
#    define STM32_FLASH_PAGESIZE      256

#  elif defined(CONFIG_STM32_LOWDENSITY)
#    define STM32_FLASH_NPAGES        32
#    define STM32_FLASH_PAGESIZE      1024

#  elif  defined(CONFIG_STM32_MEDIUMDENSITY)
#    define STM32_FLASH_NPAGES        128
#    define STM32_FLASH_PAGESIZE      1024

#  elif  defined(CONFIG_STM32_CONNECTIVITYLINE)
#    define STM32_FLASH_NPAGES        128
#    define STM32_FLASH_PAGESIZE      2048

#  elif defined(CONFIG_STM32_HIGHDENSITY)
#    define STM32_FLASH_NPAGES        256
#    define STM32_FLASH_PAGESIZE      2048

#  elif defined(CONFIG_STM32_STM32F30XX)
#    define STM32_FLASH_NPAGES        128
#    define STM32_FLASH_PAGESIZE      2048

#  elif defined(CONFIG_STM32_STM32F37XX)
#    define STM32_FLASH_NPAGES        128
#    define STM32_FLASH_PAGESIZE      2048

#  elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#      define STM32_FLASH_NPAGES      8
#      define STM32_FLASH_SIZE        _K((4 * 16) + (1 * 64) + (3 * 128))
#      define STM32_FLASH_SIZES       {_K(16), _K(16), _K(16), _K(16), \
                                      _K(64),_K(128), _K(128), _K(128)}

  /* STM32F4 has mixed page size */

#    undef STM32_FLASH_PAGESIZE
#  endif
#endif /* CONFIG_STM32_FLASH_CONFIG_DEFAULT */

/* Override of the Flash Has been Chosen */

#if !defined(CONFIG_STM32_FLASH_CONFIG_DEFAULT)

/* Define the Valid Configuration the F2 and F4  */

#  if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)

#    if defined(CONFIG_STM32_FLASH_CONFIG_B)
#      define STM32_FLASH_NPAGES      5
#      define STM32_FLASH_SIZE        _K((4 * 16) + (1 * 64))
#      define STM32_FLASH_SIZES       {_K(16), _K(16), _K(16), _K(16), \
                                      _K(64)}

#    elif defined(CONFIG_STM32_FLASH_CONFIG_C)
#      define STM32_FLASH_NPAGES      6
#      define STM32_FLASH_SIZE        _K((4 * 16) + (1 * 64) + (1 * 128))
#      define STM32_FLASH_SIZES       {_K(16), _K(16), _K(16), _K(16), \
                                       _K(64), _K(128)}

#    elif defined(CONFIG_STM32_FLASH_CONFIG_D) && defined(CONFIG_STM32_STM32F40XX)
#      define STM32_FLASH_NPAGES      7
#      define STM32_FLASH_SIZE        _K((4 * 16) + (1 * 64) + (2 * 128))
#      define STM32_FLASH_SIZES       {_K(16), _K(16), _K(16), _K(16), \
                                      _K(64), _K(128), _K(128)}

#    elif defined(CONFIG_STM32_FLASH_CONFIG_E)
#      define STM32_FLASH_NPAGES      8
#      define STM32_FLASH_SIZE        _K((4 * 16) + (1 * 64) + (3 * 128))
#      define STM32_FLASH_SIZES       {_K(16), _K(16), _K(16), _K(16),  \
                                      _K(64), _K(128), _K(128), _K(128)}

#    elif defined(CONFIG_STM32_FLASH_CONFIG_F) &&  defined(CONFIG_STM32_STM32F20XX)
#      define STM32_FLASH_NPAGES      9
#      define STM32_FLASH_SIZE        _K((4 * 16) + (1 * 64) + (4 * 128))
#      define STM32_FLASH_SIZES       {_K(16), _K(16), _K(16), _K(16),   \
                                      _K(64), _K(128), _K(128), _K(128), \
                                      _K(128)}

#    elif defined(CONFIG_STM32_FLASH_CONFIG_G)
#      define STM32_FLASH_NPAGES      12
#      define STM32_FLASH_SIZE        _K((4 * 16) + (1 * 64) + (7 * 128))
#      define STM32_FLASH_SIZES       {_K(16), _K(16), _K(16), _K(16),    \
                                      _K(64), _K(128), _K(128), _K(128),  \
                                      _K(128), _K(128), _K(128), _K(128)}

#    elif defined(CONFIG_STM32_FLASH_CONFIG_I) && defined(CONFIG_STM32_STM32F40XX)
#      define STM32_FLASH_NPAGES      24
#      define STM32_FLASH_SIZE        _K((4 * 16) + (1 * 64) + (7 * 128)) + \
                                      _K((4 * 16) + (1 * 64) + (7 * 128))
#      define STM32_FLASH_SIZES       {_K(16), _K(16), _K(16), _K(16),      \
                                      _K(64), _K(128), _K(128), _K(128),    \
                                      _K(128), _K(128), _K(128), _K(128),   \
                                      _K(16), _K(16), _K(16), _K(16),       \
                                      _K(64), _K(128), _K(128), _K(128),    \
                                      _K(128), _K(128), _K(128), _K(128)}
#    endif

/* Define the Valid Configuration the F1 and F3  */

#  else
#    if defined(CONFIG_STM32_FLASH_CONFIG_4)
#      define STM32_FLASH_NPAGES      16
#      define STM32_FLASH_PAGESIZE    1024
#    elif defined(CONFIG_STM32_FLASH_CONFIG_6)
#      define STM32_FLASH_NPAGES      32
#      define STM32_FLASH_PAGESIZE    1024
#    elif defined(CONFIG_STM32_FLASH_CONFIG_8)
#      define STM32_FLASH_NPAGES      64
#      define STM32_FLASH_PAGESIZE    1024
#    elif defined(CONFIG_STM32_FLASH_CONFIG_B)
#      define STM32_FLASH_NPAGES      128
#      define STM32_FLASH_PAGESIZE    1024
#    elif defined(CONFIG_STM32_FLASH_CONFIG_C)
#      define STM32_FLASH_NPAGES      128
#      define STM32_FLASH_PAGESIZE    2048
#    elif defined(CONFIG_STM32_FLASH_CONFIG_D)
#      define STM32_FLASH_NPAGES      192
#      define STM32_FLASH_PAGESIZE    2048
#    elif defined(CONFIG_STM32_FLASH_CONFIG_E)
#      define STM32_FLASH_NPAGES      256
#      define STM32_FLASH_PAGESIZE    2048
#    elif defined(CONFIG_STM32_FLASH_CONFIG_F)
#      define STM32_FLASH_NPAGES      384
#      define STM32_FLASH_PAGESIZE    2048
#    elif defined(CONFIG_STM32_FLASH_CONFIG_G)
#      define STM32_FLASH_NPAGES      512
#      define STM32_FLASH_PAGESIZE    2048
#    elif defined(CONFIG_STM32_FLASH_CONFIG_I)
#    endif
#  endif
#endif

#ifdef STM32_FLASH_PAGESIZE
#  define STM32_FLASH_SIZE            (STM32_FLASH_NPAGES * STM32_FLASH_PAGESIZE)
#endif /* def STM32_FLASH_PAGESIZE */

/* Register Offsets *****************************************************************/

#define STM32_FLASH_ACR_OFFSET     0x0000
#define STM32_FLASH_KEYR_OFFSET    0x0004
#define STM32_FLASH_OPTKEYR_OFFSET 0x0008
#define STM32_FLASH_SR_OFFSET      0x000c
#define STM32_FLASH_CR_OFFSET      0x0010

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) || \
    defined(CONFIG_STM32_STM32F37XX)
#  define STM32_FLASH_AR_OFFSET    0x0014
#  define STM32_FLASH_OBR_OFFSET   0x001c
#  define STM32_FLASH_WRPR_OFFSET  0x0020
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define STM32_FLASH_OPTCR_OFFSET 0x0014
#endif

#if defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429)
#  define STM32_FLASH_OPTCR1_OFFSET 0x0018
#endif

/* Register Addresses ***************************************************************/

#define STM32_FLASH_ACR            (STM32_FLASHIF_BASE+STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_KEYR           (STM32_FLASHIF_BASE+STM32_FLASH_KEYR_OFFSET)
#define STM32_FLASH_OPTKEYR        (STM32_FLASHIF_BASE+STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_SR             (STM32_FLASHIF_BASE+STM32_FLASH_SR_OFFSET)
#define STM32_FLASH_CR             (STM32_FLASHIF_BASE+STM32_FLASH_CR_OFFSET)

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) || \
    defined(CONFIG_STM32_STM32F37XX)
#  define STM32_FLASH_AR           (STM32_FLASHIF_BASE+STM32_FLASH_AR_OFFSET)
#  define STM32_FLASH_OBR          (STM32_FLASHIF_BASE+STM32_FLASH_OBR_OFFSET)
#  define STM32_FLASH_WRPR         (STM32_FLASHIF_BASE+STM32_FLASH_WRPR_OFFSET)
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define STM32_FLASH_OPTCR        (STM32_FLASHIF_BASE+STM32_FLASH_OPTCR_OFFSET)
#endif
#if defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429)
#  define STM32_FLASH_OPTCR1       (STM32_FLASHIF_BASE+STM32_FLASH_OPTCR1_OFFSET)
#endif

/* Register Bitfield Definitions ****************************************************/
/* Flash Access Control Register (ACR) */

#if defined(CONFIG_STM32_STM32L15XX)
#  define FLASH_ACR_LATENCY         (1 << 0)  /* Bit 0: Latency */
#  define FLASH_ACR_PRFTEN          (1 << 1)  /* Bit 1: Prefetch enable */
#  define FLASH_ACR_ACC64           (1 << 2)  /* Bit 2: 64-bit access */
#  define FLASH_ACR_SLEEP_PD        (1 << 3)  /* Bit 3: Flash mode during Sleep */
#  define FLASH_ACR_RUN_PD          (1 << 4)  /* Bit 4: Flash mode during Run */
#else
#  define FLASH_ACR_LATENCY_SHIFT   (0)
#  define FLASH_ACR_LATENCY_MASK    (7 << FLASH_ACR_LATENCY_SHIFT)
#    define FLASH_ACR_LATENCY(n)    ((n) << FLASH_ACR_LATENCY_SHIFT)  /* n wait states */
#    define FLASH_ACR_LATENCY_0     (0 << FLASH_ACR_LATENCY_SHIFT)    /* 000: Zero wait states */
#    define FLASH_ACR_LATENCY_1     (1 << FLASH_ACR_LATENCY_SHIFT)    /* 001: One wait state */
#    define FLASH_ACR_LATENCY_2     (2 << FLASH_ACR_LATENCY_SHIFT)    /* 010: Two wait states */
#    define FLASH_ACR_LATENCY_3     (3 << FLASH_ACR_LATENCY_SHIFT)    /* 011: Three wait states */
#    define FLASH_ACR_LATENCY_4     (4 << FLASH_ACR_LATENCY_SHIFT)    /* 100: Four wait states */
#    define FLASH_ACR_LATENCY_5     (5 << FLASH_ACR_LATENCY_SHIFT)    /* 101: Five wait states */
#    define FLASH_ACR_LATENCY_6     (6 << FLASH_ACR_LATENCY_SHIFT)    /* 110: Six wait states */
#    define FLASH_ACR_LATENCY_7     (7 << FLASH_ACR_LATENCY_SHIFT)    /* 111: Seven wait states */

#  if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) || \
      defined(CONFIG_STM32_STM32F37XX)
#    define FLASH_ACR_HLFCYA        (1 << 3)  /* Bit 3: FLASH half cycle access */
#    define FLASH_ACR_PRTFBE        (1 << 4)  /* Bit 4: FLASH prefetch enable */
#    if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
#      define FLASH_ACR_PRFTBS      (1 << 5)  /* Bit 5: FLASH prefetch buffer status */
#    endif
#  elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#    define FLASH_ACR_PRFTEN        (1 << 8)  /* FLASH prefetch enable */
#    define FLASH_ACR_ICEN          (1 << 9)  /* Bit 9: Instruction cache enable */
#    define FLASH_ACR_DCEN          (1 << 10) /* Bit 10: Data cache enable */
#    define FLASH_ACR_ICRST         (1 << 11) /* Bit 11: Instruction cache reset */
#    define FLASH_ACR_DCRST         (1 << 12) /* Bit 12: Data cache reset */
#  endif
#endif

/* Flash Status Register (SR) */

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) || \
    defined(CONFIG_STM32_STM32F37XX)
#  define FLASH_SR_BSY              (1 << 0)  /* Busy */
#  define FLASH_SR_PGERR            (1 << 2)  /* Programming Error */
#  define FLASH_SR_WRPRT_ERR        (1 << 4)  /* Write Protection Error */
#  define FLASH_SR_EOP              (1 << 5)  /* End of Operation */
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define FLASH_SR_EOP              (1 << 0)  /* Bit 0: End of operation */
#  define FLASH_SR_OPERR            (1 << 1)  /* Bit 1: Operation error */
#  define FLASH_SR_WRPERR           (1 << 4)  /* Bit 4: Write protection error */
#  define FLASH_SR_PGAERR           (1 << 5)  /* Bit 5: Programming alignment error */
#  define FLASH_SR_PGPERR           (1 << 6)  /* Bit 6: Programming parallelism error */
#  define FLASH_SR_PGSERR           (1 << 7)  /* Bit 7: Programming sequence error */
#  define FLASH_SR_BSY              (1 << 16) /* Bit 16: Busy */
#endif

/* Flash Control Register (CR) */

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) || \
    defined(CONFIG_STM32_STM32F37XX)
#  define FLASH_CR_PG               (1 << 0)  /* Bit 0: Program Page */
#  define FLASH_CR_PER              (1 << 1)  /* Bit 1: Page Erase */
#  define FLASH_CR_MER              (1 << 2)  /* Bit 2: Mass Erase */
#  define FLASH_CR_OPTPG            (1 << 4)  /* Bit 4: Option Byte Programming */
#  define FLASH_CR_OPTER            (1 << 5)  /* Bit 5: Option Byte Erase */
#  define FLASH_CR_STRT             (1 << 6)  /* Bit 6: Start Erase */
#  define FLASH_CR_LOCK             (1 << 7)  /* Bit 7: Page Locked or Lock Page */
#  define FLASH_CR_OPTWRE           (1 << 9)  /* Bit 8: Option Bytes Write Enable */
#  define FLASH_CR_ERRIE            (1 << 10) /* Bit 10: Error Interrupt Enable */
#  define FLASH_CR_EOPIE            (1 << 12) /* Bit 12: End of Program Interrupt Enable */
#  if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
#    define FLASH_CR_OBL_LAUNCH     (1 << 13) /* Bit 13: Force option byte loading */
#  endif
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define FLASH_CR_PG               (1 << 0)                /* Bit 0: Programming */
#  define FLASH_CR_SER              (1 << 1)                /* Bit 1: Sector Erase */
#  define FLASH_CR_MER              (1 << 2)                /* Bit 2: Mass Erase sectors 0..11 */
#  define FLASH_CR_SNB_SHIFT        (3)                     /* Bits 3-6: Sector number */
#  define FLASH_CR_SNB_MASK         (15 << FLASH_CR_SNB_SHIFT)
#if defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429)
#    define FLASH_CR_SNB(n)         (((n % 12) << FLASH_CR_SNB_SHIFT) | ((n / 12) << 7)) /* Sector n, n=0..23 */
#else
#    define FLASH_CR_SNB(n)         ((n) << FLASH_CR_SNB_SHIFT) /* Sector n, n=0..11 */
#endif
#  define FLASH_CR_PSIZE_SHIFT      (8)                     /* Bits 8-9: Program size */
#  define FLASH_CR_PSIZE_MASK       (3 << FLASH_CR_PSIZE_SHIFT)
#    define FLASH_CR_PSIZE_X8       (0 << FLASH_CR_PSIZE_SHIFT) /* 00 program x8 */
#    define FLASH_CR_PSIZE_X16      (1 << FLASH_CR_PSIZE_SHIFT) /* 01 program x16 */
#    define FLASH_CR_PSIZE_X32      (2 << FLASH_CR_PSIZE_SHIFT) /* 10 program x32 */
#    define FLASH_CR_PSIZE_X64      (3 << FLASH_CR_PSIZE_SHIFT) /* 11 program x64 */
#  define FLASH_CR_STRT             (1 << 16)               /* Bit 16: Start Erase */
#  define FLASH_CR_EOPIE            (1 << 24)               /* Bit 24: End of operation interrupt enable */
#  define FLASH_CR_ERRIE            (1 << 25)               /* Bit 25: Error interrupt enable */
#  define FLASH_CR_LOCK             (1 << 31)               /* Bit 31: Lock */
#endif
#if defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429)
#  define FLASH_CR_MER1             (1 << 15)               /* Bit 15: Mass Erase sectors 12..23 */
#endif

/* Flash Option Control Register (OPTCR) */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define FLASH_OPTCR_OPTLOCK       (1 << 0)               /* Bit 0: Option lock */
#  define FLASH_OPTCR_OPTSTRT       (1 << 1)               /* Bit 1: Option start */
#  define FLASH_OPTCR_BORLEV_SHIFT  (2)                    /* Bits 2-3: BOR reset Level */
#  define FLASH_OPTCR_BORLEV_MASK   (3 << FLASH_OPTCR_BORLEV_SHIFT)
#    define FLASH_OPTCR_VBOR3       (0 << FLASH_OPTCR_BORLEV_SHIFT) /* 00: BOR Level 3 */
#    define FLASH_OPTCR_VBOR2       (1 << FLASH_OPTCR_BORLEV_SHIFT) /* 01: BOR Level 2 */
#    define FLASH_OPTCR_VBOR1       (2 << FLASH_OPTCR_BORLEV_SHIFT) /* 10: BOR Level 1 */
#    define FLASH_OPTCR_VBOR0       (3 << FLASH_OPTCR_BORLEV_SHIFT) /* 11: BOR off */
#  define FLASH_OPTCR_USER_SHIFT    (5)                    /* Bits 5-7: User option bytes */
#  define FLASH_OPTCR_USER_MASK     (7 << FLASH_OPTCR_USER_SHIFT)
#    define FLASH_OPTCR_NRST_STDBY  (1 << 7)               /* Bit 7: nRST_STDBY */
#    define FLASH_OPTCR_NRST_STOP   (1 << 6)               /* Bit 6: nRST_STOP */
#    define FLASH_OPTCR_WDG_SW      (1 << 5)               /* Bit 5: WDG_SW */
#  define FLASH_OPTCR_RDP_SHIFT     (8)                    /* Bits 8-15: Read protect */
#  define FLASH_OPTCR_RDP_MASK      (0xff << FLASH_OPTCR_RDP_SHIFT)
#  define FLASH_OPTCR_NWRP_SHIFT    (16)                   /* Bits 16-27: Not write protect */
#  define FLASH_OPTCR_NWRP_MASK     (0xfff << FLASH_OPTCR_NWRP_SHIFT)
#endif

/* Flash Option Control Register (OPTCR1) */

#if defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429)
#  define FLASH_OPTCR1_NWRP_SHIFT    (16)                   /* Bits 16-27: Not write protect (high bank) */
#  define FLASH_OPTCR1_NWRP_MASK     (0xfff << FLASH_OPTCR_NWRP_SHIFT)

#  define FLASH_OPTCR1_BFB2_SHIFT    (4)                   /* Bits 4: Dual-bank Boot option byte */
#  define FLASH_OPTCR1_BFB2_MASK     (1 << FLASH_OPTCR_NWRP_SHIFT)

#endif

#if defined(CONFIG_STM32_STM32F446)
#  define FLASH_OPTCR1_NWRP_SHIFT    (16)                   /* Bits 16-23: Not write protect (high bank) */
#  define FLASH_OPTCR1_NWRP_MASK     (0xff << FLASH_OPTCR_NWRP_SHIFT)
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void stm32_flash_lock(void);
void stm32_flash_unlock(void);

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_FLASH_H */
