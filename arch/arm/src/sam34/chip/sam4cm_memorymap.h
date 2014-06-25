/************************************************************************************************
 * arch/arm/src/sam34/chip/sam4cm_memorymap.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM4CM_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM4CM_MEMORYMAP_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* Address regions */

#define SAM_CODE_BASE          0x00000000 /* 0x00000000-0x1fffffff: Code space */
#define SAM_INTSRAM_BASE       0x20000000 /* 0x20000000-0x3fffffff: Internal SRAM */
#define SAM_PERIPHERALS_BASE   0x40000000 /* 0x40000000-0x5fffffff: Peripherals */
#define SAM_EXTRAM_BASE        0x60000000 /* 0x60000000-0x9fffffff: External RAM */
#define SAM_EXTDEV_BASE        0xa0000000 /* 0xa0000000-0xdfffffff: External device */
#define SAM_SYSTEM_BASE        0xe0000000 /* 0xe0000000-0xffffffff: System */

/* Code memory region */

#define SAM_BOOTMEMORY_BASE    0x00000000 /* 0x00000000-0x00ffffff: Boot Memory */
#define SAM_INTFLASH_BASE      0x01000000 /* 0x01000000-0x01ffffff: Internal FLASH */
#define SAM_INTROM_BASE        0x02000000 /* 0x02000000-0x02ffffff: Internal ROM */

/* Internal SRAM memory region */

#define SAM_INTSRAM0_BASE      0x20000000 /* For SAM3U compatibility */
#define SAM_BBSRAM_BASE        0x22000000 /* 0x22000000-0x23ffffff: 32MB bit-band region */
                                          /* 0x24000000-0x3fffffff: Undefined */
/* Peripherals address region */

#define SAM_AES_BASE           0x40000000
#define SAM_SPI0_BASE          0x40008000
#define SAM_TC0_BASE           0x40010000
#define SAM_TC1_BASE           0x40010040
#define SAM_TC2_BASE           0x40010080
#define SAM_TC3_BASE           0x40014000
#define SAM_TC4_BASE           0x40014040
#define SAM_TC5_BASE           0x40014080
#define SAM_TWI0_BASE          0x40018000
#define SAM_TWI1_BASE          0x4001C000
#define SAM_USART0_BASE        0x40024000
#define SAM_USART1_BASE        0x40028000
#define SAM_USART2_BASE        0x4002C000
#define SAM_USART3_BASE        0x40030000
#define SAM_ADC_BASE           0x40038000
#define SAM_SLCDC_BASE         0x4003C000
#define SAM_CPKCC_BASE         0x40040000
#define SAM_ICM_BASE           0x40044000
#define SAM_TRNG_BASE          0x40048000
#define SAM_IPC0_BASE          0x4004C000
#define SAM_CMCC0_BASE         0x4007C000
#define SAM_SMC0_BASE          0x400E0000
#define SAM_MATRIX_BASE        0x400E0200
#  define SAM_MATRIX0_BASE     0x400E0200
#define SAM_PMC_BASE           0x400E0400
#define SAM_UART0_BASE         0x400E0600
#define SAM_CHIPID_BASE        0x400E0740
#define SAM_EEFC0_BASE         0x400E0A00
#define SAM_EEFC1_BASE         0x400E0C00
#define SAM_PIOA_BASE          0x400E0E00
#define SAM_PIOB_BASE          0x400E1000
#define SAM_RSTC_BASE          0x400E1400
#define SAM_SUPC_BASE          0x400E1410
#define SAM_RTT_BASE           0x400E1430
#define SAM_WDT_BASE           0x400E1450
#define SAM_RTC_BASE           0x400E1460
#define SAM_GPBR_BASE          0x400E1490
#define SAM_UART1_BASE         0x48004000
#define SAM_PWM_BASE           0x48008000
#define SAM_PIOC_BASE          0x4800C000
#define SAM_MATRIX1_BASE       0x48010000
#define SAM_IPC1_BASE          0x48014000
#define SAM_CMCC1_BASE         0x48018000
#define SAM_SMC1_BASE          0x4801C000

/* External RAM memory region */

#define SAM_EXTCS_BASE         0x60000000 /* 0x60000000-0x63ffffff: Chip selects */
#  define SAM_EXTCSN_BASE(n)   (0x60000000*((n)<<24))
#  define SAM_EXTCS0_BASE      0x60000000 /* 0x60000000-0x60ffffff:   Chip select 0 */
#  define SAM_EXTCS1_BASE      0x61000000 /* 0x61000000-0x601fffff:   Chip select 1 */
#  define SAM_EXTCS2_BASE      0x62000000 /* 0x62000000-0x62ffffff:   Chip select 2 */
#  define SAM_EXTCS3_BASE      0x63000000 /* 0x63000000-0x63ffffff:   Chip select 3 */
                                          /* 0x64000000-0x9fffffff: Reserved */
/* System memory region */

#define SAM_PRIVPERIPH_BASE    0xe0000000 /* 0xe0000000-0xe00fffff: Private peripheral bus */
#define SAM_VENDOR_BASE        0xe0100000 /* 0ex0100000-0xffffffff: Vendor-specific memory */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

static inline unsigned long SAM_PION_BASE(int n)
{
  switch(n) {
  case 0:
    return SAM_PIOA_BASE;
  case 1:
    return SAM_PIOB_BASE;
  case 2:
    return SAM_PIOC_BASE;
  default:
    return 0;
  }
}

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM4S_MEMORYMAP_H */
