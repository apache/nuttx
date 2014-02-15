/************************************************************************************
 * arch/arm/include/samd/chip.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_SAMD_CHIP_H
#define __ARCH_ARM_INCLUDE_SAMD_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

/* SAMD20 Family ********************************************************************/
/* FEATURE             SAM D20J          SAM D20G           SAM D20E
 * ------------------- ------------------ ------------------ --------
 * No. of pins         64                 48                 32
 * Flash               256/128/64/32/16KB 256/128/64/32/16KB 256/128/64/32/16KB
 * SRAM                32/16/8/4/2KB      32/16/8/4/2KB      32/16/8/4/2KB
 * Max. Freq.          48MHz              48MHz              48MHz
 * Event channels      8                  8                  8
 * Timer/counters      8                  6                  6
 * TC output channels  2                  2                  2
 * SERCOM              6                  6                  4
 * ADC channels        20                 14                 10
 * Comparators         2                  2                  2
 * DAC channels        1                  1                  1
 * RTC                 Yes                Yes                Yes
 * RTC alarms          1                  1                  1
 * RTC compare         1 32-bit/2 16-bit  1 32-bit/2 16-bit  1 32-bit/2 16-bit
 * External interrupts 16                 16                 16
 * PTC X an Y          16x16              12x10              10x6
 * Packages            QFN/TQFP           QFN/TQFP           QFN/TQFP
 * Oscillators         XOSC32, XOSC, OSC32K, OSCULP32K, OSC8M, and DFLL48M
 * SW Debug interface  Yes                Yes                Yes
 * Watchdog timer      Yes                Yes                Yes
 */

#if defined(CONFIG_ARCH_CHIP_SAMD20E14)

#  define SAMD20                    1             /* SAMD20 family */
#  define SAMD20E                   1             /* SAMD20E */
#  undef  SAMD20G
#  undef  SAMD20J

/* Internal memory */

#  define SAMD_FLASH_SIZE           (16*1024)     /* 16KB */
#  define SAMD_SRAM0_SIZE           (2*1024)      /*  2KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20E15)

#  define SAMD20                    1             /* SAMD20 family */
#  define SAMD20E                   1             /* SAMD20E */
#  undef  SAMD20G
#  undef  SAMD20J

/* Internal memory */

#  define SAMD_FLASH_SIZE           (32*1024)     /* 32KB */
#  define SAMD_SRAM0_SIZE           (4*1024)      /*  4KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20E16)

#  define SAMD20                    1             /* SAMD20 family */
#  define SAMD20E                   1             /* SAMD20E */
#  undef  SAMD20G
#  undef  SAMD20J

/* Internal memory */

#  define SAMD_FLASH_SIZE           (64*1024)     /* 64KB */
#  define SAMD_SRAM0_SIZE           (8*1024)      /*  8KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20E17)

#  define SAMD20                    1             /* SAMD20 family */
#  define SAMD20E                   1             /* SAMD20E */
#  undef  SAMD20G
#  undef  SAMD20J

/* Internal memory */

#  define SAMD_FLASH_SIZE           (128*1024)    /* 128KB */
#  define SAMD_SRAM0_SIZE           (16*1024)     /*  16KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20E18)

#  define SAMD20                    1             /* SAMD20 family */
#  define SAMD20E                   1             /* SAMD20E */
#  undef  SAMD20G
#  undef  SAMD20J

/* Internal memory */

#  define SAMD_FLASH_SIZE           (256*1024)    /* 256KB */
#  define SAMD_SRAM0_SIZE           (32*1024)     /*  32KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20G14)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  define SAMD20G                   1             /* SAMD20G */
#  undef  SAMD20J

/* Internal memory */

#  define SAMD_FLASH_SIZE           (16*1024)     /* 16KB */
#  define SAMD_SRAM0_SIZE           (2*1024)      /*  2KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20G15)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  define SAMD20G                   1             /* SAMD20G */
#  undef  SAMD20J

/* Internal memory */

#  define SAMD_FLASH_SIZE           (32*1024)     /* 32KB */
#  define SAMD_SRAM0_SIZE           (4*1024)      /*  4KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20G16)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  define SAMD20G                   1             /* SAMD20G */
#  undef  SAMD20J

/* Internal memory */

#  define SAMD_FLASH_SIZE           (64*1024)     /* 64KB */
#  define SAMD_SRAM0_SIZE           (8*1024)      /*  8KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20G17)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  define SAMD20G                   1             /* SAMD20G */
#  undef  SAMD20J

/* Internal memory */

#  define SAMD_FLASH_SIZE           (128*1024)    /* 128KB */
#  define SAMD_SRAM0_SIZE           (16*1024)     /*  16KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20G18)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  define SAMD20G                   1             /* SAMD20G */
#  undef  SAMD20J

/* Internal memory */

#  define SAMD_FLASH_SIZE           (256*1024)    /* 256KB */
#  define SAMD_SRAM0_SIZE           (32*1024)     /*  32KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20J14)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  undef  SAMD20G
#  define SAMD20J                   1             /* SAMD20J */

/* Internal memory */

#  define SAMD_FLASH_SIZE           (16*1024)     /* 16KB */
#  define SAMD_SRAM0_SIZE           (2*1024)      /*  2KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20J15)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  undef  SAMD20G
#  define SAMD20J                   1             /* SAMD20J */

/* Internal memory */

#  define SAMD_FLASH_SIZE           (32*1024)     /* 32KB */
#  define SAMD_SRAM0_SIZE           (4*1024)      /*  4KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20J16)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  undef  SAMD20G
#  define SAMD20J                   1             /* SAMD20J */

/* Internal memory */

#  define SAMD_FLASH_SIZE           (64*1024)     /* 64KB */
#  define SAMD_SRAM0_SIZE           (8*1024)      /*  8KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20J17)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  undef  SAMD20G
#  define SAMD20J                   1             /* SAMD20J */

/* Internal memory */

#  define SAMD_FLASH_SIZE           (128*1024)    /* 128KB */
#  define SAMD_SRAM0_SIZE           (16*1024)     /*  16KB */

#elif defined(CONFIG_ARCH_CHIP_SAMD20J18)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  undef  SAMD20G
#  define SAMD20J                   1             /* SAMD20J */

/* Internal memory */

#  define SAMD_FLASH_SIZE           (256*1024)    /* 256KB */
#  define SAMD_SRAM0_SIZE           (32*1024)     /*  32KB */

#endif

/* SAMD20 Peripherals */

#if defined(SAMD20E)
#  define SAMD_NEVENTS              8             /* 8 event channels */
#  define SAMD_NTC                  6             /* 6 Timer/counters */
#  define SAMD_NTCOUT               2             /* 2 TC output channels */
#  define SAMD_NSERCOM              4             /* 4 SERCOM */
#  define SAMD_NADC                 10            /* 10 ADC channels */
#  define SAMD_NCMP                 2             /* 2 Comparators */
#  define SAMD_NDAC                 1             /* 1 DAC channel */
#  define SAMD_RTC                  1             /* Have RTC */
#  define SAMD_NALARMS              1             /* 1 RTC alarm */
#  define SAMD_NRTCMP               1             /* RTC compare: 1 32-bit/2 16-bit */
#  define SAMD_NEXTINT              16            /* 16 External interrupts */
#  define SAMD_NPTCX                10            /* PTC X */
#  define SAMD_NPTCY                6             /* PTC Y */
#  define SAMD_WDT                                /* Have watchdog timer */
#elif defined(SAMD20G)
#  define SAMD_NEVENTS              8             /* 8 event channels */
#  define SAMD_NTC                  6             /* 6 Timer/counters */
#  define SAMD_NTCOUT               2             /* 2 TC output channels */
#  define SAMD_NSERCOM              6             /* 6 SERCOM */
#  define SAMD_NADC                 15            /* 14 ADC channels */
#  define SAMD_NCMP                 2             /* 2 Comparators */
#  define SAMD_NDAC                 1             /* 1 DAC channel */
#  define SAMD_RTC                  1             /* Have RTC */
#  define SAMD_NALARMS              1             /* 1 RTC alarm */
#  define SAMD_NRTCMP               1             /* RTC compare: 1 32-bit/2 16-bit */
#  define SAMD_NEXTINT              16            /* 16 External interrupts */
#  define SAMD_NPTCX                12            /* PTC X */
#  define SAMD_NPTCY                10            /* PTC Y */
#  define SAMD_WDT                                /* Have watchdog timer */
#elif defined(SAMD20J)
#  define SAMD_NEVENTS              8             /* 8 event channels */
#  define SAMD_NTC                  8             /* 8 Timer/counters */
#  define SAMD_NTCOUT               2             /* 2 TC output channels */
#  define SAMD_NSERCOM              6             /* 6 SERCOM */
#  define SAMD_NADC                 20            /* 20 ADC channels */
#  define SAMD_NCMP                 2             /* 2 Comparators */
#  define SAMD_NDAC                 1             /* 1 DAC channel */
#  define SAMD_RTC                  1             /* Have RTC */
#  define SAMD_NALARMS              1             /* 1 RTC alarm */
#  define SAMD_NRTCMP               1             /* RTC compare: 1 32-bit/2 16-bit */
#  define SAMD_NEXTINT              16            /* 16 External interrupts */
#  define SAMD_NPTCX                16            /* PTC X */
#  define SAMD_NPTCY                16            /* PTC Y */
#  define SAMD_WDT                                /* Have watchdog timer */
#endif

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0-3. The lower the value, the greater
 * the priority of the corresponding interrupt. The processor implements only
 * bits[7:6] of each field, bits[5:0] read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xc0 /* All bits[7:3] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x40 /* Five bits of interrupt priority used */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_SAMD_CHIP_H */
