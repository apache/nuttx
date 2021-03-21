/****************************************************************************
 * arch/arm/include/samd2l2/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_SAMD2L2_CHIP_H
#define __ARCH_ARM_INCLUDE_SAMD2L2_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Get customizations for each supported chip */

/* SAMD20 Family ************************************************************/

/* FEATURE             SAM D20J          SAM D20G           SAM D20E
 * ------------------- ------------------ ------------------ --------
 * No. of pins         64                 48                 32
 * Flash               256/128/64/        256/128/64/        256/128/64/
 *                     32/16KB            32/16KB            32/16KB
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
 * RTC compare         1 32-bit/          1 32-bit/          1 32-bit/
 *                     2 16-bit           2 16-bit           2 16-bit
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

#  define SAMD2L2_FLASH_SIZE        (16*1024)     /* 16KB */
#  define SAMD2L2_SRAM0_SIZE        (2*1024)      /*  2KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20E15)

#  define SAMD20                    1             /* SAMD20 family */
#  define SAMD20E                   1             /* SAMD20E */
#  undef  SAMD20G
#  undef  SAMD20J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (32*1024)     /* 32KB */
#  define SAMD2L2_SRAM0_SIZE        (4*1024)      /*  4KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20E16)

#  define SAMD20                    1             /* SAMD20 family */
#  define SAMD20E                   1             /* SAMD20E */
#  undef  SAMD20G
#  undef  SAMD20J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (64*1024)     /* 64KB */
#  define SAMD2L2_SRAM0_SIZE        (8*1024)      /*  8KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20E17)

#  define SAMD20                    1             /* SAMD20 family */
#  define SAMD20E                   1             /* SAMD20E */
#  undef  SAMD20G
#  undef  SAMD20J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (128*1024)    /* 128KB */
#  define SAMD2L2_SRAM0_SIZE        (16*1024)     /*  16KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20E18)

#  define SAMD20                    1             /* SAMD20 family */
#  define SAMD20E                   1             /* SAMD20E */
#  undef  SAMD20G
#  undef  SAMD20J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (256*1024)    /* 256KB */
#  define SAMD2L2_SRAM0_SIZE        (32*1024)     /*  32KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20G14)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  define SAMD20G                   1             /* SAMD20G */
#  undef  SAMD20J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (16*1024)     /* 16KB */
#  define SAMD2L2_SRAM0_SIZE        (2*1024)      /*  2KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20G15)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  define SAMD20G                   1             /* SAMD20G */
#  undef  SAMD20J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (32*1024)     /* 32KB */
#  define SAMD2L2_SRAM0_SIZE        (4*1024)      /*  4KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20G16)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  define SAMD20G                   1             /* SAMD20G */
#  undef  SAMD20J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (64*1024)     /* 64KB */
#  define SAMD2L2_SRAM0_SIZE        (8*1024)      /*  8KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20G17)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  define SAMD20G                   1             /* SAMD20G */
#  undef  SAMD20J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (128*1024)    /* 128KB */
#  define SAMD2L2_SRAM0_SIZE        (16*1024)     /*  16KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20G18)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  define SAMD20G                   1             /* SAMD20G */
#  undef  SAMD20J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (256*1024)    /* 256KB */
#  define SAMD2L2_SRAM0_SIZE        (32*1024)     /*  32KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20J14)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  undef  SAMD20G
#  define SAMD20J                   1             /* SAMD20J */

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (16*1024)     /* 16KB */
#  define SAMD2L2_SRAM0_SIZE        (2*1024)      /*  2KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20J15)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  undef  SAMD20G
#  define SAMD20J                   1             /* SAMD20J */

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (32*1024)     /* 32KB */
#  define SAMD2L2_SRAM0_SIZE        (4*1024)      /*  4KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20J16)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  undef  SAMD20G
#  define SAMD20J                   1             /* SAMD20J */

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (64*1024)     /* 64KB */
#  define SAMD2L2_SRAM0_SIZE        (8*1024)      /*  8KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20J17)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  undef  SAMD20G
#  define SAMD20J                   1             /* SAMD20J */

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (128*1024)    /* 128KB */
#  define SAMD2L2_SRAM0_SIZE        (16*1024)     /*  16KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD20J18)

#  define SAMD20                    1             /* SAMD20 family */
#  undef  SAMD20E
#  undef  SAMD20G
#  define SAMD20J                   1             /* SAMD20J */

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (256*1024)    /* 256KB */
#  define SAMD2L2_SRAM0_SIZE        (32*1024)     /*  32KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#else

#  undef  SAMD20                                  /* Not SAMD20 family */
#  undef  SAMD20E
#  undef  SAMD20G
#  undef  SAMD20J

#endif

/* SAMD20 Peripherals */

#if defined(SAMD20E)
#  define SAMD2L2_NEVENTS           8             /* 8 event channels */
#  define SAMD2L2_NTC               6             /* 6 Timer/counters */
#  define SAMD2L2_NTCOUT            2             /* 2 TC output channels */
#  define SAMD2L2_NTCC              0             /* No TC control channels */
#  define SAMD2L2_NTCCOUT           0             /* No TCC output channels */
#  define SAMD2L2_NDMACHAN          0             /* No DMA channels */
#  define SAMD2L2_NUSBIF            0             /* No USB interface */
#  define SAMD2L2_NAES              0             /* No AES engine */
#  define SAMD2L2_NCCL              0             /* No Counfigurable Custom Logic */
#  define SAMD2L2_NTRNG             0             /* No True random number generator */
#  define SAMD2L2_NSERCOM           4             /* 4 SERCOM */
#  define SAMD2L2_NI2S              0             /* No I2S */
#  define SAMD2L2_NADC              10            /* 10 ADC channels */
#  define SAMD2L2_NCMP              2             /* 2 Comparators */
#  define SAMD2L2_NDAC              1             /* 1 DAC channel */
#  define SAMD2L2_NOPAMP            0             /* No OpAmps */
#  define SAMD2L2_RTC               1             /* Have RTC */
#  define SAMD2L2_NALARMS           1             /* 1 RTC alarm */
#  define SAMD2L2_NRTCMP            1             /* RTC compare: 1 32-bit/2 16-bit */
#  define SAMD2L2_NEXTINT           16            /* 16 External interrupts */
#  define SAMD2L2_NPTCX             10            /* PTC X */
#  define SAMD2L2_NPTCY             6             /* PTC Y */
#  define SAMD2L2_WDT               1             /* Have watchdog timer */
#elif defined(SAMD20G)
#  define SAMD2L2_NEVENTS           8             /* 8 event channels */
#  define SAMD2L2_NTC               6             /* 6 Timer/counters */
#  define SAMD2L2_NTCOUT            2             /* 2 TC output channels */
#  define SAMD2L2_NTCC              0             /* No TC control channels */
#  define SAMD2L2_NTCCOUT           0             /* No TCC output channels */
#  define SAMD2L2_NDMACHAN          0             /* No DMA channels */
#  define SAMD2L2_NUSBIF            0             /* No USB interface */
#  define SAMD2L2_NAES              0             /* No AES engine */
#  define SAMD2L2_NCCL              0             /* No Counfigurable Custom Logic */
#  define SAMD2L2_NTRNG             0             /* No True random number generator */
#  define SAMD2L2_NSERCOM           6             /* 6 SERCOM */
#  define SAMD2L2_NI2S              0             /* No I2S */
#  define SAMD2L2_NADC              15            /* 14 ADC channels */
#  define SAMD2L2_NCMP              2             /* 2 Comparators */
#  define SAMD2L2_NDAC              1             /* 1 DAC channel */
#  define SAMD2L2_NOPAMP            0             /* No OpAmps */
#  define SAMD2L2_RTC               1             /* Have RTC */
#  define SAMD2L2_NALARMS           1             /* 1 RTC alarm */
#  define SAMD2L2_NRTCMP            1             /* RTC compare: 1 32-bit/2 16-bit */
#  define SAMD2L2_NEXTINT           16            /* 16 External interrupts */
#  define SAMD2L2_NPTCX             12            /* PTC X */
#  define SAMD2L2_NPTCY             10            /* PTC Y */
#  define SAMD2L2_WDT               1             /* Have watchdog timer */
#elif defined(SAMD20J)
#  define SAMD2L2_NEVENTS           8             /* 8 event channels */
#  define SAMD2L2_NTC               8             /* 8 Timer/counters */
#  define SAMD2L2_NTCOUT            2             /* 2 TC output channels */
#  define SAMD2L2_NTCC              0             /* No TC control channels */
#  define SAMD2L2_NTCCOUT           0             /* No TCC output channels */
#  define SAMD2L2_NDMACHAN          0             /* No DMA channels */
#  define SAMD2L2_NUSBIF            0             /* No USB interface */
#  define SAMD2L2_NAES              0             /* No AES engine */
#  define SAMD2L2_NCCL              0             /* No Counfigurable Custom Logic */
#  define SAMD2L2_NTRNG             0             /* No True random number generator */
#  define SAMD2L2_NSERCOM           6             /* 6 SERCOM */
#  define SAMD2L2_NI2S              0             /* No I2S */
#  define SAMD2L2_NADC              20            /* 20 ADC channels */
#  define SAMD2L2_NCMP              2             /* 2 Comparators */
#  define SAMD2L2_NDAC              1             /* 1 DAC channel */
#  define SAMD2L2_NOPAMP            0             /* No OpAmps */
#  define SAMD2L2_RTC               1             /* Have RTC */
#  define SAMD2L2_NALARMS           1             /* 1 RTC alarm */
#  define SAMD2L2_NRTCMP            1             /* RTC compare: 1 32-bit/2 16-bit */
#  define SAMD2L2_NEXTINT           16            /* 16 External interrupts */
#  define SAMD2L2_NPTCX             16            /* PTC X */
#  define SAMD2L2_NPTCY             16            /* PTC Y */
#  define SAMD2L2_WDT               1             /* Have watchdog timer */
#endif

/* SAMD21 Family ************************************************************/

/* FEATURE             SAM D21J          SAM D21G           SAM D21E
 * ------------------- ------------------ ------------------ --------
 * No. of pins         64                 48                 32
 * Flash               256/128/64/32/16KB 256/128/64/32KB    256/128/64/32KB
 * SRAM                32/16/8/4/2KB      32/16/8/4/2KB      32/16/8/4KB
 * Max. Freq.          48MHz              48MHz              48MHz
 * Event channels      12                 12                 12
 * Timer/counters      5                  3                  3
 * TC output channels  2                  2                  2
 * T/C Control         3                  3                  3
 * TCC output channels 2                  2                  2
 * TCC waveform output 8/4/2              8/4/2              6/4/2
 * DMA channels        12                 12                 12
 * USB interface       1                  1                  1
 * SERCOM              6                  6                  4
 * I2S                 1                  1                  1
 * ADC channels        20                 14                 10
 * Comparators         2                  2                  2
 * DAC channels        1                  1                  1
 * RTC                 Yes                Yes                Yes
 * RTC alarms          1                  1                  1
 * RTC compare         1 32-bit/          1 32-bit/          1 32-bit/
 *                     2 16-bit           2 16-bit           2 16-bit
 * External interrupts 16                 16                 16
 * PTC X an Y          16x16              12x10              10x6
 * Packages            QFN/TQFP           QFN/TQFP/WLCSP     QFN/TQFP/UFBGA
 * Oscillators         XOSC32, XOSC, OSC32K, OSCULP32K, OSC8M,
 *                                                  DFLL48M, and FDPLL96M
 * SW Debug interface  Yes                Yes                Yes
 * Watchdog timer      Yes                Yes                Yes
 */

#if defined(CONFIG_ARCH_CHIP_SAMD21E15A) || defined(CONFIG_ARCH_CHIP_SAMD21E15B)

#  define SAMD21                    1             /* SAMD21 family */
#  define SAMD21E                   1             /* SAMD21E */
#  undef  SAMD21G
#  undef  SAMD21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (32*1024)     /* 32KB */
#  define SAMD2L2_SRAM0_SIZE        (4*1024)      /* 4KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#  if defined(CONFIG_ARCH_CHIP_SAMD21E15A)
#    define SAMD2L2_FLASHRWW_SIZE   (0*1024)      /* None */
#  else
#    define SAMD2L2_FLASHRWW_SIZE   (1*1024)      /* 1KB */
#  endif

#elif defined(CONFIG_ARCH_CHIP_SAMD21E16A) || defined(CONFIG_ARCH_CHIP_SAMD21E16B)

#  define SAMD21                    1             /* SAMD21 family */
#  define SAMD21E                   1             /* SAMD21E */
#  undef  SAMD21G
#  undef  SAMD21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (64*1024)     /* 64KB */
#  define SAMD2L2_SRAM0_SIZE        (8*1024)      /* 8KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#  if defined(CONFIG_ARCH_CHIP_SAMD21E16A)
#    define SAMD2L2_FLASHRWW_SIZE   (0*1024)      /* None */
#  else
#    define SAMD2L2_FLASHRWW_SIZE   (2*1024)      /* 2KB */
#  endif

#elif defined(CONFIG_ARCH_CHIP_SAMD21E17A)

#  define SAMD21                    1             /* SAMD21 family */
#  define SAMD21E                   1             /* SAMD21E */
#  undef  SAMD21G
#  undef  SAMD21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (128*1024)    /* 128KB */
#  define SAMD2L2_FLASHRWW_SIZE     (0*1024)      /* None */
#  define SAMD2L2_SRAM0_SIZE        (16*1024)     /* 16KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD21E18A)

#  define SAMD21                    1             /* SAMD21 family */
#  define SAMD21E                   1             /* SAMD21E */
#  undef  SAMD21G
#  undef  SAMD21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (256*1024)    /* 256KB */
#  define SAMD2L2_FLASHRWW_SIZE     (0*1024)      /* None */
#  define SAMD2L2_SRAM0_SIZE        (32*1024)     /* 32KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD21G15A) || defined(CONFIG_ARCH_CHIP_SAMD21G15B)

#  define SAMD21                    1             /* SAMD21 family */
#  undef  SAMD21E
#  define SAMD21G                   1             /* SAMD21G */
#  undef  SAMD21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (32*1024)     /* 32KB */
#  define SAMD2L2_SRAM0_SIZE        (4*1024)      /* 4KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#  if defined(CONFIG_ARCH_CHIP_SAMD21G15A)
#    define SAMD2L2_FLASHRWW_SIZE   (0*1024)      /* None */
#  else
#    define SAMD2L2_FLASHRWW_SIZE   (1*1024)      /* 1KB */
#  endif

#elif defined(CONFIG_ARCH_CHIP_SAMD21G16A) || defined(CONFIG_ARCH_CHIP_SAMD21G16B)

#  define SAMD21                    1             /* SAMD21 family */
#  undef  SAMD21E
#  define SAMD21G                   1             /* SAMD21G */
#  undef  SAMD21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (64*1024)     /* 64KB */
#  define SAMD2L2_SRAM0_SIZE        (8*1024)      /* 8KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#  if defined(CONFIG_ARCH_CHIP_SAMD21G16A)
#    define SAMD2L2_FLASHRWW_SIZE   (0*1024)      /* None */
#  else
#    define SAMD2L2_FLASHRWW_SIZE   (2*1024)      /* 2KB */
#  endif

#elif defined(CONFIG_ARCH_CHIP_SAMD21G17A)

#  define SAMD21                    1             /* SAMD21 family */
#  undef  SAMD21E
#  define SAMD21G                   1             /* SAMD21G */
#  undef  SAMD21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (128*1024)    /* 128KB */
#  define SAMD2L2_FLASHRWW_SIZE     (0*1024)      /* None */
#  define SAMD2L2_SRAM0_SIZE        (16*1024)     /* 16KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD21G18A)

#  define SAMD21                    1             /* SAMD21 family */
#  undef  SAMD21E
#  define SAMD21G                   1             /* SAMD21G */
#  undef  SAMD21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (256*1024)    /* 256KB */
#  define SAMD2L2_FLASHRWW_SIZE     (0*1024)      /* None */
#  define SAMD2L2_SRAM0_SIZE        (32*1024)     /* 32KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD21J15A) || defined(CONFIG_ARCH_CHIP_SAMD21J15B)

#  define SAMD21                    1             /* SAMD21 family */
#  undef  SAMD21E
#  undef  SAMD21G
#  define SAMD21J                   1             /* SAMD21J */

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (32*1024)     /* 32KB */
#  define SAMD2L2_SRAM0_SIZE        (4*1024)      /* 4KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#  if defined(CONFIG_ARCH_CHIP_SAMD21J15A)
#    define SAMD2L2_FLASHRWW_SIZE   (0*1024)      /* None */
#  else
#    define SAMD2L2_FLASHRWW_SIZE   (1*1024)      /* 1KB */
#  endif

#elif defined(CONFIG_ARCH_CHIP_SAMD21J16A) || defined(CONFIG_ARCH_CHIP_SAMD21J16B)

#  define SAMD21                    1             /* SAMD21 family */
#  undef  SAMD21E
#  undef  SAMD21G
#  define SAMD21J                   1             /* SAMD21J */

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (64*1024)     /* 64KB */
#  define SAMD2L2_SRAM0_SIZE        (8*1024)      /* 8KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#  if defined(CONFIG_ARCH_CHIP_SAMD21J16A)
#    define SAMD2L2_FLASHRWW_SIZE   (0*1024)      /* None */
#  else
#    define SAMD2L2_FLASHRWW_SIZE   (2*1024)      /* 2KB */
#  endif

#elif defined(CONFIG_ARCH_CHIP_SAMD21J17A)

#  define SAMD21                    1             /* SAMD21 family */
#  undef  SAMD21E
#  undef  SAMD21G
#  define SAMD21J                   1             /* SAMD21J */

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (128*1024)    /* 128KB */
#  define SAMD2L2_FLASHRWW_SIZE     (0*1024)      /* None */
#  define SAMD2L2_SRAM0_SIZE        (16*1024)     /* 16KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#elif defined(CONFIG_ARCH_CHIP_SAMD21J18A)

#  define SAMD21                    1             /* SAMD21 family */
#  undef  SAMD21E
#  undef  SAMD21G
#  define SAMD21J                   1             /* SAMD21J */

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (256*1024)    /* 256KB */
#  define SAMD2L2_FLASHRWW_SIZE     (0*1024)      /* None */
#  define SAMD2L2_SRAM0_SIZE        (32*1024)     /* 32KB */
#  define SAMD2L2_LPRAM_SIZE        (0*1024)      /* None */

#else

#  undef  SAMD21                                  /* Not SAMD21 family */
#  undef  SAMD21E
#  undef  SAMD21G
#  undef  SAMD21J

#endif

#if defined(SAMD21E)
#  define SAMD2L2_NEVENTS           12            /* 12 event channels */
#  define SAMD2L2_NTC               3             /* 3 Timer/counters */
#  define SAMD2L2_NTCOUT            2             /* 2 TC output channels */
#  define SAMD2L2_NTCC              3             /* 3 TC control channels */
#  define SAMD2L2_NTCCOUT           2             /* 2 TCC output channels */
#  define SAMD2L2_TCC_NWAVEFORMS    8             /* Each TCC has a different number of outputs */
#  define SAMD2L2_NDMACHAN          12            /* 12 DMA channels */
#  define SAMD2L2_NUSBIF            1             /* 1 USB interface */
#  define SAMD2L2_NAES              1             /* 1 AES engine */
#  define SAMD2L2_NCCL              4             /* 4 Counfigurable Custom Logic */
#  define SAMD2L2_NTRNG             1             /* 1 True random number generator */
#  define SAMD2L2_NSERCOM           4             /* 4 SERCOM */
#  define SAMD2L2_NI2S              1             /* 1 I2S */
#  define SAMD2L2_NADC              10            /* 10 ADC channels */
#  define SAMD2L2_NCMP              2             /* 2 Comparators */
#  define SAMD2L2_NDAC              1             /* 1 DAC channel */
#  define SAMD2L2_NOPAMP            3             /* 3 OpAmps */
#  define SAMD2L2_RTC               1             /* Have RTC */
#  define SAMD2L2_NALARMS           1             /* 1 RTC alarm */
#  define SAMD2L2_NRTCMP            1             /* RTC compare: 1 32-bit/2 16-bit */
#  define SAMD2L2_NEXTINT           16            /* 16 External interrupts */
#  define SAMD2L2_NPTCX             10            /* 10x6 */
#  define SAMD2L2_NPTCY             6             /* 10x6*/
#  define SAMD2L2_WDT               1             /* Have watchdog timer */
#elif defined(SAMD21G)
#  define SAMD2L2_NEVENTS           12            /* 12 event channels */
#  define SAMD2L2_NTC               3             /* 3 Timer/counters */
#  define SAMD2L2_NTCOUT            2             /* 2 TC output channels */
#  define SAMD2L2_NTCC              3             /* 3 TC control channels */
#  define SAMD2L2_NTCCOUT           2             /* 2 TCC output channels */
#  define SAMD2L2_TCC_NWAVEFORMS    8             /* Each TCC has a different number of outputs */
#  define SAMD2L2_NDMACHAN          12            /* 12 DMA channels */
#  define SAMD2L2_NUSBIF            1             /* 1 USB interface */
#  define SAMD2L2_NAES              1             /* 1 AES engine */
#  define SAMD2L2_NCCL              4             /* 4 Counfigurable Custom Logic */
#  define SAMD2L2_NTRNG             1             /* 1 True random number generator */
#  define SAMD2L2_NSERCOM           6             /* 6 SERCOM */
#  define SAMD2L2_NI2S              1             /* 1 I2S */
#  define SAMD2L2_NADC              14            /* 14 ADC channels */
#  define SAMD2L2_NCMP              2             /* 2 Comparators */
#  define SAMD2L2_NDAC              1             /* 1 DAC channel */
#  define SAMD2L2_NOPAMP            3             /* 3 OpAmps */
#  define SAMD2L2_RTC               1             /* Have RTC */
#  define SAMD2L2_NALARMS           1             /* 1 RTC alarm */
#  define SAMD2L2_NRTCMP            1             /* RTC compare: 1 32-bit/2 16-bit */
#  define SAMD2L2_NEXTINT           16            /* 16 External interrupts */
#  define SAMD2L2_NPTCX             12            /* 12x10 */
#  define SAMD2L2_NPTCY             10            /* 12x10 */
#  define SAMD2L2_WDT               1             /* Have watchdog timer */
#elif defined(SAMD21J)
#  define SAMD2L2_NEVENTS           12            /* 12 event channels */
#  define SAMD2L2_NTC               5             /* 5 Timer/counters */
#  define SAMD2L2_NTCOUT            2             /* 2 TC output channels */
#  define SAMD2L2_NTCC              3             /* 3 TC control channels */
#  define SAMD2L2_NTCCOUT           2             /* 2 TCC output channels */
#  define SAMD2L2_TCC_NWAVEFORMS    8             /* Each TCC has a different number of outputs */
#  define SAMD2L2_NDMACHAN          12            /* 12 DMA channels */
#  define SAMD2L2_NUSBIF            1             /* 1 USB interface */
#  define SAMD2L2_NAES              1             /* 1 AES engine */
#  define SAMD2L2_NCCL              4             /* 4 Counfigurable Custom Logic */
#  define SAMD2L2_NTRNG             1             /* 1 True random number generator */
#  define SAMD2L2_NSERCOM           6             /* 6 SERCOM */
#  define SAMD2L2_NI2S              1             /* 1 I2S */
#  define SAMD2L2_NADC              20            /* 20 ADC channels */
#  define SAMD2L2_NCMP              2             /* 2 Comparators */
#  define SAMD2L2_NDAC              1             /* 1 DAC channel */
#  define SAMD2L2_NOPAMP            3             /* 3 OpAmps */
#  define SAMD2L2_RTC               1             /* Have RTC */
#  define SAMD2L2_NALARMS           1             /* 1 RTC alarm */
#  define SAMD2L2_NRTCMP            1             /* RTC compare: 1 32-bit/2 16-bit */
#  define SAMD2L2_NEXTINT           16            /* 16 External interrupts */
#  define SAMD2L2_NPTCX             16            /* 16x16 */
#  define SAMD2L2_NPTCY             16            /* 16x16*/
#  define SAMD2L2_WDT               1             /* Have watchdog timer */
#endif

/* SAML21 Family ************************************************************/

/* FEATURE             SAM L21J          SAM L21G           SAM L21E
 * ------------------- ------------------ ------------------ --------
 * No. of pins         64                 48                 32
 * Flash               256/128/64KB       256/128/64KB       256/128/64/32KB
 * Flash RWW           8/4/2KB            8/4/2KB            8/4/2/1KB
 * SRAM                32/16/8KB          32/16/8KB          32/16/8/4KB
 * Max. Freq.          48MHz              48MHz              48MHz
 * Event channels      12                 12                 12
 * Timer/counters      5                  3                  3
 * TC output channels  2                  2                  2
 * T/C Control         3                  3                  3
 * TCC output channels 2                  2                  2
 * TCC waveform output 8/4/2              8/4/2              6/4/2
 * DMA channels        16                 16                 16
 * USB interface       1                  1                  1
 * AES engine          1                  1                  1
 * CCLs                4                  4                  4
 * TRNG                1                  1                  1
 * SERCOM              6                  6                  4
 * ADC channels        20                 14                 10
 * Comparators         2                  2                  2
 * DAC channels        2                  2                  2
 * OPAMP               3                  3                  3
 * RTC                 Yes                Yes                Yes
 * RTC alarms          1                  1                  1
 * RTC compare         1 32-bit/          1 32-bit/          1 32-bit/
 *                     2 16-bit           2 16-bit           2 16-bit
 * External interrupts 16                 16                 16
 * PTC X an Y          12x16              8x12               6x10
 *                     16x12              12x8               10x6
 * Packages            QFN/TQFP           QFN/TQFP           QFN/TQFP
 * Oscillators         XOSC32, XOSC, OSC32K, OSCULP32K,
 *                                     OSC16M, DFLL48M, and FDPLL96M
 * SW Debug interface  Yes                Yes                Yes
 * Watchdog timer      Yes                Yes                Yes
 */

#if defined(CONFIG_ARCH_CHIP_SAML21E15)

#  define SAML21                    1             /* SAML21 family */
#  define SAML21E                   1             /* SAML21E */
#  undef  SAML21G
#  undef  SAML21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (32*1024)     /* 32KB */
#  define SAMD2L2_FLASHRWW_SIZE     (1*1024)      /*  1KB */
#  define SAMD2L2_SRAM0_SIZE        (4*1024)      /*  4KB */
#  define SAMD2L2_LPRAM_SIZE        (2*1024)      /*  2KB */

/* TCC waveform outputs */

#  define SAMD2L2_TCC_NWAVEFORMS    2             /* 2 TCC waveform outputs */

#elif defined(CONFIG_ARCH_CHIP_SAML21E16)

#  define SAML21                    1             /* SAML21 family */
#  define SAML21E                   1             /* SAML21E */
#  undef  SAML21G
#  undef  SAML21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (64*1024)     /* 64KB */
#  define SAMD2L2_FLASHRWW_SIZE     (2*1024)      /*  2KB */
#  define SAMD2L2_SRAM0_SIZE        (8*1024)      /*  8KB */
#  define SAMD2L2_LPRAM_SIZE        (4*1024)      /*  4KB */

/* TCC waveform outputs */

#  define SAMD2L2_TCC_NWAVEFORMS    2             /* 2 TCC waveform outputs */

#elif defined(CONFIG_ARCH_CHIP_SAML21E17)

#  define SAML21                    1             /* SAML21 family */
#  define SAML21E                   1             /* SAML21E */
#  undef  SAML21G
#  undef  SAML21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (128*1024)    /* 128KB */
#  define SAMD2L2_FLASHRWW_SIZE     (4*1024)      /*   4KB */
#  define SAMD2L2_SRAM0_SIZE        (16*1024)     /*  16KB */
#  define SAMD2L2_LPRAM_SIZE        (8*1024)      /*   8KB */

/* TCC waveform outputs */

#  define SAMD2L2_TCC_NWAVEFORMS    4             /* 4 TCC waveform outputs */

#elif defined(CONFIG_ARCH_CHIP_SAML21E18)

#  define SAML21                    1             /* SAML21 family */
#  define SAML21E                   1             /* SAML21E */
#  undef  SAML21G
#  undef  SAML21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (256*1024)    /* 256KB */
#  define SAMD2L2_FLASHRWW_SIZE     (8*1024)      /*   8KB */
#  define SAMD2L2_SRAM0_SIZE        (32*1024)     /*  32KB */
#  define SAMD2L2_LPRAM_SIZE        (8*1024)      /*   8KB */

/* TCC waveform outputs */

#  define SAMD2L2_TCC_NWAVEFORMS    6             /* 6 TCC waveform outputs */

#elif defined(CONFIG_ARCH_CHIP_SAML21G16)

#  define SAML21                    1             /* SAML21 family */
#  undef  SAML21E
#  define SAML21G                   1             /* SAML21G */
#  undef  SAML21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (64*1024)     /* 64KB */
#  define SAMD2L2_FLASHRWW_SIZE     (2*1024)      /*  2KB */
#  define SAMD2L2_SRAM0_SIZE        (8*1024)      /*  8KB */
#  define SAMD2L2_LPRAM_SIZE        (4*1024)      /*  4KB */

/* TCC waveform outputs */

#  define SAMD2L2_TCC_NWAVEFORMS    2             /* 2 TCC waveform outputs */

#elif defined(CONFIG_ARCH_CHIP_SAML21G17)

#  define SAML21                    1             /* SAML21 family */
#  undef  SAML21E
#  define SAML21G                   1             /* SAML21G */
#  undef  SAML21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (128*1024)    /* 128KB */
#  define SAMD2L2_FLASHRWW_SIZE     (4*1024)      /*   4KB */
#  define SAMD2L2_SRAM0_SIZE        (16*1024)     /*  16KB */
#  define SAMD2L2_LPRAM_SIZE        (8*1024)      /*   8KB */

/* TCC waveform outputs */

#  define SAMD2L2_TCC_NWAVEFORMS    4              /* 4 TCC waveform outputs */

#elif defined(CONFIG_ARCH_CHIP_SAML21G18)

#  define SAML21                    1             /* SAML21 family */
#  undef  SAML21E
#  define SAML21G                   1             /* SAML21G */
#  undef  SAML21J

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (256*1024)    /* 256KB */
#  define SAMD2L2_FLASHRWW_SIZE     (8*1024)      /*   8KB */
#  define SAMD2L2_SRAM0_SIZE        (32*1024)     /*  32KB */
#  define SAMD2L2_LPRAM_SIZE        (8*1024)      /*   8KB */

/* TCC waveform outputs */

#  define SAMD2L2_TCC_NWAVEFORMS    8              /* 8 TCC waveform outputs */

#elif defined(CONFIG_ARCH_CHIP_SAML21J16)

#  define SAML21                    1             /* SAML21 family */
#  undef  SAML21E
#  undef  SAML21G
#  define SAML21J                   1             /* SAML21J */

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (64*1024)     /* 64KB */
#  define SAMD2L2_FLASHRWW_SIZE     (2*1024)      /*  2KB */
#  define SAMD2L2_SRAM0_SIZE        (8*1024)      /*  8KB */
#  define SAMD2L2_LPRAM_SIZE        (8*1024)      /*  4KB */

/* TCC waveform outputs */

#  define SAMD2L2_TCC_NWAVEFORMS    2             /* 2 TCC waveform outputs */

#elif defined(CONFIG_ARCH_CHIP_SAML21J17)

#  define SAML21                    1             /* SAML21 family */
#  undef  SAML21E
#  undef  SAML21G
#  define SAML21J                   1             /* SAML21J */

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (128*1024)    /* 128KB */
#  define SAMD2L2_FLASHRWW_SIZE     (4*1024)      /*   4KB */
#  define SAMD2L2_SRAM0_SIZE        (16*1024)     /*  16KB */
#  define SAMD2L2_LPRAM_SIZE        (8*1024)      /*   8KB */

/* TCC waveform outputs */

#  define SAMD2L2_TCC_NWAVEFORMS    4             /* 4 TCC waveform outputs */

#elif defined(CONFIG_ARCH_CHIP_SAML21J18)

#  define SAML21                    1             /* SAML21 family */
#  undef  SAML21E
#  undef  SAML21G
#  define SAML21J                   1             /* SAML21J */

/* Internal memory */

#  define SAMD2L2_FLASH_SIZE        (256*1024)    /* 256KB */
#  define SAMD2L2_FLASHRWW_SIZE     (8*1024)      /*   8KB */
#  define SAMD2L2_SRAM0_SIZE        (32*1024)     /*  32KB */
#  define SAMD2L2_LPRAM_SIZE        (8*1024)      /*   8KB */

/* TCC waveform outputs */

#  define SAMD2L2_TCC_NWAVEFORMS    8             /* 8 TCC waveform outputs */

#else

#  undef  SAML21                                  /* Not SAML21 family */
#  undef  SAML21E
#  undef  SAML21G
#  undef  SAML21J

#endif

#if defined(SAML21E)
#  define SAMD2L2_NEVENTS           12            /* 12 event channels */
#  define SAMD2L2_NTC               3             /* 3 Timer/counters */
#  define SAMD2L2_NTCOUT            2             /* 2 TC output channels */
#  define SAMD2L2_NTCC              3             /* 3 TC control channels */
#  define SAMD2L2_NTCCOUT           2             /* 2 TCC output channels */
#  define SAMD2L2_NDMACHAN          16            /* 16 DMA channels */
#  define SAMD2L2_NUSBIF            1             /* 1 USB interface */
#  define SAMD2L2_NAES              1             /* 1 AES engine */
#  define SAMD2L2_NCCL              4             /* 4 Counfigurable Custom Logic */
#  define SAMD2L2_NTRNG             1             /* 1 True random number generator */
#  define SAMD2L2_NSERCOM           4             /* 4 SERCOM */
#  define SAMD2L2_NI2S              0             /* No I2S */
#  define SAMD2L2_NADC              10            /* 10 ADC channels */
#  define SAMD2L2_NCMP              2             /* 2 Comparators */
#  define SAMD2L2_NDAC              2             /* 2 DAC channels */
#  define SAMD2L2_NOPAMP            3             /* 3 OpAmps */
#  define SAMD2L2_RTC               1             /* Have RTC */
#  define SAMD2L2_NALARMS           1             /* 1 RTC alarm */
#  define SAMD2L2_NRTCMP            1             /* RTC compare: 1 32-bit/2 16-bit */
#  define SAMD2L2_NEXTINT           16            /* 16 External interrupts */
#  define SAMD2L2_NPTCX             10            /* PTC X 6 or 10 */
#  define SAMD2L2_NPTCY             10            /* PTC Y 6 or 10*/
#  define SAMD2L2_WDT               1             /* Have watchdog timer */
#elif defined(SAML21G)
#  define SAMD2L2_NEVENTS           12            /* 12 event channels */
#  define SAMD2L2_NTC               3             /* 3 Timer/counters */
#  define SAMD2L2_NTCOUT            2             /* 2 TC output channels */
#  define SAMD2L2_NTCC              3             /* 3 TC control channels */
#  define SAMD2L2_NTCCOUT           2             /* 2 TCC output channels */
#  define SAMD2L2_NDMACHAN          16            /* 16 DMA channels */
#  define SAMD2L2_NUSBIF            1             /* 1 USB interface */
#  define SAMD2L2_NAES              1             /* 1 AES engine */
#  define SAMD2L2_NCCL              4             /* 4 Counfigurable Custom Logic */
#  define SAMD2L2_NTRNG             1             /* 1 True random number generator */
#  define SAMD2L2_NSERCOM           6             /* 6 SERCOM */
#  define SAMD2L2_NI2S              0             /* No I2S */
#  define SAMD2L2_NADC              14            /* 14 ADC channels */
#  define SAMD2L2_NCMP              2             /* 2 Comparators */
#  define SAMD2L2_NDAC              2             /* 2 DAC channels */
#  define SAMD2L2_NOPAMP            3             /* 3 OpAmps */
#  define SAMD2L2_RTC               1             /* Have RTC */
#  define SAMD2L2_NALARMS           1             /* 1 RTC alarm */
#  define SAMD2L2_NRTCMP            1             /* RTC compare: 1 32-bit/2 16-bit */
#  define SAMD2L2_NEXTINT           16            /* 16 External interrupts */
#  define SAMD2L2_NPTCX             12            /* PTC X 8 or 12 */
#  define SAMD2L2_NPTCY             12            /* PTC Y 8 or 12*/
#  define SAMD2L2_WDT               1             /* Have watchdog timer */
#elif defined(SAML21J)
#  define SAMD2L2_NEVENTS           12            /* 12 event channels */
#  define SAMD2L2_NTC               5             /* 5 Timer/counters */
#  define SAMD2L2_NTCOUT            2             /* 2 TC output channels */
#  define SAMD2L2_NTCC              3             /* 3 TC control channels */
#  define SAMD2L2_NTCCOUT           2             /* 2 TCC output channels */
#  define SAMD2L2_NDMACHAN          16            /* 16 DMA channels */
#  define SAMD2L2_NUSBIF            1             /* 1 USB interface */
#  define SAMD2L2_NAES              1             /* 1 AES engine */
#  define SAMD2L2_NCCL              4             /* 4 Counfigurable Custom Logic */
#  define SAMD2L2_NTRNG             1             /* 1 True random number generator */
#  define SAMD2L2_NSERCOM           6             /* 6 SERCOM */
#  define SAMD2L2_NI2S              0             /* No I2S */
#  define SAMD2L2_NADC              20            /* 20 ADC channels */
#  define SAMD2L2_NCMP              2             /* 2 Comparators */
#  define SAMD2L2_NDAC              2             /* 2 DAC channels */
#  define SAMD2L2_NOPAMP            3             /* 3 OpAmps */
#  define SAMD2L2_RTC               1             /* Have RTC */
#  define SAMD2L2_NALARMS           1             /* 1 RTC alarm */
#  define SAMD2L2_NRTCMP            1             /* RTC compare: 1 32-bit/2 16-bit */
#  define SAMD2L2_NEXTINT           16            /* 16 External interrupts */
#  define SAMD2L2_NPTCX             16            /* PTC X 12 or 16 */
#  define SAMD2L2_NPTCY             16            /* PTC Y 12 or 16*/
#  define SAMD2L2_WDT               1             /* Have watchdog timer */
#endif

/* NVIC priority levels *****************************************************/

/* Each priority field holds a priority value, 0-3. The lower the value, the
 * greater the priority of the corresponding interrupt. The processor
 * implements only bits[7:6] of each field, bits[5:0] read as zero and ignore
 * writes.
 */

#define NVIC_SYSH_PRIORITY_MIN      0xc0 /* All bits[7:6] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT  0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX      0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP     0x40 /* Five bits of interrupt priority used */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_SAMD2L2_CHIP_H */
