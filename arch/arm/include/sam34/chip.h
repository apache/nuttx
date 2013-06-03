/************************************************************************************
 * arch/arm/include/sam34/chip.h
 *
 *   Copyright (C) 2009-2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_SAM34_CHIP_H
#define __ARCH_ARM_INCLUDE_SAM34_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

/* AT91SAM3U Family *****************************************************************/

#if defined(CONFIG_ARCH_CHIP_AT91SAM3U4E)

/* Internal memory */

#  define CONFIG_SAM34_FLASH_SIZE    (256*1024)  /* 256Kb */
#  define CONFIG_SAM34_SRAM0_SIZE    (32*1024)   /*  32Kb */
#  define CONFIG_SAM34_SRAM1_SIZE    (16*1024)   /*  16Kb */
#  define CONFIG_SAM34_NFCSRAM_SIZE  (4*1024)    /*   4Kb */

/* Peripherals */

#  define SAM34_NDMACHAN              4          /* 4 DMA Channels */
#  define SAM34_NMCI2                 1          /* 1 memory card interface */
#  define SAM32_NSLCD                 0          /* No segment LCD interface */
#  define SAM32_NAESA                 0          /* No advanced encryption standard */
#  define SAM32_NUDPHS                1          /* One USB high speed device */
#  define SAM32_NUDPFS                0          /* No USB full speed device */
#  define SAM32_NUHPFS                0          /* No USB full speed embedded host */

/* AT91SAM4L Family *****************************************************************/
/* Sub-family differences:
 *
 *   FEATURE                 ATSAM4LCxx    ATSAM4LSxx
 *   ----------------------- ------------- -------------
 *   SEGMENT LCD             Yes           No
 *   AESA                    Yes           No
 *   USB                     Device + Host Device Only
 *
 *   Note:  The SEGMENT LCD capability varies with packaging.
 *
 *   FEATURE                 ATSAM4Lx2x    ATSAM4Lx4x
 *   ----------------------- ------------- -------------
 *   FLASH                   256KB         128KB
 *   SRAM                    32KB          32KB
 *
 * Packaging differences:
 *
 *   FEATURE                 ATSAM4LxxC ATSAM4LxxB ATSAM4LxxA
 *   ----------------------- ---------- ---------- ----------
 *   Number of Pins          100        64         48
 *   Max Frequency           48MHz      48MHz      48MHz
 *   SEGMENT LCD             4x40       4x23       4x13
 *   GPIO                    75         43         27
 *   High-drive pins         6          3          1
 *   External Interrupts     8+NMI      8+NMI      8+NMI
 *   TWI Masters             2          2          1
 *   TWI Master/Slave        2          2          1
 *   USART                   4          4          3
 *   PICOUART                1          1          1
 *   Peripheral DMA Channels 16         16         16
 *   Peripheral Even System  1          1          1
 *   SPI                     1          1          1
 *   Asynchronous Timers     1          1          1
 *   Timer/Counter Channels  6          3          3
 *   Parallel Capture Inputs 8          8          8
 *   Frequency Meter         1          1          1
 *   Watchdog Timer          1          1          1
 *   Power Manager           1          1          1
 *   Glue Logic LUT          2          2          1
 *   ADC                     15-channel 7-channel  3-channel
 *   DAC                     1-channel  1-channel  1-channel
 *   Analog Comparators      4          2          1
 *   CATB Sensors            32         32         26
 *   Audio Bitstream DAC     1          1          1
 *   IIS Controller          1          1          1
 *   Packages                TQFP/VFBGA TQFP/QFN   TQFP/QFN
 */

#elif defined(CONFIG_ARCH_CHIP_AT91SAM4LC2)

/* Internal memory */

#  define CONFIG_SAM34_FLASH_SIZE    (128*1024)  /* 128Kb */
#  define CONFIG_SAM34_SRAM0_SIZE    (32*1024)   /*  32Kb */
#  define CONFIG_SAM34_SRAM1_SIZE    (0)         /* None */
#  define CONFIG_SAM34_NFCSRAM_SIZE  (0)         /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              16         /* 16 Peripheral DMA Channels */
#  define SAM34_NMCI2                 0          /* No memory card interface */
#  define SAM32_NSLCD                 1          /* 1 segment LCD interface */
#  define SAM32_NAESA                 1          /* 1 advanced encryption standard */
#  define SAM32_NUDPHS                0          /* No USB high speed device */
#  define SAM32_NUDPFS                1          /* 1 USB full speed device */
#  define SAM32_NUHPFS                1          /* 1 USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_AT91SAM4LC4)

/* Internal memory */

#  define CONFIG_SAM34_FLASH_SIZE    (256*1024)  /* 256Kb */
#  define CONFIG_SAM34_SRAM0_SIZE    (32*1024)   /*  32Kb */
#  define CONFIG_SAM34_SRAM1_SIZE    (0)         /* None */
#  define CONFIG_SAM34_NFCSRAM_SIZE  (0)         /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              16         /* 16 Peripheral DMA Channels */
#  define SAM34_NMCI2                 0          /* No memory card interface */
#  define SAM32_NSLCD                 1          /* 1 segment LCD interface */
#  define SAM32_NAESA                 1          /* 1 advanced encryption standard */
#  define SAM32_NUDPHS                0          /* No USB high speed device */
#  define SAM32_NUDPFS                1          /* 1 USB full speed device */
#  define SAM32_NUHPFS                1          /* 1 USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_AT91SAM4LS2)

/* Internal memory */

#  define CONFIG_SAM34_FLASH_SIZE    (128*1024)  /* 128Kb */
#  define CONFIG_SAM34_SRAM0_SIZE    (32*1024)   /*  32Kb */
#  define CONFIG_SAM34_SRAM1_SIZE    (0)         /* None */
#  define CONFIG_SAM34_NFCSRAM_SIZE  (0)         /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              16         /* 16 Peripheral DMA Channels */
#  define SAM34_NMCI2                 0          /* No memory card interface */
#  define SAM32_NSLCD                 0          /* No segment LCD interface */
#  define SAM32_NAESA                 0          /* No advanced encryption standard */
#  define SAM32_NUDPHS                0          /* No USB high speed device */
#  define SAM32_NUDPFS                1          /* 1 USB full speed device */
#  define SAM32_NUHPFS                0          /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_AT91SAM4LS4)

/* Internal memory */

#  define CONFIG_SAM34_FLASH_SIZE    (256*1024)  /* 256Kb */
#  define CONFIG_SAM34_SRAM0_SIZE    (32*1024)   /*  32Kb */
#  define CONFIG_SAM34_SRAM1_SIZE    (0)         /* None */
#  define CONFIG_SAM34_NFCSRAM_SIZE  (0)         /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              16         /* 16 Peripheral DMA Channels */
#  define SAM34_NMCI2                 0          /* No memory card interface */
#  define SAM32_NSLCD                 0          /* No segment LCD interface */
#  define SAM32_NAESA                 0          /* No advanced encryption standard */
#  define SAM32_NUDPHS                0          /* No USB high speed device */
#  define SAM32_NUDPFS                1          /* 1 USB full speed device */
#  define SAM32_NUHPFS                0          /* No USB full speed embedded host */

#else
#  error "Unknown SAM3/4 chip type"
#endif

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0-15. The lower the value, the greater
 * the priority of the corresponding interrupt. The processor implements only
 * bits[7:4] of each field, bits[3:0] read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN       0xf0 /* All bits[7:4] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT   0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX       0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP      0x10 /* Four bits of interrupt priority used */

#define NVIC_SYSH_DISABLE_PRIORITY   (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#define NVIC_SYSH_SVCALL_PRIORITY    NVIC_SYSH_PRIORITY_MAX

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_SAM34_CHIP_H */
