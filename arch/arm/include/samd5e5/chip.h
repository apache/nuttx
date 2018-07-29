/************************************************************************************
 * arch/arm/include/samd5e5/chip.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_SAMD5E5_CHIP_H
#define __ARCH_ARM_INCLUDE_SAMD5E5_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip.  Only sizes and numbers of things are
 * provided here.  See arch/arm/src/samd5e5/Kconfig for other, boolean configuration
 * settings.
 */

#if defined(CONFIG_ARCH_CHIP_SAMD51P20)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (1024*1024)   /* 1024KB */
#  define SAMD5E5_SRAM_SIZE         (256*1024)    /*  256KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           8             /* Sercomm0-7 */
#  define SAMD5E5_NTC               8             /* TC0-TC7 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NSDHC             2             /* SDHC0-1 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           14            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         16            /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           5             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAMD51P19)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (512*1024)   /* 512KB */
#  define SAMD5E5_SRAM_SIZE         (192*1024)   /* 192KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           8             /* Sercomm0-7 */
#  define SAMD5E5_NTC               8             /* TC0-TC7 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NSDHC             2             /* SDHC0-1 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           14            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         6             /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           5             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAMD51N20)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (1024*1024)   /* 1024KB */
#  define SAMD5E5_SRAM_SIZE         (256*1024)    /*  256KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           8             /* Sercomm0-7 */
#  define SAMD5E5_NTC               8             /* TC0-TC7 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NSDHC             2             /* SDHC0-1 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           14            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         12            /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           5             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAMD51N19)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (512*1024)   /* 512KB */
#  define SAMD5E5_SRAM_SIZE         (192*1024)   /* 192KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           8             /* Sercomm0-7 */
#  define SAMD5E5_NTC               8             /* TC0-TC7 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NSDHC             2             /* SDHC0-1 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           14            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         12            /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           5             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAMD51J20)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (1024*1024)   /* 1024KB */
#  define SAMD5E5_SRAM_SIZE         (256*1024)    /*  256KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           6             /* Sercomm0-5 */
#  define SAMD5E5_NTC               6             /* TC0-TC5 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NSDHC             1             /* SDHC0 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           10            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         8             /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           3             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAMD51J19)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (512*1024)   /* 512KB */
#  define SAMD5E5_SRAM_SIZE         (192*1024)   /* 192KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           6             /* Sercomm0-5 */
#  define SAMD5E5_NTC               6             /* TC0-TC5 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NSDHC             1             /* SDHC0 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           10            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         8             /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           3             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAMD51J18)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (256*1024)   /* 1256KB */
#  define SAMD5E5_SRAM_SIZE         (128*1024)   /* 128KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           6             /* Sercomm0-5 */
#  define SAMD5E5_NTC               6             /* TC0-TC5 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NSDHC             1             /* SDHC0 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           10            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         8             /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           3             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAMD51G19)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (512*1024)   /* 512KB */
#  define SAMD5E5_SRAM_SIZE         (192*1024)   /* 192KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           6             /* Sercomm0-5 */
#  define SAMD5E5_NTC               4             /* TC0-TC3 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            1             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             1             /* SDHC0 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           10            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         4             /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           2             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAMD51G18)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (256*1024)   /* 1256KB */
#  define SAMD5E5_SRAM_SIZE         (128*1024)   /* 128KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           6             /* Sercomm0-5 */
#  define SAMD5E5_NTC               4             /* TC0-TC3 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            1             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             1             /* SDHC0 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           10            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         4             /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           2             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAME51N20)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (1024*1024)   /* 1024KB */
#  define SAMD5E5_SRAM_SIZE         (256*1024)    /*  256KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           8             /* Sercomm0-7 */
#  define SAMD5E5_NTC               4             /* TC0-TC3 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            3             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             1             /* SDHC0 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           14            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         12            /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           5             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAME51N19)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (512*1024)   /* 512KB */
#  define SAMD5E5_SRAM_SIZE         (192*1024)   /* 192KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           8             /* Sercomm0-7 */
#  define SAMD5E5_NTC               4             /* TC0-TC3 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            3             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             1             /* SDHC0 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           14            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         12            /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           5             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAME51J20)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (1024*1024)   /* 1024KB */
#  define SAMD5E5_SRAM_SIZE         (256*1024)    /*  256KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           6             /* Sercomm0-5 */
#  define SAMD5E5_NTC               6             /* TC0-TC5 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            3             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             1             /* SDHC0 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           10            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         8             /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           3             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAME51J19)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (512*1024)   /* 512KB */
#  define SAMD5E5_SRAM_SIZE         (192*1024)   /* 192KB */

/* Peripherals -- to be provided */

#elif defined(CONFIG_ARCH_CHIP_SAME51J18)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (256*1024)   /* 1256KB */
#  define SAMD5E5_SRAM_SIZE         (128*1024)   /* 128KB */

/* Peripherals -- to be provided */

#elif defined(CONFIG_ARCH_CHIP_SAME53N20)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (1024*1024)   /* 1024KB */
#  define SAMD5E5_SRAM_SIZE         (256*1024)    /*  256KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           8             /* Sercomm0-7 */
#  define SAMD5E5_NTC               8             /* TC0-TC7 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            3             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             2             /* SDHC0-1 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           14            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         12            /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           5             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAME53N19)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (512*1024)   /* 512KB */
#  define SAMD5E5_SRAM_SIZE         (192*1024)   /* 192KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           8             /* Sercomm0-7 */
#  define SAMD5E5_NTC               8             /* TC0-TC7 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            3             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             2             /* SDHC0-1 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           14            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         12            /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           5             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAME53J20)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (1024*1024)   /* 1024KB */
#  define SAMD5E5_SRAM_SIZE         (256*1024)    /*  256KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           6             /* Sercomm0-5 */
#  define SAMD5E5_NTC               6             /* TC0-TC5 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            3             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             1             /* SDHC0 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           10            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         8             /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           3             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAME53J19)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (512*1024)   /* 512KB */
#  define SAMD5E5_SRAM_SIZE         (192*1024)   /* 192KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           6             /* Sercomm0-5 */
#  define SAMD5E5_NTC               6             /* TC0-TC5 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            3             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             1             /* SDHC0 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           10            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         8             /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           3             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAME53J18)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (256*1024)   /* 1256KB */
#  define SAMD5E5_SRAM_SIZE         (128*1024)   /* 128KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           6             /* Sercomm0-5 */
#  define SAMD5E5_NTC               6             /* TC0-TC5 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            3             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             1             /* SDHC0 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           10            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         8             /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           3             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAME54P20)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (1024*1024)   /* 1024KB */
#  define SAMD5E5_SRAM_SIZE         (256*1024)    /*  256KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           8             /* Sercomm0-7 */
#  define SAMD5E5_NTC               8             /* TC0-TC7 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            3             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             2             /* SDHC0-1 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           14            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         16            /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           5             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAME54P19)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (512*1024)   /* 512KB */
#  define SAMD5E5_SRAM_SIZE         (192*1024)   /* 192KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           8             /* Sercomm0-7 */
#  define SAMD5E5_NTC               8             /* TC0-TC7 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            3             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             2             /* SDHC0-1 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           14            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         16            /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           5             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAME54N20)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (1024*1024)   /* 1024KB */
#  define SAMD5E5_SRAM_SIZE         (256*1024)    /*  256KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           8             /* Sercomm0-7 */
#  define SAMD5E5_NTC               8             /* TC0-TC7 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            3             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             1             /* SDHC0 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           14            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         12            /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           5             /* Tamper pins */

#elif defined(CONFIG_ARCH_CHIP_SAME54N19)

/* Internal memory */

#  define SAMD5E5_FLASH_SIZE        (512*1024)   /* 512KB */
#  define SAMD5E5_SRAM_SIZE         (192*1024)   /* 192KB */

/* Peripherals */

#  define SAMD5E5_NSERCOM           8             /* Sercomm0-7 */
#  define SAMD5E5_NTC               8             /* TC0-TC7 */
#  define SAMD5E5_NTCCOMP           2             /* TC compare */
#  define SAMD5E5_NTCC24            2             /* TCC 24-bit */
#  define SAMD5E5_NTCC16            3             /* TCC 16-bit */
#  define SAMD5E5_NSDHC             1             /* SDHC0 */
#  define SAMD5E5_NDMACHAN          32            /* 32 DMA channels */
#  define SAMD5E5_PCCSIZE           14            /* PCC data size */
#  define SAMD5E5_NCCL              4             /* CCL */
#  define SAMD5E5_NEVTCHAN          32            /* Event system channels */
#  define SAMD5E5_NADC0CHAN         16            /* ADC0 channels */
#  define SAMD5E5_NADC1CHAN         12            /* ADC1 channels */
#  define SAMD5E5_NACMP             4             /* Analog comparators */
#  define SAMD5E5_NDACCHAN          2             /* DAC channels */
#  define SAMD5E5_NPCTMANCHAN       256           /* PTC manual channels */
#  define SAMD5E5_NPCTSELFCHAN      32            /* PTC self-capacitance channels */
#  define SAMD5E5_NTAMPER           5             /* Tamper pins */

#else
#  error Unrecognized SAMD5x/Ex chip
#endif

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0x00-0xe0. The lower the value, the
 * greater the priority of the corresponding interrupt. The processor implements only
 * bits[7:4] of each field, bits[6:0] read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN      0xe0 /* All bits[7:5] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT  0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX      0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP     0x20 /* Eight priority levels in steps 0x20 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_SAMD5E5_CHIP_H */
