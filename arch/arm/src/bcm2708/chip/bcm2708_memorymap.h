/************************************************************************************
 * arch/arm/src/bcm2708/chip/bcm2708_memorymap.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __ARCH_ARM_SRC_BCM_CHIP_BCM_MEMORYMAP_H
#define __ARCH_ARM_SRC_BCM_CHIP_BCM_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* BCM2835 Physical (unmapped) Memory Map *******************************************/
/* BCM2835 System 1-Mbyte aligned physical sections (PSECTIONS) */

#define BCM_SDRAM_PSECTION    0x00000000 /* Beginning of SDRAM section */
#define BCM_VCSDRAM_PSECTION  0x16000000 /* The VideoCore SDRAM is part of SDRAM */
#define BCM_PERIPH_PSECTION   0x20000000 /* Beginning of the peripheral section */

/* BCM2708 Peripherals PSECTION Offsets */

#define BCM_SYSTMR_OFFSET     0x00003000 /* System Timer peripheral */
#define BCM_MPHI_OFFSET       0x00006000 /* Message Parallel Host Interface */
#define BCM_DMA_OFFSET        0x00007000 /* DMA Controller */
#define BCM_IRQ_OFFSET        0x0000b000 /* ARM interrupt register */
#define BCM_TIMER_OFFSET      0x0000b400 /* Timer peripheral */
#define BCM_PM_OFFSET         0x00100000 /* Power Management, Reset and Watchdog */
#define BCM_CMGPCLK_OFFSET    0x00101070 /* Clock Manager General Purpose Clocks*/
#define BCM_PCM_CLK_OFFSET    0x00101098 /* PCM Clock */
#define BCM_RNG_OFFSET        0x00104000 /* Hardware RNG */
#define BCM_GPIO_OFFSET       0x00200000 /* GPIO peripheral */
#define BCM_PL011_OFFSET      0x00201000 /* PL011 UART peripheral */
#define BCM_MMCI0_OFFSET      0x00202000 /* MMC0 peripheral */
#define BCM_I2S_OFFSET        0x00203000 /* PCM/I2S audio interface */
#define BCM_SPI0_OFFSET       0x00204000 /* Serial interface peripheral */
#define BCM_BSC0_OFFSET       0x00205000 /* Broadcom Serial Controller 0 (BSC0) */
#define BCM_PWM_OFFSET        0x0020c000 /* Pulse Width Modulator interface */
#define BCM_BSCSPI_OFFSET     0x00214000 /* BSC/SPI peripheral */
#define BCM_AUX_OFFSET        0x00215000 /* AUX/Mini-UART/SPI peripherals */
#define BCM_EMMC_OFFSET       0x00300000 /* External Mass Media Controller */
#define BCM_SMI_OFFSET        0x00600000 /* SMI */
#define BCM_BSC1_OFFSET       0x00804000 /* Broadcom Serial Controller 1 (BSC1) */
#define BCM_BSC2_OFFSET       0x00805000 /* Broadcom Serial Controller 2 (BSC2) */
#define BCM_USB_OFFSET        0x00980000 /* USB Controller */

/* Sizes of memory regions in bytes. */

#define BCM_SDRAM_SIZE        (352*1024*1024) /* 00000000-15ffffff: 352MiB RAM */
#define BCM_VCSDRAM_SIZE      (160*1024*1024) /* 16000000-1fffffff: 160MiB Video RAM*/
#define BCM_PERIPH_SIZE       (10*1024*1024)  /* 20000000-209fffff: Peripherals */

/* Convert size in bytes to number of sections (in Mb). */

#define _NSECTIONS(b)         (((b)+0x000fffff) >> 20)

/* Sizes of memory regions in sections */

#define BCM_SDRAM_NSECTIONS   _NSECTIONS(BCM_SDRAM_SIZE)
#define BCM_VCSDRAM_NSECTIONS _NSECTIONS(BCM_VCSDRAM_SIZE)
#define BCM_PERIPH_NSECTIONS  _NSECTIONS(BCM_PERIPH_SIZE)

/* Section MMU Flags
 *
 * SDRAM is a special case because it requires non-cached access of its
 * initial configuration, then cached access thereafter.
 */

#define BCM_SDRAM_MMUFLAGS    MMU_MEMFLAGS
#define BCM_VCRAM_MMUFLAGS    MMU_IOFLAGS
#define BCM_PERIPH_MMUFLAGS   MMU_IOFLAGS

/* BCM2835 System 1-Mbyte aligned virtual sections (VSECTIONS). */

#ifdef CONFIG_ARCH_ROMPGTABLE
#  error This configuration does not support a ROM page table
#endif

#define BCM_SDRAM_VSECTION    0x00000000  /* Virtual section of the SDRAM */
#define BCM_VCSDRAM_VSECTION  0x16000000  /* Virtual section of the GPU RAM: 160MiB */
#define BCM_PERIPH_VSECTION   0xf2000000  /* Virtual section of the peripherals */

/* BCM2708 Peripherals Virtual Base Addresses */

#define BCM_SYSTMR_VBASE      (BCM_PERIPH_VSECTION + BCM_SYSTMR_OFFSET)
#define BCM_MPHI_VBASE        (BCM_PERIPH_VSECTION + BCM_MPHI_OFFSET)
#define BCM_DMA_VBASE         (BCM_PERIPH_VSECTION + BCM_DMA_OFFSET)
#define BCM_IRQ_VBASE         (BCM_PERIPH_VSECTION + BCM_IRQ_OFFSET)
#define BCM_TIMER_VBASE       (BCM_PERIPH_VSECTION + BCM_TIMER_OFFSET)
#define BCM_PM_VBASE          (BCM_PERIPH_VSECTION + BCM_PM_OFFSET)
#define BCM_CMGPCLK_VBASE     (BCM_PERIPH_VSECTION + BCM_CMGPCLK_OFFSET)
#define BCM_PCM_CLK_VBASE     (BCM_PERIPH_VSECTION + BCM_PCM_CLK_OFFSET)
#define BCM_RNG_VBASE         (BCM_PERIPH_VSECTION + BCM_RNG_OFFSET)
#define BCM_GPIO_VBASE        (BCM_PERIPH_VSECTION + BCM_GPIO_OFFSET)
#define BCM_PL011_VBASE       (BCM_PERIPH_VSECTION + BCM_PL011_OFFSET)
#define BCM_MMCI0_VBASE       (BCM_PERIPH_VSECTION + BCM_MMCI0_OFFSET)
#define BCM_I2S_VBASE         (BCM_PERIPH_VSECTION + BCM_I2S_OFFSET)
#define BCM_SPI0_VBASE        (BCM_PERIPH_VSECTION + BCM_SPI0_OFFSET)
#define BCM_BSC0_VBASE        (BCM_PERIPH_VSECTION + BCM_BSC0_OFFSET)
#define BCM_PWM_VBASE         (BCM_PERIPH_VSECTION + BCM_PWM_OFFSET)
#define BCM_BSCSPI_VBASE      (BCM_PERIPH_VSECTION + BCM_BSCSPI_OFFSET)
#define BCM_AUX_VBASE         (BCM_PERIPH_VSECTION + BCM_AUX_OFFSET)
#define BCM_EMMC_VBASE        (BCM_PERIPH_VSECTION + BCM_EMMC_OFFSET)
#define BCM_SMI_VBASE         (BCM_PERIPH_VSECTION + BCM_SMI_OFFSET)
#define BCM_BSC1_VBASE        (BCM_PERIPH_VSECTION + BCM_BSC1_OFFSET)
#define BCM_BSC2_VBASE        (BCM_PERIPH_VSECTION + BCM_BSC2_OFFSET)
#define BCM_USB_VBASE         (BCM_PERIPH_VSECTION + BCM_USB_OFFSET)

/* Vector table:
 *
 *   BCM_VECTOR_PADDR  - Unmapped, physical address of vector table in SDRAM
 *   BCM_VECTOR_VSDRAM - Virtual address of vector table in SDRAM
 *   BCM_VECTOR_VADDR  - Virtual address of vector table (must be 0x00000000)
 *
 * NOTE: Vectors in hight memory are not supported.
 */

#ifndef CONFIG_ARCH_LOWVECTORS
#  errno CONFIG_ARCH_LOWVECTORS is required for this memory configuration.
#endif

#define VECTOR_TABLE_SIZE     0x00004000
#define BCM_VECTOR_PADDR      BCM_SDRAM_PSECTION
#define BCM_VECTOR_VSDRAM     BCM_SDRAM_VSECTION
#define BCM_VECTOR_VADDR      0x00000000
#define BCM_VECTOR_VCOARSE    0x00000000

/* Page Table.  The page table immediately follows the vector table.  .text
 * should then be positioned after the page table.
 *
 * The L1 page table must be on a physical address that is 16KiB aligned.
 * This is assured by the value of VECTOR_TABLE_SIZE.   The size of the L1
 * page table is also 16KiB: 16KiB / (4bytes/entry) * 1MB gives a 4GB
 * address space.
 */

#define  PGTABLE_BASE_PADDR    (BCM_VECTOR_PADDR + VECTOR_TABLE_SIZE)
#define  PGTABLE_BASE_VADDR    (BCM_VECTOR_VADDR + VECTOR_TABLE_SIZE)

/* The boot logic will create a temporarily mapping based on where NuttX is
 * executing in memory.  In this case, there is support only for NuttX
 * running from SDRAM.  NuttX is assumed to be positioned by the linker
 * script to lie immediately following the L1 page table.
 */

#if defined(CONFIG_BOOT_RUNFROMSDRAM)
#  define NUTTX_START_VADDR    (PGTABLE_BASE_VADDR + PGTABLE_SIZE)
#  define NUTTX_START_PADDR    (PGTABLE_BASE_PADDR + PGTABLE_SIZE)
#else
#  error Unsupported boot memory
#endif

#endif /* __ARCH_ARM_SRC_BCM_CHIP_BCM_MEMORYMAP_H */
