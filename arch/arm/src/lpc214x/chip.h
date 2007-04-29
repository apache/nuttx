/************************************************************************************
 * lpc214x/chip.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#ifndef __LPC214X_CHIP_H
#define __LPC214X_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/* Memory Map ***********************************************************************/

#define LPC214X_FLASH_BASE      0x00000000
#define LPC214X_ONCHIP_RAM_BASE 0x40000000
#define LPC214X_EXTMEM_BASE     0x80000000

/* Interrupts **********************************************************************/

/* Peripheral Registers ************************************************************/

/* Register block base addresses */

#define LPC214X_MAM_BASE        0xe01fc000  /* Memory Accelerator Module (MAM) Base Address */
#define LPC214X_MEMMAP          0xe01fc040  /* Memory Mapping Control */
#define LPC214X_PLL_BASE        0xe01fc080  /* Phase Locked Loop (PLL) Base Address */
#define LPC214X_VPBDIV          0xe01fc100  /* VPBDIV Address */
#define LPC214X_PINSEL2         0xe002c014  /* PINSEL2 Address */
#define LPC214X_EMC_BASE        0xffe00000  /* External Memory Controller (EMC) Base Address */

/* Memory Accelerator Module (MAM) Regiser Offsets */

#define LPC214X_MAMCR_OFFSET    0x00        /* MAM Control Offset*/
#define LPC214x_MAMTIM_OFFSET   0x04        /* MAM Timing Offset */

/* Phase Locked Loop (PLL) register offsets */

#define LPC214X_PLLCON_OFFSET   0x00        /* PLL Control Offset*/
#define LPC214X_PLLCFG_OFFSET   0x04        /* PLL Configuration Offset */
#define LPC214X_PLLSTAT_OFFSET  0x08        /* PLL Status Offset */
#define LPC214X_PLLFEED_OFFSET  0x0c        /* PLL Feed Offset */

/* PLL Control Register Bit Settings */

#define LPC214X_PLLCON_PLLE     (1<<0)      /* PLL Enable */
#define LPC214X_PLLCON_PLLC     (1<<1)      /* PLL Connect */

/* PLL Configuration Register Bit Settings */

#define LPC214X_PLLCFG_MSEL     (0x1f<<0)   /* PLL Multiplier */
#define LPC214X_PLLCFG_PSEL     (0x03<<5)   /* PLL Divider */
#define LPC214X_PLLSTAT_PLOCK   (1<<10)     /* PLL Lock Status */

/* External Memory Controller (EMC) definitions */

#define LPC214X_BCFG0_OFFSET    0x00 /* BCFG0 Offset */
#define LPC214X_BCFG1_OFFSET    0x04 /* BCFG1 Offset */
#define LPC214X_BCFG2_OFFSET    0x08 /* BCFG2 Offset */
#define LPC214X_BCFG3_OFFSET    0x0c /* BCFG3 Offset */

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Global Function Prototypes
 ************************************************************************************/

#endif  /* __LPC214X_CHIP_H */
