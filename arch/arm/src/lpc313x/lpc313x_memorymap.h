/************************************************************************************
 * arch/arm/src/lpc313x/chip.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC313X_MEMORYMAP_H
#define __ARCH_ARM_SRC_LPC313X_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Physical (unmapped) memory map */

#define LPC313X_SHADOWSPACE_PSECTION  0x00000000 /* 0x00000000-0x00000fff: Shadow Area 4Kb */
                                                 /* 0x00001000-0xff027fff: Reserved */
#define LPC313X_INTSRAM0_PSECTION     0x11028000 /* 0x11028000-0x1103ffff: Internal SRAM 0 96Kb */
#define LPC313X_INTSRAM1_PSECTION     0x11040000 /* 0x11040000-0x11057fff: Internal SRAM 1 96Kb */
                                                 /* 0x11058000-11ffffffff: Reserved */
#define LPC313X_INTSROM0_PSECTION     0x12000000 /* 0x12000000-0x1201ffff: Internal SROM 0 128Kb */
                                                 /* 0x12020000-0x12ffffff: Reserved */
#define LPC313X_APB0_PSECTION         0x13000000 /* 0x13000000-0x13007fff: APB0 32Kb */
#define LPC313X_APB1_PSECTION         0x13008000 /* 0x13008000-0x1300bfff: APB1 16Kb */
                                                 /* 0x1300c000-0x14ffffff: Reserved */
#define LPC313X_APB2_PSECTION         0x15000000 /* 0x15000000-0x15003fff: APB2 16Kb */
#define LPC313X_APB3_PSECTION         0x16000000 /* 0x16000000-0x160003ff: APB3 1Kb */
#define LPC313X_APB4_PSECTION         0x17000000 /* 0x17000000-0x17000fff: APB4 4Kb */
#define LPC313X_MPMC_PSECTION         0x17008000 /* 0x17008000-0x17008fff: MPMC cfg 4Kb */
                                                 /* 0x17009000-0x17ffffff: Reserved */
#define LPC313X_MCI_PSECTION          0x18000000 /* 0x18000000 0x180003ff: MCI/SD/SDIO 1Kb */
                                                 /* 0x18000900-0x18ffffff: Reserved */
#define LPC313X_USBOTG_PSECTION       0x19000000 /* 0x19000000-0x19000fff: USB OTG 4Kb */
                                                 /* 0x19001000-0x1fffffff: Reserved */
#define LPC313X_EXTSRAM0_PSECTION     0x20000000 /* 0x20000000-0x2001ffff: External SRAM 0 64-128Kb */
#define LPC313X_EXTSRAM1_PSECTION     0x20020000 /* 0x20020000-0x2003ffff: External SRAM 1 64-128Kb */
#define LPC313X_EXTSDRAM0_PSECTION    0x30000000 /* 0x30000000-0x37ffffff: External SDRAM 0 128Mb */
                                                 /* 0x40000000-0x5fffffff: Reserved */
#define LPC313X_INTC_PSECTION         0x60000000 /* 0x60000000-0x60000fff: Interrupt controller 4Kb */
                                                 /* 0x60001000-0x6fffffff: Reserved */
#define LPC313X_NAND_PSECTION         0x70000000 /* 0x70000000-0x700007ff: NANDFLASH Ctrl 2Kb */
                                                 /* 0x70000800-0xffffffff: Reserved */
/* APB0-4 Domain Offsets */

#define LPC313X_APB0_EVNTRTR_OFFSET   0x00000000 /* Event Router */
#define LPC313X_APB0_ADC_OFFSET       0x00002000 /* ADC 10-bit */
#define LPC313X_APB0_WDT_OFFSET       0x00002400 /* WDT */
#define LPC313X_APB0_SYSCONFIG_OFFSET 0x00002800 /* SYSCONFIG register */
#define LPC313X_APB0_IOCONFIG_OFFSET  0x00003000 /* IOCONFIG */
#define LPC313X_APB0_GCU_OFFSET       0x00004000 /* GCU */
                                   /* 0x00005000    Reserved */
#define LPC313X_APB0_RNG_OFFSET       0x00006000 /* RNG */

#define LPC313X_APB1_TIMER0_OFFSET    0x00000000 /* TIMER0 */
#define LPC313X_APB1_TIMER1_OFFSET    0x00000400 /* TIMER1 */
#define LPC313X_APB1_TIMER2_OFFSET    0x00000800 /* TIMER2 */
#define LPC313X_APB1_TIMER3_OFFSET    0x00000c00 /* TIMER3 */
#define LPC313X_APB1_PWM_OFFSET       0x00001000 /* PWM */
#define LPC313X_APB1_I2C0_OFFSET      0x00002000 /* I2C0 */
#define LPC313X_APB1_I2C1_OFFSET      0x00002400 /* I2C1 */

#define LPC313X_APB2_PCM_OFFSET       0x00000000 /* PCM */
#define LPC313X_APB2_LCD_OFFSET       0x00000400 /* LCD */
                                   /* 0x00000800    Reserved */
#define LPC313X_APB2_UART_OFFSET      0x00001000 /* UART */
#define LPC313X_APB2_SPI_OFFSET       0x00002000 /* SPI */
                                   /* 0x00003000    Reserved */

#define LPC313X_APB3_I2SCONFIG_OFFSET 0x00000000 /* I2S System Configuration */
#define LPC313X_APB3_I2STX0_OFFSET    0x00000080 /* I2S TX0 */
#define LPC313X_APB3_I2STX1_OFFSET    0x00000100 /* I2S TX1 */
#define LPC313X_APB3_I2SRX0_OFFSET    0x00000180 /* I2S RX0 */
#define LPC313X_APB3_I2SRX1_OFFSET    0x00000200 /* I2S RX1 */
                                   /* 0x00000280    Reserved */

#define LPC313X_APB4_DMA_OFFSET       0x00000000 /* DMA */
#define LPC313X_APB4_NAND_OFFSET      0x00000800 /* NAND FLASH Controller */
                                   /* 0x00001000    Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC313X_MEMORYMAP_H */
