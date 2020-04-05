/*************************************************************************************
 * arch/arm/include/kinetis/kinetis_k40irq.h
 *
 *   Copyright (C) 2011, 2015-2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_KINETIS_KINETIS_40KIRQ_H
#define __ARCH_ARM_INCLUDE_KINETIS_KINETIS_40KIRQ_H

/*************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/*************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be found
 * in the file nuttx/arch/arm/include/kinets/irq.h which includes this file
 *
 * External interrupts (vectors >= 16)
 *
 * K40 Family ************************************************************************
 *
 * The interrupt vectors  for the following parts is defined in Freescale
 * document K40P144M100SF2RM
 */

#define KINETIS_IRQ_DMACH0   (KINETIS_IRQ_FIRST + 0)   /* 0: DMA channel 0 transfer complete */
#define KINETIS_IRQ_DMACH1   (KINETIS_IRQ_FIRST + 1)   /* 1: DMA channel 1 transfer complete */
#define KINETIS_IRQ_DMACH2   (KINETIS_IRQ_FIRST + 2)   /* 2: DMA channel 2 transfer complete */
#define KINETIS_IRQ_DMACH3   (KINETIS_IRQ_FIRST + 3)   /* 3: DMA channel 3 transfer complete */
#define KINETIS_IRQ_DMACH4   (KINETIS_IRQ_FIRST + 4)   /* 4: DMA channel 4 transfer complete */
#define KINETIS_IRQ_DMACH5   (KINETIS_IRQ_FIRST + 5)   /* 5: DMA channel 5 transfer complete */
#define KINETIS_IRQ_DMACH6   (KINETIS_IRQ_FIRST + 6)   /* 6: DMA channel 6 transfer complete */
#define KINETIS_IRQ_DMACH7   (KINETIS_IRQ_FIRST + 7)   /* 7: DMA channel 7 transfer complete */
#define KINETIS_IRQ_DMACH8   (KINETIS_IRQ_FIRST + 8)   /* 8: DMA channel 8 transfer complete */
#define KINETIS_IRQ_DMACH9   (KINETIS_IRQ_FIRST + 9)   /* 9: DMA channel 9 transfer complete */
#define KINETIS_IRQ_DMACH10  (KINETIS_IRQ_FIRST + 10)  /* 10: DMA channel 10 transfer complete */
#define KINETIS_IRQ_DMACH11  (KINETIS_IRQ_FIRST + 11)  /* 11: DMA channel 11 transfer complete */
#define KINETIS_IRQ_DMACH12  (KINETIS_IRQ_FIRST + 12)  /* 12: DMA channel 12 transfer complete */
#define KINETIS_IRQ_DMACH13  (KINETIS_IRQ_FIRST + 13)  /* 13: DMA channel 13 transfer complete */
#define KINETIS_IRQ_DMACH14  (KINETIS_IRQ_FIRST + 14)  /* 14: DMA channel 14 transfer complete */
#define KINETIS_IRQ_DMACH15  (KINETIS_IRQ_FIRST + 15)  /* 15: DMA channel 15 transfer complete */
#define KINETIS_IRQ_DMAERR   (KINETIS_IRQ_FIRST + 16)  /* 16: DMA error interrupt channels 0-15 */
#define KINETIS_IRQ_MCM      (KINETIS_IRQ_FIRST + 17)  /* 17: MCM Normal interrupt */
#define KINETIS_IRQ_FLASHCC  (KINETIS_IRQ_FIRST + 18)  /* 18: Flash memory command complete */
#define KINETIS_IRQ_FLASHRC  (KINETIS_IRQ_FIRST + 19)  /* 19: Flash memory read collision */
#define KINETIS_IRQ_SMCLVD   (KINETIS_IRQ_FIRST + 20)  /* 20: Mode Controller low-voltage
                                                        *     detect, low-voltage warning */
#define KINETIS_IRQ_LLWU     (KINETIS_IRQ_FIRST + 21)  /* 21: LLWU Normal Low Leakage Wakeup */
#define KINETIS_IRQ_WDOG     (KINETIS_IRQ_FIRST + 22)  /* 22: Watchdog */
#define KINETIS_IRQ_RESVD23  (KINETIS_IRQ_FIRST + 23)  /* 23: Reserved */
#define KINETIS_IRQ_I2C0     (KINETIS_IRQ_FIRST + 24)  /* 24: I2C0 */
#define KINETIS_IRQ_I2C1     (KINETIS_IRQ_FIRST + 25)  /* 25: I2C1 */
#define KINETIS_IRQ_SPI0     (KINETIS_IRQ_FIRST + 26)  /* 26: SPI0 all sources */
#define KINETIS_IRQ_SPI1     (KINETIS_IRQ_FIRST + 27)  /* 27: SPI1 all sources */
#define KINETIS_IRQ_SPI2     (KINETIS_IRQ_FIRST + 28)  /* 28: SPI2 all sources */
#define KINETIS_IRQ_CAN0MB   (KINETIS_IRQ_FIRST + 29)  /* 29: CAN0 OR'ed Message buffer (0-15) */
#define KINETIS_IRQ_CAN0BO   (KINETIS_IRQ_FIRST + 30)  /* 30: CAN0 Bus Off */
#define KINETIS_IRQ_CAN0ERR  (KINETIS_IRQ_FIRST + 31)  /* 31: CAN0 Error */
#define KINETIS_IRQ_CAN0TW   (KINETIS_IRQ_FIRST + 32)  /* 32: CAN0 Transmit Warning */
#define KINETIS_IRQ_CAN0RW   (KINETIS_IRQ_FIRST + 33)  /* 33: CAN0 Receive Warning */
#define KINETIS_IRQ_CAN0WU   (KINETIS_IRQ_FIRST + 34)  /* 34: CAN0 Wake UP */
#define KINETIS_IRQ_RESVD35  (KINETIS_IRQ_FIRST + 35)  /* 35: Reserved */
#define KINETIS_IRQ_RESVD36  (KINETIS_IRQ_FIRST + 36)  /* 36: Reserved */
#define KINETIS_IRQ_CAN1MB   (KINETIS_IRQ_FIRST + 37)  /* 37: CAN1 OR'ed Message buffer (0-15) */
#define KINETIS_IRQ_CAN1BO   (KINETIS_IRQ_FIRST + 38)  /* 38: CAN1 Bus Off */
#define KINETIS_IRQ_CAN1ERR  (KINETIS_IRQ_FIRST + 39)  /* 39: CAN1 Error */
#define KINETIS_IRQ_CAN1TW   (KINETIS_IRQ_FIRST + 40)  /* 40: CAN1 Transmit Warning */
#define KINETIS_IRQ_CAN1RW   (KINETIS_IRQ_FIRST + 41)  /* 41: CAN1 Receive Warning */
#define KINETIS_IRQ_CAN1WU   (KINETIS_IRQ_FIRST + 42)  /*  42: CAN1 Wake UP */
#define KINETIS_IRQ_RESVD43  (KINETIS_IRQ_FIRST + 43)  /* 43: Reserved */
#define KINETIS_IRQ_RESVD44  (KINETIS_IRQ_FIRST + 44)  /* 44: Reserved */
#define KINETIS_IRQ_UART0S   (KINETIS_IRQ_FIRST + 45)  /* 45: UART0 status */
#define KINETIS_IRQ_UART0E   (KINETIS_IRQ_FIRST + 46)  /* 46: UART0 error */
#define KINETIS_IRQ_UART1S   (KINETIS_IRQ_FIRST + 47)  /* 47: UART1 status */
#define KINETIS_IRQ_UART1E   (KINETIS_IRQ_FIRST + 48)  /* 48: UART1 error */
#define KINETIS_IRQ_UART2S   (KINETIS_IRQ_FIRST + 49)  /* 49: UART2 status */
#define KINETIS_IRQ_UART2E   (KINETIS_IRQ_FIRST + 50)  /* 50: UART2 error */
#define KINETIS_IRQ_UART3S   (KINETIS_IRQ_FIRST + 51)  /* 51: UART3 status */
#define KINETIS_IRQ_UART3E   (KINETIS_IRQ_FIRST + 52)  /* 52: UART3 error */
#define KINETIS_IRQ_UART4S   (KINETIS_IRQ_FIRST + 53)  /* 53: UART4 status */
#define KINETIS_IRQ_UART4E   (KINETIS_IRQ_FIRST + 54)  /* 54: UART4 error */
#define KINETIS_IRQ_UART5S   (KINETIS_IRQ_FIRST + 55)  /* 55: UART5 status */
#define KINETIS_IRQ_UART5E   (KINETIS_IRQ_FIRST + 56)  /* 56: UART5 error */
#define KINETIS_IRQ_ADC0     (KINETIS_IRQ_FIRST + 57)  /* 57: ADC0 */
#define KINETIS_IRQ_ADC1     (KINETIS_IRQ_FIRST + 58)  /* 58: ADC1 */
#define KINETIS_IRQ_CMP0     (KINETIS_IRQ_FIRST + 59)  /* 59: CMP0 */
#define KINETIS_IRQ_CMP1     (KINETIS_IRQ_FIRST + 60)  /* 60: CMP1 */
#define KINETIS_IRQ_CMP2     (KINETIS_IRQ_FIRST + 61)  /* 61: CMP2 */
#define KINETIS_IRQ_FTM0     (KINETIS_IRQ_FIRST + 62)  /* 62: FTM0 all sources */
#define KINETIS_IRQ_FTM1     (KINETIS_IRQ_FIRST + 63)  /* 63: FTM1 all sources */
#define KINETIS_IRQ_FTM2     (KINETIS_IRQ_FIRST + 64)  /* 64: FTM2 all sources */
#define KINETIS_IRQ_CMT      (KINETIS_IRQ_FIRST + 65)  /* 65: CMT */
#define KINETIS_IRQ_RTC      (KINETIS_IRQ_FIRST + 66)  /* 66: RTC alarm interrupt */
#define KINETIS_IRQ_RESVD67  (KINETIS_IRQ_FIRST + 67)  /* 67: Reserved */
#define KINETIS_IRQ_PITCH0   (KINETIS_IRQ_FIRST + 68)  /* 68: PIT channel 0 */
#define KINETIS_IRQ_PITCH1   (KINETIS_IRQ_FIRST + 69)  /* 69: PIT channel 1 */
#define KINETIS_IRQ_PITCH2   (KINETIS_IRQ_FIRST + 70)  /* 70: PIT channel 2 */
#define KINETIS_IRQ_PITCH3   (KINETIS_IRQ_FIRST + 71)  /* 71: PIT channel 3 */
#define KINETIS_IRQ_PDB      (KINETIS_IRQ_FIRST + 72)  /* 72: PDB */
#define KINETIS_IRQ_USBOTG   (KINETIS_IRQ_FIRST + 73)  /* 73: USB OTG */
#define KINETIS_IRQ_USBCD    (KINETIS_IRQ_FIRST + 74)  /* 74: USB charger detect */
#define KINETIS_IRQ_RESVD75  (KINETIS_IRQ_FIRST + 75)  /* 75: Reserved */
#define KINETIS_IRQ_RESVD76  (KINETIS_IRQ_FIRST + 76)  /* 76: Reserved */
#define KINETIS_IRQ_RESVD77  (KINETIS_IRQ_FIRST + 77)  /* 77: Reserved */
#define KINETIS_IRQ_RESVD78  (KINETIS_IRQ_FIRST + 78)  /* 78: Reserved */
#define KINETIS_IRQ_I2S0     (KINETIS_IRQ_FIRST + 79)  /* 79: I2S0 */
#define KINETIS_IRQ_SDHC     (KINETIS_IRQ_FIRST + 80)  /* 80: SDHC */
#define KINETIS_IRQ_DAC0     (KINETIS_IRQ_FIRST + 81)  /* 81: DAC0 */
#define KINETIS_IRQ_DAC1     (KINETIS_IRQ_FIRST + 82)  /* 82: DAC1 */
#define KINETIS_IRQ_TSI      (KINETIS_IRQ_FIRST + 83)  /* 83: TSI all sources */
#define KINETIS_IRQ_MCG      (KINETIS_IRQ_FIRST + 84)  /* 84: MCG */
#define KINETIS_IRQ_LPT      (KINETIS_IRQ_FIRST + 85)  /* 85: Low power timer */
#define KINETIS_IRQ_SLCD     (KINETIS_IRQ_FIRST + 86)  /* 86: Segment LCD all sources */
#define KINETIS_IRQ_PORTA    (KINETIS_IRQ_FIRST + 87)  /* 87: Pin detect port A */
#define KINETIS_IRQ_PORTB    (KINETIS_IRQ_FIRST + 88)  /* 88: Pin detect port B */
#define KINETIS_IRQ_PORTC    (KINETIS_IRQ_FIRST + 89)  /* 89: Pin detect port C */
#define KINETIS_IRQ_PORTD    (KINETIS_IRQ_FIRST + 90)  /* 90: Pin detect port D */
#define KINETIS_IRQ_PORTE    (KINETIS_IRQ_FIRST + 91)  /* 91: Pin detect port E */
#define KINETIS_IRQ_RESVD92  (KINETIS_IRQ_FIRST + 92)  /* 92: Reserved */
#define KINETIS_IRQ_RESVD93  (KINETIS_IRQ_FIRST + 93)  /* 93: Reserved */
#define KINETIS_IRQ_SWI      (KINETIS_IRQ_FIRST + 94)  /* 94: Software interrupt */

#define KINETIS_IRQ_NEXTINTS 95                        /* 95 Non core IRQs */
#define KINETIS_IRQ_NVECTORS (KINETIS_IRQ_FIRST + KINETIS_IRQ_NEXTINTS) /* 111 vectors */

/* EXTI interrupts (Do not use IRQ numbers) */

#define NR_IRQS              KINETIS_IRQ_NVECTORS

/*************************************************************************************
 * Public Types
 ************************************************************************************/

/*************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/*************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_KINETIS_KINETIS_40KIRQ_H */
