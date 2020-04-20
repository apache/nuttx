/************************************************************************************
 * arch/arm/include/kinetis/kinetis_k66irq.h
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

#ifndef __ARCH_ARM_INCLUDE_KINETIS_KINETIS_K66IRQ_H
#define __ARCH_ARM_INCLUDE_KINETIS_KINETIS_K66IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
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
 * K66 Family ************************************************************************
 *
 * The interrupt vectors  for the following parts is defined in Freescale
 * document K66P144M180SF5RMV2
 */

#define KINETIS_IRQ_DMACH0   (KINETIS_IRQ_FIRST + 0)   /* 0: DMA channel 0, 16 transfer complete */
#define KINETIS_IRQ_DMACH1   (KINETIS_IRQ_FIRST + 1)   /* 1: DMA channel 1, 17 transfer complete */
#define KINETIS_IRQ_DMACH2   (KINETIS_IRQ_FIRST + 2)   /* 2: DMA channel 2, 18 transfer complete */
#define KINETIS_IRQ_DMACH3   (KINETIS_IRQ_FIRST + 3)   /* 3: DMA channel 3, 19 transfer complete */
#define KINETIS_IRQ_DMACH4   (KINETIS_IRQ_FIRST + 4)   /* 4: DMA channel 4, 20 transfer complete */
#define KINETIS_IRQ_DMACH5   (KINETIS_IRQ_FIRST + 5)   /* 5: DMA channel 5, 21 transfer complete */
#define KINETIS_IRQ_DMACH6   (KINETIS_IRQ_FIRST + 6)   /* 6: DMA channel 6, 11 transfer complete */
#define KINETIS_IRQ_DMACH7   (KINETIS_IRQ_FIRST + 7)   /* 7: DMA channel 7, 23 transfer complete */
#define KINETIS_IRQ_DMACH8   (KINETIS_IRQ_FIRST + 8)   /* 8: DMA channel 8, 24 transfer complete */
#define KINETIS_IRQ_DMACH9   (KINETIS_IRQ_FIRST + 9)   /* 9: DMA channel 9, 25 transfer complete */
#define KINETIS_IRQ_DMACH10  (KINETIS_IRQ_FIRST + 10)  /* 10: DMA channel 10, 26 transfer complete */
#define KINETIS_IRQ_DMACH11  (KINETIS_IRQ_FIRST + 11)  /* 11: DMA channel 11, 27 transfer complete */
#define KINETIS_IRQ_DMACH12  (KINETIS_IRQ_FIRST + 12)  /* 12: DMA channel 12, 28 transfer complete */
#define KINETIS_IRQ_DMACH13  (KINETIS_IRQ_FIRST + 13)  /* 13: DMA channel 13, 29 transfer complete */
#define KINETIS_IRQ_DMACH14  (KINETIS_IRQ_FIRST + 14)  /* 14: DMA channel 14, 30 transfer complete */
#define KINETIS_IRQ_DMACH15  (KINETIS_IRQ_FIRST + 15)  /* 15: DMA channel 15, 31 transfer complete */
#define KINETIS_IRQ_DMAERR   (KINETIS_IRQ_FIRST + 16)  /* 16: DMA error interrupt channels 0-31 */
#define KINETIS_IRQ_MCM      (KINETIS_IRQ_FIRST + 17)  /* 17: MCM Normal interrupt */
#define KINETIS_IRQ_FLASHCC  (KINETIS_IRQ_FIRST + 18)  /* 18: Flash memory command complete */
#define KINETIS_IRQ_FLASHRC  (KINETIS_IRQ_FIRST + 19)  /* 19: Flash memory read collision */
#define KINETIS_IRQ_SMCLVD   (KINETIS_IRQ_FIRST + 20)  /* 20: Mode Controller low-voltage
                                                        *     detect, low-voltage warning */
#define KINETIS_IRQ_LLWU     (KINETIS_IRQ_FIRST + 21)  /* 21: LLWU Normal Low Leakage Wakeup */
#define KINETIS_IRQ_WDOG     (KINETIS_IRQ_FIRST + 22)  /* 22: Watchdog or EWM */
#define KINETIS_IRQ_RNGB     (KINETIS_IRQ_FIRST + 23)  /* 23: Random number generator */
#define KINETIS_IRQ_I2C0     (KINETIS_IRQ_FIRST + 24)  /* 24: I2C0 */
#define KINETIS_IRQ_I2C1     (KINETIS_IRQ_FIRST + 25)  /* 25: I2C1 */
#define KINETIS_IRQ_SPI0     (KINETIS_IRQ_FIRST + 26)  /* 26: SPI0 all sources */
#define KINETIS_IRQ_SPI1     (KINETIS_IRQ_FIRST + 27)  /* 27: SPI1 all sources */
#define KINETIS_IRQ_I2S0     (KINETIS_IRQ_FIRST + 28)  /* 28: 12S0 Transmit */
#define KINETIS_IRQ_I2S1     (KINETIS_IRQ_FIRST + 29)  /* 29: 12S0 Receive */
#define KINETIS_IRQ_RESVD30  (KINETIS_IRQ_FIRST + 30)  /* 30: Reserved */
#define KINETIS_IRQ_UART0S   (KINETIS_IRQ_FIRST + 31)  /* 31: UART0 status */
#define KINETIS_IRQ_UART0E   (KINETIS_IRQ_FIRST + 32)  /* 32: UART0 error */
#define KINETIS_IRQ_UART1S   (KINETIS_IRQ_FIRST + 33)  /* 33: UART1 status */
#define KINETIS_IRQ_UART1E   (KINETIS_IRQ_FIRST + 34)  /* 34: UART1 error */
#define KINETIS_IRQ_UART2S   (KINETIS_IRQ_FIRST + 35)  /* 35: UART2 status */
#define KINETIS_IRQ_UART2E   (KINETIS_IRQ_FIRST + 36)  /* 36: UART2 error */
#define KINETIS_IRQ_UART3S   (KINETIS_IRQ_FIRST + 37)  /* 37: UART3 status */
#define KINETIS_IRQ_UART3E   (KINETIS_IRQ_FIRST + 38)  /* 38: UART3 error */
#define KINETIS_IRQ_ADC0     (KINETIS_IRQ_FIRST + 39)  /* 39: ADC0 */
#define KINETIS_IRQ_CMP0     (KINETIS_IRQ_FIRST + 40)  /* 40: CMP0 */
#define KINETIS_IRQ_CMP1     (KINETIS_IRQ_FIRST + 41)  /* 41: CMP1 */
#define KINETIS_IRQ_FTM0     (KINETIS_IRQ_FIRST + 42)  /* 42: FTM0 all sources */
#define KINETIS_IRQ_FTM1     (KINETIS_IRQ_FIRST + 43)  /* 43: FTM1 all sources */
#define KINETIS_IRQ_FTM2     (KINETIS_IRQ_FIRST + 44)  /* 44: FTM2 all sources */
#define KINETIS_IRQ_CMT      (KINETIS_IRQ_FIRST + 45)  /* 45: CMT */
#define KINETIS_IRQ_RTC      (KINETIS_IRQ_FIRST + 46)  /* 46: RTC alarm interrupt */
#define KINETIS_IRQ_RTCS     (KINETIS_IRQ_FIRST + 47)  /* 47: RTC seconds interrupt */
#define KINETIS_IRQ_PITCH0   (KINETIS_IRQ_FIRST + 48)  /* 48: PIT channel 0 */
#define KINETIS_IRQ_PITCH1   (KINETIS_IRQ_FIRST + 49)  /* 49: PIT channel 1 */
#define KINETIS_IRQ_PITCH2   (KINETIS_IRQ_FIRST + 50)  /* 50: PIT channel 2 */
#define KINETIS_IRQ_PITCH3   (KINETIS_IRQ_FIRST + 51)  /* 51: PIT channel 3 */
#define KINETIS_IRQ_PDB      (KINETIS_IRQ_FIRST + 52)  /* 52: PDB */
#define KINETIS_IRQ_USBOTG   (KINETIS_IRQ_FIRST + 53)  /* 53: USB OTG */
#define KINETIS_IRQ_USBCD    (KINETIS_IRQ_FIRST + 54)  /* 54: USB charger detect */
#define KINETIS_IRQ_RESVD55  (KINETIS_IRQ_FIRST + 55)  /* 55: Reserved */
#define KINETIS_IRQ_DAC0     (KINETIS_IRQ_FIRST + 56)  /* 56: DAC0 */
#define KINETIS_IRQ_MCG      (KINETIS_IRQ_FIRST + 57)  /* 57: MCG */
#define KINETIS_IRQ_LPT      (KINETIS_IRQ_FIRST + 58)  /* 58: Low power timer */
#define KINETIS_IRQ_PORTA    (KINETIS_IRQ_FIRST + 59)  /* 59: Pin detect port A */
#define KINETIS_IRQ_PORTB    (KINETIS_IRQ_FIRST + 60)  /* 60: Pin detect port B */
#define KINETIS_IRQ_PORTC    (KINETIS_IRQ_FIRST + 61)  /* 61: Pin detect port C */
#define KINETIS_IRQ_PORTD    (KINETIS_IRQ_FIRST + 62)  /* 62: Pin detect port D */
#define KINETIS_IRQ_PORTE    (KINETIS_IRQ_FIRST + 63)  /* 63: Pin detect port E */
#define KINETIS_IRQ_SWI      (KINETIS_IRQ_FIRST + 64)  /* 64: Software interrupt */
#define KINETIS_IRQ_SPI2     (KINETIS_IRQ_FIRST + 65)  /* 65: SPI2 all sources */
#define KINETIS_IRQ_UART4S   (KINETIS_IRQ_FIRST + 66)  /* 66: UART4 status */
#define KINETIS_IRQ_UART4E   (KINETIS_IRQ_FIRST + 67)  /* 67: UART4 error */
#define KINETIS_IRQ_RESVD68  (KINETIS_IRQ_FIRST + 68)  /* 68: Reserved */
#define KINETIS_IRQ_RESVD69  (KINETIS_IRQ_FIRST + 69)  /* 69: Reserved */
#define KINETIS_IRQ_CMP2     (KINETIS_IRQ_FIRST + 70)  /* 70: CMP2 */
#define KINETIS_IRQ_FTM3     (KINETIS_IRQ_FIRST + 71)  /* 71: FTM3 all sources */
#define KINETIS_IRQ_DAC1     (KINETIS_IRQ_FIRST + 72)  /* 72: DAC1 */
#define KINETIS_IRQ_ADC1     (KINETIS_IRQ_FIRST + 73)  /* 73: ADC1 */
#define KINETIS_IRQ_I2C2     (KINETIS_IRQ_FIRST + 74)  /* 74: I2C2 */
#define KINETIS_IRQ_CAN0MB   (KINETIS_IRQ_FIRST + 75)  /* 75: CAN0 OR'ed Message buffer (0-15) */
#define KINETIS_IRQ_CAN0BO   (KINETIS_IRQ_FIRST + 76)  /* 76: CAN0 Bus Off */
#define KINETIS_IRQ_CAN0ERR  (KINETIS_IRQ_FIRST + 77)  /* 77: CAN0 Error */
#define KINETIS_IRQ_CAN0TW   (KINETIS_IRQ_FIRST + 78)  /* 78: CAN0 Transmit Warning */
#define KINETIS_IRQ_CAN0RW   (KINETIS_IRQ_FIRST + 79)  /* 79: CAN0 Receive Warning */
#define KINETIS_IRQ_CAN0WU   (KINETIS_IRQ_FIRST + 80)  /* 80: CAN0 Wake UP */
#define KINETIS_IRQ_SDHC     (KINETIS_IRQ_FIRST + 81)  /* 81: SDHC */
#define KINETIS_IRQ_EMACTMR  (KINETIS_IRQ_FIRST + 82)  /* 82: Ethernet MAC IEEE 1588 timer interrupt */
#define KINETIS_IRQ_EMACTX   (KINETIS_IRQ_FIRST + 83)  /* 83: Ethernet MAC transmit interrupt */
#define KINETIS_IRQ_EMACRX   (KINETIS_IRQ_FIRST + 84)  /* 84: Ethernet MAC receive interrupt */
#define KINETIS_IRQ_EMACMISC (KINETIS_IRQ_FIRST + 85)  /* 85: Ethernet MAC error and misc interrupt */
#define KINETIS_IRQ_LPUART0  (KINETIS_IRQ_FIRST + 86)  /* 86: LPUART0 Status and error */
#define KINETIS_IRQ_TSI0     (KINETIS_IRQ_FIRST + 87)  /* 87: TSI0 */
#define KINETIS_IRQ_TPM1     (KINETIS_IRQ_FIRST + 88)  /* 88: TPM1 */
#define KINETIS_IRQ_TPM2     (KINETIS_IRQ_FIRST + 89)  /* 89: TPM2 */
#define KINETIS_IRQ_USBHSDCD (KINETIS_IRQ_FIRST + 90)  /* 90: shared by USBHS DCD & USBHS Phy modules */
#define KINETIS_IRQ_I2C3     (KINETIS_IRQ_FIRST + 91)  /* 91: I2C3 */
#define KINETIS_IRQ_CMP3     (KINETIS_IRQ_FIRST + 92)  /* 92: CMP3 */
#define KINETIS_IRQ_USBHSOTG (KINETIS_IRQ_FIRST + 93)  /* 93: USBHS OTG*/
#define KINETIS_IRQ_CAN1MB   (KINETIS_IRQ_FIRST + 94)  /* 94: CAN1 OR'ed Message buffer (0-15) */
#define KINETIS_IRQ_CAN1BO   (KINETIS_IRQ_FIRST + 95)  /* 95: CAN1 Bus Off */
#define KINETIS_IRQ_CAN1ERR  (KINETIS_IRQ_FIRST + 96)  /* 96: CAN1 Error */
#define KINETIS_IRQ_CAN1TW   (KINETIS_IRQ_FIRST + 97)  /* 97: CAN1 Transmit Warning */
#define KINETIS_IRQ_CAN1RW   (KINETIS_IRQ_FIRST + 98)  /* 98: CAN1 Receive Warning */
#define KINETIS_IRQ_CAN1WU   (KINETIS_IRQ_FIRST + 99)  /* 99: CAN1 Wake UP */


#define KINETIS_IRQ_NEXTINTS 100                       /* 100 Non core IRQs*/
#define KINETIS_IRQ_NVECTORS (KINETIS_IRQ_FIRST + KINETIS_IRQ_NEXTINTS) /* 116 vectors */

/* EXTI interrupts (Do not use IRQ numbers) */

#define NR_IRQS              KINETIS_IRQ_NVECTORS

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
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

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_KINETIS_KINETIS_K66IRQ_H */
