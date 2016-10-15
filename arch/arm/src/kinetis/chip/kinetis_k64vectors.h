/************************************************************************************
 * arch/arm/src/kinetis/chip/kinetis_k64vectors.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

/************************************************************************************
 * Pre-processor definitions
 ************************************************************************************/
/* This file is included by kinetis_vectors.S.  It provides the macro VECTOR that
 * supplies ach K64 vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/kinetis/kinetis_k64irq.h.
 * kinetis_vectors.S will defined the VECTOR in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 */

#if defined(CONFIG_ARCH_FAMILY_K64)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#  ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve interrupt table entries for I/O interrupts. */

#    define ARMV7M_PERIPHERAL_INTERRUPTS NR_INTERRUPTS

#  else
VECTOR(kinetis_dmach0, KINETIS_IRQ_DMACH0)     /* Vector 16: DMA channel 0 transfer complete */
VECTOR(kinetis_dmach1, KINETIS_IRQ_DMACH1)     /* Vector 17: DMA channel 1 transfer complete */
VECTOR(kinetis_dmach2, KINETIS_IRQ_DMACH2)     /* Vector 18: DMA channel 2 transfer complete */
VECTOR(kinetis_dmach3, KINETIS_IRQ_DMACH3)     /* Vector 19: DMA channel 3 transfer complete */
VECTOR(kinetis_dmach4, KINETIS_IRQ_DMACH4)     /* Vector 20: DMA channel 4 transfer complete */
VECTOR(kinetis_dmach5, KINETIS_IRQ_DMACH5)     /* Vector 21: DMA channel 5 transfer complete */
VECTOR(kinetis_dmach6, KINETIS_IRQ_DMACH6)     /* Vector 22: DMA channel 6 transfer complete */
VECTOR(kinetis_dmach7, KINETIS_IRQ_DMACH7)     /* Vector 23: DMA channel 7 transfer complete */
VECTOR(kinetis_dmach8, KINETIS_IRQ_DMACH8)     /* Vector 24: DMA channel 8 transfer complete */
VECTOR(kinetis_dmach9, KINETIS_IRQ_DMACH9)     /* Vector 25: DMA channel 9 transfer complete */
VECTOR(kinetis_dmach10, KINETIS_IRQ_DMACH10)   /* Vector 26: DMA channel 10 transfer complete */
VECTOR(kinetis_dmach11, KINETIS_IRQ_DMACH11)   /* Vector 27: DMA channel 11 transfer complete */
VECTOR(kinetis_dmach12, KINETIS_IRQ_DMACH12)   /* Vector 28: DMA channel 12 transfer complete */
VECTOR(kinetis_dmach13, KINETIS_IRQ_DMACH13)   /* Vector 29: DMA channel 13 transfer complete */
VECTOR(kinetis_dmach14, KINETIS_IRQ_DMACH14)   /* Vector 30: DMA channel 14 transfer complete */
VECTOR(kinetis_dmach15, KINETIS_IRQ_DMACH15)   /* Vector 31: DMA channel 15 transfer complete */
VECTOR(kinetis_dmaerr, KINETIS_IRQ_DMAERR)     /* Vector 32: DMA error interrupt channels 0-15 */
VECTOR(kinetis_mcm, KINETIS_IRQ_MCM)           /* Vector 33: MCM Normal interrupt */
VECTOR(kinetis_flashcc, KINETIS_IRQ_FLASHCC)   /* Vector 34: Flash memory command complete */
VECTOR(kinetis_flashrc, KINETIS_IRQ_FLASHRC)   /* Vector 35: Flash memory read collision */
VECTOR(kinetis_smclvd, KINETIS_IRQ_SMCLVD)     /* Vector 36: Mode Controller low-voltage detect, low-voltage warning */
VECTOR(kinetis_llwu, KINETIS_IRQ_LLWU)         /* Vector 37: LLWU Normal Low Leakage Wakeup */
VECTOR(kinetis_wdog, KINETIS_IRQ_WDOG)         /* Vector 38: Watchdog */
VECTOR(kinetis_rngb, KINETIS_IRQ_RNGB)         /* Vector 39: Random number generator */
VECTOR(kinetis_i2c0, KINETIS_IRQ_I2C0)         /* Vector 40: I2C0 */
VECTOR(kinetis_i2c1, KINETIS_IRQ_I2C1)         /* Vector 41: I2C1 */
VECTOR(kinetis_spi0, KINETIS_IRQ_SPI0)         /* Vector 42: SPI0 all sources */
VECTOR(kinetis_spi1, KINETIS_IRQ_SPI1)         /* Vector 43: SPI1 all sources */
VECTOR(kinetis_i2s0, KINETIS_IRQ_I2S0)         /* Vector 44: Transmit */
VECTOR(kinetis_i2s1, KINETIS_IRQ_I2S1)         /* Vector 45: Transmit */
UNUSED(KINETIS_IRQ_RESVD30)                    /* Vector 46: Reserved */
VECTOR(kinetis_uart0s, KINETIS_IRQ_UART0S)     /* Vector 47: UART0 status */
VECTOR(kinetis_uart0e, KINETIS_IRQ_UART0E)     /* Vector 48: UART0 error */
VECTOR(kinetis_uart1s, KINETIS_IRQ_UART1S)     /* Vector 49: UART1 status */
VECTOR(kinetis_uart1e, KINETIS_IRQ_UART1E)     /* Vector 50: UART1 error */
VECTOR(kinetis_uart2s, KINETIS_IRQ_UART2S)     /* Vector 51: UART2 status */
VECTOR(kinetis_uart2e, KINETIS_IRQ_UART2E)     /* Vector 52: UART2 error */
VECTOR(kinetis_uart3s, KINETIS_IRQ_UART3S)     /* Vector 53: UART3 status */
VECTOR(kinetis_uart3e, KINETIS_IRQ_UART3E)     /* Vector 54: UART3 error */
VECTOR(kinetis_adc0, KINETIS_IRQ_ADC0)         /* Vector 55: ADC0 */
VECTOR(kinetis_cmp0, KINETIS_IRQ_CMP0)         /* Vector 56: CMP0 */
VECTOR(kinetis_cmp1, KINETIS_IRQ_CMP1)         /* Vector 57: CMP1 */
VECTOR(kinetis_ftm0, KINETIS_IRQ_FTM0)         /* Vector 58: FTM0 all sources */
VECTOR(kinetis_ftm1, KINETIS_IRQ_FTM1)         /* Vector 59: FTM1 all sources */
VECTOR(kinetis_ftm2, KINETIS_IRQ_FTM2)         /* Vector 60: FTM2 all sources */
VECTOR(kinetis_cmt, KINETIS_IRQ_CMT)           /* Vector 61: CMT */
VECTOR(kinetis_rtc0, KINETIS_IRQ_RTC)          /* Vector 62: RTC alarm interrupt */
VECTOR(kinetis_rtc1, KINETIS_IRQ_RTCS)         /* Vector 63: RTC seconds interrupt */
VECTOR(kinetis_pitch0, KINETIS_IRQ_PITCH0)     /* Vector 64: PIT channel 0 */
VECTOR(kinetis_pitch1, KINETIS_IRQ_PITCH1)     /* Vector 65: PIT channel 1 */
VECTOR(kinetis_pitch2, KINETIS_IRQ_PITCH2)     /* Vector 66: PIT channel 2 */
VECTOR(kinetis_pitch3, KINETIS_IRQ_PITCH3)     /* Vector 67: PIT channel 3 */
VECTOR(kinetis_pdb, KINETIS_IRQ_PDB)           /* Vector 68: PDB */
VECTOR(kinetis_usbotg, KINETIS_IRQ_USBOTG)     /* Vector 69: USB OTG */
VECTOR(kinetis_usbcd, KINETIS_IRQ_USBCD)       /* Vector 70: USB charger detect */
UNUSED(KINETIS_IRQ_RESVD55)                    /* Vector 71: Reserved */
VECTOR(kinetis_dac0, KINETIS_IRQ_DAC0)         /* Vector 72: DAC0 */
VECTOR(kinetis_mcg, KINETIS_IRQ_MCG)           /* Vector 73: MCG */
VECTOR(kinetis_lpt, KINETIS_IRQ_LPT)           /* Vector 74: Low power timer */
VECTOR(kinetis_porta, KINETIS_IRQ_PORTA)       /* Vector 75: Pin detect port A */
VECTOR(kinetis_portb, KINETIS_IRQ_PORTB)       /* Vector 76: Pin detect port B */
VECTOR(kinetis_portc, KINETIS_IRQ_PORTC)       /* Vector 77: Pin detect port C */
VECTOR(kinetis_portd, KINETIS_IRQ_PORTD)       /* Vector 78: Pin detect port D */
VECTOR(kinetis_porte, KINETIS_IRQ_PORTE)       /* Vector 79: Pin detect port E */
VECTOR(kinetis_software, KINETIS_IRQ_SWI)      /* Vector 80: Software interrupt */
VECTOR(kinetis_spi2, KINETIS_IRQ_SPI2)         /* Vector 81: SPI2 all sources */
VECTOR(kinetis_uart4s, KINETIS_IRQ_UART4S)     /* Vector 82: UART4 status */
VECTOR(kinetis_uart4e, KINETIS_IRQ_UART4E)     /* Vector 83: UART4 error */
VECTOR(kinetis_uart5s, KINETIS_IRQ_UART5S)     /* Vector 84: UART5 status */
VECTOR(kinetis_uart5e, KINETIS_IRQ_UART5E)     /* Vector 85: UART5 error */
VECTOR(kinetis_cmp2, KINETIS_IRQ_CMP2)         /* Vector 86: CMP2 */
VECTOR(kinetis_ftm3, KINETIS_IRQ_FTM3)         /* Vector 87: FTM3 all sources */
VECTOR(kinetis_dac1, KINETIS_IRQ_DAC1)         /* Vector 88: DAC1 */
VECTOR(kinetis_adc1, KINETIS_IRQ_ADC1)         /* Vector 89: ADC1 */
VECTOR(kinetis_i2c2, KINETIS_IRQ_I2C2)         /* Vector 90: I2C2 */
VECTOR(kinetis_can0mb, KINETIS_IRQ_CAN0MB)     /* Vector 91: CAN0 ORed Message buffer (0-15) */
VECTOR(kinetis_can0bo, KINETIS_IRQ_CAN0BO)     /* Vector 92: CAN0 Bus Off */
VECTOR(kinetis_can0err, KINETIS_IRQ_CAN0ERR)   /* Vector 93: CAN0 Error */
VECTOR(kinetis_can0tw, KINETIS_IRQ_CAN0TW)     /* Vector 94: CAN0 Transmit Warning */
VECTOR(kinetis_can0rw, KINETIS_IRQ_CAN0RW)     /* Vector 95: CAN0 Receive Warning */
VECTOR(kinetis_can0wu, KINETIS_IRQ_CAN0WU)     /* Vector 96: CAN0 Wake UP */
VECTOR(kinetis_sdhc, KINETIS_IRQ_SDHC)         /* Vector 97: SDHC */
VECTOR(kinetis_emactmr, KINETIS_IRQ_EMACTMR)   /* Vector 91: Ethernet MAC IEEE 1588 timer interrupt */
VECTOR(kinetis_emactx, KINETIS_IRQ_EMACTX)     /* Vector 92: Ethernet MAC transmit interrupt */
VECTOR(kinetis_emacrx, KINETIS_IRQ_EMACRX)     /* Vector 93: Ethernet MAC receive interrupt */
VECTOR(kinetis_emacmisc, KINETIS_IRQ_EMACMISC) /* Vector 94: Ethernet MAC error and misc interrupt */
#  endif /* CONFIG_ARMV7M_CMNVECTOR */
#endif /* CONFIG_ARCH_FAMILY_K64 */
