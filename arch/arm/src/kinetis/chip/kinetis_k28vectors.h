/************************************************************************************
 * arch/arm/src/kinetis/chip/kinetis_k28vectors.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 * supplies ach K28 vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/kinetis/kinetis_k28irq.h.
 * kinetis_vectors.S will defined the VECTOR in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 */

#if defined(CONFIG_ARCH_FAMILY_K28)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#  ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve interrupt table entries for I/O interrupts. */

#    define ARMV7M_PERIPHERAL_INTERRUPTS NR_INTERRUPTS

#  else

VECTOR(kinetis_dmach0,  KINETIS_IRQ_DMACH0)   /*  0: DMA channel 0, 16 transfer complete */
VECTOR(kinetis_dmach1,  KINETIS_IRQ_DMACH1)   /*  1: DMA channel 1, 17 transfer complete */
VECTOR(kinetis_dmach2,  KINETIS_IRQ_DMACH2)   /*  2: DMA channel 2, 18 transfer complete */
VECTOR(kinetis_dmach3,  KINETIS_IRQ_DMACH3)   /*  3: DMA channel 3, 19 transfer complete */
VECTOR(kinetis_dmach4,  KINETIS_IRQ_DMACH4)   /*  4: DMA channel 4, 20 transfer complete */
VECTOR(kinetis_dmach5,  KINETIS_IRQ_DMACH5)   /*  5: DMA channel 5, 21 transfer complete */
VECTOR(kinetis_dmach6,  KINETIS_IRQ_DMACH6)   /*  6: DMA channel 6, 11 transfer complete */
VECTOR(kinetis_dmach7,  KINETIS_IRQ_DMACH7)   /*  7: DMA channel 7, 23 transfer complete */
VECTOR(kinetis_dmach8,  KINETIS_IRQ_DMACH8)   /*  8: DMA channel 8, 24 transfer complete */
VECTOR(kinetis_dmach9,  KINETIS_IRQ_DMACH9)   /*  9: DMA channel 9, 25 transfer complete */

VECTOR(kinetis_dmach10, KINETIS_IRQ_DMACH10)  /* 10: DMA channel 10, 26 transfer complete */
VECTOR(kinetis_dmach11, KINETIS_IRQ_DMACH11)  /* 11: DMA channel 11, 27 transfer complete */
VECTOR(kinetis_dmach12, KINETIS_IRQ_DMACH12)  /* 12: DMA channel 12, 28 transfer complete */
VECTOR(kinetis_dmach13, KINETIS_IRQ_DMACH13)  /* 13: DMA channel 13, 29 transfer complete */
VECTOR(kinetis_dmach14, KINETIS_IRQ_DMACH14)  /* 14: DMA channel 14, 30 transfer complete */
VECTOR(kinetis_dmach15, KINETIS_IRQ_DMACH15)  /* 15: DMA channel 15, 31 transfer complete */
VECTOR(kinetis_dmaerr,  KINETIS_IRQ_DMAERR)   /* 16: DMA error interrupt channels 0-31 */
VECTOR(kinetis_mcm,     KINETIS_IRQ_MCM)      /* 17: MCM or RDC interrupt */
VECTOR(kinetis_flashcc, KINETIS_IRQ_FLASHCC)  /* 18: Flash memory command complete */
VECTOR(kinetis_flashrc, KINETIS_IRQ_FLASHRC)  /* 19: Flash memory read collision */

VECTOR(kinetis_smclvd,  KINETIS_IRQ_SMCLVD)   /* 20: Mode Controller low-voltage */
VECTOR(kinetis_llwu,    KINETIS_IRQ_LLWU)     /* 21: LLWU Normal Low Leakage Wakeup */
VECTOR(kinetis_wdog,    KINETIS_IRQ_WDOG)     /* 22: Watchdog or EWM */
VECTOR(kinetis_rngb,    KINETIS_IRQ_RNGB)     /* 23: True random number generator (TRNG) */
VECTOR(kinetis_i2c0,    KINETIS_IRQ_I2C0)     /* 24: I2C0 */
VECTOR(kinetis_i2c1,    KINETIS_IRQ_I2C1)     /* 25: I2C1 */
VECTOR(kinetis_spi0,    KINETIS_IRQ_SPI0)     /* 26: SPI0 all sources */
VECTOR(kinetis_spi1,    KINETIS_IRQ_SPI1)     /* 27: SPI1 all sources */
VECTOR(kinetis_i2s0,    KINETIS_IRQ_I2S0)     /* 28: 12S0 Transmit */
VECTOR(kinetis_i2s1,    KINETIS_IRQ_I2S1)     /* 29: 12S0 Receive */

VECTOR(kinetis_lpuart0, KINETIS_IRQ_LPUART0)  /* 30: LPUART0 Status and error */
VECTOR(kinetis_lpuart1, KINETIS_IRQ_LPUART1)  /* 31: LPUART1 Status and error */
VECTOR(kinetis_lpuart2, KINETIS_IRQ_LPUART2)  /* 32: LPUART2 Status and error */
VECTOR(kinetis_lpuart3, KINETIS_IRQ_LPUART3)  /* 33: LPUART3 Status and error */
VECTOR(kinetis_lpuart4, KINETIS_IRQ_LPUART4)  /* 34: LPUART4 Status and error */
UNUSED(KINETIS_IRQ_RESVD35)
UNUSED(KINETIS_IRQ_RESVD36)
UNUSED(KINETIS_IRQ_RESVD37)
UNUSED(KINETIS_IRQ_RESVD38)
VECTOR(kinetis_adc0,    KINETIS_IRQ_ADC0)     /* 39: ADC0 */

VECTOR(kinetis_cmp0,    KINETIS_IRQ_CMP0)     /* 40: CMP0 */
VECTOR(kinetis_cmp1,    KINETIS_IRQ_CMP1)     /* 41: CMP1 */
VECTOR(kinetis_ftm0,    KINETIS_IRQ_FTM0)     /* 42: FTM0 all sources */
VECTOR(kinetis_ftm1,    KINETIS_IRQ_FTM1)     /* 43: FTM1 all sources */
VECTOR(kinetis_ftm2,    KINETIS_IRQ_FTM2)     /* 44: FTM2 all sources */
VECTOR(kinetis_cmt,     KINETIS_IRQ_CMT)      /* 45: CMT */
VECTOR(kinetis_rtc,     KINETIS_IRQ_RTC)      /* 46: RTC alarm interrupt */
VECTOR(kinetis_rtcs,    KINETIS_IRQ_RTCS)     /* 47: RTC seconds interrupt */
VECTOR(kinetis_pitch0,  KINETIS_IRQ_PITCH0)   /* 48: PIT channel 0 */
VECTOR(kinetis_pitch1,  KINETIS_IRQ_PITCH1)   /* 49: PIT channel 1 */

VECTOR(kinetis_pitch2,  KINETIS_IRQ_PITCH2)   /* 50: PIT channel 2 */
VECTOR(kinetis_pitch3,  KINETIS_IRQ_PITCH3)   /* 51: PIT channel 3 */
VECTOR(kinetis_pdb,     KINETIS_IRQ_PDB)      /* 52: PDB */
VECTOR(kinetis_usbotg,  KINETIS_IRQ_USBOTG)   /* 53: USB OTG */
VECTOR(kinetis_usbcd,   KINETIS_IRQ_USBCD)    /* 54: USB charger detect */
UNUSED(KINETIS_IRQ_RESVD55)
VECTOR(kinetis_dac0,    KINETIS_IRQ_DAC0)     /* 56: DAC0 */
VECTOR(kinetis_mcg,     KINETIS_IRQ_MCG)      /* 57: MCG */
VECTOR(kinetis_lpt,     KINETIS_IRQ_LPT)      /* 58: Low power timer LPTMR0 and LPTMR1 */
VECTOR(kinetis_porta,   KINETIS_IRQ_PORTA)    /* 59: Pin detect port A */

VECTOR(kinetis_portb,   KINETIS_IRQ_PORTB)    /* 60: Pin detect port B */
VECTOR(kinetis_portc,   KINETIS_IRQ_PORTC)    /* 61: Pin detect port C */
VECTOR(kinetis_portd,   KINETIS_IRQ_PORTD)    /* 62: Pin detect port D */
VECTOR(kinetis_porte,   KINETIS_IRQ_PORTE)    /* 63: Pin detect port E */
VECTOR(kinetis_swi,     KINETIS_IRQ_SWI)      /* 64: Software interrupt */
VECTOR(kinetis_spi2,    KINETIS_IRQ_SPI2)     /* 65: SPI2 all sources */
VECTOR(kinetis_spi3,    KINETIS_IRQ_SPI3)     /* 66: SPI3 all sources */
UNUSED(KINETIS_IRQ_RESVD67)
VECTOR(kinetis_i2s1tx,  KINETIS_IRQ_I2S1TX)   /* 68: I2S1 Transmit */
VECTOR(kinetis_i2s1rx,  KINETIS_IRQ_I2S1RX)   /* 69: I2S1 Receive */

VECTOR(kinetis_flexio,  KINETIS_IRQ_FLEXIO)   /* 70: FlexIO */
VECTOR(kinetis_ftm3,    KINETIS_IRQ_FTM3)     /* 71: FTM3 all sources */
UNUSED(KINETIS_IRQ_RESVD72)
UNUSED(KINETIS_IRQ_RESVD73)
VECTOR(kinetis_i2c2,    KINETIS_IRQ_I2C2)     /* 74: I2C2 */
UNUSED(KINETIS_IRQ_RESVD75)
UNUSED(KINETIS_IRQ_RESVD76)
UNUSED(KINETIS_IRQ_RESVD77)
UNUSED(KINETIS_IRQ_RESVD78)
UNUSED(KINETIS_IRQ_RESVD79)

UNUSED(KINETIS_IRQ_RESVD80)
VECTOR(kinetis_sdhc,    KINETIS_IRQ_SDHC)     /* 81: SDHC */
UNUSED(KINETIS_IRQ_RESVD82)
UNUSED(KINETIS_IRQ_RESVD83)
UNUSED(KINETIS_IRQ_RESVD84)
UNUSED(KINETIS_IRQ_RESVD85)
UNUSED(KINETIS_IRQ_RESVD86)
UNUSED(KINETIS_IRQ_RESVD87)
VECTOR(kinetis_tpm1,    KINETIS_IRQ_TPM1)     /* 88: TPM1 */
VECTOR(kinetis_tpm2,    KINETIS_IRQ_TPM2)     /* 89: TPM2 */

VECTOR(kinetis_usbhsdcd, KINETIS_IRQ_USBHSDCD) /* 90: USBHS DCD or USBHS Phy modules */
VECTOR(kinetis_i2c3,    KINETIS_IRQ_I2C3)     /* 91: I2C3 */
UNUSED(KINETIS_IRQ_RESVD92)
VECTOR(kinetis_usb1otg, KINETIS_IRQ_USB1OTG)  /* 93: USB1 OTG*/
UNUSED(KINETIS_IRQ_RESVD94)
UNUSED(KINETIS_IRQ_RESVD95)
UNUSED(KINETIS_IRQ_RESVD96)
UNUSED(KINETIS_IRQ_RESVD97)
UNUSED(KINETIS_IRQ_RESVD98)
UNUSED(KINETIS_IRQ_RESVD99)

VECTOR(kinetis_qspi0,   KINETIS_QSPI0)       /* 100: QSPI0 all sources */
UNUSED(KINETIS_IRQ_RESVD101)
UNUSED(KINETIS_IRQ_RESVD102)
UNUSED(KINETIS_IRQ_RESVD103)
UNUSED(KINETIS_IRQ_RESVD104)
UNUSED(KINETIS_IRQ_RESVD105)
UNUSED(KINETIS_IRQ_RESVD106)

#  endif /* CONFIG_ARMV7M_CMNVECTOR */
#endif /* CONFIG_ARCH_FAMILY_K28 */
