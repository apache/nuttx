/****************************************************************************************************
 * arch/arm/src/lpc54xx/lpc54_inputmux.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_INPUTMUX_H
#define __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_INPUTMUX_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/lpc54_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register offsets *********************************************************************************/

#define LPC54_MUX_SCT0_INMUX_OFFSET(n)      (0x0000 + ((n) << 2))
#define LPC54_MUX_SCT0_INMUX0_OFFSET        0x0000  /* Input mux register for SCT0 input 0 */
#define LPC54_MUX_SCT0_INMUX1_OFFSET        0x0004  /* Input mux register for SCT0 input 1 */
#define LPC54_MUX_SCT0_INMUX2_OFFSET        0x0008  /* Input mux register for SCT0 input 2 */
#define LPC54_MUX_SCT0_INMUX3_OFFSET        0x000c  /* Input mux register for SCT0 input 3 */
#define LPC54_MUX_SCT0_INMUX4_OFFSET        0x0010  /* Input mux register for SCT0 input 4 */
#define LPC54_MUX_SCT0_INMUX5_OFFSET        0x0014  /* Input mux register for SCT0 input 5 */
#define LPC54_MUX_SCT0_INMUX6_OFFSET        0x0018  /* Input mux register for SCT0 input 6 */

#define LPC54_MUX_PINTSEL_OFFSET(n)         (0x00c0 + ((n) << 2))
#define LPC54_MUX_PINTSEL0_OFFSET           0x00c0  /* Pin interrupt select register 0 */
#define LPC54_MUX_PINTSEL1_OFFSET           0x00c4  /* Pin interrupt select register 1 */
#define LPC54_MUX_PINTSEL2_OFFSET           0x00c8  /* Pin interrupt select register 2 */
#define LPC54_MUX_PINTSEL3_OFFSET           0x00cc  /* Pin interrupt select register 3 */
#define LPC54_MUX_PINTSEL4_OFFSET           0x00d0  /* Pin interrupt select register 4 */
#define LPC54_MUX_PINTSEL5_OFFSET           0x00d4  /* Pin interrupt select register 5 */
#define LPC54_MUX_PINTSEL6_OFFSET           0x00d8  /* Pin interrupt select register 6 */
#define LPC54_MUX_PINTSEL7_OFFSET           0x00dc  /* Pin interrupt select register 7 */

#define LPC54_MUX_DMA_ITRIG_INMUX_OFFSET(n) (0x00e0 + ((n) << 2))
#define LPC54_MUX_DMA_ITRIG_INMUX0_OFFSET   0x00e0  /* Trigger select register for DMA channel 0 */
#define LPC54_MUX_DMA_ITRIG_INMUX1_OFFSET   0x00e4  /* Trigger select register for DMA channel 1 */
#define LPC54_MUX_DMA_ITRIG_INMUX2_OFFSET   0x00e8  /* Trigger select register for DMA channel 2 */
#define LPC54_MUX_DMA_ITRIG_INMUX3_OFFSET   0x00ec  /* Trigger select register for DMA channel 3 */
#define LPC54_MUX_DMA_ITRIG_INMUX4_OFFSET   0x00f0  /* Trigger select register for DMA channel 4 */
#define LPC54_MUX_DMA_ITRIG_INMUX5_OFFSET   0x00f4  /* Trigger select register for DMA channel 5 */
#define LPC54_MUX_DMA_ITRIG_INMUX6_OFFSET   0x00f8  /* Trigger select register for DMA channel 6 */
#define LPC54_MUX_DMA_ITRIG_INMUX7_OFFSET   0x00fc  /* Trigger select register for DMA channel 7 */
#define LPC54_MUX_DMA_ITRIG_INMUX8_OFFSET   0x0100  /* Trigger select register for DMA channel 8 */
#define LPC54_MUX_DMA_ITRIG_INMUX9_OFFSET   0x0104  /* Trigger select register for DMA channel 9 */
#define LPC54_MUX_DMA_ITRIG_INMUX10_OFFSET  0x0108  /* Trigger select register for DMA channel 10 */
#define LPC54_MUX_DMA_ITRIG_INMUX11_OFFSET  0x010c  /* Trigger select register for DMA channel 11 */
#define LPC54_MUX_DMA_ITRIG_INMUX12_OFFSET  0x0110  /* Trigger select register for DMA channel 12 */
#define LPC54_MUX_DMA_ITRIG_INMUX13_OFFSET  0x0114  /* Trigger select register for DMA channel 13 */
#define LPC54_MUX_DMA_ITRIG_INMUX14_OFFSET  0x0118  /* Trigger select register for DMA channel 14 */
#define LPC54_MUX_DMA_ITRIG_INMUX15_OFFSET  0x011c  /* Trigger select register for DMA channel 15 */
#define LPC54_MUX_DMA_ITRIG_INMUX16_OFFSET  0x0120  /* Trigger select register for DMA channel 16 */
#define LPC54_MUX_DMA_ITRIG_INMUX17_OFFSET  0x0124  /* Trigger select register for DMA channel 17 */
#define LPC54_MUX_DMA_ITRIG_INMUX18_OFFSET  0x0128  /* Trigger select register for DMA channel 18 */
#define LPC54_MUX_DMA_ITRIG_INMUX19_OFFSET  0x012c  /* Trigger select register for DMA channel 19 */
#define LPC54_MUX_DMA_ITRIG_INMUX20_OFFSET  0x0130  /* Trigger select register for DMA channel 20 */
#define LPC54_MUX_DMA_ITRIG_INMUX21_OFFSET  0x0134  /* Trigger select register for DMA channel 21 */
#define LPC54_MUX_DMA_ITRIG_INMUX22_OFFSET  0x0138  /* Trigger select register for DMA channel 22 */
#define LPC54_MUX_DMA_ITRIG_INMUX23_OFFSET  0x013c  /* Trigger select register for DMA channel 23 */
#define LPC54_MUX_DMA_ITRIG_INMUX24_OFFSET  0x0140  /* Trigger select register for DMA channel 24 */
#define LPC54_MUX_DMA_ITRIG_INMUX25_OFFSET  0x0144  /* Trigger select register for DMA channel 25 */
#define LPC54_MUX_DMA_ITRIG_INMUX26_OFFSET  0x0148  /* Trigger select register for DMA channel 26 */
#define LPC54_MUX_DMA_ITRIG_INMUX27_OFFSET  0x014c  /* Trigger select register for DMA channel 27 */
#define LPC54_MUX_DMA_ITRIG_INMUX28_OFFSET  0x0150  /* Trigger select register for DMA channel 28 */
#define LPC54_MUX_DMA_ITRIG_INMUX29_OFFSET  0x0154  /* Trigger select register for DMA channel 29 */

#define LPC54_MUX_DMA_OTRIG_INMUX_OFFSET(n) (0x0160 + ((n) << 2))
#define LPC54_MUX_DMA_OTRIG_INMUX0_OFFSET   0x0160  /* DMA output trigger selection to become DMA trigger 18 */
#define LPC54_MUX_DMA_OTRIG_INMUX1_OFFSET   0x0164  /* DMA output trigger selection to become DMA trigger 19 */
#define LPC54_MUX_DMA_OTRIG_INMUX2_OFFSET   0x0168  /* DMA output trigger selection to become DMA trigger 20 */
#define LPC54_MUX_DMA_OTRIG_INMUX3_OFFSET   0x016c  /* DMA output trigger selection to become DMA trigger 21 */

#define LPC54_MUX_FREQMEAS_REF_OFFSET       0x0180  /* Selection for frequency measurement reference clock */
#define LPC54_MUX_FREQMEAS_TARGET_OFFSET    0x0184  /* Selection for frequency measurement target clock */

/* Register addresses *******************************************************************************/

#define LPC54_MUX_SCT0_INMUX(n)             (LPC54_MUX_BASE + LPC54_MUX_SCT0_INMUX_OFFSET(n))
#define LPC54_MUX_SCT0_INMUX0               (LPC54_MUX_BASE + LPC54_MUX_SCT0_INMUX0_OFFSET)
#define LPC54_MUX_SCT0_INMUX1               (LPC54_MUX_BASE + LPC54_MUX_SCT0_INMUX1_OFFSET)
#define LPC54_MUX_SCT0_INMUX2               (LPC54_MUX_BASE + LPC54_MUX_SCT0_INMUX2_OFFSET)
#define LPC54_MUX_SCT0_INMUX3               (LPC54_MUX_BASE + LPC54_MUX_SCT0_INMUX3_OFFSET)
#define LPC54_MUX_SCT0_INMUX4               (LPC54_MUX_BASE + LPC54_MUX_SCT0_INMUX4_OFFSET)
#define LPC54_MUX_SCT0_INMUX5               (LPC54_MUX_BASE + LPC54_MUX_SCT0_INMUX5_OFFSET)
#define LPC54_MUX_SCT0_INMUX6               (LPC54_MUX_BASE + LPC54_MUX_SCT0_INMUX6_OFFSET)

#define LPC54_MUX_PINTSEL(n)                (LPC54_MUX_BASE + LPC54_MUX_PINTSEL_OFFSET(n))
#define LPC54_MUX_PINTSEL0                  (LPC54_MUX_BASE + LPC54_MUX_PINTSEL0_OFFSET)
#define LPC54_MUX_PINTSEL1                  (LPC54_MUX_BASE + LPC54_MUX_PINTSEL1_OFFSET)
#define LPC54_MUX_PINTSEL2                  (LPC54_MUX_BASE + LPC54_MUX_PINTSEL2_OFFSET)
#define LPC54_MUX_PINTSEL3                  (LPC54_MUX_BASE + LPC54_MUX_PINTSEL3_OFFSET)
#define LPC54_MUX_PINTSEL4                  (LPC54_MUX_BASE + LPC54_MUX_PINTSEL4_OFFSET)
#define LPC54_MUX_PINTSEL5                  (LPC54_MUX_BASE + LPC54_MUX_PINTSEL5_OFFSET)
#define LPC54_MUX_PINTSEL6                  (LPC54_MUX_BASE + LPC54_MUX_PINTSEL6_OFFSET)
#define LPC54_MUX_PINTSEL7                  (LPC54_MUX_BASE + LPC54_MUX_PINTSEL7_OFFSET)

#define LPC54_MUX_DMA_ITRIG_INMUX(n)        (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX_OFFSET(n))
#define LPC54_MUX_DMA_ITRIG_INMUX0          (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX0_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX1          (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX1_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX2          (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX2_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX3          (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX3_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX4          (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX4_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX5          (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX5_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX6          (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX6_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX7          (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX7_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX8          (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX8_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX9          (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX9_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX10         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX10_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX11         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX11_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX12         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX12_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX13         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX13_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX14         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX14_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX15         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX15_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX16         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX16_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX17         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX17_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX18         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX18_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX19         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX19_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX20         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX20_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX21         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX21_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX22         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX22_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX23         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX23_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX24         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX24_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX25         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX25_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX26         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX26_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX27         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX27_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX28         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX28_OFFSET)
#define LPC54_MUX_DMA_ITRIG_INMUX29         (LPC54_MUX_BASE + LPC54_MUX_DMA_ITRIG_INMUX29_OFFSET)

#define LPC54_MUX_DMA_OTRIG_INMUX(n)        (LPC54_MUX_BASE + LPC54_MUX_DMA_OTRIG_INMUX_OFFSET(n))
#define LPC54_MUX_DMA_OTRIG_INMUX0          (LPC54_MUX_BASE + LPC54_MUX_DMA_OTRIG_INMUX0_OFFSET)
#define LPC54_MUX_DMA_OTRIG_INMUX1          (LPC54_MUX_BASE + LPC54_MUX_DMA_OTRIG_INMUX1_OFFSET)
#define LPC54_MUX_DMA_OTRIG_INMUX2          (LPC54_MUX_BASE + LPC54_MUX_DMA_OTRIG_INMUX2_OFFSET)
#define LPC54_MUX_DMA_OTRIG_INMUX3          (LPC54_MUX_BASE + LPC54_MUX_DMA_OTRIG_INMUX3_OFFSET)

#define LPC54_MUX_FREQMEAS_REF              (LPC54_MUX_BASE + LPC54_MUX_FREQMEAS_REF_OFFSET)
#define LPC54_MUX_FREQMEAS_TARGET           (LPC54_MUX_BASE + LPC54_MUX_FREQMEAS_TARGET_OFFSET)

/* Register bit definitions *************************************************************************/

/* Input mux register for SCT0 input 0-6 */

#define MUX_SCT0_INMUX_SHIFT                (0)      /* Bits 0-4: Input number to SCT0 inputs 0 to 6 */
#define MUX_SCT0_INMUX_MASK                 (31 << MUX_SCT0_INMUX_SHIFT)
#  define MUX_SCT0_INMUX_SCTGPI0            (0  << MUX_SCT0_INMUX_SHIFT) /* function selected from IOCON register */
#  define MUX_SCT0_INMUX_SCTGPI1            (1  << MUX_SCT0_INMUX_SHIFT) /* function selected from IOCON register */
#  define MUX_SCT0_INMUX_SCTGPI2            (2  << MUX_SCT0_INMUX_SHIFT) /* function selected from IOCON register */
#  define MUX_SCT0_INMUX_SCTGPI3            (3  << MUX_SCT0_INMUX_SHIFT) /* function selected from IOCON register */
#  define MUX_SCT0_INMUX_SCTGPI4            (4  << MUX_SCT0_INMUX_SHIFT) /* function selected from IOCON register */
#  define MUX_SCT0_INMUX_SCTGPI5            (5  << MUX_SCT0_INMUX_SHIFT) /* function selected from IOCON register */
#  define MUX_SCT0_INMUX_SCTGPI6            (6  << MUX_SCT0_INMUX_SHIFT) /* function selected from IOCON register */
#  define MUX_SCT0_INMUX_SCTGPI7            (7  << MUX_SCT0_INMUX_SHIFT) /* function selected from IOCON register */
#  define MUX_SCT0_INMUX_T0OUT0             (8  << MUX_SCT0_INMUX_SHIFT) /* T0_OUT0 */
#  define MUX_SCT0_INMUX_T1OUT0             (9  << MUX_SCT0_INMUX_SHIFT) /* T1_OUT0 */
#  define MUX_SCT0_INMUX_T2OUT0             (10 << MUX_SCT0_INMUX_SHIFT) /* T2_OUT0 */
#  define MUX_SCT0_INMUX_T3OUT0             (11 << MUX_SCT0_INMUX_SHIFT) /* T3_OUT0 */
#  define MUX_SCT0_INMUX_T4OUT0             (12 << MUX_SCT0_INMUX_SHIFT) /* T4_OUT0 */
#  define MUX_SCT0_INMUX_ADCTHCMP           (13 << MUX_SCT0_INMUX_SHIFT) /* ADC_THCMP_IRQ */
#  define MUX_SCT0_INMUX_BMATCH             (14 << MUX_SCT0_INMUX_SHIFT) /* GPIOINT_BMATCH */
#  define MUX_SCT0_INMUX_USB0               (15 << MUX_SCT0_INMUX_SHIFT) /* USB0_FRAME_TOGGLE */
#  define MUX_SCT0_INMUX_USB1               (16 << MUX_SCT0_INMUX_SHIFT) /* USB1_FRAME_TOGGLE */
#  define MUX_SCT0_INMUX_ARMTXEV            (17 << MUX_SCT0_INMUX_SHIFT) /* ARM_TXEV */
#  define MUX_SCT0_INMUX_HALTED             (18 << MUX_SCT0_INMUX_SHIFT) /* DEBUG_HALTED */
#  define MUX_SCT0_INMUX_SC0TX              (19 << MUX_SCT0_INMUX_SHIFT) /* SMARTCARD0_TX_ACTIVE */
#  define MUX_SCT0_INMUX_SC0RX              (20 << MUX_SCT0_INMUX_SHIFT) /* SMARTCARD0_RX_ACTIVE */
#  define MUX_SCT0_INMUX_SC1TX              (21 << MUX_SCT0_INMUX_SHIFT) /* SMARTCARD1_TX_ACTIVE */
#  define MUX_SCT0_INMUX_S10RX              (22 << MUX_SCT0_INMUX_SHIFT) /* SMARTCARD1_RX_ACTIVE */
#  define MUX_SCT0_INMUX_I2S6SCLK           (23 << MUX_SCT0_INMUX_SHIFT) /* I2S6_SCLK */
#  define MUX_SCT0_INMUX_I2S7SCLK           (24 << MUX_SCT0_INMUX_SHIFT) /* I2S7_SCLK */

/* Pin interrupt select register 0-7
 *
 * Pin number select for pin interrupt or pattern match engine input.
 * For PIOx_y: pin = (x * 32) + y.
 * PIO0_0 to PIO1_31 correspond to numbers 0 to 63.
 */

#define MUX_PINTSEL(n)                      (1 << (n))

/* Trigger select register for DMA channel 0-29 */

#define ITRIG_INMUX_ADC0A                   (0)      /*  ADC0 Sequence A interrupt */
#define ITRIG_INMUX_ADC0B                   (1)      /* ADC0 Sequence B interrupt */
#define ITRIG_INMUX_SCT0DMA0                (2)      /* SCT0 DMA request 0 */
#define ITRIG_INMUX_SCT0DMA1                (3)      /* SCT0 DMA request 1 */
#define ITRIG_INMUX_PININT0                 (4)      /* Pin interrupt 0 */
#define ITRIG_INMUX_PININT1                 (5)      /* Pin interrupt 1 */
#define ITRIG_INMUX_PININT2                 (6)      /* Pin interrupt 2 */
#define ITRIG_INMUX_PININT3                 (7)      /* Pin interrupt 3 */
#define ITRIG_INMUX_CTIMER0MAT0             (8)      /* Timer CTIMER0 Match 0 */
#define ITRIG_INMUX_CTIMER0MAT1             (9)      /* Timer CTIMER0 Match 1 */
#define ITRIG_INMUX_CTIMER1MAT0             (10)     /* Timer CTIMER1 Match 0 */
#define ITRIG_INMUX_CTIMER1MAT1             (11)     /* Timer CTIMER1 Match 1 */
#define ITRIG_INMUX_CTIMER2MAT0             (12)     /* Timer CTIMER2 Match 0 */
#define ITRIG_INMUX_CTIMER2MAT1             (13)     /* Timer CTIMER2 Match 1 */
#define ITRIG_INMUX_CTIMER3MAT0             (14)     /* Timer CTIMER3 Match 0 */
#define ITRIG_INMUX_CTIMER3MAT1             (15)     /* Timer CTIMER3 Match 1 */
#define ITRIG_INMUX_CTIMER4MAT0             (16)     /* Timer CTIMER4 Match 0 */
#define ITRIG_INMUX_CTIMER4MAT1             (17)     /* Timer CTIMER4 Match 1 */
#define ITRIG_INMUX_DMAMUX0                 (18)     /* DMA output trigger mux 0 */
#define ITRIG_INMUX_DMAMUX1                 (19)     /* DMA output trigger mux 1 */
#define ITRIG_INMUX_DMAMUX2                 (20)     /* DMA output trigger mux 2 */
#define ITRIG_INMUX_DMAMUX3                 (21)     /* DMA output trigger mux 3 */

#define MUX_DMA_ITRIG_INMUX_SHIFT           (0)      /* Bit 0-4: Trigger input number for DMA channel n (n = 0 to 29) */
#define MUX_DMA_ITRIG_INMUX_MASK            (31 << MUX_DMA_ITRIG_INMUX_SHIFT)
#  define MUX_DMA_ITRIG_INMUX(n)            ((uint32_t)(n) << MUX_DMA_ITRIG_INMUX_SHIFT)
#  define MUX_DMA_ITRIG_INMUX_ADC0A         (0 << MUX_DMA_ITRIG_INMUX_SHIFT) /*  ADC0 Sequence A interrupt */
#  define MUX_DMA_ITRIG_INMUX_ADC0B         (1  << MUX_DMA_ITRIG_INMUX_SHIFT) /* ADC0 Sequence B interrupt */
#  define MUX_DMA_ITRIG_INMUX_SCT0DMA0      (2  << MUX_DMA_ITRIG_INMUX_SHIFT) /* SCT0 DMA request 0 */
#  define MUX_DMA_ITRIG_INMUX_SCT0DMA1      (3  << MUX_DMA_ITRIG_INMUX_SHIFT) /* SCT0 DMA request 1 */
#  define MUX_DMA_ITRIG_INMUX_PININT0       (4  << MUX_DMA_ITRIG_INMUX_SHIFT) /* Pin interrupt 0 */
#  define MUX_DMA_ITRIG_INMUX_PININT1       (5  << MUX_DMA_ITRIG_INMUX_SHIFT) /* Pin interrupt 1 */
#  define MUX_DMA_ITRIG_INMUX_PININT2       (6  << MUX_DMA_ITRIG_INMUX_SHIFT) /* Pin interrupt 2 */
#  define MUX_DMA_ITRIG_INMUX_PININT3       (7  << MUX_DMA_ITRIG_INMUX_SHIFT) /* Pin interrupt 3 */
#  define MUX_DMA_ITRIG_INMUX_CTIMER0MAT0   (8  << MUX_DMA_ITRIG_INMUX_SHIFT) /* Timer CTIMER0 Match 0 */
#  define MUX_DMA_ITRIG_INMUX_CTIMER0MAT1   (9  << MUX_DMA_ITRIG_INMUX_SHIFT) /* Timer CTIMER0 Match 1 */
#  define MUX_DMA_ITRIG_INMUX_CTIMER1MAT0   (10 << MUX_DMA_ITRIG_INMUX_SHIFT) /* Timer CTIMER1 Match 0 */
#  define MUX_DMA_ITRIG_INMUX_CTIMER1MAT1   (11 << MUX_DMA_ITRIG_INMUX_SHIFT) /* Timer CTIMER1 Match 1 */
#  define MUX_DMA_ITRIG_INMUX_CTIMER2MAT0   (12 << MUX_DMA_ITRIG_INMUX_SHIFT) /* Timer CTIMER2 Match 0 */
#  define MUX_DMA_ITRIG_INMUX_CTIMER2MAT1   (13 << MUX_DMA_ITRIG_INMUX_SHIFT) /* Timer CTIMER2 Match 1 */
#  define MUX_DMA_ITRIG_INMUX_CTIMER3MAT0   (14 << MUX_DMA_ITRIG_INMUX_SHIFT) /* Timer CTIMER3 Match 0 */
#  define MUX_DMA_ITRIG_INMUX_CTIMER3MAT1   (15 << MUX_DMA_ITRIG_INMUX_SHIFT) /* Timer CTIMER3 Match 1 */
#  define MUX_DMA_ITRIG_INMUX_CTIMER4MAT0   (16 << MUX_DMA_ITRIG_INMUX_SHIFT) /* Timer CTIMER4 Match 0 */
#  define MUX_DMA_ITRIG_INMUX_CTIMER4MAT1   (17 << MUX_DMA_ITRIG_INMUX_SHIFT) /* Timer CTIMER4 Match 1 */
#  define MUX_DMA_ITRIG_INMUX_DMAMUX0       (18 << MUX_DMA_ITRIG_INMUX_SHIFT) /* DMA output trigger mux 0 */
#  define MUX_DMA_ITRIG_INMUX_DMAMUX1       (19 << MUX_DMA_ITRIG_INMUX_SHIFT) /* DMA output trigger mux 1 */
#  define MUX_DMA_ITRIG_INMUX_DMAMUX2       (20 << MUX_DMA_ITRIG_INMUX_SHIFT) /* DMA output trigger mux 2 */
#  define MUX_DMA_ITRIG_INMUX_DMAMUX3       (21 << MUX_DMA_ITRIG_INMUX_SHIFT) /* DMA output trigger mux 3 */

/* DMA output trigger selection registers 0-3 */

#define MUX_DMA_OTRIG_INMUX_SHIFT           (0)      /* Bits 0-4:  DMA trigger output number for DMA channel n=0..29 */
#define MUX_DMA_OTRIG_INMUX_MASK            (31 << MUX_DMA_OTRIG_INMUX_SHIFT)
#  define MUX_DMA_OTRIG_INMUX(n)            ((uint32_t)(n) << MUX_DMA_OTRIG_INMUX_SHIFT)

/* Selection for frequency measurement reference clock */

#define MUX_FREQMEAS_REF_SHIFT              (0)      /* Bits 0-4: Clock source for frequency measure farget clock */
#define MUX_FREQMEAS_REF_MASK               (31 << MUX_FREQMEAS_REF_SHIFT)
#  define MUX_FREQMEAS_REF_CLKIN            (0 << MUX_FREQMEAS_REF_SHIFT) /* External crystal oscillator (clk_in) */
#  define MUX_FREQMEAS_REF_FRO12M           (1 << MUX_FREQMEAS_REF_SHIFT) /* FRO 12 MHz oscillator (fro_12m) */
#  define MUX_FREQMEAS_REF_FROHF            (2 << MUX_FREQMEAS_REF_SHIFT) /* FRO 96 or 48 MHz (fro_hf) */
#  define MUX_FREQMEAS_REF_WDTCLK           (3 << MUX_FREQMEAS_REF_SHIFT) /* Watchdog oscillator (wdt_clk) */
#  define MUX_FREQMEAS_REF_32KCLK           (4 << MUX_FREQMEAS_REF_SHIFT) /* 32 kHz RTC oscillator (32k_clk) */
#  define MUX_FREQMEAS_REF_MAINCLK          (5 << MUX_FREQMEAS_REF_SHIFT) /* Main clock (main_clk) */
#  define MUX_FREQMEAS_REF_GPIOCLKA         (6 << MUX_FREQMEAS_REF_SHIFT) /* FREQME_GPIO_CLK_A */
#  define MUX_FREQMEAS_REF_GPIOCLKB         (7 << MUX_FREQMEAS_REF_SHIFT) /* FREQME_GPIO_CLK_B */

/* Selection for frequency measurement target clock */

#define MUX_FREQMEAS_TARGET_SHIFT           (0)      /* Bits 0-4: Selects  target clock of the frequency measure function */
#define MUX_FREQMEAS_TARGET_MASK            (31 << MUX_FREQMEAS_TARGET_SHIFT)
#  define MUX_FREQMEAS_TARGET_CLKIN         (0 << MUX_FREQMEAS_TARGET_SHIFT) /* External crystal oscillator (clk_in) */
#  define MUX_FREQMEAS_TARGET_FRO12M        (1 << MUX_FREQMEAS_TARGET_SHIFT) /* FRO 12 MHz oscillator (fro_12m) */
#  define MUX_FREQMEAS_TARGET_FROHF         (2 << MUX_FREQMEAS_TARGET_SHIFT) /* FRO 96 or 48 MHz (fro_hf) */
#  define MUX_FREQMEAS_TARGET_WDTCLK        (3 << MUX_FREQMEAS_TARGET_SHIFT) /* Watchdog oscillator (wdt_clk) */
#  define MUX_FREQMEAS_TARGET_32KCLK        (4 << MUX_FREQMEAS_TARGET_SHIFT) /* 32 kHz RTC oscillator (32k_clk) */
#  define MUX_FREQMEAS_TARGET_MAINCLK       (5 << MUX_FREQMEAS_TARGET_SHIFT) /* Main clock (main_clk) */
#  define MUX_FREQMEAS_TARGET_GPIOCLKA      (6 << MUX_FREQMEAS_TARGET_SHIFT) /* FREQME_GPIO_CLK_A */
#  define MUX_FREQMEAS_TARGET_ PIOCLKB      (7 << MUX_FREQMEAS_TARGET_SHIFT) /* FREQME_GPIO_CLK_B */

#endif /* __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_INPUTMUX_H */
