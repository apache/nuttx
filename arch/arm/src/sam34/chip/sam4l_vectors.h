/************************************************************************************************
 * arch/arm/src/sam34/chip/sam4l_vectors.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/
/* This file is included by sam_vectors.S.  It provides the macro VECTOR that
 * supplies ach SAM3U vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/sam/sam4l_irq.h.
 * sam_vectors.S will defined the VECTOR in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 */

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 80 interrupt table entries for I/O interrupts. */

# define ARMV7M_PERIPHERAL_INTERRUPTS 80

#else
  VECTOR(sam_hflashc, SAM_IRQ_HFLASHC)           /* Vector 16+0:   Flash Controller */
  VECTOR(sam_pdca0, SAM_IRQ_PDCA0)               /* Vector 16+1:   Peripheral DMA Controller 0 */
  VECTOR(sam_pdca1, SAM_IRQ_PDCA1)               /* Vector 16+2:   Peripheral DMA Controller 1 */
  VECTOR(sam_pdca2, SAM_IRQ_PDCA2)               /* Vector 16+3:   Peripheral DMA Controller 2 */
  VECTOR(sam_pdca3, SAM_IRQ_PDCA3)               /* Vector 16+4:   Peripheral DMA Controller 3 */
  VECTOR(sam_pdca4, SAM_IRQ_PDCA4)               /* Vector 16+5:   Peripheral DMA Controller 4 */
  VECTOR(sam_pdca5, SAM_IRQ_PDCA5)               /* Vector 16+6:   Peripheral DMA Controller 5 */
  VECTOR(sam_pdca6, SAM_IRQ_PDCA6)               /* Vector 16+7:   Peripheral DMA Controller 6 */
  VECTOR(sam_pdca7, SAM_IRQ_PDCA7)               /* Vector 16+8:   Peripheral DMA Controller 7 */
  VECTOR(sam_pdca8, SAM_IRQ_PDCA8)               /* Vector 16+9:   Peripheral DMA Controller 8 */
  VECTOR(sam_pdca9, SAM_IRQ_PDCA9)               /* Vector 16+10:  Peripheral DMA Controller 9 */
  VECTOR(sam_pdca10, SAM_IRQ_PDCA10)             /* Vector 16+11:  Peripheral DMA Controller 10 */
  VECTOR(sam_pdca11, SAM_IRQ_PDCA11)             /* Vector 16+12:  Peripheral DMA Controller 11 */
  VECTOR(sam_pdca12, SAM_IRQ_PDCA12)             /* Vector 16+13:  Peripheral DMA Controller 12 */
  VECTOR(sam_pdca13, SAM_IRQ_PDCA13)             /* Vector 16+14:  Peripheral DMA Controller 13 */
  VECTOR(sam_pdca14, SAM_IRQ_PDCA14)             /* Vector 16+15:  Peripheral DMA Controller 14 */
  VECTOR(sam_pdca15, SAM_IRQ_PDCA15)             /* Vector 16+16:  Peripheral DMA Controller 15 */
  VECTOR(sam_crccu, SAM_IRQ_CRCCU)               /* Vector 16+17:  CRC Calculation Unit */
  VECTOR(sam_usbc, SAM_IRQ_USBC)                 /* Vector 16+18:  USB 2.0 Interface */
  VECTOR(sam_pevc_tr, SAM_IRQ_PEVC_TR)           /* Vector 16+19:  Peripheral Event Controller TR */
  VECTOR(sam_pevc_ov, SAM_IRQ_PEVC_OV)           /* Vector 16+20:  Peripheral Event Controller OV */
  VECTOR(sam_aesa, SAM_IRQ_AESA)                 /* Vector 16+21:  Advanced Encryption Standard */
  VECTOR(sam_pm, SAM_IRQ_PM)                     /* Vector 16+22:  Power Manager */
  VECTOR(sam_scif, SAM_IRQ_SCIF)                 /* Vector 16+23:  System Control Interface */
  VECTOR(sam_freqm, SAM_IRQ_FREQM)               /* Vector 16+24:  Frequency Meter */
  VECTOR(sam_gpio0, SAM_IRQ_GPIO0)               /* Vector 16+25:  General-Purpose Input/Output Controller 0 */
  VECTOR(sam_gpio1, SAM_IRQ_GPIO1)               /* Vector 16+26:  General-Purpose Input/Output Controller 1 */
  VECTOR(sam_gpio2, SAM_IRQ_GPIO2)               /* Vector 16+27:  General-Purpose Input/Output Controller 2 */
  VECTOR(sam_gpio3, SAM_IRQ_GPIO3)               /* Vector 16+28:  General-Purpose Input/Output Controller 3 */
  VECTOR(sam_gpio4, SAM_IRQ_GPIO4)               /* Vector 16+29:  General-Purpose Input/Output Controller 4 */
  VECTOR(sam_gpio5, SAM_IRQ_GPIO5)               /* Vector 16+30:  General-Purpose Input/Output Controller 5 */
  VECTOR(sam_gpio6, SAM_IRQ_GPIO6)               /* Vector 16+31:  General-Purpose Input/Output Controller 6 */
  VECTOR(sam_gpio7, SAM_IRQ_GPIO7)               /* Vector 16+32:  General-Purpose Input/Output Controller 7 */
  VECTOR(sam_gpio8, SAM_IRQ_GPIO8)               /* Vector 16+33:  General-Purpose Input/Output Controller 8 */
  VECTOR(sam_gpio9, SAM_IRQ_GPIO9)               /* Vector 16+34:  General-Purpose Input/Output Controller 9 */
  VECTOR(sam_gpio10, SAM_IRQ_GPIO10)             /* Vector 16+35:  General-Purpose Input/Output Controller 10 */
  VECTOR(sam_gpio11, SAM_IRQ_GPIO11)             /* Vector 16+36:  General-Purpose Input/Output Controller 11 */
  VECTOR(sam_bpm, SAM_IRQ_BPM)                   /* Vector 16+37:  Backup Power Manager */
  VECTOR(sam_bscif, SAM_IRQ_BSCIF)               /* Vector 16+38:  Backup System Control Interface */
  VECTOR(sam_ast_alarm, SAM_IRQ_AST_ALARM)       /* Vector 16+39:  Asynchronous Timer ALARM */
  VECTOR(sam_ast_per, SAM_IRQ_AST_PER)           /* Vector 16+40:  Asynchronous Timer PER */
  VECTOR(sam_ast_ovf, SAM_IRQ_AST_OVF)           /* Vector 16+41:  Asynchronous Timer OVF */
  VECTOR(sam_ast_ready, SAM_IRQ_AST_READY)       /* Vector 16+42:  Asynchronous Timer READY */
  VECTOR(sam_ast_clkready, SAM_IRQ_AST_CLKREADY) /* Vector 16+43:  Asynchronous Timer CLKREADY */
  VECTOR(sam_wdt, SAM_IRQ_WDT)                   /* Vector 16+44:  Watchdog Timer */
  VECTOR(sam_eic1, SAM_IRQ_EIC1)                 /* Vector 16+45:  External Interrupt Controller 1 */
  VECTOR(sam_eic2, SAM_IRQ_EIC2)                 /* Vector 16+46:  External Interrupt Controller 2 */
  VECTOR(sam_eic3, SAM_IRQ_EIC3)                 /* Vector 16+47:  External Interrupt Controller 3 */
  VECTOR(sam_eic4, SAM_IRQ_EIC4)                 /* Vector 16+48:  External Interrupt Controller 4 */
  VECTOR(sam_eic5, SAM_IRQ_EIC5)                 /* Vector 16+49:  External Interrupt Controller 5 */
  VECTOR(sam_eic6, SAM_IRQ_EIC6)                 /* Vector 16+50:  External Interrupt Controller 6 */
  VECTOR(sam_eic7, SAM_IRQ_EIC7)                 /* Vector 16+51:  External Interrupt Controller 7 */
  VECTOR(sam_eic8, SAM_IRQ_EIC8)                 /* Vector 16+52:  External Interrupt Controller 8 */
  VECTOR(sam_iisc, SAM_IRQ_IISC)                 /* Vector 16+53:  Inter-IC Sound (I2S) Controller */
  VECTOR(sam_spi0, SAM_IRQ_SPI0)                 /* Vector 16+54:  Serial Peripheral Interface */
  VECTOR(sam_tc00, SAM_IRQ_TC00)                 /* Vector 16+55:  Timer/Counter 0 */
  VECTOR(sam_tc01, SAM_IRQ_TC01)                 /* Vector 16+56:  Timer/Counter 1 */
  VECTOR(sam_tc02, SAM_IRQ_TC02)                 /* Vector 16+57:  Timer/Counter 2 */
  VECTOR(sam_tc10, SAM_IRQ_TC10)                 /* Vector 16+58:  Timer/Counter 10 */
  VECTOR(sam_tc11, SAM_IRQ_TC11)                 /* Vector 16+59:  Timer/Counter 11 */
  VECTOR(sam_tc12, SAM_IRQ_TC12)                 /* Vector 16+60:  Timer/Counter 12 */
  VECTOR(sam_twim0, SAM_IRQ_TWIM0)               /* Vector 16+61:  Two-wire Master Interface TWIM0 */
  VECTOR(sam_twis0, SAM_IRQ_TWIS0)               /* Vector 16+62:  Two-wire Slave Interface TWIS0 */
  VECTOR(sam_twim1, SAM_IRQ_TWIM1)               /* Vector 16+63:  Two-wire Master Interface TWIM1 */
  VECTOR(sam_twis1, SAM_IRQ_TWIS1)               /* Vector 16+64:  Two-wire Slave Interface TWIS1 */
  VECTOR(sam_usart0, SAM_IRQ_USART0)             /* Vector 16+65:  USART0 */
  VECTOR(sam_usart1, SAM_IRQ_USART1)             /* Vector 16+66:  USART1 */
  VECTOR(sam_usart2, SAM_IRQ_USART2)             /* Vector 16+67:  USART2 */
  VECTOR(sam_usart3, SAM_IRQ_USART3)             /* Vector 16+68:  USART3 */
  VECTOR(sam_adcife, SAM_IRQ_ADCIFE)             /* Vector 16+69:  ADC controller interface  */
  VECTOR(sam_dacc, SAM_IRQ_DACC)                 /* Vector 16+70:  DAC Controller */
  VECTOR(sam_acifc, SAM_IRQ_ACIFC)               /* Vector 16+71:  Analog Comparator Interface */
  VECTOR(sam_abdacb, SAM_IRQ_ABDACB)             /* Vector 16+72:  Audio Bitstream DAC */
  VECTOR(sam_trng, SAM_IRQ_TRNG)                 /* Vector 16+73:  True Random Number Generator */
  VECTOR(sam_parc, SAM_IRQ_PARC)                 /* Vector 16+74:  Parallel Capture */
  VECTOR(sam_catb, SAM_IRQ_CATB)                 /* Vector 16+75:  Capacitive Touch Module B */
  VECTOR(sam_twim2, SAM_IRQ_TWIM2)               /* Vector 16+77:  Two-wire Master Interface */
  VECTOR(sam_twim3, SAM_IRQ_TWIM3)               /* Vector 16+78:  Two-wire Master Interface */
  VECTOR(sam_lcdca, SAM_IRQ_LCDCA)               /* Vector 16+79:  LCD Controller A */
#endif
