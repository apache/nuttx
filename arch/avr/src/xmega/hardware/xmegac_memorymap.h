/****************************************************************************
 * arch/avr/src/xmega/hardware/xmegac_memorymap.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_AVR_SRC_XMEGA_HARDWARE_XMEGAC_MEMORYMAP_H
#define __ARCH_AVR_SRC_XMEGA_HARDWARE_XMEGAC_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/xmega/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* XMega C data memory map */

#define XMEGA_IOMEM_BASE     0x0000 /* I/O Memory (Up to 4 KB) */
#define XMEGA_EEPROM_BASE    0x1000 /* EEPROM (Up to 4 KB) */
#define XMEGA_ISRAM_BASE     0x2000 /* Internal SRAM */

#define XMEGA_GPIO_BASE      0x0000 /* General Purpose IO Registers */
#define XMEGA_VPORT0_BASE    0x0010 /* Virtual Port 0 */
#define XMEGA_VPORT1_BASE    0x0014 /* Virtual Port 1 */
#define XMEGA_VPORT2_BASE    0x0018 /* Virtual Port 2 */
#define XMEGA_VPORT3_BASE    0x001c /* Virtual Port 3 */
#define XMEGA_CPU_BASE       0x0030 /* CPU */
#define XMEGA_CLK_BASE       0x0040 /* Clock Control */
#define XMEGA_SLEEP_BASE     0x0048 /* Sleep Controller */
#define XMEGA_OSC_BASE       0x0050 /* Oscillator Control */
#define XMEGA_DFLLRC32M_BASE 0x0060 /* DFLL for the 32 MHz Internal RC Oscillator */
#define XMEGA_DFLLRC2M_BASE  0x0068 /* DFLL for the 2 MHz RC Oscillator */
#define XMEGA_PR_BASE        0x0070 /* Power Reduction */
#define XMEGA_RST_BASE       0x0078 /* Reset Controller */
#define XMEGA_WDT_BASE       0x0080 /* Watch-Dog Timer */
#define XMEGA_MCU_BASE       0x0090 /* MCU Control */
#define XMEGA_PMIC_BASE      0x00a0 /* Programmable Multilevel Interrupt Controller */
#define XMEGA_PORTCFG_BASE   0x00b0 /* Port Configuration */
#define XMEGA_AES_BASE       0x00c0 /* AES Module */
#define XMEGA_DMA_BASE       0x0100 /* DMA Controller */
#define XMEGA_EVSYS_BASE     0x0180 /* Event System */
#define XMEGA_NVM_BASE       0x01C0 /* Non Volatile Memory (NVM) Controller */
#define XMEGA_ADCA_BASE      0x0200 /* Analog to Digital Converter on port A */
#define XMEGA_ACA_BASE       0x0380 /* Analog Comparator pair on port A */
#define XMEGA_RTC_BASE       0x0400 /* Real Time Counter */
#define XMEGA_TWIC_BASE      0x0480 /* Two Wire Interface on port C */
#define XMEGA_TWIE_BASE      0x04a0 /* Two Wire Interface on port E */
#define XMEGA_PORTA_BASE     0x0600 /* Port A */
#define XMEGA_PORTB_BASE     0x0620 /* Port B */
#define XMEGA_PORTC_BASE     0x0640 /* Port C */
#define XMEGA_PORTD_BASE     0x0660 /* Port D */
#define XMEGA_PORTE_BASE     0x0680 /* Port E */
#define XMEGA_PORTF_BASE     0x06a0 /* Port F */
#define XMEGA_PORTR_BASE     0x07e0 /* Port R */
#define XMEGA_TCC0_BASE      0x0800 /* Timer/Counter 0 on port C */
#define XMEGA_TCC1_BASE      0x0840 /* Timer/Counter 1 on port C */
#define XMEGA_AWEXC_BASE     0x0880 /* Advanced Waveform Extension on port C */
#define XMEGA_HIRESC_BASE    0x0890 /* High Resolution Extension on port C */
#define XMEGA_USARTC0_BASE   0x08a0 /* USART 0 on port C */
#define XMEGA_USARTC1_BASE   0x08b0 /* USART 1 on port C */
#define XMEGA_SPIC_BASE      0x08c0 /* Serial Peripheral Interface on port C */
#define XMEGA_IRCOM_BASE     0x08f0 /* Infrared Communication Module */
#define XMEGA_TCD0_BASE      0x0900 /* Timer/Counter 0 on port D */
#define XMEGA_USARTD0_BASE   0x09a0 /* USART 0 on port D */
#define XMEGA_SPID_BASE      0x09c0 /* Serial Peripheral Interface on port D */
#define XMEGA_TCE0_BASE      0x0a00 /* Timer/Counter 0 on port E */
#define XMEGA_USARTE0_BASE   0x0aa0 /* USART 0 on port E */
#define XMEGA_TCF0_BASE      0x0b00 /* Timer/Counter 0 on port F */
#define XMEGA_USARTF0_BASE   0x0ba0 /* USART 0 on port F */

#endif /* __ARCH_AVR_SRC_XMEGA_HARDWARE_XMEGAC_MEMORYMAP_H */
