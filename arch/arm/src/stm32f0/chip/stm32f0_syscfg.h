/****************************************************************************************************
 * arch/arm/src/stm32f0/chip/stm32f0_syscfg.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __ARCH_ARM_SRC_STM32F0_CHIP_STM32F0_SYSCFG_H
#define __ARCH_ARM_SRC_STM32F0_CHIP_STM32F0_SYSCFG_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define STM32F0_SYSCFG_CFGR1_OFFSET      0x0000 /* SYSCFG configuration register 1 */
#define STM32F0_SYSCFG_EXTICR_OFFSET(p)  (0x0008 + ((p) & 0x000c)) /* Registers are displaced by 4! */
#define STM32F0_SYSCFG_EXTICR1_OFFSET    0x0008 /* SYSCFG external interrupt configuration register 1 */
#define STM32F0_SYSCFG_EXTICR2_OFFSET    0x000c /* SYSCFG external interrupt configuration register 2 */
#define STM32F0_SYSCFG_EXTICR3_OFFSET    0x0010 /* SYSCFG external interrupt configuration register 3 */
#define STM32F0_SYSCFG_EXTICR4_OFFSET    0x0014 /* SYSCFG external interrupt configuration register 4 */
#define STM32F0_SYSCFG_CFGR2_OFFSET      0x0018 /* SYSCFG configuration register 2 */
#define STM32F0_SYSCFG_ITLINE0_OFFSET    0x0080 /* SYSCFG interrupt line 0 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE1_OFFSET    0x0084 /* SYSCFG interrupt line 1 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE2_OFFSET    0x0088 /* SYSCFG interrupt line 2 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE3_OFFSET    0x008c /* SYSCFG interrupt line 3 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE4_OFFSET    0x0090 /* SYSCFG interrupt line 4 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE5_OFFSET    0x0094 /* SYSCFG interrupt line 5 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE6_OFFSET    0x0098 /* SYSCFG interrupt line 6 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE7_OFFSET    0x009c /* SYSCFG interrupt line 7 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE8_OFFSET    0x00a0 /* SYSCFG interrupt line 8 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE9_OFFSET    0x00a4 /* SYSCFG interrupt line 9 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE10_OFFSET   0x00a8 /* SYSCFG interrupt line 10 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE11_OFFSET   0x00ac /* SYSCFG interrupt line 11 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE12_OFFSET   0x00b0 /* SYSCFG interrupt line 12 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE13_OFFSET   0x00b4 /* SYSCFG interrupt line 13 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE14_OFFSET   0x00b8 /* SYSCFG interrupt line 14 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE15_OFFSET   0x00bc /* SYSCFG interrupt line 15 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE16_OFFSET   0x00c0 /* SYSCFG interrupt line 16 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE17_OFFSET   0x00c4 /* SYSCFG interrupt line 17 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE18_OFFSET   0x00c8 /* SYSCFG interrupt line 18 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE19_OFFSET   0x00cc /* SYSCFG interrupt line 19 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE20_OFFSET   0x00d0 /* SYSCFG interrupt line 20 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE21_OFFSET   0x00d4 /* SYSCFG interrupt line 21 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE22_OFFSET   0x00d8 /* SYSCFG interrupt line 22 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE23_OFFSET   0x00dc /* SYSCFG interrupt line 23 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE24_OFFSET   0x00e0 /* SYSCFG interrupt line 24 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE25_OFFSET   0x00e4 /* SYSCFG interrupt line 25 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE26_OFFSET   0x00e8 /* SYSCFG interrupt line 26 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE27_OFFSET   0x00ec /* SYSCFG interrupt line 27 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE28_OFFSET   0x00f0 /* SYSCFG interrupt line 28 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE29_OFFSET   0x00f4 /* SYSCFG interrupt line 29 status register (STM32F09x) */
#define STM32F0_SYSCFG_ITLINE30_OFFSET   0x00f8 /* SYSCFG interrupt line 30 status register (STM32F09x) */

/* Register Addresses *******************************************************************************/

#define STM32F0_SYSCFG_CFGR1             (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_CFGR1_OFFSET)

#define STM32F0_SYSCFG_EXTICR(p)         (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_EXTICR_OFFSET(p))
#define STM32F0_SYSCFG_EXTICR1           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_EXTICR1_OFFSET)
#define STM32F0_SYSCFG_EXTICR2           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_EXTICR2_OFFSET)
#define STM32F0_SYSCFG_EXTICR3           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_EXTICR3_OFFSET)
#define STM32F0_SYSCFG_EXTICR4           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_EXTICR4_OFFSET)

#define STM32F0_SYSCFG_CFGR2             (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_CFGR2_OFFSET)

#define STM32F0_SYSCFG_ITLINE0           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE0_OFFSET)
#define STM32F0_SYSCFG_ITLINE1           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE1_OFFSET)
#define STM32F0_SYSCFG_ITLINE2           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE2_OFFSET)
#define STM32F0_SYSCFG_ITLINE3           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE3_OFFSET)
#define STM32F0_SYSCFG_ITLINE4           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE4_OFFSET)
#define STM32F0_SYSCFG_ITLINE5           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE5_OFFSET)
#define STM32F0_SYSCFG_ITLINE6           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE6_OFFSET)
#define STM32F0_SYSCFG_ITLINE7           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE7_OFFSET)
#define STM32F0_SYSCFG_ITLINE8           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE8_OFFSET)
#define STM32F0_SYSCFG_ITLINE9           (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE9_OFFSET)
#define STM32F0_SYSCFG_ITLINE10          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE10_OFFSET)
#define STM32F0_SYSCFG_ITLINE11          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE11_OFFSET)
#define STM32F0_SYSCFG_ITLINE12          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE12_OFFSET)
#define STM32F0_SYSCFG_ITLINE13          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE13_OFFSET)
#define STM32F0_SYSCFG_ITLINE14          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE14_OFFSET)
#define STM32F0_SYSCFG_ITLINE15          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE15_OFFSET)
#define STM32F0_SYSCFG_ITLINE16          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE16_OFFSET)
#define STM32F0_SYSCFG_ITLINE17          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE17_OFFSET)
#define STM32F0_SYSCFG_ITLINE18          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE18_OFFSET)
#define STM32F0_SYSCFG_ITLINE19          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE19_OFFSET)
#define STM32F0_SYSCFG_ITLINE20          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE20_OFFSET)
#define STM32F0_SYSCFG_ITLINE21          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE21_OFFSET)
#define STM32F0_SYSCFG_ITLINE22          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE22_OFFSET)
#define STM32F0_SYSCFG_ITLINE23          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE23_OFFSET)
#define STM32F0_SYSCFG_ITLINE24          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE24_OFFSET)
#define STM32F0_SYSCFG_ITLINE25          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE25_OFFSET)
#define STM32F0_SYSCFG_ITLINE26          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE26_OFFSET)
#define STM32F0_SYSCFG_ITLINE27          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE27_OFFSET)
#define STM32F0_SYSCFG_ITLINE28          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE28_OFFSET)
#define STM32F0_SYSCFG_ITLINE29          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE29_OFFSET)
#define STM32F0_SYSCFG_ITLINE30          (STM32F0_SYSCFG_BASE+STM32F0_SYSCFG_ITLINE30_OFFSET)

/* Register Bitfield Definitions ********************************************************************/

/* SYSCFG memory remap register */

#define SYSCFG_CFGR1_MEMMODE_SHIFT     (0)       /* Bits 1:0 MEM_MODE: Memory mapping selection */
#define SYSCFG_CFGR1_MEMMODE_MASK      (3 << SYSCFG_CFGR1_MEMMODE_SHIFT)
#  define SYSCFG_CFGR1_MEMMODE_FLASH   (0 << SYSCFG_CFGR1_MEMMODE_SHIFT) /* 00: Main Flash at 0x00000000 */
#  define SYSCFG_CFGR1_MEMMODE_SYSTEM  (1 << SYSCFG_CFGR1_MEMMODE_SHIFT) /* 01: System Flash at 0x00000000 */
#  define SYSCFG_CFGR1_MEMMODE_SRAM    (3 << SYSCFG_CFGR1_MEMMODE_SHIFT) /* 11: Embedded SRAM at 0x00000000 */
#define SYSCFG_CFGR1_PA11_PA12_RMP     (1 << 4)  /* Bit 4:  PA11 and PA12 remapping bit for small packages */
#define SYSCFG_CFGR1_IRMOD_SHIFT       (6)       /* Bits 6-7: IR Modulation Envelope signal selection */
#define SYSCFG_CFGR1_IRMOD_MASK        (3 << SYSCFG_CFGR1_IRMOD_SHIFT)
#  define SYSCFG_CFGR1_IRMOD_TIM16     (0 << SYSCFG_CFGR1_IRMOD_SHIFT) /* 00: TIM16 selected */
#  define SYSCFG_CFGR1_IRMOD_USART1    (1 << SYSCFG_CFGR1_IRMOD_SHIFT) /* 01: USART1 selected */
#  define SYSCFG_CFGR1_IRMOD_USART4    (2 << SYSCFG_CFGR1_IRMOD_SHIFT) /* 10: USART1 selected */
#define SYSCFG_CFGR1_ADC_DMARMP        (1 << 8)  /* Bit 8:  ADC DMA remapping bit. Only STM32F03x/F04x/F05x/F07x */
#define SYSCFG_CFGR1_USART1_TXDMARMP   (1 << 9)  /* Bit 9: USART1_TX_DMA request remapping bit. Only STM32F03x/F04x/F05x/F07x */
#define SYSCFG_CFGR1_USART1_RXDMARMP   (1 << 10) /* Bit 10: USART1_TX_DMA request remapping bit. Only STM32F03x/F04x/F05x/F07x */
#define SYSCFG_CFGR1_TIM16_DMARMP      (1 << 11) /* Bit 11: TIM16 DMA request remapping bit */
#define SYSCFG_CFGR1_TIM17_DMARMP      (1 << 12) /* Bit 12: TIM17 DMA request remapping bit */
#define SYSCFG_CFGR1_TIM16_DMARMP2     (1 << 13) /* Bit 13: TIM16 alternate DMA request remapping bit */
#define SYSCFG_CFGR1_TIM17_DMARMP2     (1 << 14) /* Bit 14: TIM17 alternate DMA request remapping bit */
#define SYSCFG_CFGR1_I2C_PBXFMP_SHIFT  (16)      /* Bits 16-19: Fast Mode Plus (FM+) driving capability */
#define SYSCFG_CFGR1_I2C_PBXFMP_MASK   (15 << SYSCFG_CFGR1_I2C_PBXFMP_SHIFT)
#define SYSCFG_CFGR1_I2C1_FMP          (1 << 20) /* Bit 20: I2C1 fast mode Plus driving capability */
#define SYSCFG_CFGR1_I2C2_FMP          (1 << 21) /* Bit 21: I2C2 fast mode Plus driving capability */
#define SYSCFG_CFGR1_I2C_PAXFMP_SHIFT  (22)      /* Bits 22-23: Fast Mode Plus (FM+) driving capability */
#define SYSCFG_CFGR1_I2C_PAXFMP_MASK   (3 << SYSCFG_CFGR1_I2C_PAXFMP_SHIFT)
#define SYSCFG_CFGR1_SPI2_DMARMP       (1 << 24) /* Bit 24: SPI2 DMA request remapping bit. */
#define SYSCFG_CFGR1_USART2_DMARMP     (1 << 25) /* Bit 25: USART2 DMA request remapping bit. */
#define SYSCFG_CFGR1_USART3_DMARMP     (1 << 26) /* Bit 26: USART3 DMA request remapping bit. */
#define SYSCFG_CFGR1_I2C1_DMARMP       (1 << 27) /* Bit 27: I2C1 DMA request remapping bit. */
#define SYSCFG_CFGR1_TIM1_DMARMP       (1 << 28) /* Bit 28: TIM1 DMA request remapping bit. */
#define SYSCFG_CFGR1_TIM2_DMARMP       (1 << 29) /* Bit 29: TIM2 DMA request remapping bit. */
#define SYSCFG_CFGR1_TIM3_DMARMP       (1 << 30) /* Bit 30: TIM3 DMA request remapping bit. */

/* SYSCFG external interrupt configuration register 1-4 */

#define SYSCFG_EXTICR_PORTA           (0)       /* 0000: PA[x] pin */
#define SYSCFG_EXTICR_PORTB           (1)       /* 0001: PB[x] pin */
#define SYSCFG_EXTICR_PORTC           (2)       /* 0010: PC[x] pin */
#define SYSCFG_EXTICR_PORTD           (3)       /* 0011: PD[x] pin */
#define SYSCFG_EXTICR_PORTE           (4)       /* 0100: PE[x] pin */
#define SYSCFG_EXTICR_PORTF           (5)       /* 0101: PF[x] pin */

#define SYSCFG_EXTICR_PORT_MASK       (15)
#define SYSCFG_EXTICR_EXTI_SHIFT(g)   (((g) & 3) << 2)
#define SYSCFG_EXTICR_EXTI_MASK(g)    (SYSCFG_EXTICR_PORT_MASK << (SYSCFG_EXTICR_EXTI_SHIFT(g)))

#define SYSCFG_EXTICR1_EXTI0_SHIFT    (0)       /* Bits 0-3: EXTI 0 coinfiguration */
#define SYSCFG_EXTICR1_EXTI0_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI0_SHIFT)
#define SYSCFG_EXTICR1_EXTI1_SHIFT    (4)       /* Bits 4-7: EXTI 1 coinfiguration */
#define SYSCFG_EXTICR1_EXTI1_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI1_SHIFT)
#define SYSCFG_EXTICR1_EXTI2_SHIFT    (8)       /* Bits 8-11: EXTI 2 coinfiguration */
#define SYSCFG_EXTICR1_EXTI2_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI2_SHIFT)
#define SYSCFG_EXTICR1_EXTI3_SHIFT    (12)      /* Bits 12-15: EXTI 3 coinfiguration */
#define SYSCFG_EXTICR1_EXTI3_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI3_SHIFT)

#define SYSCFG_EXTICR2_EXTI4_SHIFT    (0)       /* Bits 0-3: EXTI 4 coinfiguration */
#define SYSCFG_EXTICR2_EXTI4_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI4_SHIFT)
#define SYSCFG_EXTICR2_EXTI5_SHIFT    (4)       /* Bits 4-7: EXTI 5 coinfiguration */
#define SYSCFG_EXTICR2_EXTI5_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI5_SHIFT)
#define SYSCFG_EXTICR2_EXTI6_SHIFT    (8)       /* Bits 8-11: EXTI 6 coinfiguration */
#define SYSCFG_EXTICR2_EXTI6_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI6_SHIFT)
#define SYSCFG_EXTICR2_EXTI7_SHIFT    (12)      /* Bits 12-15: EXTI 7 coinfiguration */
#define SYSCFG_EXTICR2_EXTI7_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI7_SHIFT)

#define SYSCFG_EXTICR3_EXTI8_SHIFT    (0)       /* Bits 0-3: EXTI 8 coinfiguration */
#define SYSCFG_EXTICR3_EXTI8_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI8_SHIFT)
#define SYSCFG_EXTICR3_EXTI9_SHIFT    (4)       /* Bits 4-7: EXTI 9 coinfiguration */
#define SYSCFG_EXTICR3_EXTI9_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI9_SHIFT)
#define SYSCFG_EXTICR3_EXTI10_SHIFT   (8)       /* Bits 8-11: EXTI 10 coinfiguration */
#define SYSCFG_EXTICR3_EXTI10_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI10_SHIFT)
#define SYSCFG_EXTICR3_EXTI11_SHIFT   (12)      /* Bits 12-15: EXTI 11 coinfiguration */
#define SYSCFG_EXTICR3_EXTI11_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI11_SHIFT)

#define SYSCFG_EXTICR4_EXTI12_SHIFT   (0)       /* Bits 0-3: EXTI 12 coinfiguration */
#define SYSCFG_EXTICR4_EXTI12_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI12_SHIFT)
#define SYSCFG_EXTICR4_EXTI13_SHIFT   (4)       /* Bits 4-7: EXTI 13 coinfiguration */
#define SYSCFG_EXTICR4_EXTI13_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI13_SHIFT)
#define SYSCFG_EXTICR4_EXTI14_SHIFT   (8)       /* Bits 8-11: EXTI 14 coinfiguration */
#define SYSCFG_EXTICR4_EXTI14_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI14_SHIFT)
#define SYSCFG_EXTICR4_EXTI15_SHIFT   (12)      /* Bits 12-15: EXTI 15 coinfiguration */
#define SYSCFG_EXTICR4_EXTI15_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI15_SHIFT)

/* SYSCFG configuration register 2 */

#define SYSCFG_CFGR2_LOCKUPLOCK       (1 << 0)  /* Bit 0: Cortex-M0 Hardfault output bit enable */
#define SYSCFG_CFGR2_SRAM_PARITYLOCK  (1 << 1)  /* Bit 1: RAM parity lock */
#define SYSCFG_CFGR2_PVDLOCK          (1 << 2)  /* Bit 2: PVD lock enable */
#define SYSCFG_CFGR2_SRAM_PEF         (1 << 8)  /* Bit 8: SRAM parity error */

/* SYSCFG interrupt line 0 status register */

#define SYSCFG_ITLINE0_WWDG           (1 << 0)  /* Bit 0: Window Watchdog interrupt pending flag */

/* SYSCFG interrupt line 1 status register */

#define SYSCFG_ITLINE1_PVDOUT         (1 << 0)  /* Bit 0: PVD supply monitoring interrupt request pending (EXTI line 16) */
#define SYSCFG_ITLINE1_VDDIO2         (1 << 0)  /* Bit 1: VDDIO2 supply monitoring interrupt request pending (EXTI line 31) */

/* SYSCFG interrupt line 2 status register */

#define SYSCFG_ITLINE2_RTC_WAKEUP     (1 << 0)  /* Bit 0: RTC Wake Up interrupt request pending (EXTI line 20) */
#define SYSCFG_ITLINE2_RTC_TSTAMP     (1 << 1)  /* Bit 1: RTC Tamper and TimeStamp interrupt request pending (EXTI line 19) */
#define SYSCFG_ITLINE2_RTC_ALRA       (1 << 2)  /* Bit 2: RTC Alarm interrupt request pending (EXTI line 17) */

/* SYSCFG interrupt line 3 status register */

#define SYSCFG_ITLINE3_FLASH_ITF      (1 << 0)  /* Bit 0: Flash interface interrupt request pending */

/* SYSCFG interrupt line 4 status register */

#define SYSCFG_ITLINE4_RCC            (1 << 0)  /* Bit 0: Reset and clock control interrupt request pending */
#define SYSCFG_ITLINE4_CRS            (1 << 1)  /* Bit 1: Clock recovery system interrupt request pending */

/* SYSCFG interrupt line 5 status register */

#define SYSCFG_ITLINE5_EXTI0          (1 << 0)  /* Bit 0: EXTI line 0 interrupt request pending */
#define SYSCFG_ITLINE5_EXTI1          (1 << 1)  /* Bit 1: EXTI line 1 interrupt request pending */

/* SYSCFG interrupt line 6 status register */

#define SYSCFG_ITLINE6_EXTI2          (1 << 0)  /* Bit 0: EXTI line 2 interrupt request pending */
#define SYSCFG_ITLINE6_EXTI3          (1 << 1)  /* Bit 1: EXTI line 3 interrupt request pending */

/* SYSCFG interrupt line 7 status register */

#define SYSCFG_ITLINE7_EXTI4          (1 << 0)  /* Bit 0: EXTI line 4 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI5          (1 << 1)  /* Bit 1: EXTI line 5 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI6          (1 << 2)  /* Bit 2: EXTI line 6 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI7          (1 << 3)  /* Bit 3: EXTI line 7 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI8          (1 << 4)  /* Bit 4: EXTI line 8 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI9          (1 << 5)  /* Bit 5: EXTI line 9 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI10         (1 << 6)  /* Bit 6: EXTI line 10 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI11         (1 << 7)  /* Bit 7: EXTI line 11 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI12         (1 << 8)  /* Bit 8: EXTI line 12 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI13         (1 << 9)  /* Bit 9: EXTI line 13 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI14         (1 << 10) /* Bit 10: EXTI line 14 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI15         (1 << 11) /* Bit 11: EXTI line 15 interrupt request pending */

/* SYSCFG interrupt line 8 status register */

#define SYSCFG_ITLINE8_TCS_MCE        (1 << 0)  /* Bit 0: Touch sensing controller max count error interrupt request pending */
#define SYSCFG_ITLINE8_TCS_EOA        (1 << 1)  /* Bit 1: Touch sensing controller end of acquisition interrupt request pending */

/* SYSCFG interrupt line 9 status register */

#define SYSCFG_ITLINE9_DMA1_CH1       (1 << 0)  /* Bit 0: DMA1 channel 1 interrupt request pending */

/* SYSCFG interrupt line 10 status register */

#define SYSCFG_ITLINE10_DMA1_CH2       (1 << 0)  /* Bit 0: DMA1 channel 2 interrupt request pending */
#define SYSCFG_ITLINE10_DMA1_CH3       (1 << 1)  /* Bit 1: DMA1 channel 3 interrupt request pending */
#define SYSCFG_ITLINE10_DMA2_CH1       (1 << 2)  /* Bit 0: DMA2 channel 1 interrupt request pending */
#define SYSCFG_ITLINE10_DMA2_CH2       (1 << 3)  /* Bit 1: DMA2 channel 2 interrupt request pending */

/* SYSCFG interrupt line 11 status register */

#define SYSCFG_ITLINE11_DMA1_CH4       (1 << 0)  /* Bit 0: DMA1 channel 4 interrupt request pending */
#define SYSCFG_ITLINE11_DMA1_CH5       (1 << 1)  /* Bit 1: DMA1 channel 5 interrupt request pending */
#define SYSCFG_ITLINE11_DMA1_CH6       (1 << 2)  /* Bit 2: DMA1 channel 6 interrupt request pending */
#define SYSCFG_ITLINE11_DMA1_CH7       (1 << 3)  /* Bit 3: DMA1 channel 7 interrupt request pending */
#define SYSCFG_ITLINE11_DMA2_CH3       (1 << 4)  /* Bit 4: DMA2 channel 3 interrupt request pending */
#define SYSCFG_ITLINE11_DMA2_CH4       (1 << 5)  /* Bit 5: DMA2 channel 4 interrupt request pending */
#define SYSCFG_ITLINE11_DMA2_CH5       (1 << 6)  /* Bit 6: DMA2 channel 5 interrupt request pending */

/* SYSCFG interrupt line 12 status register */

#define SYSCFG_ITLINE12_ADC            (1 << 0)  /* Bit 0: ADC interrupt request pending */
#define SYSCFG_ITLINE12_COMP1          (1 << 1)  /* Bit 1: Comparator 1 interrupt request pending */
#define SYSCFG_ITLINE12_COMP2          (1 << 2)  /* Bit 2: Comparator 2 interrupt request pending */

/* SYSCFG interrupt line 13 status register */

#define SYSCFG_ITLINE13_TIM1_CCU       (1 << 0)  /* Bit 0: TIM1 commutation interrupt request pending */
#define SYSCFG_ITLINE13_TIM1_TRG       (1 << 1)  /* Bit 1: TIM1 triggerinterrupt request pending */
#define SYSCFG_ITLINE13_TIM1_UPD       (1 << 2)  /* Bit 2: TIM1 update interrupt request pending */
#define SYSCFG_ITLINE13_TIM1_BRK       (1 << 3)  /* Bit 3: TIM1 break interrupt request pending */

/* SYSCFG interrupt line 14 status register */

#define SYSCFG_ITLINE14_TIM1_CC        (1 << 0)  /* Bit 0: TIM1 capture compare interrupt request pending */

/* SYSCFG interrupt line 15 status register */

#define SYSCFG_ITLINE15_TIM2           (1 << 0)  /* Bit 0: Timer 2 interrupt request pending */

/* SYSCFG interrupt line 16 status register */

#define SYSCFG_ITLINE16_TIM3           (1 << 0)  /* Bit 0: Timer 3 interrupt request pending */

/* SYSCFG interrupt line 17 status register */

#define SYSCFG_ITLINE17_TIM6           (1 << 0)  /* Bit 0: Timer 6 interrupt request pending */
#define SYSCFG_ITLINE17_DAC            (1 << 1)  /* Bit 1: DAC underrun interrupt request pending */

/* SYSCFG interrupt line 18 status register */

#define SYSCFG_ITLINE18_TIM7           (1 << 0)  /* Bit 0: Timer 7 interrupt request pending */

/* SYSCFG interrupt line 19 status register */

#define SYSCFG_ITLINE19_TIM14          (1 << 0)  /* Bit 0: Timer 14 interrupt request pending */

/* SYSCFG interrupt line 20 status register */

#define SYSCFG_ITLINE20_TIM15          (1 << 0)  /* Bit 0: Timer 15 interrupt request pending */

/* SYSCFG interrupt line 21 status register */

#define SYSCFG_ITLINE21_TIM16          (1 << 0)  /* Bit 0: Timer 16 interrupt request pending */

/* SYSCFG interrupt line 22 status register */

#define SYSCFG_ITLINE22_TIM17          (1 << 0)  /* Bit 0: Timer 17 interrupt request pending */

/* SYSCFG interrupt line 23 status register */

#define SYSCFG_ITLINE23_I2C1           (1 << 0)  /* Bit 0: I2C1 interrupt request pending, combined with EXTI line 23 */

/* SYSCFG interrupt line 24 status register */

#define SYSCFG_ITLINE24_I2C2           (1 << 0)  /* Bit 0: I2C2 interrupt request pending */

/* SYSCFG interrupt line 25 status register */

#define SYSCFG_ITLINE25_SPI1           (1 << 0)  /* Bit 0: SPI1 interrupt request pending */

/* SYSCFG interrupt line 26 status register */

#define SYSCFG_ITLINE26_SPI2           (1 << 0)  /* Bit 0: SPI2 interrupt request pending */

/* SYSCFG interrupt line 27 status register */

#define SYSCFG_ITLINE27_USART1         (1 << 0)  /* Bit 0: USART1 interrupt request pending */

/* SYSCFG interrupt line 28 status register */

#define SYSCFG_ITLINE28_USART2         (1 << 0)  /* Bit 0: USART2 interrupt request pending */

/* SYSCFG interrupt line 29 status register */

#define SYSCFG_ITLINE29_USART3         (1 << 0)  /* Bit 0: USART3 interrupt request pending */
#define SYSCFG_ITLINE29_USART4         (1 << 1)  /* Bit 1: USART4 interrupt request pending */
#define SYSCFG_ITLINE29_USART5         (1 << 2)  /* Bit 2: USART5 interrupt request pending */
#define SYSCFG_ITLINE29_USART6         (1 << 3)  /* Bit 3: USART6 interrupt request pending */
#define SYSCFG_ITLINE29_USART7         (1 << 4)  /* Bit 4: USART7 interrupt request pending */
#define SYSCFG_ITLINE29_USART8         (1 << 5)  /* Bit 5: USART8 interrupt request pending */

/* SYSCFG interrupt line 30 status register */

#define SYSCFG_ITLINE30_CEC            (1 << 0)  /* Bit 0: CEC interrupt request pending, combined with EXTI line 27 */
#define SYSCFG_ITLINE30_CAN            (1 << 1)  /* Bit 1: CAN interrupt request pending */

#endif /* __ARCH_ARM_SRC_STM32F0_CHIP_STM32F0_SYSCFG_H */
