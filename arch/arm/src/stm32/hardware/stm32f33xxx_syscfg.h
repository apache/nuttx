/****************************************************************************************************
 * arch/arm/src/stm32/hardware/stm32f33xxx_syscfg.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Modified for STM32F334 by Mateusz Szafoni <raiden00@railab.me>
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32F33XXX_SYSCFG_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32F33XXX_SYSCFG_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#ifdef CONFIG_STM32_STM32F33XX

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define STM32_SYSCFG_CFGR1_OFFSET      0x0000 /* SYSCFG configuration register 1 */
#define STM32_SYSCFG_RCR_OFFSET        0x0004 /* SYSCFG CCM SRAM protection register */

#define STM32_SYSCFG_EXTICR_OFFSET(p)  (0x0008 + ((p) & 0x000c)) /* Registers are displaced by 4! */
#define STM32_SYSCFG_EXTICR1_OFFSET    0x0008 /* SYSCFG external interrupt configuration register 1 */
#define STM32_SYSCFG_EXTICR2_OFFSET    0x000c /* SYSCFG external interrupt configuration register 2 */
#define STM32_SYSCFG_EXTICR3_OFFSET    0x0010 /* SYSCFG external interrupt configuration register 3 */
#define STM32_SYSCFG_EXTICR4_OFFSET    0x0014 /* SYSCFG external interrupt configuration register 4 */

#define STM32_SYSCFG_CFGR2_OFFSET      0x0018 /* SYSCFG configuration register 2 */
#define STM32_SYSCFG_CFGR3_OFFSET      0x0050 /* SYSCFG configuration register 3 */

/* Register Addresses *******************************************************************************/

#define STM32_SYSCFG_CFGR1             (STM32_SYSCFG_BASE+STM32_SYSCFG_CFGR1_OFFSET)
#define STM32_SYSCFG_RCR               (STM32_SYSCFG_BASE+STM32_SYSCFG_RCR_OFFSET)

#define STM32_SYSCFG_EXTICR(p)         (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR_OFFSET(p))
#define STM32_SYSCFG_EXTICR1           (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR1_OFFSET)
#define STM32_SYSCFG_EXTICR2           (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR2_OFFSET)
#define STM32_SYSCFG_EXTICR3           (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR3_OFFSET)
#define STM32_SYSCFG_EXTICR4           (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR4_OFFSET)

#define STM32_SYSCFG_CFGR2             (STM32_SYSCFG_BASE+STM32_SYSCFG_CFGR2_OFFSET)
#define STM32_SYSCFG_CFGR3             (STM32_SYSCFG_BASE+STM32_SYSCFG_CFGR3_OFFSET)

/* Register Bitfield Definitions ********************************************************************/

/* SYSCFG memory remap register */

#define SYSCFG_CFGR1_MEMMODE_SHIFT     (0)       /* Bits 1:0 MEM_MODE: Memory mapping selection */
#define SYSCFG_CFGR1_MEMMODE_MASK      (3 << SYSCFG_CFGR1_MEMMODE_SHIFT)
#  define SYSCFG_CFGR1_MEMMODE_FLASH   (0 << SYSCFG_CFGR1_MEMMODE_SHIFT) /* 00: Main Flash at 0x00000000 */
#  define SYSCFG_CFGR1_MEMMODE_SYSTEM  (1 << SYSCFG_CFGR1_MEMMODE_SHIFT) /* 01: System Flash at 0x00000000 */
#  define SYSCFG_CFGR1_MEMMODE_SRAM    (3 << SYSCFG_CFGR1_MEMMODE_SHIFT) /* 11: Embedded SRAM at 0x00000000 */
#define SYSCFG_CFGR1_TIM1_ITR3RMP      (1 << 6)  /* Bit 6:  Timer 1 ITR3 selection */
#define SYSCFG_CFGR1_DAC_TRIGRMP       (1 << 7)  /* Bit 7:  DAC trigger remap (when TSEL = 001) */
#define SYSCFG_CFGR1_TIM16_DMARMP      (1 << 11) /* Bit 11: TIM16 DMA request remapping bit */
#define SYSCFG_CFGR1_TIM17_DMARMP      (1 << 12) /* Bit 12: TIM17 DMA request remapping bit */
#define SYSCFG_CFGR1_TIM6_DMARMP       (1 << 13) /* Bit 13: TIM6 DMA remap, or */
#define SYSCFG_CFGR1_DAC1CH1_DMARMP    (1 << 13) /* Bit 13: DAC1 channel1 DMA remap */
#define SYSCFG_CFGR1_TIM7_DMARMP       (1 << 14) /* Bit 14: TIM7 DMA remap */
#define SYSCFG_CFGR1_DAC1CH2_DMARMP    (1 << 14) /* Bit 14: DAC1 channel2 DMA remap */
#define SYSCFG_CFGR1_DAC2CH1_DMARMP    (1 << 15) /* Bit 15: DAC2 channel1 DMA remap */
#define SYSCFG_CFGR1_I2C_PBXFMP_SHIFT  (16)      /* Bits 16-19: Fast Mode Plus (FM+) driving capability */
#define SYSCFG_CFGR1_I2C_PBXFMP_MASK   (15 << SYSCFG_CFGR1_I2C_PBXFMP_SHIFT)
#define SYSCFG_CFGR1_I2C1_FMP          (1 << 20) /* Bit 20: I2C1 fast mode Plus driving capability */
#define SYSCFG_CFGR1_I2C2_FMP          (1 << 21) /* Bit 21: I2C2 fast mode Plus driving capability */
#define SYSCFG_CFGR1_ENCMODE_SHIFT     (22)       /* Bits 22-23: Encoder mode */
#define SYSCFG_CFGR1_ENCMODE_MASK      (3 << SYSCFG_CFGR1_ENCMODE_SHIFT)
#  define SYSCFG_CFGR1_ENCMODE_NONE    (0 << SYSCFG_CFGR1_ENCMODE_SHIFT) /* No redirection */
#  define SYSCFG_CFGR1_ENCMODE_TIM2    (1 << SYSCFG_CFGR1_ENCMODE_SHIFT) /* TIM2 I2C1-2 -> TIM15 IC1/2 */
#  define SYSCFG_CFGR1_ENCMODE_TIM3    (2 << SYSCFG_CFGR1_ENCMODE_SHIFT) /* TIM3 I2C1-2 -> TIM15 IC1/2 */
#  define SYSCFG_CFGR1_ENCMODE_TIM4    (3 << SYSCFG_CFGR1_ENCMODE_SHIFT) /* TIM4 I2C1-2 -> TIM15 IC1/2 */
#define SYSCFG_CFGR1_FPUIE_SHIFT       (26)       /* Bits 26-31: Floating Point Unit interrupts enable bits */
#define SYSCFG_CFGR1_FPUIE_MASK        (63 << SYSCFG_CFGR1_FPUIE_SHIFT)
#  define SYSCFG_CFGR1_FPUIE_INVALIDOP (1 << SYSCFG_CFGR1_FPUIE_SHIFT)  /* Invalid operation interrupt enable */
#  define SYSCFG_CFGR1_FPUIE_DIVZERO   (2 << SYSCFG_CFGR1_FPUIE_SHIFT)  /* Divide-by-zero interrupt enable */
#  define SYSCFG_CFGR1_FPUIE_UNDERFLOW (4 << SYSCFG_CFGR1_FPUIE_SHIFT)  /* Underflow interrupt enable */
#  define SYSCFG_CFGR1_FPUIE_OVERFLOW  (8 << SYSCFG_CFGR1_FPUIE_SHIFT)  /* Overflow interrupt enable */
#  define SYSCFG_CFGR1_FPUIE_DENORMAL  (16 << SYSCFG_CFGR1_FPUIE_SHIFT) /* Input denormal interrupt enable */
#  define SYSCFG_CFGR1_FPUIE_INEXACT   (32 << SYSCFG_CFGR1_FPUIE_SHIFT) /* Inexact interrupt enable */

/* SYSCFG CCM SRAM protection register */

#define SYSCFG_RCR(page)               (1 << (page)) /* Bit n: Write protection page n */

/* SYSCFG external interrupt configuration register 1-4 */

#define SYSCFG_EXTICR_PORTA           (0)       /* 0000: PA[x] pin */
#define SYSCFG_EXTICR_PORTB           (1)       /* 0001: PB[x] pin */
#define SYSCFG_EXTICR_PORTC           (2)       /* 0010: PC[x] pin */
#define SYSCFG_EXTICR_PORTD           (3)       /* 0011: PD[x] pin */
#define SYSCFG_EXTICR_PORTE           (4)       /* 0100: PE[x] pin */

#define SYSCFG_EXTICR_PORT_MASK       (15)
#define SYSCFG_EXTICR_EXTI_SHIFT(g)   (((g) & 3) << 2)
#define SYSCFG_EXTICR_EXTI_MASK(g)    (SYSCFG_EXTICR_PORT_MASK << (SYSCFG_EXTICR_EXTI_SHIFT(g)))

#define SYSCFG_EXTICR1_EXTI0_SHIFT    (0)       /* Bits 0-3: EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI0_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI0_SHIFT)
#define SYSCFG_EXTICR1_EXTI1_SHIFT    (4)       /* Bits 4-7: EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI1_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI1_SHIFT)
#define SYSCFG_EXTICR1_EXTI2_SHIFT    (8)       /* Bits 8-11: EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI2_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI2_SHIFT)
#define SYSCFG_EXTICR1_EXTI3_SHIFT    (12)      /* Bits 12-15: EXTI 3 configuration */
#define SYSCFG_EXTICR1_EXTI3_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI3_SHIFT)

#define SYSCFG_EXTICR2_EXTI4_SHIFT    (0)       /* Bits 0-3: EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI4_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI4_SHIFT)
#define SYSCFG_EXTICR2_EXTI5_SHIFT    (4)       /* Bits 4-7: EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI5_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI5_SHIFT)
#define SYSCFG_EXTICR2_EXTI6_SHIFT    (8)       /* Bits 8-11: EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI6_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI6_SHIFT)
#define SYSCFG_EXTICR2_EXTI7_SHIFT    (12)      /* Bits 12-15: EXTI 7 configuration */
#define SYSCFG_EXTICR2_EXTI7_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI7_SHIFT)

#define SYSCFG_EXTICR3_EXTI8_SHIFT    (0)       /* Bits 0-3: EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI8_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI8_SHIFT)
#define SYSCFG_EXTICR3_EXTI9_SHIFT    (4)       /* Bits 4-7: EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI9_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI9_SHIFT)
#define SYSCFG_EXTICR3_EXTI10_SHIFT   (8)       /* Bits 8-11: EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI10_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI10_SHIFT)
#define SYSCFG_EXTICR3_EXTI11_SHIFT   (12)      /* Bits 12-15: EXTI 11 configuration */
#define SYSCFG_EXTICR3_EXTI11_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI11_SHIFT)

#define SYSCFG_EXTICR4_EXTI12_SHIFT   (0)       /* Bits 0-3: EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI12_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI12_SHIFT)
#define SYSCFG_EXTICR4_EXTI13_SHIFT   (4)       /* Bits 4-7: EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI13_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI13_SHIFT)
#define SYSCFG_EXTICR4_EXTI14_SHIFT   (8)       /* Bits 8-11: EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI14_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI14_SHIFT)
#define SYSCFG_EXTICR4_EXTI15_SHIFT   (12)      /* Bits 12-15: EXTI 15 configuration */
#define SYSCFG_EXTICR4_EXTI15_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI15_SHIFT)

/* SYSCFG configuration register 2 */

#define SYSCFG_CFGR2_LOCKUPLOCK       (1 << 0)  /* Bit 0: Cortex-M4 Hardfault output bit enable */
#define SYSCFG_CFGR2_SRAM_PARITYLOCK  (1 << 1)  /* Bit 1: RAM parity lock */
#define SYSCFG_CFGR2_PVDLOCK          (1 << 2)  /* Bit 2: PVD lock enable */
#define SYSCFG_CFGR2_BYPADDPAR        (1 << 4)  /* Bit 4: Bypass address bit 29 in parity calculation */
#define SYSCFG_CFGR2_SRAM_PEF         (1 << 8)  /* Bit 8: SRAM parity error */

/* SYSCFG configuration register 3 */

#define SYSCFG_CFGR3_SPI1_RX_DMA_RMP_SHIFT   (0)  /* Bits 0-1: SPI1_RX_DMA remap */
#define SYSCFG_CFGR3_SPI1_RX_DMA_RMP_MASK    (3 << SYSCFG_CFGR3_SPI1_RX_DMA_RMP_SHIFT)
#  define SYSCFG_CFGR3_SPI1_RX_DMA_RMP_0     (0 << SYSCFG_CFGR3_SPI1_RX_DMA_RMP_SHIFT) /* 00: SPI1_RX mapped on DMA1CH2 */
#  define SYSCFG_CFGR3_SPI1_RX_DMA_RMP_1     (1 << SYSCFG_CFGR3_SPI1_RX_DMA_RMP_SHIFT) /* 01: SPI1_RX mapped on DMA1CH2 */
#  define SYSCFG_CFGR3_SPI1_RX_DMA_RMP_2     (2 << SYSCFG_CFGR3_SPI1_RX_DMA_RMP_SHIFT) /* 10: SPI1_RX mapped on DMA1CH6 */
#  define SYSCFG_CFGR3_SPI1_RX_DMA_RMP_3     (3 << SYSCFG_CFGR3_SPI1_RX_DMA_RMP_SHIFT) /* 11: SPI1_RX mapped on DMA1CH2 */
#define SYSCFG_CFGR3_SPI1_TX_DMA_RMP_SHIFT   (2)  /* Bits 2-3: SPI1_TX_DMA remap */
#define SYSCFG_CFGR3_SPI1_TX_DMA_RMP_MASK    (3 << SYSCFG_CFGR3_SPI1_TX_DMA_RMP_SHIFT)
#  define SYSCFG_CFGR3_SPI1_TX_DMA_RMP_0     (0 << SYSCFG_CFGR3_SPI1_TX_DMA_RMP_SHIFT) /* 00: SPI1_TX mapped on DMA1CH3 */
#  define SYSCFG_CFGR3_SPI1_TX_DMA_RMP_1     (1 << SYSCFG_CFGR3_SPI1_TX_DMA_RMP_SHIFT) /* 01: SPI1_TX mapped on DMA1CH5 */
#  define SYSCFG_CFGR3_SPI1_TX_DMA_RMP_2     (2 << SYSCFG_CFGR3_SPI1_TX_DMA_RMP_SHIFT) /* 10: SPI1_TX mapped on DMA1CH7 */
#  define SYSCFG_CFGR3_SPI1_TX_DMA_RMP_3     (3 << SYSCFG_CFGR3_SPI1_TX_DMA_RMP_SHIFT) /* 11: SPI1_TX mapped on DMA1CH3 */
#define SYSCFG_CFGR3_I2C1_RX_DMA_RMP_SHIFT   (4)  /* Bits 4-5: I2C1_RX_DMA remap */
#define SYSCFG_CFGR3_I2C1_RX_DMA_RMP_MASK    (3 << SYSCFG_CFGR3_I2C1_RX_DMA_RMP_SHIFT)
#  define SYSCFG_CFGR3_I2C1_RX_DMA_RMP_0     (0 << SYSCFG_CFGR3_I2C1_RX_DMA_RMP_SHIFT) /* 00: I2C1_RX mapped on DMA1CH7 */
#  define SYSCFG_CFGR3_I2C1_RX_DMA_RMP_1     (1 << SYSCFG_CFGR3_I2C1_RX_DMA_RMP_SHIFT) /* 01: I2C1_RX mapped on DMA1CH3 */
#  define SYSCFG_CFGR3_I2C1_RX_DMA_RMP_2     (2 << SYSCFG_CFGR3_I2C1_RX_DMA_RMP_SHIFT) /* 10: I2C1_RX mapped on DMA1CH5 */
#  define SYSCFG_CFGR3_I2C1_RX_DMA_RMP_3     (3 << SYSCFG_CFGR3_I2C1_RX_DMA_RMP_SHIFT) /* 11: I2C1_RX mapped on DMA1CH7 */
#define SYSCFG_CFGR3_I2C1_TX_DMA_RMP_SHIFT   (6)  /* Bits 6-7: I2C1_TX_DMA remap */
#define SYSCFG_CFGR3_I2C1_TX_DMA_RMP_MASK    (3 << SYSCFG_CFGR3_I2C1_TX_DMA_RMP_SHIFT)
#  define SYSCFG_CFGR3_I2C1_TX_DMA_RMP_0     (0 << SYSCFG_CFGR3_I2C1_TX_DMA_RMP_SHIFT) /* 00: I2C1_TX mapped on DMA1CH6 */
#  define SYSCFG_CFGR3_I2C1_TX_DMA_RMP_1     (1 << SYSCFG_CFGR3_I2C1_TX_DMA_RMP_SHIFT) /* 01: I2C1_TX mapped on DMA1CH2 */
#  define SYSCFG_CFGR3_I2C1_TX_DMA_RMP_2     (2 << SYSCFG_CFGR3_I2C1_TX_DMA_RMP_SHIFT) /* 10: I2C1_TX mapped on DMA1CH4 */
#  define SYSCFG_CFGR3_I2C1_TX_DMA_RMP_3     (3 << SYSCFG_CFGR3_I2C1_TX_DMA_RMP_SHIFT) /* 11: I2C1_TX mapped on DMA1CH6 */
#define SYSCFG_CFGR3_ADC2_DMA_RMP            (8)  /* Bits 8: ADC2 mapped on DMA1CH1 remap */
#define SYSCFG_CFGR3_DAC1_TRIG3_RMP          (1 << 16)  /* Bit 16: HRTIM1_DAC1_TRIG1 remap */
#define SYSCFG_CFGR3_DAC1_TRIG3_RMP          (1 << 16)  /* Bit 16: HRTIM1_DAC1_TRIG1 remap */
#define SYSCFG_CFGR3_DAC1_TRIG5_RMP          (1 << 17)  /* Bit 17: HRTIM1_DAC1_TRIG2 remap */

#endif /* CONFIG_STM32_STM32F33XX */
#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32F33XXX_SYSCFG_H */
