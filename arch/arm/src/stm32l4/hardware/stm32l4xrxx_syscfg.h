/****************************************************************************************************
 * arch/arm/src/stm32l4/hardware/stm32l4xrxx_syscfg.h
 *
 *   Copyright (C) 2014-2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4XRXX_SYSCFG_H
#define __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4XRXX_SYSCFG_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#if defined(CONFIG_STM32L4_STM32L4XR)

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define STM32L4_SYSCFG_MEMRMP_OFFSET    0x0000 /* SYSCFG memory remap register */
#define STM32L4_SYSCFG_CFGR1_OFFSET     0x0004 /* SYSCFG configuration register 1 */
#define STM32L4_SYSCFG_EXTICR_OFFSET(p) (0x0008 + ((p) & 0x000c)) /* Registers are displaced by 4! */
#define STM32L4_SYSCFG_EXTICR1_OFFSET   0x0008 /* SYSCFG external interrupt configuration register 1 */
#define STM32L4_SYSCFG_EXTICR2_OFFSET   0x000c /* SYSCFG external interrupt configuration register 2 */
#define STM32L4_SYSCFG_EXTICR3_OFFSET   0x0010 /* SYSCFG external interrupt configuration register 3 */
#define STM32L4_SYSCFG_EXTICR4_OFFSET   0x0014 /* SYSCFG external interrupt configuration register 4 */
#define STM32L4_SYSCFG_SCSR_OFFSET      0x0018 /* SYSCFG SRAM2 control and status register */
#define STM32L4_SYSCFG_CFGR2_OFFSET     0x001c /* SYSCFG configuration register 2 */
#define STM32L4_SYSCFG_SWPR_OFFSET      0x0020 /* SYSCFG SRAM2 write protection register */
#define STM32L4_SYSCFG_SKR_OFFSET       0x0024 /* SYSCFG SRAM2 key register */
#define STM32L4_SYSCFG_SWPR2_OFFSET     0x0028 /* SYSCFG SRAM2 write protection register 2 */

/* Register Addresses *******************************************************************************/

#define STM32L4_SYSCFG_MEMRMP           (STM32L4_SYSCFG_BASE+STM32L4_SYSCFG_MEMRMP_OFFSET)
#define STM32L4_SYSCFG_CFGR1            (STM32L4_SYSCFG_BASE+STM32L4_SYSCFG_CFGR1_OFFSET)
#define STM32L4_SYSCFG_EXTICR(p)        (STM32L4_SYSCFG_BASE+STM32L4_SYSCFG_EXTICR_OFFSET(p))
#define STM32L4_SYSCFG_EXTICR1          (STM32L4_SYSCFG_BASE+STM32L4_SYSCFG_EXTICR1_OFFSET)
#define STM32L4_SYSCFG_EXTICR2          (STM32L4_SYSCFG_BASE+STM32L4_SYSCFG_EXTICR2_OFFSET)
#define STM32L4_SYSCFG_EXTICR3          (STM32L4_SYSCFG_BASE+STM32L4_SYSCFG_EXTICR3_OFFSET)
#define STM32L4_SYSCFG_EXTICR4          (STM32L4_SYSCFG_BASE+STM32L4_SYSCFG_EXTICR4_OFFSET)
#define STM32L4_SYSCFG_SCSR             (STM32L4_SYSCFG_BASE+STM32L4_SYSCFG_SCSR_OFFSET)
#define STM32L4_SYSCFG_CFGR2            (STM32L4_SYSCFG_BASE+STM32L4_SYSCFG_CFGR2_OFFSET)
#define STM32L4_SYSCFG_SWPR             (STM32L4_SYSCFG_BASE+STM32L4_SYSCFG_SWPR_OFFSET)
#define STM32L4_SYSCFG_SKR              (STM32L4_SYSCFG_BASE+STM32L4_SYSCFG_SKR_OFFSET)
#define STM32L4_SYSCFG_SWPR2            (STM32L4_SYSCFG_BASE+STM32L4_SYSCFG_SWPR2_OFFSET)

/* Register Bitfield Definitions ********************************************************************/

/* SYSCFG memory remap register */

#define SYSCFG_MEMRMP_SHIFT           (0)       /* Bits 2:0 MEM_MODE: Memory mapping selection */
#define SYSCFG_MEMRMP_MASK            (7 << SYSCFG_MEMRMP_SHIFT)
#  define SYSCFG_MEMRMP_FLASH         (0 << SYSCFG_MEMRMP_SHIFT) /* 000: Main Flash memory mapped at 0x0000 0000 */
#  define SYSCFG_MEMRMP_SYSTEM        (1 << SYSCFG_MEMRMP_SHIFT) /* 001: System Flash memory mapped at 0x0000 0000 */
#  define SYSCFG_MEMRMP_FMC           (2 << SYSCFG_MEMRMP_SHIFT) /* 010: FSMC Bank1 (NOR/PSRAM 1 and 2) mapped at 0x0000 0000 */
#  define SYSCFG_MEMRMP_SRAM          (3 << SYSCFG_MEMRMP_SHIFT) /* 011: SRAM1 mapped at 0x0000 0000 */
#  define SYSCFG_MEMRMP_OCTOSPI1      (4 << SYSCFG_MEMRMP_SHIFT) /* 100: OCTOSPI1 mapped at 0x0000 0000 */
#  define SYSCFG_MEMRMP_OCTOSPI2      (5 << SYSCFG_MEMRMP_SHIFT) /* 101: OCTOSPI2 mapped at 0x0000 0000 */
#define SYSCFG_FBMODE                 (1 << 8) /* Bit 8: Flash Bank mode selection */

/* SYSCFG configuration register 1 */

#define SYSCFG_CFGR1_FWDIS            (1 <<  0) /* Bit  0: Firewall disable */
#define SYSCFG_CFGR1_BOOSTEN          (1 <<  8) /* Bit  8: I/O analog switch voltage booster enable (use when vdd is low) */
#define SYSCFG_CFGR1_ANASWVDD         (1 <<  9) /* Bit  9: GPIO analog switch control voltage selection */
#define SYSCFG_CFGR1_I2C_PB6_FMP      (1 << 16) /* Bit 16: Fast-mode Plus (Fm+) driving capability activation on PB6 */
#define SYSCFG_CFGR1_I2C_PB7_FMP      (1 << 17) /* Bit 17: Fast-mode Plus (Fm+) driving capability activation on PB7 */
#define SYSCFG_CFGR1_I2C_PB8_FMP      (1 << 18) /* Bit 18: Fast-mode Plus (Fm+) driving capability activation on PB8 */
#define SYSCFG_CFGR1_I2C_PB9_FMP      (1 << 19) /* Bit 19: Fast-mode Plus (Fm+) driving capability activation on PB9 */
#define SYSCFG_CFGR1_I2C1_FMP         (1 << 20) /* Bit 20: I2C1 Fast-mode Plus (Fm+) driving capability activation */
#define SYSCFG_CFGR1_I2C2_FMP         (1 << 21) /* Bit 21: I2C2 Fast-mode Plus (Fm+) driving capability activation */
#define SYSCFG_CFGR1_I2C3_FMP         (1 << 22) /* Bit 22: I2C3 Fast-mode Plus (Fm+) driving capability activation */
#define SYSCFG_CFGR1_I2C4_FMP         (1 << 23) /* Bit 23: I2C4 Fast-mode Plus (Fm+) driving capability activation */
#define SYSCFG_CFGR1_FPU_IE0          (1 << 26) /* Bit 26: FPU Invalid operation interrupt enable */
#define SYSCFG_CFGR1_FPU_IE1          (1 << 27) /* Bit 27: FPU Divide-by-zero interrupt enable */
#define SYSCFG_CFGR1_FPU_IE2          (1 << 28) /* Bit 28: FPU Underflow interrupt enable */
#define SYSCFG_CFGR1_FPU_IE3          (1 << 29) /* Bit 29: FPU Overflow interrupt enable */
#define SYSCFG_CFGR1_FPU_IE4          (1 << 30) /* Bit 30: FPU Input denormal interrupt enable */
#define SYSCFG_CFGR1_FPU_IE5          (1 << 31) /* Bit 31: FPU Inexact interrupt enable */

/* SYSCFG external interrupt configuration register 1-4 */

#define SYSCFG_EXTICR_PORTA           (0)       /* 0000: PA[x] pin */
#define SYSCFG_EXTICR_PORTB           (1)       /* 0001: PB[x] pin */
#define SYSCFG_EXTICR_PORTC           (2)       /* 0010: PC[x] pin */
#define SYSCFG_EXTICR_PORTD           (3)       /* 0011: PD[x] pin */
#define SYSCFG_EXTICR_PORTE           (4)       /* 0100: PE[x] pin */
#define SYSCFG_EXTICR_PORTF           (5)       /* 0101: PF[C] pin */
#define SYSCFG_EXTICR_PORTG           (6)       /* 0110: PG[x] pin */
#define SYSCFG_EXTICR_PORTH           (7)       /* 0111: PH[x] pin */
#define SYSCFG_EXTICR_PORTI           (8)       /* 1000: PI[x] pin */

#define SYSCFG_EXTICR_PORT_MASK       (15)
#define SYSCFG_EXTICR_EXTI_SHIFT(g)   (((g) & 3) << 2)
#define SYSCFG_EXTICR_EXTI_MASK(g)    (SYSCFG_EXTICR_PORT_MASK << (SYSCFG_EXTICR_EXTI_SHIFT(g)))

#define SYSCFG_EXTICR1_EXTI0_SHIFT    (0)       /* Bits 0-2: EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI0_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI0_SHIFT)
#define SYSCFG_EXTICR1_EXTI1_SHIFT    (4)       /* Bits 4-6: EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI1_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI1_SHIFT)
#define SYSCFG_EXTICR1_EXTI2_SHIFT    (8)       /* Bits 8-10: EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI2_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI2_SHIFT)
#define SYSCFG_EXTICR1_EXTI3_SHIFT    (12)      /* Bits 12-14: EXTI 3 configuration */
#define SYSCFG_EXTICR1_EXTI3_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI3_SHIFT)

#define SYSCFG_EXTICR2_EXTI4_SHIFT    (0)       /* Bits 0-2: EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI4_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI4_SHIFT)
#define SYSCFG_EXTICR2_EXTI5_SHIFT    (4)       /* Bits 4-6: EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI5_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI5_SHIFT)
#define SYSCFG_EXTICR2_EXTI6_SHIFT    (8)       /* Bits 8-10: EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI6_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI6_SHIFT)
#define SYSCFG_EXTICR2_EXTI7_SHIFT    (12)      /* Bits 12-14: EXTI 7 configuration */
#define SYSCFG_EXTICR2_EXTI7_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI7_SHIFT)

#define SYSCFG_EXTICR3_EXTI8_SHIFT    (0)       /* Bits 0-2: EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI8_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI8_SHIFT)
#define SYSCFG_EXTICR3_EXTI9_SHIFT    (4)       /* Bits 4-6: EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI9_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI9_SHIFT)
#define SYSCFG_EXTICR3_EXTI10_SHIFT   (8)       /* Bits 8-10: EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI10_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI10_SHIFT)
#define SYSCFG_EXTICR3_EXTI11_SHIFT   (12)      /* Bits 12-14: EXTI 11 configuration */
#define SYSCFG_EXTICR3_EXTI11_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI11_SHIFT)

#define SYSCFG_EXTICR4_EXTI12_SHIFT   (0)       /* Bits 0-2: EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI12_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI12_SHIFT)
#define SYSCFG_EXTICR4_EXTI13_SHIFT   (4)       /* Bits 4-6: EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI13_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI13_SHIFT)
#define SYSCFG_EXTICR4_EXTI14_SHIFT   (8)       /* Bits 8-10: EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI14_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI14_SHIFT)
#define SYSCFG_EXTICR4_EXTI15_SHIFT   (12)      /* Bits 12-14: EXTI 15 configuration */
#define SYSCFG_EXTICR4_EXTI15_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI15_SHIFT)

/* SYSCFG SRAM2 control and status register */

#define SYSCFG_SCSR_SRAM2ER           (1 <<  0) /* Bit  0: SRAM2 Erase */
#define SYSCFG_SCSR_SRAM2BSY          (1 <<  1) /* Bit  1: SRAM2 busy in erase operation */

/* SYSCFG configuration register 2 */

#define SYSCFG_CFGR2_CLL              (1 <<  0) /* Bit  0: Cortex-M4 LOCKUP (Hardfault) output enable (TIMx break enable, see refman) */
#define SYSCFG_CFGR2_SPL              (1 <<  1) /* Bit  1: SRAM2 parity lock enable (same) */
#define SYSCFG_CFGR2_PVDL             (1 <<  2) /* Bit  2: PVD lock enable (same) */
#define SYSCFG_CFGR2_ECCL             (1 <<  3) /* Bit  3: ECC lock enable (same) */
#define SYSCFG_CFGR2_SPF              (1 <<  8) /* Bit  8: SRAM2 parity error flag */

/* SYSCFG SRAM2 write protection register */
/* There is one bit per SRAM2 page (0 to 31) */

/* SYSCFG SRAM2 key register */

#define SYSCFG_SKR_SHIFT              0
#define SYSCFG_SKR_MASK               (0xFF << SYSCFG_SKR_SHIFT)

/* SYSCFG SRAM2 write protection register 2 */
/* There is one bit per SRAM2 page (32 to 63) */

#endif /* CONFIG_STM32L4_STM32L4XR */
#endif /* __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4XRXX_SYSCFG_H */
