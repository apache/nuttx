/****************************************************************************
 * arch/arm/src/stm32wb/hardware/stm32wb_syscfg.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_SYSCFG_H
#define __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32WB_SYSCFG_MEMRMP_OFFSET    0x0000 /* SYSCFG memory remap register */
#define STM32WB_SYSCFG_CFGR1_OFFSET     0x0004 /* SYSCFG configuration register 1 */

#define STM32WB_SYSCFG_EXTICR_OFFSET(p) (0x0008 + ((p) & 0x0c)) /* Pin p = 0..15 */

#define STM32WB_SYSCFG_EXTICR1_OFFSET   0x0008 /* SYSCFG external interrupt configuration register 1 */
#define STM32WB_SYSCFG_EXTICR2_OFFSET   0x000c /* SYSCFG external interrupt configuration register 2 */
#define STM32WB_SYSCFG_EXTICR3_OFFSET   0x0010 /* SYSCFG external interrupt configuration register 3 */
#define STM32WB_SYSCFG_EXTICR4_OFFSET   0x0014 /* SYSCFG external interrupt configuration register 4 */

#define STM32WB_SYSCFG_SCSR_OFFSET      0x0018 /* SYSCFG SRAM2 control and status register */
#define STM32WB_SYSCFG_CFGR2_OFFSET     0x001c /* SYSCFG configuration register 2 */
#define STM32WB_SYSCFG_SWPR1_OFFSET     0x0020 /* SYSCFG SRAM2 write protection register 1 */
#define STM32WB_SYSCFG_SKR_OFFSET       0x0024 /* SYSCFG SRAM2 key register */
#define STM32WB_SYSCFG_SWPR2_OFFSET     0x0028 /* SYSCFG SRAM2 write protection register 2 */

#define STM32WB_SYSCFG_IMR1_OFFSET      0x0100 /* SYSCFG Interrupt mask register 1 */
#define STM32WB_SYSCFG_IMR2_OFFSET      0x0104 /* SYSCFG Interrupt mask register 2 */
#define STM32WB_SYSCFG_C2IMR1_OFFSET    0x0108 /* SYSCFG CPU2 Interrupt mask register 1 */
#define STM32WB_SYSCFG_C2IMR2_OFFSET    0x010c /* SYSCFG CPU2 Interrupt mask register 2 */
#define STM32WB_SYSCFG_SIPCR_OFFSET     0x0110 /* SYSCFG Secure IP control register */

/* Register Addresses *******************************************************/

#define STM32WB_SYSCFG_MEMRMP           (STM32WB_SYSCFG_BASE + STM32WB_SYSCFG_MEMRMP_OFFSET)
#define STM32WB_SYSCFG_CFGR1            (STM32WB_SYSCFG_BASE + STM32WB_SYSCFG_CFGR1_OFFSET)
#define STM32WB_SYSCFG_EXTICR(p)        (STM32WB_SYSCFG_BASE + STM32WB_SYSCFG_EXTICR_OFFSET(p))
#define STM32WB_SYSCFG_EXTICR1          (STM32WB_SYSCFG_BASE + STM32WB_SYSCFG_EXTICR1_OFFSET)
#define STM32WB_SYSCFG_EXTICR2          (STM32WB_SYSCFG_BASE + STM32WB_SYSCFG_EXTICR2_OFFSET)
#define STM32WB_SYSCFG_EXTICR3          (STM32WB_SYSCFG_BASE + STM32WB_SYSCFG_EXTICR3_OFFSET)
#define STM32WB_SYSCFG_EXTICR4          (STM32WB_SYSCFG_BASE + STM32WB_SYSCFG_EXTICR4_OFFSET)
#define STM32WB_SYSCFG_SCSR             (STM32WB_SYSCFG_BASE + STM32WB_SYSCFG_SCSR_OFFSET)
#define STM32WB_SYSCFG_CFGR2            (STM32WB_SYSCFG_BASE + STM32WB_SYSCFG_CFGR2_OFFSET)
#define STM32WB_SYSCFG_SWPR1            (STM32WB_SYSCFG_BASE + STM32WB_SYSCFG_SWPR1_OFFSET)
#define STM32WB_SYSCFG_SKR              (STM32WB_SYSCFG_BASE + STM32WB_SYSCFG_SKR_OFFSET)
#define STM32WB_SYSCFG_SWPR2            (STM32WB_SYSCFG_BASE + STM32WB_SYSCFG_SWPR2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SYSCFG memory remap register */

#define SYSCFG_MEMRMP_SHIFT           (0)       /* Bits 2-0: Memory mapping selection */
#define SYSCFG_MEMRMP_MASK            (0x7 << SYSCFG_MEMRMP_SHIFT)
#  define SYSCFG_MEMRMP_FLASH         (0x0 << SYSCFG_MEMRMP_SHIFT) /* 000: Main Flash memory mapped at 0x00000000 */
#  define SYSCFG_MEMRMP_SYSTEM        (0x1 << SYSCFG_MEMRMP_SHIFT) /* 001: System Flash memory mapped at 0x00000000 */
#  define SYSCFG_MEMRMP_SRAM          (0x3 << SYSCFG_MEMRMP_SHIFT) /* 011: SRAM1 mapped at 0x00000000 */
#  define SYSCFG_MEMRMP_QSPI          (0x6 << SYSCFG_MEMRMP_SHIFT) /* 110: QSPI memory mapped at 0x00000000 */

/* SYSCFG configuration register 1 */

#define SYSCFG_CFGR1_BOOSTEN          (1 << 8)  /* Bit  8: I/O analog switch voltage booster enable */
#define SYSCFG_CFGR1_I2C_PB6_FMP      (1 << 16) /* Bit 16: Fast-mode Plus (Fm+) driving capability activation on PB6 */
#define SYSCFG_CFGR1_I2C_PB7_FMP      (1 << 17) /* Bit 17: Fast-mode Plus (Fm+) driving capability activation on PB7 */
#define SYSCFG_CFGR1_I2C_PB8_FMP      (1 << 18) /* Bit 18: Fast-mode Plus (Fm+) driving capability activation on PB8 */
#define SYSCFG_CFGR1_I2C_PB9_FMP      (1 << 19) /* Bit 19: Fast-mode Plus (Fm+) driving capability activation on PB9 */
#define SYSCFG_CFGR1_I2C1_FMP         (1 << 20) /* Bit 20: I2C1 Fast-mode Plus (Fm+) driving capability activation */
#define SYSCFG_CFGR1_I2C3_FMP         (1 << 22) /* Bit 22: I2C3 Fast-mode Plus (Fm+) driving capability activation */
#define SYSCFG_CFGR1_FPU_IE0          (1 << 26) /* Bit 26: FPU Invalid operation interrupt enable */
#define SYSCFG_CFGR1_FPU_IE1          (1 << 27) /* Bit 27: FPU Divide-by-zero interrupt enable */
#define SYSCFG_CFGR1_FPU_IE2          (1 << 28) /* Bit 28: FPU Underflow interrupt enable */
#define SYSCFG_CFGR1_FPU_IE3          (1 << 29) /* Bit 29: FPU Overflow interrupt enable */
#define SYSCFG_CFGR1_FPU_IE4          (1 << 30) /* Bit 30: FPU Input denormal interrupt enable */
#define SYSCFG_CFGR1_FPU_IE5          (1 << 31) /* Bit 31: FPU Inexact interrupt enable */

/* SYSCFG external interrupt configuration register 1-4 */

#define SYSCFG_EXTICR_PORTA           (0x0)     /* 000: PA[x] pin */
#define SYSCFG_EXTICR_PORTB           (0x1)     /* 001: PB[x] pin */
#define SYSCFG_EXTICR_PORTC           (0x2)     /* 010: PC[x] pin */
#define SYSCFG_EXTICR_PORTD           (0x3)     /* 011: PD[x] pin */
#define SYSCFG_EXTICR_PORTE           (0x4)     /* 100: PE[x] pin */
#define SYSCFG_EXTICR_PORTH           (0x7)     /* 111: PH[x] pin */

#define SYSCFG_EXTICR_PORT_MASK       (0x7)
#define SYSCFG_EXTICR_EXTI_SHIFT(g)   (((g) & 0x3) << 2)
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

#define SYSCFG_SCSR_SRAM2ER           (1 << 0)  /* Bit  0: SRAM2 and PKA RAM Erase */
#define SYSCFG_SCSR_SRAM2BSY          (1 << 1)  /* Bit  1: SRAM2 and PKA RAM busy by erase operation */
#define SYSCFG_SCSR_C2RFD             (1 << 31) /* Bit 31: CPU2 SRAM fetch disable */

/* SYSCFG configuration register 2 */

#define SYSCFG_CFGR2_CLL              (1 << 0)  /* Bit 0: LOCKUP (Hardfault) output enable */
#define SYSCFG_CFGR2_SPL              (1 << 1)  /* Bit 1: SRAM2 parity lock enable  */
#define SYSCFG_CFGR2_PVDL             (1 << 2)  /* Bit 2: PVD lock enable */
#define SYSCFG_CFGR2_ECCL             (1 << 3)  /* Bit 3: ECC lock enable */
#define SYSCFG_CFGR2_SPF              (1 << 8)  /* Bit 8: SRAM2 parity error flag */

/* SYSCFG SRAM2 write protection register 1 */

#define SYSCFG_SWPR1_PWP(x)           (1 << (x)) /* Bits 0-31: SRAM2 1Kb page x = 0..31 write protection */

/* SYSCFG SRAM2 key register */

#define SYSCFG_SKR_SHIFT              (0)
#define SYSCFG_SKR_MASK               (0xff << SYSCFG_SKR_SHIFT)
#define SYSCFG_SKR_KEY1               0xca
#define SYSCFG_SKR_KEY2               0x53

/* SYSCFG SRAM2 write protection register 2 */

#define SYSCFG_SWPR2_PWP(x)           (1 << ((x) - 32)) /* Bits 0-31: SRAM2 1Kb page x = 32..63 write protection */

/* SYSCFG Interrupt mask register 1 */

#define SYSCFG_IMR1_EXTIIM(n)         (1 << ((n) + 16)) /* EXTI[n] interrupt mask, n = 5..15 */

#define SYSCFG_IMR1_TIM1IM            (1 << 13) /* Bit 13: TIM1 interrupt mask */
#define SYSCFG_IMR1_TIM16IM           (1 << 14) /* Bit 14: TIM16 interrupt mask */
#define SYSCFG_IMR1_TIM17IM           (1 << 15) /* Bit 15: TIM17 interrupt mask */

/* SYSCFG Interrupt mask register 2 */

#define SYSCFG_IMR2_PVM1IM            (1 << 16) /* Bit 16: PVM1 interrupt mask */
#define SYSCFG_IMR2_PVM3IM            (1 << 18) /* Bit 18: PVM3 interrupt mask */
#define SYSCFG_IMR2_PVDIM             (1 << 20) /* Bit 20: PVD interrupt mask */

/* SYSCFG CPU2 Interrupt mask register 1 */

#define SYSCFG_C2IMR1_RTCSTLSECSSIM   (1 << 0)  /* Bit 0: CPU2 RTC, STAMP, TAMP, LSECSS interrupt mask */
#define SYSCFG_C2IMR1_RTCWKUPIM       (1 << 3)  /* Bit 3: CPU2 RTC Wakeup interrupt mask */
#define SYSCFG_C2IMR1_RTCALRMIM       (1 << 4)  /* Bit 4: CPU2 RTC Alarm interrupt mask */
#define SYSCFG_C2IMR1_RCCIM           (1 << 5)  /* Bit 5: CPU2 RCC interrupt mask */
#define SYSCFG_C2IMR1_FLASHIM         (1 << 6)  /* Bit 6: CPU2 FLASH interrupt mask */
#define SYSCFG_C2IMR1_PKAIM           (1 << 8)  /* Bit 8: CPU2 PKA interrupt mask */
#define SYSCFG_C2IMR1_RNGIM           (1 << 9)  /* Bit 9: CPU2 RNG interrupt mask */
#define SYSCFG_C2IMR1_AES1IM          (1 << 10) /* Bit 10: CPU2 AES1 interrupt mask */
#define SYSCFG_C2IMR1_COMPIM          (1 << 11) /* Bit 11: CPU2 COMP interrupt mask */
#define SYSCFG_C2IMR1_ADCIM           (1 << 12) /* Bit 12: CPU2 ADC interrupt mask */

#define SYSCFG_C2IMR1_EXTIIM(n)       (1 << ((n) + 16)) /* CPU2 EXTI[n] interrupt mask, n = 0..15 */

/* SYSCFG CPU2 Interrupt mask register 2 */

#define SYSCFG_C2IMR2_DMA1IM(n)       (1 << ((n) - 1)) /* CPU2 DMA1[n] interrupt mask, n = 1..7 */

#define SYSCFG_C2IMR2_DMA2IM(n)       (1 << ((n) + 7)) /* CPU2 DMA2[n] interrupt mask, n = 1..7 */

#define SYSCFG_C2IMR2_DMAMUX1IM       (1 << 15) /* Bit 15: CPU2 DMAMUX1 interrupt mask */
#define SYSCFG_C2IMR2_PVM1IM          (1 << 16) /* Bit 16: CPU2 PVM1 interrupt mask */
#define SYSCFG_C2IMR2_PVM3IM          (1 << 18) /* Bit 18: CPU2 PVM3 interrupt mask */
#define SYSCFG_C2IMR2_PVDIM           (1 << 20) /* Bit 20: CPU2 PVD interrupt mask */
#define SYSCFG_C2IMR2_TSCIM           (1 << 21) /* Bit 21: CPU2 TSC interrupt mask */
#define SYSCFG_C2IMR2_LCDIM           (1 << 22) /* Bit 22: CPU2 LCD interrupt mask */

/* SYSCFG Secure IP control register */

#define SYSCFG_SIPCR_SAES1            (1 << 0) /* Bit 0: AES1 KEY[7:0] Security enable */
#define SYSCFG_SIPCR_SAES2            (1 << 1) /* Bit 1: AES2 Security enable */
#define SYSCFG_SIPCR_SPKA             (1 << 2) /* Bit 2: PKA Security enable */
#define SYSCFG_SIPCR_SRNG             (1 << 3) /* Bit 3: RNG Security enable */

#endif /* __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_SYSCFG_H */
