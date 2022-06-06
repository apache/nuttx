/****************************************************************************
 * arch/arm/src/stm32wl5/hardware/stm32wl5_syscfg.h
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

#ifndef __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_SYSCFG_H
#define __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32WL5_SYSCFG_MEMRMP_OFFSET    0x0000 /* SYSCFG memory remap register */
#define STM32WL5_SYSCFG_CFGR1_OFFSET     0x0004 /* SYSCFG configuration register 1 */

#define STM32WL5_SYSCFG_EXTICR_OFFSET(p) (0x0008 + ((p) & 0x000c)) /* Registers are displaced by 4! */

#define STM32WL5_SYSCFG_EXTICR1_OFFSET   0x0008 /* SYSCFG external interrupt configuration register 1 */
#define STM32WL5_SYSCFG_EXTICR2_OFFSET   0x000c /* SYSCFG external interrupt configuration register 2 */
#define STM32WL5_SYSCFG_EXTICR3_OFFSET   0x0010 /* SYSCFG external interrupt configuration register 3 */
#define STM32WL5_SYSCFG_EXTICR4_OFFSET   0x0014 /* SYSCFG external interrupt configuration register 4 */
#define STM32WL5_SYSCFG_SCSR_OFFSET      0x0018 /* SYSCFG SRAM2 control and status register */
#define STM32WL5_SYSCFG_CFGR2_OFFSET     0x001c /* SYSCFG configuration register 2 */
#define STM32WL5_SYSCFG_SWPR_OFFSET      0x0020 /* SYSCFG SRAM2 write protection register */
#define STM32WL5_SYSCFG_SKR_OFFSET       0x0024 /* SYSCFG SRAM2 key register */
#define STM32WL5_SYSCFG_IMR1_OFFSET      0x0100 /* SYSCFG cpu1 interrupt mask register 1 */
#define STM32WL5_SYSCFG_IMR2_OFFSET      0x0104 /* SYSCFG cpu1 interrupt mask register 2 */
#define STM32WL5_SYSCFG_C2IMR1_OFFSET    0x0108 /* SYSCFG cpu2 interrupt mask register 1 */
#define STM32WL5_SYSCFG_C2IMR2_OFFSET    0x010c /* SYSCFG cpu2 interrupt mask register 2 */
#define STM32WL5_SYSCFG_RFDCR_OFFSET     0x0208 /* SYSCFG radio debug control register */

/* Register Addresses *******************************************************/

#define STM32WL5_SYSCFG_MEMRMP          (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_MEMRMP_OFFSET)
#define STM32WL5_SYSCFG_CFGR1           (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_CFGR1_OFFSET)
#define STM32WL5_SYSCFG_EXTICR(p)       (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_EXTICR_OFFSET(p))
#define STM32WL5_SYSCFG_EXTICR1         (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_EXTICR1)
#define STM32WL5_SYSCFG_EXTICR2         (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_EXTICR2)
#define STM32WL5_SYSCFG_EXTICR3         (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_EXTICR3)
#define STM32WL5_SYSCFG_EXTICR4         (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_EXTICR4)
#define STM32WL5_SYSCFG_SCSR            (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_SCSR)
#define STM32WL5_SYSCFG_CFGR2           (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_CFGR2)
#define STM32WL5_SYSCFG_SWPR            (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_SWPR)
#define STM32WL5_SYSCFG_SKR             (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_SKR)
#define STM32WL5_SYSCFG_IMR1            (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_IMR1)
#define STM32WL5_SYSCFG_IMR2            (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_IMR2)
#define STM32WL5_SYSCFG_C2IMR1          (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_C2IMR1)
#define STM32WL5_SYSCFG_C2IMR2          (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_C2IMR2)
#define STM32WL5_SYSCFG_RFDCR           (STM32WL5_SYSCFG_BASE+STM32WL5_SYSCFG_RFDCR)

/* Register Bitfield Definitions ********************************************/

/* SYSCFG memory remap register */

#define SYSCFG_MEMRMP_SHIFT           (0)       /* Bits 2:0 MEM_MODE: Memory mapping selection */
#define SYSCFG_MEMRMP_MASK            (7 << SYSCFG_MEMRMP_SHIFT)
#  define SYSCFG_MEMRMP_FLASH         (0 << SYSCFG_MEMRMP_SHIFT) /* 000: Main Flash memory mapped at cpu1 0x0000 0000 */
#  define SYSCFG_MEMRMP_SYSTEM        (1 << SYSCFG_MEMRMP_SHIFT) /* 001: System Flash memory mapped at cpu1 0x0000 0000 */
#  define SYSCFG_MEMRMP_SRAM          (3 << SYSCFG_MEMRMP_SHIFT) /* 011: SRAM1 mapped at cpu1 0x0000 0000 */

/* SYSCFG configuration register 1 */

#define SYSCFG_CFGR1_BOOSTEN          (1 <<  8) /* Bit  8: I/O analog switch voltage booster enable (use when vdd is low) */
#define SYSCFG_CFGR1_I2C_PB6_FMP      (1 << 16) /* Bit 16: Fast-mode Plus (Fm+) driving capability activation on PB6 */
#define SYSCFG_CFGR1_I2C_PB7_FMP      (1 << 17) /* Bit 17: Fast-mode Plus (Fm+) driving capability activation on PB7 */
#define SYSCFG_CFGR1_I2C_PB8_FMP      (1 << 18) /* Bit 18: Fast-mode Plus (Fm+) driving capability activation on PB8 */
#define SYSCFG_CFGR1_I2C_PB9_FMP      (1 << 19) /* Bit 19: Fast-mode Plus (Fm+) driving capability activation on PB9 */
#define SYSCFG_CFGR1_I2C1_FMP         (1 << 20) /* Bit 20: I2C1 Fast-mode Plus (Fm+) driving capability activation */
#define SYSCFG_CFGR1_I2C2_FMP         (1 << 21) /* Bit 21: I2C2 Fast-mode Plus (Fm+) driving capability activation */
#define SYSCFG_CFGR1_I2C3_FMP         (1 << 22) /* Bit 22: I2C3 Fast-mode Plus (Fm+) driving capability activation */

/* SYSCFG external interrupt configuration register 1-4 */

#define SYSCFG_EXTICR_PORTA           (0)       /* 0000: PA[x] pin */
#define SYSCFG_EXTICR_PORTB           (1)       /* 0001: PB[x] pin */
#define SYSCFG_EXTICR_PORTC           (2)       /* 0010: PC[x] pin */
#define SYSCFG_EXTICR_PORTH           (7)       /* 0111: PH[x] pin */

#define SYSCFG_EXTICR_PORT_MASK       (7)
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

/* SYSCFG SRAM control and status register */

#define SYSCFG_SCSR_SRAM2ER           (1 <<  0) /* Bit  0: SRAM2 Erase */
#define SYSCFG_SCSR_SRAMBSY           (1 <<  1) /* Bit  1: SRAM1/2 busy in erase operation */
#define SYSCFG_SCSR_PKASRAMBSY        (1 <<  8) /* Bit  8: PKA SRAM busy in erase operation */

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

/* SYSCFG cpu1 interrupt mask register 1 (IMR1) */

#define SYSCFG_IMR1_RTCSTAMPTAMPLSECSSIM  (1 <<  0) /* Bit  0: RTCSTAMPTAMPLSECSS interrupt to cpu1 masked */
#define SYSCFG_IMR1_RTCSSRUIM         (1 <<  2)     /* Bit  2: RTC SSRU interrupt to cpu1 masked */
#define SYSCFG_IMR1_EXTI5IM           (1 << 21)     /* Bit 21: Disable EXTI5IM interrupt to cpu1 */
#define SYSCFG_IMR1_EXTI6IM           (1 << 22)     /* Bit 22: Disable EXTI6IM interrupt to cpu1 */
#define SYSCFG_IMR1_EXTI7IM           (1 << 23)     /* Bit 23: Disable EXTI7IM interrupt to cpu1 */
#define SYSCFG_IMR1_EXTI8IM           (1 << 24)     /* Bit 24: Disable EXTI8IM interrupt to cpu1 */
#define SYSCFG_IMR1_EXTI9IM           (1 << 25)     /* Bit 25: Disable EXTI9IM interrupt to cpu1 */
#define SYSCFG_IMR1_EXTI10IM          (1 << 26)     /* Bit 26: Disable EXTI10IM interrupt to cpu1 */
#define SYSCFG_IMR1_EXTI11IM          (1 << 27)     /* Bit 27: Disable EXTI11IM interrupt to cpu1 */
#define SYSCFG_IMR1_EXTI12IM          (1 << 28)     /* Bit 28: Disable EXTI12IM interrupt to cpu1 */
#define SYSCFG_IMR1_EXTI13IM          (1 << 29)     /* Bit 29: Disable EXTI13IM interrupt to cpu1 */
#define SYSCFG_IMR1_EXTI14IM          (1 << 30)     /* Bit 30: Disable EXTI14IM interrupt to cpu1 */
#define SYSCFG_IMR1_EXTI15IM          (1 << 31)     /* Bit 31: Disable EXTI15IM interrupt to cpu1 */

/* SYSCFG cpu1 interrupt mask register 2 (IMR2) */

#define SYSCFG_IMR2_PCM3IM            (1 << 18) /* Bit 18: Disable PVM3 interrupt to cpu1 */
#define SYSCFG_IMR2_PVDIM             (1 << 20) /* Bit 20: Disable PVD interrupt to cpu1 */

/* SYSCFG cpu2 interrupt mask register 1 (C2IMR1) */

#define SYSCFG_C2IMR1_RTCSTAMPTAMPLSECSSIM (1 << 0) /* Bit  0: RTCSTAMPTAMPLSECSS interrupt to CPU2 masked */
#define SYSCFG_C2IMR1_RTCALARMIM      (1 <<  1)     /* Bit  1: Disable rtc alarm interrupt to cpu2 */
#define SYSCFG_C2IMR1_RTCSSRUIM       (1 <<  2)     /* Bit  2: Disable rtc ssru interrupt to cpu2 */
#define SYSCFG_C2IMR1_RTCWKUPIM       (1 <<  3)     /* Bit  3: Disable rtc wkup interrupt to cpu2 */
#define SYSCFG_C2IMR1_RCCIM           (1 <<  5)     /* Bit  5: Disable rcc interrupt to cpu2 */
#define SYSCFG_C2IMR1_FLASHIM         (1 <<  6)     /* Bit  6: Disable flash interrupt to cpu2 */
#define SYSCFG_C2IMR1_PKAIM           (1 <<  8)     /* Bit  8: Disable pka interrupt to cpu2 */
#define SYSCFG_C2IMR1_AESIM           (1 << 10)     /* Bit 10: Disable aes interrupt to cpu2 */
#define SYSCFG_C2IMR1_COMPIM          (1 << 11)     /* Bit 11: Disable comp interrupt to cpu2 */
#define SYSCFG_C2IMR1_ADCIM           (1 << 12)     /* Bit 12: Disable adc interrupt to cpu2 */
#define SYSCFG_C2IMR1_DACIM           (1 << 13)     /* Bit 13: Disable dac interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI0IM         (1 << 16)     /* Bit 16: Disable EXTI0IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI1IM         (1 << 17)     /* Bit 17: Disable EXTI1IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI2IM         (1 << 18)     /* Bit 18: Disable EXTI2IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI3IM         (1 << 19)     /* Bit 19: Disable EXTI3IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI4IM         (1 << 20)     /* Bit 20: Disable EXTI4IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI5IM         (1 << 21)     /* Bit 21: Disable EXTI5IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI6IM         (1 << 22)     /* Bit 22: Disable EXTI6IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI7IM         (1 << 23)     /* Bit 23: Disable EXTI7IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI8IM         (1 << 24)     /* Bit 24: Disable EXTI8IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI9IM         (1 << 25)     /* Bit 25: Disable EXTI9IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI10IM        (1 << 26)     /* Bit 26: Disable EXTI10IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI11IM        (1 << 27)     /* Bit 27: Disable EXTI11IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI12IM        (1 << 28)     /* Bit 28: Disable EXTI12IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI13IM        (1 << 29)     /* Bit 29: Disable EXTI13IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI14IM        (1 << 30)     /* Bit 30: Disable EXTI14IM interrupt to cpu2 */
#define SYSCFG_C2IMR1_EXTI15IM        (1 << 31)     /* Bit 31: Disable EXTI15IM interrupt to cpu2 */

/* SYSCFG cpu2 interrupt mask register 2 (C2IMR2) */

#define SYSCFG_C2IMR2_DMA1CH1IM       (1 <<  0) /* Bit  0: Disable DMA1CH1 interrupt to cpu2 */
#define SYSCFG_C2IMR2_DMA1CH2IM       (1 <<  1) /* Bit  1: Disable DMA1CH2 interrupt to cpu2 */
#define SYSCFG_C2IMR2_DMA1CH3IM       (1 <<  2) /* Bit  2: Disable DMA1CH3 interrupt to cpu2 */
#define SYSCFG_C2IMR2_DMA1CH4IM       (1 <<  3) /* Bit  3: Disable DMA1CH4 interrupt to cpu2 */
#define SYSCFG_C2IMR2_DMA1CH5IM       (1 <<  4) /* Bit  4: Disable DMA1CH5 interrupt to cpu2 */
#define SYSCFG_C2IMR2_DMA1CH6IM       (1 <<  5) /* Bit  5: Disable DMA1CH6 interrupt to cpu2 */
#define SYSCFG_C2IMR2_DMA1CH7IM       (1 <<  6) /* Bit  6: Disable DMA1CH7 interrupt to cpu2 */

#define SYSCFG_C2IMR2_DMA2CH1IM       (1 <<  8) /* Bit  8: Disable DMA2CH1 interrupt to cpu2 */
#define SYSCFG_C2IMR2_DMA2CH2IM       (1 <<  9) /* Bit  9: Disable DMA2CH2 interrupt to cpu2 */
#define SYSCFG_C2IMR2_DMA2CH3IM       (1 << 10) /* Bit 10: Disable DMA2CH3 interrupt to cpu2 */
#define SYSCFG_C2IMR2_DMA2CH4IM       (1 << 11) /* Bit 11: Disable DMA2CH4 interrupt to cpu2 */
#define SYSCFG_C2IMR2_DMA2CH5IM       (1 << 12) /* Bit 12: Disable DMA2CH5 interrupt to cpu2 */
#define SYSCFG_C2IMR2_DMA2CH6IM       (1 << 13) /* Bit 13: Disable DMA2CH6 interrupt to cpu2 */
#define SYSCFG_C2IMR2_DMA2CH7IM       (1 << 14) /* Bit 14: Disable DMA2CH7 interrupt to cpu2 */

#define SYSCFG_C2IMR2_DMAMUX1IM       (1 << 15) /* Bit 15: Disable DMAMUX1 interrupt to cpu2 */

#define SYSCFG_C2IMR2_PCM3IM          (1 << 18) /* Bit 18: Disable PVM3 interrupt to cpu2 */
#define SYSCFG_C2IMR2_PVDIM           (1 << 20) /* Bit 20: Disable PVD interrupt to cpu2 */

/* SYSCFG radio debug control register (RFDCR) */

#define SYSCFG_RFDCR_RFTBSEL          (1 <<  0) /* Bit  0: Analog test bus on RF[ADTB[3:0] */

#endif /* __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_SYSCFG_H */
