/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32l15xxx_syscfg.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32L15XXX_SYSCFG_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32L15XXX_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_SYSCFG_MEMRMP_OFFSET     0x0000                    /* SYSCFG memory remap register */
#define STM32_SYSCFG_PMC_OFFSET        0x0004                    /* SYSCFG peripheral mode configuration register */

#define STM32_SYSCFG_EXTICR_OFFSET(p)  (0x0008 + ((p) & 0x000c)) /* Registers are displaced by 4! */
#define STM32_SYSCFG_EXTICR1_OFFSET    0x0008                    /* SYSCFG external interrupt configuration register 1 */
#define STM32_SYSCFG_EXTICR2_OFFSET    0x000c                    /* SYSCFG external interrupt configuration register 2 */
#define STM32_SYSCFG_EXTICR3_OFFSET    0x0010                    /* SYSCFG external interrupt configuration register 3 */
#define STM32_SYSCFG_EXTICR4_OFFSET    0x0014                    /* SYSCFG external interrupt configuration register 4 */

/* Register Addresses *******************************************************/

#define STM32_SYSCFG_MEMRMP            (STM32_SYSCFG_BASE+STM32_SYSCFG_MEMRMP_OFFSET)
#define STM32_SYSCFG_PMC               (STM32_SYSCFG_BASE+STM32_SYSCFG_PMC_OFFSET)

#define STM32_SYSCFG_EXTICR(p)         (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR_OFFSET(p))
#define STM32_SYSCFG_EXTICR1           (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR1_OFFSET)
#define STM32_SYSCFG_EXTICR2           (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR2_OFFSET)
#define STM32_SYSCFG_EXTICR3           (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR3_OFFSET)
#define STM32_SYSCFG_EXTICR4           (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR4_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SYSCFG memory remap register */

#define SYSCFG_MEMRMP_MEMMODE_SHIFT     (0)                                 /* Bits 0-1: Memory mapping selection */
#define SYSCFG_MEMRMP_MEMMODE_MASK      (3 << SYSCFG_MEMRMP_MEMMODE_SHIFT)
#  define SYSCFG_MEMRMP_MEMMODE_FLASH   (0 << SYSCFG_MEMRMP_MEMMODE_SHIFT)  /* 00: Main Flash memory mapped at 0x00000000 */
#  define SYSCFG_MEMRMP_MEMMODE_SYSTEM  (1 << SYSCFG_MEMRMP_MEMMODE_SHIFT)  /* 01: System Flash memory mapped at 0x00000000 */
#  define SYSCFG_MEMRMP_MEMMODE_FSMC    (2 << SYSCFG_MEMRMP_MEMMODE_SHIFT)  /* 10: FSMC */
#  define SYSCFG_MEMRMP_MEMMODE_SRAM    (3 << SYSCFG_MEMRMP_MEMMODE_SHIFT)  /* 11: SRAM mapped at 0x00000000 */
                                                                            /* Bits 2-7: Reserved */
#define SYSCFG_MEMRMP_BOOTMODE_SHIFT    (0)                                 /* Bits 8-9: Boot mode selected by the boot pins */
#define SYSCFG_MEMRMP_BOOTMODE_MASK     (3 << SYSCFG_MEMRMP_BOOTMODE_SHIFT)
#  define SYSCFG_MEMRMP_BOOTMODE_FLASH  (0 << SYSCFG_MEMRMP_BOOTMODE_SHIFT) /* 00: Main Flash memory boot mode */
#  define SYSCFG_MEMRMP_BOOTMODE_SYSTEM (1 << SYSCFG_MEMRMP_BOOTMODE_SHIFT) /* 01: System Flash memory boot mode */
#  define SYSCFG_MEMRMP_BOOTMODE_SRAM   (3 << SYSCFG_MEMRMP_BOOTMODE_SHIFT) /* 11: Embedded SRAM boot mode */
                                                                            /* Bits 10-31: Reserved */

/* SYSCFG peripheral mode configuration register */

#define SYSCFG_PMC_USBPU                (1 << 0)                           /* Bit 0: USB pull-up enable on DP line */
#define SYSCFG_PMC_LCDCAPA_SHIFT        (1)                                /* Bits 1-5: LCD  decoupling capacitance connection */
#define SYSCFG_PMC_LCDCAPA_MASK         (0x1f << SYSCFG_PMC_LCDCAPA_SHIFT)
#  define SYSCFG_PMC_LCDCAPA_PB2        (0x01 << SYSCFG_PMC_LCDCAPA_SHIFT) /* Bit 1: Controls VLCDrail2 on PB2/LCD_VCAP2 */
#  define SYSCFG_PMC_LCDCAPA_PB12       (0x02 << SYSCFG_PMC_LCDCAPA_SHIFT) /* Bit 2: Controls VLCDrail1 on PB12/LCD_VCAP1 */
#  define SYSCFG_PMC_LCDCAPA_PB0        (0x04 << SYSCFG_PMC_LCDCAPA_SHIFT) /* Bit 3: Controls VLCDrail3 on PB0/LCD_VCAP3 */
#  define SYSCFG_PMC_LCDCAPA_PE11       (0x08 << SYSCFG_PMC_LCDCAPA_SHIFT) /* Bit 4: Controls VLCDrail1 on PE11/LCD_VCAP1 */
#  define SYSCFG_PMC_LCDCAPA_PE12       (0x10 << SYSCFG_PMC_LCDCAPA_SHIFT) /* Bit 5: Controls VLCDrail3 on PE12/LCD_VCAP3 */
                                                                           /* Bits 6-31: Reserved */

/* SYSCFG external interrupt configuration register 1-4 */

#define SYSCFG_EXTICR_PORTA           (0)       /* 0000: PA[x] pin */
#define SYSCFG_EXTICR_PORTB           (1)       /* 0001: PB[x] pin */
#define SYSCFG_EXTICR_PORTC           (2)       /* 0010: PC[x] pin */
#define SYSCFG_EXTICR_PORTD           (3)       /* 0011: PD[x] pin */
#define SYSCFG_EXTICR_PORTE           (4)       /* 0100: PE[x] pin */
#define SYSCFG_EXTICR_PORTH           (5)       /* 0101: PH[x] pin */
#define SYSCFG_EXTICR_PORTF           (6)       /* 0110: PF[x] pin */
#define SYSCFG_EXTICR_PORTG           (7)       /* 0111: PG[x] pin */

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
                                                /* Bits 16-31: Reserved */

#define SYSCFG_EXTICR2_EXTI4_SHIFT    (0)       /* Bits 0-3: EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI4_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI4_SHIFT)
#define SYSCFG_EXTICR2_EXTI5_SHIFT    (4)       /* Bits 4-7: EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI5_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI5_SHIFT)
#define SYSCFG_EXTICR2_EXTI6_SHIFT    (8)       /* Bits 8-11: EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI6_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI6_SHIFT)
#define SYSCFG_EXTICR2_EXTI7_SHIFT    (12)      /* Bits 12-15: EXTI 7 configuration */
#define SYSCFG_EXTICR2_EXTI7_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI7_SHIFT)
                                                /* Bits 16-31: Reserved */

#define SYSCFG_EXTICR3_EXTI8_SHIFT    (0)       /* Bits 0-3: EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI8_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI8_SHIFT)
#define SYSCFG_EXTICR3_EXTI9_SHIFT    (4)       /* Bits 4-7: EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI9_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI9_SHIFT)
#define SYSCFG_EXTICR3_EXTI10_SHIFT   (8)       /* Bits 8-11: EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI10_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI10_SHIFT)
#define SYSCFG_EXTICR3_EXTI11_SHIFT   (12)      /* Bits 12-15: EXTI 11 configuration */
#define SYSCFG_EXTICR3_EXTI11_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI11_SHIFT)
                                                /* Bits 16-31: Reserved */

#define SYSCFG_EXTICR4_EXTI12_SHIFT   (0)       /* Bits 0-3: EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI12_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI12_SHIFT)
#define SYSCFG_EXTICR4_EXTI13_SHIFT   (4)       /* Bits 4-7: EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI13_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI13_SHIFT)
#define SYSCFG_EXTICR4_EXTI14_SHIFT   (8)       /* Bits 8-11: EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI14_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI14_SHIFT)
#define SYSCFG_EXTICR4_EXTI15_SHIFT   (12)      /* Bits 12-15: EXTI 15 configuration */
#define SYSCFG_EXTICR4_EXTI15_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI15_SHIFT)
                                                /* Bits 16-31: Reserved */

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32L15XXX_SYSCFG_H */
