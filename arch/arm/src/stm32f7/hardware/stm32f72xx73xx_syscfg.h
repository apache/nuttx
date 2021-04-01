/****************************************************************************
 * arch/arm/src/stm32f7/hardware/stm32f72xx73xx_syscfg.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F72XX73XX_SYSCFG_H
#define __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F72XX73XX_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#if defined(CONFIG_STM32F7_STM32F72XX) || defined(CONFIG_STM32F7_STM32F73XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_SYSCFG_MEMRMP_OFFSET    0x0000 /* SYSCFG memory remap register */
#define STM32_SYSCFG_PMC_OFFSET       0x0004 /* SYSCFG peripheral mode configuration register */

#define STM32_SYSCFG_EXTICR_OFFSET(p) (0x0008 + ((p) & 0x000c)) /* Registers are displaced by 4! */

#define STM32_SYSCFG_EXTICR1_OFFSET   0x0008 /* SYSCFG external interrupt configuration register 1 */
#define STM32_SYSCFG_EXTICR2_OFFSET   0x000c /* SYSCFG external interrupt configuration register 2 */
#define STM32_SYSCFG_EXTICR3_OFFSET   0x0010 /* SYSCFG external interrupt configuration register 3 */
#define STM32_SYSCFG_EXTICR4_OFFSET   0x0014 /* SYSCFG external interrupt configuration register 4 */

#define STM32_SYSCFG_CMPCR_OFFSET     0x0020 /* Compensation cell control register */

/* Register Addresses *******************************************************/

#define STM32_SYSCFG_MEMRMP           (STM32_SYSCFG_BASE+STM32_SYSCFG_MEMRMP_OFFSET)
#define STM32_SYSCFG_PMC              (STM32_SYSCFG_BASE+STM32_SYSCFG_PMC_OFFSET)

#define STM32_SYSCFG_EXTICR(p)        (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR_OFFSET(p))
#define STM32_SYSCFG_EXTICR1          (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR1_OFFSET)
#define STM32_SYSCFG_EXTICR2          (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR2_OFFSET)
#define STM32_SYSCFG_EXTICR3          (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR3_OFFSET)
#define STM32_SYSCFG_EXTICR4          (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR4_OFFSET)

#define STM32_SYSCFG_CMPCR            (STM32_SYSCFG_BASE+STM32_SYSCFG_CMPCR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SYSCFG memory remap register */

#define SYSCFG_MEMRMP_MEMBOOT         (1 << 0)  /* Bit 0:  Memory boot mapping */
#define SYSCFG_MEMRMP_MEMBOOT_ADD0    (0 << 0)  /*         0=BOOT_ADD0 defines */
#define SYSCFG_MEMRMP_MEMBOOT_ADD1    (1 << 0)  /*         1=BOOT_ADD1 defines */
#define SYSCFG_MEMRMP_SWPFMC_SHIFT    (10)      /* Bits 10-11: FMC memory mapping swap */
#define SYSCFG_MEMRMP_SWPFMC_MASK     (3 << SYSCFG_MEMRMP_SWPFMC_SHIFT)
#  define SYSCFG_MEMRMP_SWPFMC_NONE   (0 << SYSCFG_MEMRMP_SWPFMC_SHIFT) /* No FMC memory mapping swapping */
#  define SYSCFG_MEMRMP_SWPFMC_NORRAM (1 << SYSCFG_MEMRMP_SWPFMC_SHIFT) /* NOR/RAM and SDRAM memory mapping swapped */

/* SYSCFG peripheral mode configuration register */

#define SYSCFG_PMC_I2C1_FMP           (1 << 0)  /* Bit 0: Forces FM+ drive capability on I2C1 SCL & SDA */
#define SYSCFG_PMC_I2C2_FMP           (1 << 1)  /* Bit 1: Forces FM+ drive capability on I2C2 SCL & SDA */
#define SYSCFG_PMC_I2C3_FMP           (1 << 2)  /* Bit 2: Forces FM+ drive capability on I2C3 SCL & SDA */
#define SYSCFG_PMC_PB6_FMP            (1 << 4)  /* Bit 4: Forces FM+ drive capability on the PB6 pin */
#define SYSCFG_PMC_PB7_FMP            (1 << 5)  /* Bit 5: Forces FM+ drive capability on the PB7 pin */
#define SYSCFG_PMC_PB8_FMP            (1 << 6)  /* Bit 7: Forces FM+ drive capability on the PB8 pin */
#define SYSCFG_PMC_PB9_FMP            (1 << 7)  /* Bit 4: Forces FM+ drive capability on the PB9 pin */
#define SYSCFG_PMC_ADC1DC2            (1 << 16) /* Bit 16: See AN4073 */
#define SYSCFG_PMC_ADC2DC2            (1 << 17) /* Bit 17: See AN4073 */
#define SYSCFG_PMC_ADC3DC2            (1 << 18) /* Bit 18: See AN4073 */

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

/* Compensation cell control register */

#define SYSCFG_CMPCR_CMPPD            (1 << 0)  /* Bit 0: Compensation cell power-down */
#define SYSCFG_CMPCR_READY            (1 << 8)  /* Bit 8: Compensation cell ready flag */

#endif /* CONFIG_STM32F7_STM32F72XX || CONFIG_STM32F7_STM32F73XX */
#endif /* __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F72XX73XX_SYSCFG_H */
