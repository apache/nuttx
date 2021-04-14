/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7x3xx_syscfg.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_SYSCFG_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "hardware/stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_SYSCFG_PMC_OFFSET       0x0004 /* SYSCFG peripheral mode configuration register */

#define STM32_SYSCFG_EXTICR_OFFSET(p) (0x0008 + ((p) & 0x000c)) /* Registers are displaced by 4! */

#define STM32_SYSCFG_EXTICR1_OFFSET   0x0008 /* SYSCFG external interrupt configuration register 1 */
#define STM32_SYSCFG_EXTICR2_OFFSET   0x000c /* SYSCFG external interrupt configuration register 2 */
#define STM32_SYSCFG_EXTICR3_OFFSET   0x0010 /* SYSCFG external interrupt configuration register 3 */
#define STM32_SYSCFG_EXTICR4_OFFSET   0x0014 /* SYSCFG external interrupt configuration register 4 */

#define STM32_SYSCFG_CFGR_OFFSET      0x0018 /* SYSCFG configuration register */
#define STM32_SYSCFG_CCCSR_OFFSET     0x0020 /* Compensation cell control/status register */
#define STM32_SYSCFG_CCVR_OFFSET      0x0024 /* Compensation cell value register */
#define STM32_SYSCFG_CCCR_OFFSET      0x0028 /* Compensation cell code register */
#define STM32_SYSCFG_PWRCR_OFFSET     0x002c /* Power Control register */

#define STM32_SYSCFG_PKGR_OFFSET      0x0124 /* Compensation cell code register */

#define STM32_SYSCFG_UR_OFFSET(n)     (0x0300 + ((n) << 2))
#define STM32_SYSCFG_UR0_OFFSET       0x0300 /* User register 0 */
#define STM32_SYSCFG_UR2_OFFSET       0x0308 /* User register 2 */
#define STM32_SYSCFG_UR3_OFFSET       0x030c /* User register 3 */
#define STM32_SYSCFG_UR4_OFFSET       0x0310 /* User register 4 */
#define STM32_SYSCFG_UR5_OFFSET       0x0314 /* User register 5 */
#define STM32_SYSCFG_UR6_OFFSET       0x0318 /* User register 6 */
#define STM32_SYSCFG_UR7_OFFSET       0x031c /* User register 7 */
#define STM32_SYSCFG_UR8_OFFSET       0x0320 /* User register 8 */
#define STM32_SYSCFG_UR9_OFFSET       0x0324 /* User register 9 */
#define STM32_SYSCFG_UR10_OFFSET      0x0328 /* User register 10 */
#define STM32_SYSCFG_UR11_OFFSET      0x032c /* User register 11 */
#define STM32_SYSCFG_UR12_OFFSET      0x0330 /* User register 12 */
#define STM32_SYSCFG_UR13_OFFSET      0x0334 /* User register 13 */
#define STM32_SYSCFG_UR14_OFFSET      0x0338 /* User register 14 */
#define STM32_SYSCFG_UR15_OFFSET      0x033c /* User register 15 */
#define STM32_SYSCFG_UR16_OFFSET      0x0340 /* User register 16 */
#define STM32_SYSCFG_UR17_OFFSET      0x0344 /* User register 17 */

/* Register Addresses *******************************************************/

#define STM32_SYSCFG_PMC              (STM32_SYSCFG_BASE + STM32_SYSCFG_PMC_OFFSET)

#define STM32_SYSCFG_EXTICR(p)        (STM32_SYSCFG_BASE + STM32_SYSCFG_EXTICR_OFFSET(p))
#define STM32_SYSCFG_EXTICR1          (STM32_SYSCFG_BASE + STM32_SYSCFG_EXTICR1_OFFSET)
#define STM32_SYSCFG_EXTICR2          (STM32_SYSCFG_BASE + STM32_SYSCFG_EXTICR2_OFFSET)
#define STM32_SYSCFG_EXTICR3          (STM32_SYSCFG_BASE + STM32_SYSCFG_EXTICR3_OFFSET)
#define STM32_SYSCFG_EXTICR4          (STM32_SYSCFG_BASE + STM32_SYSCFG_EXTICR4_OFFSET)

#define STM32_SYSCFG_CCCSR            (STM32_SYSCFG_BASE + STM32_SYSCFG_CCCSR_OFFSET)
#define STM32_SYSCFG_CCVR             (STM32_SYSCFG_BASE + STM32_SYSCFG_CCVR_OFFSET)
#define STM32_SYSCFG_CCCR             (STM32_SYSCFG_BASE + STM32_SYSCFG_CCCR_OFFSET)
#define STM32_SYSCFG_PWRCR            (STM32_SYSCFG_BASE + STM32_SYSCFG_PWRCR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SYSCFG peripheral mode configuration register */

#define SYSCFG_PMC_I2C1_FMP           (1 << 0)  /* Bit 0: I2C1 Fast-mode Plus Enable */
#define SYSCFG_PMC_I2C2_FMP           (1 << 1)  /* Bit 1: I2C2 Fast-mode Plus Enable */
#define SYSCFG_PMC_I2C3_FMP           (1 << 2)  /* Bit 2: I2C3 Fast-mode Plus Enable */
#define SYSCFG_PMC_I2C4_FMP           (1 << 3)  /* Bit 3: I2C4 Fast-mode Plus Enable */
#define SYSCFG_PMC_PB6_FMP            (1 << 4)  /* Bit 4: PB6 IO pads Fast-mode Plus Enable */
#define SYSCFG_PMC_PB7_FMP            (1 << 5)  /* Bit 5: PB7 IO pads Fast-mode Plus Enable */
#define SYSCFG_PMC_PB8_FMP            (1 << 6)  /* Bit 6: PB8 IO pads Fast-mode Plus Enable */
#define SYSCFG_PMC_PB9_FMP            (1 << 7)  /* Bit 7: PB9 IO pads Fast-mode Plus Enable */
#define SYSCFG_PMC_BOOSTE             (1 << 8)  /* Bit 8: Booster Enable */

#define SYSCFG_PMC_EPIS_SHIFT         (21)      /* Bits 21-23: Ethernet PHY Interface Selection */
#define SYSCFG_PMC_EPIS_MASK          (7 << SYSCFG_PMC_EPIS_SHIFT)
#  define SYSCFG_PMC_EPIS_MII         (0 << SYSCFG_PMC_EPIS_SHIFT)
#  define SYSCFG_PMC_EPIS_RMII        (4 << SYSCFG_PMC_EPIS_SHIFT)
#define SYSCFG_PMC_PA0SO              (1 << 24) /* Bit 24: PA0 Switch Open */
#define SYSCFG_PMC_PA1SO              (1 << 25) /* Bit 25: PA1 Switch Open */
#define SYSCFG_PMC_PC2SO              (1 << 26) /* Bit 26: PC2 Switch Open */
#define SYSCFG_PMC_PC3SO              (1 << 27) /* Bit 27: PC3 Switch Open */

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
#define SYSCFG_EXTICR_PORTJ           (9)       /* 1001: PJ[x] pin */
#define SYSCFG_EXTICR_PORTK           (10)      /* 1010: PK[x] pin */

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

/* Compensation cell control/status register */

#define SYSCFG_CCCSR_EN               (1 << 0)  /* Bit 0: Compensation Cell enable */
#define SYSCFG_CCCSR_CS               (1 << 1)  /* Bit 1: Compensation Cell code selection */
#define SYSCFG_CCCSR_READY            (1 << 8)  /* Bit 8: Compensation Cell ready flag */
#define SYSCFG_CCCSR_HSLV             (1 << 16) /* Bit 16: High-speed at low-voltage */

/* Compensation cell value register */

/* REVISIT:  Missing bitfield definitions */

#define SYSCFG_CCVR_

/* Compensation cell code register */

/* REVISIT:  Missing bitfield definitions */

#define SYSCFG_CCCR_

/* Power control register */

#define SYSCFG_PWRCR_ODEN            (1 << 0)  /* Bit 0: Overdrive enable, this bit allows to activate the LDO regulator overdrive mode */

/* User registers 0-17 */

/* REVISIT:  Missing bitfield definitions */

#define SYSCFG_UR0_
#define SYSCFG_UR2_
#define SYSCFG_UR3_
#define SYSCFG_UR4_
#define SYSCFG_UR5_
#define SYSCFG_UR6_
#define SYSCFG_UR7_
#define SYSCFG_UR8_
#define SYSCFG_UR9_
#define SYSCFG_UR10_
#define SYSCFG_UR11_
#define SYSCFG_UR12_
#define SYSCFG_UR13_
#define SYSCFG_UR14_
#define SYSCFG_UR15_
#define SYSCFG_UR16_
#define SYSCFG_UR17_

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_SYSCFG_H */
