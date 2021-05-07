/****************************************************************************
 * arch/arm/src/tiva/hardware/lm/lm4f_memorymap.h
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_LM_LM4F_MEMORYMAP_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_LM_LM4F_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory map ***************************************************************/

#if defined(CONFIG_ARCH_CHIP_LM4F120)
#  define TIVA_FLASH_BASE     0x00000000 /* -0x0003ffff: On-chip FLASH */
                                         /* -0x00ffffff: Reserved */
#  define TIVA_ROM_BASE       0x01000000 /* -0x1fffffff: Reserved for ROM */
#  define TIVA_SRAM_BASE      0x20000000 /* -0x20007fff: Bit-banded on-chip SRAM */
                                         /* -0x21ffffff: Reserved */
#  define TIVA_ASRAM_BASE     0x22000000 /* -0x220fffff: Bit-band alias of 20000000- */
                                         /* -0x3fffffff: Reserved */
#  define TIVA_PERIPH_BASE    0x40000000 /* -0x4001ffff: FiRM Peripherals */
                                         /* -0x41ffffff: Peripherals */
#  define TIVA_APERIPH_BASE   0x42000000 /* -0x43ffffff: Bit-band alias of 40000000- */
                                         /* -0xdfffffff: Reserved */
#  define TIVA_ITM_BASE       0xe0000000 /* -0xe0000fff: Instrumentation Trace Macrocell */
#  define TIVA_DWT_BASE       0xe0001000 /* -0xe0001fff: Data Watchpoint and Trace */
#  define TIVA_FPB_BASE       0xe0002000 /* -0xe0002fff: Flash Patch and Breakpoint */
                                         /* -0xe000dfff: Reserved */
#  define TIVA_NVIC_BASE      0xe000e000 /* -0xe000efff: Nested Vectored Interrupt Controller */
                                         /* -0xe003ffff: Reserved */
#  define TIVA_TPIU_BASE      0xe0040000 /* -0xe0040fff: Trace Port Interface Unit */
#  define TIVA_ETM_BASE       0xe0041000 /* -0xe0041fff: Embedded Trace Macrocell */
                                         /* -0xffffffff: Reserved */
#else
#  error "Memory map not specified for this LM4F chip"
#endif

/* Peripheral base addresses ************************************************/

#if defined(CONFIG_ARCH_CHIP_LM4F120)
/* FiRM Peripheral Base Addresses */

#  define TIVA_WDOG0_BASE     (TIVA_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer 0 */
#  define TIVA_WDOG1_BASE     (TIVA_PERIPH_BASE + 0x01000) /* -0x00fff: Watchdog Timer 1 */
                                                           /* -0x03fff: Reserved */
#  define TIVA_GPIOA_BASE     (TIVA_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define TIVA_GPIOB_BASE     (TIVA_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define TIVA_GPIOC_BASE     (TIVA_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define TIVA_GPIOD_BASE     (TIVA_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define TIVA_SSI0_BASE      (TIVA_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
#  define TIVA_SSI1_BASE      (TIVA_PERIPH_BASE + 0x09000) /* -0x09fff: SSI1 */
#  define TIVA_SSI2_BASE      (TIVA_PERIPH_BASE + 0x0a000) /* -0x0afff: SSI2 */
#  define TIVA_SSI3_BASE      (TIVA_PERIPH_BASE + 0x0b000) /* -0x0bfff: SSI3 */
#  define TIVA_UART0_BASE     (TIVA_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define TIVA_UART1_BASE     (TIVA_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
#  define TIVA_UART2_BASE     (TIVA_PERIPH_BASE + 0x0e000) /* -0x0efff: UART2 */
#  define TIVA_UART3_BASE     (TIVA_PERIPH_BASE + 0x0f000) /* -0x0ffff: UART3 */
#  define TIVA_UART4_BASE     (TIVA_PERIPH_BASE + 0x10000) /* -0x10fff: UART4 */
#  define TIVA_UART5_BASE     (TIVA_PERIPH_BASE + 0x11000) /* -0x11fff: UART5 */
#  define TIVA_UART6_BASE     (TIVA_PERIPH_BASE + 0x12000) /* -0x12fff: UART6 */
#  define TIVA_UART7_BASE     (TIVA_PERIPH_BASE + 0x13000) /* -0x13fff: UART7 */
                                                           /* -0x1ffff: Reserved */

/* Peripheral Base Addresses */

#  define TIVA_I2C0_BASE      (TIVA_PERIPH_BASE + 0x20000)  /* -0x20fff: I2C0 */
#  define TIVA_I2C1_BASE      (TIVA_PERIPH_BASE + 0x21000)  /* -0x21fff: I2C1 */
#  define TIVA_I2C2_BASE      (TIVA_PERIPH_BASE + 0x22000)  /* -0x22fff: I2C2 */
#  define TIVA_I2C3_BASE      (TIVA_PERIPH_BASE + 0x23000)  /* -0x23fff: I2C3 */
#  define TIVA_GPIOE_BASE     (TIVA_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define TIVA_GPIOF_BASE     (TIVA_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
                                                            /* -0x2ffff: Reserved */
#  define TIVA_TIMER0_BASE    (TIVA_PERIPH_BASE + 0x30000)  /* -0x30fff: 16/32 Timer 0 */
#  define TIVA_TIMER1_BASE    (TIVA_PERIPH_BASE + 0x31000)  /* -0x31fff: 16/32 Timer 1 */
#  define TIVA_TIMER2_BASE    (TIVA_PERIPH_BASE + 0x32000)  /* -0x32fff: 16/32 Timer 2 */
#  define TIVA_TIMER3_BASE    (TIVA_PERIPH_BASE + 0x33000)  /* -0x33fff: 16/32 Timer 3 */
#  define TIVA_TIMER4_BASE    (TIVA_PERIPH_BASE + 0x34000)  /* -0x34fff: 16/32 Timer 4 */
#  define TIVA_TIMER5_BASE    (TIVA_PERIPH_BASE + 0x35000)  /* -0x35fff: 16/32 Timer 5 */
#  define TIVA_WTIMER0_BASE   (TIVA_PERIPH_BASE + 0x36000)  /* -0x36fff: 32/64 Wide Timer 0 */
#  define TIVA_WTIMER1_BASE   (TIVA_PERIPH_BASE + 0x37000)  /* -0x37fff: 32/64 Wide Timer 1 */
#  define TIVA_ADC0_BASE      (TIVA_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC 0 */
#  define TIVA_ADC1_BASE      (TIVA_PERIPH_BASE + 0x39000)  /* -0x39fff: ADC 1 */
                                                            /* -0x3bfff: Reserved */
#  define TIVA_CMP_BASE       (TIVA_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
                                                            /* -0x43fff: Reserved */
#  define TIVA_CAN0_BASE      (TIVA_PERIPH_BASE + 0x40000)  /* -0x40fff: CAN Controller */
                                                            /* -0x4bfff: Reserved */
#  define TIVA_WTIMER2_BASE   (TIVA_PERIPH_BASE + 0x4c000)  /* -0x4cfff: 32/64 Wide Timer 2 */
#  define TIVA_WTIMER3_BASE   (TIVA_PERIPH_BASE + 0x4d000)  /* -0x4dfff: 32/64 Wide Timer 3 */
#  define TIVA_WTIMER4_BASE   (TIVA_PERIPH_BASE + 0x4e000)  /* -0x4efff: 32/64 Wide Timer 4 */
#  define TIVA_WTIMER5_BASE   (TIVA_PERIPH_BASE + 0x4f000)  /* -0x4ffff: 32/64 Wide Timer 5 */
#  define TIVA_USB_BASE       (TIVA_PERIPH_BASE + 0x50000)  /* -0x50fff: USB */
                                                            /* -0x57fff: Reserved */
#  define TIVA_GPIOAAHB_BASE  (TIVA_PERIPH_BASE + 0x58000)  /* -0x58fff: GPIO Port A (AHB aperture) */
#  define TIVA_GPIOBAHB_BASE  (TIVA_PERIPH_BASE + 0x59000)  /* -0x59fff: GPIO Port B (AHB aperture) */
#  define TIVA_GPIOCAHB_BASE  (TIVA_PERIPH_BASE + 0x5a000)  /* -0x5afff: GPIO Port C (AHB aperture) */
#  define TIVA_GPIODAHB_BASE  (TIVA_PERIPH_BASE + 0x5b000)  /* -0x5bfff: GPIO Port D (AHB aperture) */
#  define TIVA_GPIOEAHB_BASE  (TIVA_PERIPH_BASE + 0x5c000)  /* -0x5cfff: GPIO Port E (AHB aperture) */
#  define TIVA_GPIOFAHB_BASE  (TIVA_PERIPH_BASE + 0x5d000)  /* -0x5dfff: GPIO Port F (AHB aperture) */
                                                            /* -0xaefff: Reserved */
#  define TIVA_EEPROM_BASE    (TIVA_PERIPH_BASE + 0xaf000)  /* -0xaffff: EEPROM and Key Locker */
                                                            /* -0xf8fff: Reserved */
#  define TIVA_SYSEXC_BASE    (TIVA_PERIPH_BASE + 0xf9000)  /* -0xf9fff: System Exception Control */
                                                            /* -0xfbfff: Reserved */
#  define TIVA_HIBERNATE_BASE (TIVA_PERIPH_BASE + 0xfc000)  /* -0xfcfff: Hibernation Controller */
#  define TIVA_FLASHCON_BASE  (TIVA_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define TIVA_SYSCON_BASE    (TIVA_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
#  define TIVA_UDMA_BASE      (TIVA_PERIPH_BASE + 0xff000)  /* -0xfffff: Micro Direct Memory Access */
#else
#  error "Peripheral base addresses not specified for this Stellaris chip"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_LM_LM4F_MEMORYMAP_H */
