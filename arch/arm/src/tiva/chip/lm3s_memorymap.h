/************************************************************************************
 * arch/arm/src/tiva/chip/lm3s_memorymap.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_CHIP_LM3S_MEMORYMAP_H
#define __ARCH_ARM_SRC_TIVA_CHIP_LM3S_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Memory map ***********************************************************************/

#if defined(CONFIG_ARCH_CHIP_LM3S6918) || defined(CONFIG_ARCH_CHIP_LM3S6432) || \
    defined(CONFIG_ARCH_CHIP_LM3S6965) || defined(CONFIG_ARCH_CHIP_LM3S8962)
#  define TIVA_FLASH_BASE     0x00000000 /* -0x0003ffff: On-chip FLASH */
                                         /* -0x1fffffff: Reserved */
#  define TIVA_SRAM_BASE      0x20000000 /* -0x2000ffff: Bit-banded on-chip SRAM */
                                         /* -0x21ffffff: Reserved */
#  define TIVA_ASRAM_BASE     0x22000000 /* -0x221fffff: Bit-band alias of 20000000- */
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
                                         /* -0xffffffff: Reserved */
#elif defined(CONFIG_ARCH_CHIP_LM3S9B96)
#  define TIVA_FLASH_BASE     0x00000000 /* -0x0003ffff: On-chip FLASH */
                                         /* -0x1fffffff: Reserved */
#  define TIVA_SRAM_BASE      0x20000000 /* -0x2000ffff: Bit-banded on-chip SRAM */
                                         /* -0x21ffffff: Reserved */
#  define TIVA_ASRAM_BASE     0x22000000 /* -0x221fffff: Bit-band alias of 20000000- */
                                         /* -0x3fffffff: Reserved */
#  define TIVA_PERIPH_BASE    0x40000000 /* -0x4001ffff: FiRM Peripherals */
                                         /* -0x41ffffff: Peripherals */
#  define TIVA_APERIPH_BASE   0x42000000 /* -0x43ffffff: Bit-band alise of 40000000- */
                                         /* -0x5fffffff: Reserved */
#  define TIVA_EPI0RAM_BASE   0x60000000 /* -0xdfffffff: EPI0 mapped peripheral and RAM */
#  define TIVA_ITM_BASE       0xe0000000 /* -0xe0000fff: Instrumentation Trace Macrocell */
#  define TIVA_DWT_BASE       0xe0001000 /* -0xe0001fff: Data Watchpoint and Trace */
#  define TIVA_FPB_BASE       0xe0002000 /* -0xe0002fff: Flash Patch and Breakpoint */
                                        /* -0xe000dfff: Reserved */
#  define TIVA_NVIC_BASE      0xe000e000 /* -0xe000efff: Nested Vectored Interrupt Controller */
                                         /* -0xe003ffff: Reserved */
#  define TIVA_TPIU_BASE      0xe0040000 /* -0xe0040fff: Trace Port Interface Unit */
                                         /* -0xffffffff: Reserved */
#else
#  error "Memory map not specified for this LM3S chip"
#endif

/* Peripheral base addresses ********************************************************/
/* The LM3S6918 and LM3S6965 differ by only the presence or absence of a few differnt
 * peripheral modules.  They could probably be combined into one peripheral memory
 * map.  However, keeping them separate does also provide so early, compile-time
 * error detection that makes the duplication worthwhile.
 */

#if defined(CONFIG_ARCH_CHIP_LM3S6918)
/* FiRM Peripheral Base Addresses */

#  define TIVA_WDOG_BASE      (TIVA_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer */
                                                           /* -0x03fff: Reserved */
#  define TIVA_GPIOA_BASE     (TIVA_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define TIVA_GPIOB_BASE     (TIVA_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define TIVA_GPIOC_BASE     (TIVA_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define TIVA_GPIOD_BASE     (TIVA_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define TIVA_SSI0_BASE      (TIVA_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
#  define TIVA_SSI1_BASE      (TIVA_PERIPH_BASE + 0x09000) /* -0x09fff: SSI1 */
                                                           /* -0x0bfff: Reserved */
#  define TIVA_UART0_BASE     (TIVA_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define TIVA_UART1_BASE     (TIVA_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define TIVA_I2C0_BASE      (TIVA_PERIPH_BASE + 0x20000)  /* -0x20fff: I2C0 */
#  define TIVA_I2C1_BASE      (TIVA_PERIPH_BASE + 0x21000)  /* -0x21fff: I2C1 */
                                                            /* -0x23fff: Reserved */
#  define TIVA_GPIOE_BASE     (TIVA_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define TIVA_GPIOF_BASE     (TIVA_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define TIVA_GPIOG_BASE     (TIVA_PERIPH_BASE + 0x26000)  /* -0x26fff: GPIO Port G */
#  define TIVA_GPIOH_BASE     (TIVA_PERIPH_BASE + 0x27000)  /* -0x27fff: GPIO Port H */
                                                            /* -0x2ffff: Reserved */
#  define TIVA_TIMER0_BASE    (TIVA_PERIPH_BASE + 0x30000)  /* -0x30fff: Timer 0 */
#  define TIVA_TIMER1_BASE    (TIVA_PERIPH_BASE + 0x31000)  /* -0x31fff: Timer 1 */
#  define TIVA_TIMER2_BASE    (TIVA_PERIPH_BASE + 0x32000)  /* -0x32fff: Timer 2 */
#  define TIVA_TIMER3_BASE    (TIVA_PERIPH_BASE + 0x33000)  /* -0x33fff: Timer 3 */
                                                            /* -0x37fff: Reserved */
#  define TIVA_ADC_BASE       (TIVA_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC */
                                                            /* -0x3bfff: Reserved */
#  define TIVA_CMP_BASE       (TIVA_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
                                                            /* -0x47fff: Reserved */
#  define TIVA_ETHCON_BASE    (TIVA_PERIPH_BASE + 0x48000)  /* -0x48fff: Ethernet Controller */
                                                            /* -0xfcfff: Reserved */
#  define TIVA_HIBERNATE_BASE (TIVA_PERIPH_BASE + 0xfc000)  /* -0xfcfff: Hibernation Controller */
#  define TIVA_FLASHCON_BASE  (TIVA_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define TIVA_SYSCON_BASE    (TIVA_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
                                                            /* -0x1ffffff: Reserved */
#elif defined(CONFIG_ARCH_CHIP_LM3S6432)
/* FiRM Peripheral Base Addresses */

#  define TIVA_WDOG_BASE      (TIVA_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer */
                                                           /* -0x03fff: Reserved */
#  define TIVA_GPIOA_BASE     (TIVA_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define TIVA_GPIOB_BASE     (TIVA_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define TIVA_GPIOC_BASE     (TIVA_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define TIVA_GPIOD_BASE     (TIVA_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define TIVA_SSI0_BASE      (TIVA_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
                                                           /* -0x0bfff: Reserved */
#  define TIVA_UART0_BASE     (TIVA_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define TIVA_UART1_BASE     (TIVA_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define TIVA_I2C0_BASE      (TIVA_PERIPH_BASE + 0x20000)  /* -0x20fff: I2C0 */
                                                            /* -0x23fff: Reserved */
#  define TIVA_GPIOE_BASE     (TIVA_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define TIVA_GPIOF_BASE     (TIVA_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define TIVA_GPIOG_BASE     (TIVA_PERIPH_BASE + 0x26000)  /* -0x26fff: GPIO Port G */
                                                            /* -0x27fff: Reserved */
#  define TIVA_PWM0_BASE      (TIVA_PERIPH_BASE + 0x28000)  /* -0x28fff: PWM */
                                                            /* -0x2ffff: Reserved */
#  define TIVA_TIMER0_BASE    (TIVA_PERIPH_BASE + 0x30000)  /* -0x30fff: Timer 0 */
#  define TIVA_TIMER1_BASE    (TIVA_PERIPH_BASE + 0x31000)  /* -0x31fff: Timer 1 */
#  define TIVA_TIMER2_BASE    (TIVA_PERIPH_BASE + 0x32000)  /* -0x32fff: Timer 2 */
                                                            /* -0x37fff: Reserved */
#  define TIVA_ADC_BASE       (TIVA_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC */
                                                            /* -0x3bfff: Reserved */
#  define TIVA_CMP_BASE       (TIVA_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
                                                            /* -0x47fff: Reserved */
#  define TIVA_ETHCON_BASE    (TIVA_PERIPH_BASE + 0x48000)  /* -0x48fff: Ethernet Controller */
                                                            /* -0xfcfff: Reserved */
#  define TIVA_HIBERNATE_BASE (TIVA_PERIPH_BASE + 0xfc000)  /* -0xfcfff: Hibernation Controller */
#  define TIVA_FLASHCON_BASE  (TIVA_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define TIVA_SYSCON_BASE    (TIVA_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
                                                            /* -0x1ffffff: Reserved */

#elif defined(CONFIG_ARCH_CHIP_LM3S6965)
/* FiRM Peripheral Base Addresses */

#  define TIVA_WDOG_BASE      (TIVA_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer */
                                                           /* -0x03fff: Reserved */
#  define TIVA_GPIOA_BASE     (TIVA_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define TIVA_GPIOB_BASE     (TIVA_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define TIVA_GPIOC_BASE     (TIVA_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define TIVA_GPIOD_BASE     (TIVA_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define TIVA_SSI0_BASE      (TIVA_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
                                                           /* -0x0bfff: Reserved */
#  define TIVA_UART0_BASE     (TIVA_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define TIVA_UART1_BASE     (TIVA_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
#  define TIVA_UART2_BASE     (TIVA_PERIPH_BASE + 0x0e000) /* -0x0dfff: UART2 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define TIVA_I2C0_BASE      (TIVA_PERIPH_BASE + 0x20000)  /* -0x20fff: I2C0 */
#  define TIVA_I2C1_BASE      (TIVA_PERIPH_BASE + 0x21000)  /* -0x21fff: I2C1 */
                                                            /* -0x23fff: Reserved */
#  define TIVA_GPIOE_BASE     (TIVA_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define TIVA_GPIOF_BASE     (TIVA_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define TIVA_GPIOG_BASE     (TIVA_PERIPH_BASE + 0x26000)  /* -0x26fff: GPIO Port G */
                                                            /* -0x27fff: Reserved */
#  define TIVA_PWM0_BASE      (TIVA_PERIPH_BASE + 0x28000)  /* -0x28fff: PWM */
                                                            /* -0x2bfff: Reserved */
#  define TIVA_QEI0_BASE      (TIVA_PERIPH_BASE + 0x2c000)  /* -0x2cfff: QEI0 */
#  define TIVA_QEI1_BASE      (TIVA_PERIPH_BASE + 0x2d000)  /* -0x2dfff: QEI1 */
                                                            /* -0x2ffff: Reserved */
#  define TIVA_TIMER0_BASE    (TIVA_PERIPH_BASE + 0x30000)  /* -0x30fff: Timer 0 */
#  define TIVA_TIMER1_BASE    (TIVA_PERIPH_BASE + 0x31000)  /* -0x31fff: Timer 1 */
#  define TIVA_TIMER2_BASE    (TIVA_PERIPH_BASE + 0x32000)  /* -0x32fff: Timer 2 */
#  define TIVA_TIMER3_BASE    (TIVA_PERIPH_BASE + 0x33000)  /* -0x33fff: Timer 3 */
                                                            /* -0x37fff: Reserved */
#  define TIVA_ADC_BASE       (TIVA_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC */
                                                            /* -0x3bfff: Reserved */
#  define TIVA_CMP_BASE       (TIVA_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
                                                            /* -0x47fff: Reserved */
#  define TIVA_ETHCON_BASE    (TIVA_PERIPH_BASE + 0x48000)  /* -0x48fff: Ethernet Controller */
                                                            /* -0xfcfff: Reserved */
#  define TIVA_HIBERNATE_BASE (TIVA_PERIPH_BASE + 0xfc000)  /* -0xfcfff: Hibernation Controller */
#  define TIVA_FLASHCON_BASE  (TIVA_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define TIVA_SYSCON_BASE    (TIVA_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
                                                            /* -0x1ffffff: Reserved */
#elif defined(CONFIG_ARCH_CHIP_LM3S8962)
/* FiRM Peripheral Base Addresses */

#  define TIVA_WDOG_BASE      (TIVA_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer */
                                                           /* -0x03fff: Reserved */
#  define TIVA_GPIOA_BASE     (TIVA_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define TIVA_GPIOB_BASE     (TIVA_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define TIVA_GPIOC_BASE     (TIVA_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define TIVA_GPIOD_BASE     (TIVA_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define TIVA_SSI0_BASE      (TIVA_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
                                                           /* -0x0bfff: Reserved */
#  define TIVA_UART0_BASE     (TIVA_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define TIVA_UART1_BASE     (TIVA_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define TIVA_I2C0_BASE      (TIVA_PERIPH_BASE + 0x20000)  /* -0x20fff: I2C0 */
                                                            /* -0x23fff: Reserved */
#  define TIVA_GPIOE_BASE     (TIVA_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define TIVA_GPIOF_BASE     (TIVA_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define TIVA_GPIOG_BASE     (TIVA_PERIPH_BASE + 0x26000)  /* -0x26fff: GPIO Port G */
                                                            /* -0x27fff: Reserved */
#  define TIVA_PWM0_BASE      (TIVA_PERIPH_BASE + 0x28000)  /* -0x28fff: PWM */
                                                            /* -0x2bfff: Reserved */
#  define TIVA_QEI0_BASE      (TIVA_PERIPH_BASE + 0x2c000)  /* -0x2cfff: QEI0 */
#  define TIVA_QEI1_BASE      (TIVA_PERIPH_BASE + 0x2d000)  /* -0x2dfff: QEI1 */
                                                            /* -0x2ffff: Reserved */
#  define TIVA_TIMER0_BASE    (TIVA_PERIPH_BASE + 0x30000)  /* -0x30fff: Timer 0 */
#  define TIVA_TIMER1_BASE    (TIVA_PERIPH_BASE + 0x31000)  /* -0x31fff: Timer 1 */
#  define TIVA_TIMER2_BASE    (TIVA_PERIPH_BASE + 0x32000)  /* -0x32fff: Timer 2 */
#  define TIVA_TIMER3_BASE    (TIVA_PERIPH_BASE + 0x33000)  /* -0x33fff: Timer 3 */
                                                            /* -0x37fff: Reserved */
#  define TIVA_ADC_BASE       (TIVA_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC */
                                                            /* -0x3bfff: Reserved */
#  define TIVA_CMP_BASE       (TIVA_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
                                                            /* -0x3fffff: Reserved */
#  define TIVA_CAN0_BASE      (TIVA_PERIPH_BASE + 0x40000)  /* -0x40fff: CAN Controller */
                                                            /* -0x47fff: Reserved */
#  define TIVA_ETHCON_BASE    (TIVA_PERIPH_BASE + 0x48000)  /* -0x48fff: Ethernet Controller */
                                                            /* -0xfcfff: Reserved */
#  define TIVA_HIBERNATE_BASE (TIVA_PERIPH_BASE + 0xfc000)  /* -0xfcfff: Hibernation Controller */
#  define TIVA_FLASHCON_BASE  (TIVA_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define TIVA_SYSCON_BASE    (TIVA_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
                                                            /* -0x1ffffff: Reserved */
#elif defined(CONFIG_ARCH_CHIP_LM3S9B96)
/* FiRM Peripheral Base Addresses */

#  define TIVA_WDOG_BASE      (TIVA_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer */
                                                           /* -0x03fff: Reserved */
#  define TIVA_GPIOA_BASE     (TIVA_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define TIVA_GPIOB_BASE     (TIVA_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define TIVA_GPIOC_BASE     (TIVA_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define TIVA_GPIOD_BASE     (TIVA_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define TIVA_SSI0_BASE      (TIVA_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
#  define TIVA_SSI1_BASE      (TIVA_PERIPH_BASE + 0x09000) /* -0x09fff: SSI0 */
                                                           /* -0x0bfff: Reserved */
#  define TIVA_UART0_BASE     (TIVA_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define TIVA_UART1_BASE     (TIVA_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
#  define TIVA_UART2_BASE     (TIVA_PERIPH_BASE + 0x0e000) /* -0x0dfff: UART2 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define TIVA_I2C0_BASE      (TIVA_PERIPH_BASE + 0x20000)  /* -0x207ff: I2C0 */
#  define TIVA_I2C1_BASE      (TIVA_PERIPH_BASE + 0x21000)  /* -0x217ff: I2C1 */
                                                            /* -0x23fff: Reserved */
#  define TIVA_GPIOE_BASE     (TIVA_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define TIVA_GPIOF_BASE     (TIVA_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define TIVA_GPIOG_BASE     (TIVA_PERIPH_BASE + 0x26000)  /* -0x26fff: GPIO Port G */
#  define TIVA_GPIOH_BASE     (TIVA_PERIPH_BASE + 0x27000)  /* -0x27fff: GPIO Port H */

#  define TIVA_PWM0_BASE      (TIVA_PERIPH_BASE + 0x28000)  /* -0x28fff: PWM */
                                                            /* -0x2bfff: Reserved */
#  define TIVA_QEI0_BASE      (TIVA_PERIPH_BASE + 0x2c000)  /* -0x2cfff: QEI0 */
#  define TIVA_QEI1_BASE      (TIVA_PERIPH_BASE + 0x2d000)  /* -0x2dfff: QEI1 */
                                                            /* -0x2ffff: Reserved */
#  define TIVA_TIMER0_BASE    (TIVA_PERIPH_BASE + 0x30000)  /* -0x30fff: Timer 0 */
#  define TIVA_TIMER1_BASE    (TIVA_PERIPH_BASE + 0x31000)  /* -0x31fff: Timer 1 */
#  define TIVA_TIMER2_BASE    (TIVA_PERIPH_BASE + 0x32000)  /* -0x32fff: Timer 2 */
#  define TIVA_TIMER3_BASE    (TIVA_PERIPH_BASE + 0x33000)  /* -0x33fff: Timer 3 */
                                                            /* -0x37fff: Reserved */
#  define TIVA_ADC0_BASE       (TIVA_PERIPH_BASE + 0x38000) /* -0x38fff: ADC 0 */
#  define TIVA_ADC1_BASE       (TIVA_PERIPH_BASE + 0x39000) /* -0x39fff: ADC 1 */
                                                            /* -0x3bfff: Reserved */
#  define TIVA_CMP_BASE       (TIVA_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
#  define TIVA_GPIOJ_BASE     (TIVA_PERIPH_BASE + 0x3d000)  /* -0x3dfff: GPIO Port J */
                                                            /* -0x3ffff: Reserved */
#  define TIVA_CAN0_BASE      (TIVA_PERIPH_BASE + 0x40000)  /* -0x40fff: CAN 0 */
#  define TIVA_CAN1_BASE      (TIVA_PERIPH_BASE + 0x41000)  /* -0x41fff: CAN 1 */
                                                            /* -0x47fff: Reserved */
#  define TIVA_ETHCON_BASE    (TIVA_PERIPH_BASE + 0x48000)  /* -0x48fff: Ethernet Controller */
                                                            /* -0x49fff: Reserved */
#  define TIVA_USB_BASE       (TIVA_PERIPH_BASE + 0x50000)  /* -0x50fff: USB */
                                                            /* -0x53fff: Reserved */
#  define TIVA_I2S0_BASE      (TIVA_PERIPH_BASE + 0x54000)  /* -0x54fff: I2S 0 */
                                                            /* -0x57fff: Reserved */
#  define TIVA_GPIOAAHB_BASE  (TIVA_PERIPH_BASE + 0x58000)  /* -0x58fff: GPIO Port A (AHB aperture) */
#  define TIVA_GPIOBAHB_BASE  (TIVA_PERIPH_BASE + 0x59000)  /* -0x59fff: GPIO Port B (AHB aperture) */
#  define TIVA_GPIOCAHB_BASE  (TIVA_PERIPH_BASE + 0x5a000)  /* -0x5afff: GPIO Port C (AHB aperture) */
#  define TIVA_GPIODAHB_BASE  (TIVA_PERIPH_BASE + 0x5b000)  /* -0x5bfff: GPIO Port D (AHB aperture) */
#  define TIVA_GPIOEAHB_BASE  (TIVA_PERIPH_BASE + 0x5c000)  /* -0x5cfff: GPIO Port E (AHB aperture) */
#  define TIVA_GPIOFAHB_BASE  (TIVA_PERIPH_BASE + 0x5d000)  /* -0x5dfff: GPIO Port F (AHB aperture) */
#  define TIVA_GPIOGAHB_BASE  (TIVA_PERIPH_BASE + 0x5e000)  /* -0x5efff: GPIO Port G (AHB aperture) */
#  define TIVA_GPIOHAHB_BASE  (TIVA_PERIPH_BASE + 0x5f000)  /* -0x5ffff: GPIO Port H (AHB aperture) */
#  define TIVA_GPIOJAHB_BASE  (TIVA_PERIPH_BASE + 0x60000)  /* -0x60fff: GPIO Port J (AHB aperture) */
                                                            /* -0xcffff: Reserved */
#  define TIVA_EPI0_BASE      (TIVA_PERIPH_BASE + 0xd0000)  /* -0xd0fff: EPI 0 */
                                                            /* -0xfcfff: Reserved */
#  define TIVA_FLASHCON_BASE  (TIVA_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define TIVA_SYSCON_BASE    (TIVA_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
#  define TIVA_UDMA_BASE      (TIVA_PERIPH_BASE + 0xff000)  /* -0xfffff: Micro Direct Memory Access */
                                                            /* -0x1ffffff: Reserved */
#else
#  error "Peripheral base addresses not specified for this Stellaris chip"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_CHIP_LM3S_MEMORYMAP_H */
